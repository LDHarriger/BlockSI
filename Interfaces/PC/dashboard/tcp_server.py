"""
Async TCP Server — ESP32 connects to the dashboard.
"""
from __future__ import annotations

import asyncio
import re
import time
from datetime import datetime
from typing import Optional

from dashboard.state import (
    S, log, _notify,
    parse_data_line, apply_telemetry,
    data_buf, compute_effective_o2_pct,
    DEFAULT_PORT, DIAGNOSTICS_DIR,
)
from dashboard.data_io import csv_logger, _save_cal_csv

# Forward reference: set by k_d_cal module after import
_cstr_debug_file_ref = None


def _get_cstr_debug_file():
    """Get the current CSTR debug file path (set by k_d_cal module)."""
    from dashboard import k_d_cal
    return k_d_cal._cstr_debug_file


class TCPServer:
    def __init__(self, port: int = DEFAULT_PORT) -> None:
        self.port = port
        self._server: Optional[asyncio.AbstractServer] = None
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._pending_responses: dict[str, tuple[asyncio.Event, list]] = {}
        self._pending_lock = asyncio.Lock()
        self._running = False

    # -- lifecycle --------------------------------------------------------
    async def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._server = await asyncio.start_server(
            self._on_connect, "0.0.0.0", self.port
        )
        log(f"TCP server listening on port {self.port}")

    async def stop(self) -> None:
        self._running = False
        await self._close_client()
        if self._server:
            self._server.close()
            await self._server.wait_closed()
            self._server = None

    async def _close_client(self) -> None:
        if self._writer:
            try:
                self._writer.close()
                await self._writer.wait_closed()
            except Exception:
                pass
            self._writer = None
            self._reader = None
        S.connected = False
        S.time_synced = False

    # -- connection handler -----------------------------------------------
    async def _on_connect(
        self,
        reader: asyncio.StreamReader,
        writer: asyncio.StreamWriter,
    ) -> None:
        addr = writer.get_extra_info("peername")
        log(f"ESP32 connected from {addr}")
        await self._close_client()
        self._reader = reader
        self._writer = writer
        S.connected = True
        S.backfill_active = False
        S.backfill_expected = 0
        S.backfill_received = 0

        csv_logger.reset(datetime.now())

        pc_ms = int(time.time() * 1000)
        await self._send_raw(f"CMD,time_sync,{pc_ms}\n")

        buf = ""
        try:
            while self._running:
                try:
                    raw = await asyncio.wait_for(reader.read(4096), timeout=1.0)
                except asyncio.TimeoutError:
                    continue
                if not raw:
                    break
                buf += raw.decode("utf-8", errors="ignore")
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    await self._dispatch(line)
        except Exception as exc:
            log(f"TCP read error: {exc}", "error")
        finally:
            if self._writer is writer:
                log("ESP32 disconnected", "warn")
                await self._close_client()
                S.backfill_active = False
                S.backfill_expected = 0
                S.backfill_received = 0
                if S.sequence_active:
                    S.sequence_active = False
                    S.seq_confirmed = False
                    S.pending_prompt_id = ""
                    S.pending_prompt_text = ""
                    _notify("Sequence aborted — ESP32 disconnected", "negative")
            else:
                log(f"Old connection from {addr} cleaned up (new connection active)", "info")

    # -- dispatch ---------------------------------------------------------
    async def _dispatch(self, line: str) -> None:
        prefix = line.split(",", 1)[0]
        try:
            if prefix == "BACKFILL_START":
                self._handle_backfill_start(line)
            elif prefix == "BACKFILL_END":
                self._handle_backfill_end()
            elif prefix == "DATA":
                sample = parse_data_line(line)
                if sample:
                    if S.backfill_active:
                        S.backfill_received += 1
                        data_buf.append(sample)
                        csv_logger.write(sample)
                    else:
                        apply_telemetry(sample)
                        data_buf.append(sample)
                        csv_logger.write(sample)
            elif prefix == "RSP":
                self._handle_rsp(line)
            elif prefix == "STATE":
                self._handle_state(line)
            elif prefix == "SEQ":
                await self._handle_seq(line)
            elif prefix == "DIAG":
                self._handle_diag(line)
            else:
                log(f"Unknown line: {line[:80]}", "warn")
        except Exception as exc:
            log(f"Dispatch error ({prefix}): {exc}  line={line[:80]}", "error")

    # -- Backfill handlers ------------------------------------------------
    def _handle_backfill_start(self, line: str) -> None:
        parts = line.split(",")
        n = int(parts[1]) if len(parts) >= 2 else 0
        S.backfill_active = True
        S.backfill_start_time = time.time()
        S.backfill_expected = n
        S.backfill_received = 0
        log(f"Backfill started — expecting {n} cached DATA lines", "info")

    def _handle_backfill_end(self) -> None:
        log(
            f"Backfill complete — received {S.backfill_received}"
            f"/{S.backfill_expected} samples",
            "info",
        )
        _notify(
            f"Backfill: {S.backfill_received} cached samples recovered",
            "positive",
        )
        S.backfill_active = False
        S.backfill_expected = 0
        S.backfill_received = 0

    # -- RSP handler ------------------------------------------------------
    def _handle_rsp(self, line: str) -> None:
        matched = False
        if "time_sync" in line and "esp=" in line:
            m_esp = re.search(r"esp=(\d+)", line)
            m_pc = re.search(r"pc=(\d+)", line)
            if m_esp and m_pc:
                S.esp_time_offset_ms = (
                    int(m_pc.group(1)) - int(m_esp.group(1))
                )
                S.time_synced = True
                log(f"Time synced (offset={S.esp_time_offset_ms} ms)")
            matched = True
        if "relay_get" in line:
            for part in line.split(","):
                if "=" not in part:
                    continue
                k, v = part.split("=", 1)
                on = v.strip() == "1"
                if k == "ozone_gen":
                    S.relay_o3_gen = on
                elif k == "o2_conc":
                    S.relay_o2_conc = on
                elif k == "air_comp":
                    S.relay_air_comp = on
        log(f"<- {line}", "recv")
        parts = line.split(",")
        rsp_cmd = parts[2].strip() if len(parts) >= 3 else ""
        if rsp_cmd and rsp_cmd in self._pending_responses:
            event, result_box = self._pending_responses[rsp_cmd]
            result_box.append(line)
            event.set()
            matched = True
        if not matched:
            log(f"Unmatched RSP (no waiter for '{rsp_cmd}'): {line[:60]}", "warn")

    # -- STATE push -------------------------------------------------------
    def _handle_state(self, line: str) -> None:
        for part in line.split(","):
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            k = k.strip()
            try:
                if k == "ozone_gen":
                    S.relay_o3_gen = v.strip() == "1"
                elif k == "o2_conc":
                    S.relay_o2_conc = v.strip() == "1"
                elif k == "air_comp":
                    S.relay_air_comp = v.strip() == "1"
                elif k == "power":
                    S.power_actual_pct = float(v)
                elif k == "flow":
                    S.flow_lpm = float(v)
            except (ValueError, IndexError):
                pass
        log("STATE sync from ESP32", "state")

    # -- DIAG handler -----------------------------------------------------
    _diag_log_file: Optional[str] = None
    _DIAG_SUBTYPES = {"drift_sample", "drift_summary", "noise_stats", "power_mismatch", "power_resolved",
                       "room_o3_raw", "room_o3_delay", "room_o3_regs"}

    def _handle_diag(self, line: str) -> None:
        log(f"DIAG: {line}", "warn")
        _notify(f"FW DIAG: {line}", "warning")
        cstr_debug_file = _get_cstr_debug_file()
        if cstr_debug_file:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            try:
                with open(cstr_debug_file, "a") as _f:
                    _f.write(f"[{ts}] {line}\n")
            except OSError:
                pass
        # Persist power-diagnostic subtypes to Data/Diagnostics/
        parts = line.split(",")
        if len(parts) >= 2:
            subtype = parts[1]
            if subtype in self._DIAG_SUBTYPES:
                self._write_diag_log(line)

    def _write_diag_log(self, line: str) -> None:
        """Append a DIAG line to a timestamped file in Data/Diagnostics/."""
        import os
        if self._diag_log_file is None:
            ts = datetime.now().strftime("%Y-%m-%d_%H%M%S")
            self._diag_log_file = os.path.join(DIAGNOSTICS_DIR, f"{ts}_diag.log")
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        try:
            with open(self._diag_log_file, "a") as f:
                f.write(f"[{ts}] {line}\n")
        except OSError:
            pass

    # -- SEQ handler ------------------------------------------------------
    async def _handle_seq(self, line: str) -> None:
        parts = line.split(",")
        if len(parts) < 3:
            log(f"Malformed SEQ: {line[:80]}", "warn")
            return

        seq_type = parts[1]
        action = parts[2]

        if action == "STARTED":
            S.sequence_active = True
            S.seq_type = seq_type
            S.seq_phase = "started"
            S.seq_progress = 0.0
            S.seq_elapsed = 0.0
            S.seq_step_idx = 0
            for part in parts[3:]:
                if "=" not in part:
                    continue
                k, v = part.split("=", 1)
                try:
                    if k.strip() == "steps":
                        S.seq_step_total = int(v)
                    elif k.strip() == "flow":
                        if seq_type == "calibrate":
                            S.cal_lpm = float(v)
                except ValueError:
                    pass
            log(f"Sequence '{seq_type}' started ({S.seq_step_total} steps)", "seq")
            _notify(f"{seq_type.title()} sequence started")

        elif action == "RELAY":
            S.seq_phase = "relay_setup"
            if len(parts) >= 5:
                k = parts[3].strip()
                on = parts[4].strip() in ("1", "ON")
                if k == "ozone_gen":
                    S.relay_o3_gen = on
                elif k == "o2_conc":
                    S.relay_o2_conc = on
                elif k == "air_comp":
                    S.relay_air_comp = on
            log(f"SEQ relay: {','.join(parts[3:])}", "seq")

        elif action == "STATUS":
            status = parts[3].strip() if len(parts) > 3 else ""
            if status == "relay_stabilizing":
                S.seq_phase = "stabilizing"
            log(f"SEQ status: {status}", "seq")

        elif action == "STEP":
            if len(parts) >= 6:
                try:
                    S.seq_step_idx = int(parts[3])
                    S.seq_power = float(parts[4])
                    S.seq_phase = parts[5].strip()
                    if S.seq_step_total > 0:
                        S.seq_progress = (
                            S.seq_step_idx / S.seq_step_total
                        ) * 100
                except (ValueError, IndexError):
                    pass
                if len(parts) >= 7:
                    try:
                        S.relay_air_comp = parts[6].strip() == "1"
                    except (ValueError, IndexError):
                        pass
            log(
                f"STEP {S.seq_step_idx}/{S.seq_step_total}: "
                f"{S.seq_phase} @ {S.seq_power:.0f}%",
                "seq",
            )

        elif action == "SAMPLE":
            if len(parts) >= 8:
                try:
                    air = int(parts[8]) if len(parts) >= 9 else 0
                    sample = {
                        "step_idx": int(parts[3]),
                        "sample_num": int(parts[4]),
                        "o3_pct": float(parts[5]),
                        "temp_c": float(parts[6]),
                        "power_actual": float(parts[7]),
                        "air_comp": air,
                        "phase": S.seq_phase,
                        "power_target": S.seq_power,
                    }
                    if seq_type == "calibrate":
                        S.cal_samples.append(sample)
                    elif seq_type == "verify":
                        S.verify_samples.append(sample)
                    elif seq_type == "process_batch":
                        S.batch_samples.append(sample)
                except (ValueError, IndexError):
                    log(f"Bad SAMPLE: {line[:80]}", "error")

        elif action == "PROMPT":
            S.pending_prompt_id = parts[3] if len(parts) > 3 else ""
            S.pending_prompt_text = (
                ",".join(parts[4:]) if len(parts) > 4 else ""
            )
            log(f"Prompt requested: {S.pending_prompt_id}", "seq")

        elif action == "COMPLETE":
            elapsed = 0.0
            if len(parts) > 3:
                try:
                    elapsed = float(parts[3])
                except ValueError:
                    pass
            S.seq_elapsed = elapsed
            log(f"Sequence '{seq_type}' complete ({elapsed:.0f}s)", "seq")
            S.seq_phase = "saving"
            try:
                if seq_type == "calibrate":
                    if S.cal_samples:
                        o2 = compute_effective_o2_pct(
                            S.cal_lpm, S.relay_air_comp
                        )
                        S.cal_file = _save_cal_csv(
                            S.cal_samples, S.cal_lpm, o2
                        )
                        _notify(
                            f"Calibration: {len(S.cal_samples)} samples saved",
                            "positive",
                        )
                    else:
                        _notify(
                            "Calibration complete (no samples)", "warning"
                        )
                else:
                    _notify(
                        f"{seq_type.title()} completed ({elapsed:.0f}s)",
                        "positive",
                    )
            except Exception as exc:
                log(f"Post-sequence analysis error: {exc}", "error")
                _notify(f"Sequence error: {exc}", "negative")
            finally:
                S.pending_prompt_id = ""
                S.pending_prompt_text = ""
                S.seq_phase = "complete"
                S.sequence_active = False
                S.seq_cleanup_pending = True
                log(
                    "Sequence complete — cleanup deferred to _tick",
                    "seq",
                )

        elif action == "ABORTED":
            reason = ",".join(parts[3:]) if len(parts) > 3 else "unknown"
            log(f"Sequence '{seq_type}' aborted: {reason}", "seq")
            _notify(f"{seq_type.title()} aborted: {reason}", "warning")
            S.pending_prompt_id = ""
            S.pending_prompt_text = ""
            S.seq_phase = "aborted"
            S.sequence_active = False
            S.seq_cleanup_pending = True
            log("Sequence aborted — cleanup deferred to _tick", "seq")

        else:
            log(f"Unknown SEQ action: {action} in {line[:80]}", "warn")

    # -- sending ----------------------------------------------------------
    async def _send_raw(self, text: str) -> bool:
        if self._writer is None:
            return False
        try:
            self._writer.write(text.encode())
            await self._writer.drain()
            return True
        except Exception:
            return False

    async def send_command(self, cmd: str, timeout: float = 2.0) -> Optional[str]:
        if not S.connected:
            return None
        cmd_name = cmd.split(",", 1)[0]
        event = asyncio.Event()
        result_box: list[str] = []
        async with self._pending_lock:
            if cmd_name in self._pending_responses:
                old_event, _ = self._pending_responses[cmd_name]
                old_event.set()
            self._pending_responses[cmd_name] = (event, result_box)
        if not await self._send_raw(f"CMD,{cmd}\n"):
            async with self._pending_lock:
                self._pending_responses.pop(cmd_name, None)
            return None
        log(f"-> {cmd}", "send")
        try:
            await asyncio.wait_for(event.wait(), timeout)
            return result_box[0] if result_box else None
        except asyncio.TimeoutError:
            log(f"timeout: {cmd}", "error")
            return None
        finally:
            async with self._pending_lock:
                self._pending_responses.pop(cmd_name, None)


# Module-level singleton — imported by commands.py, ui_main.py, etc.
# Created eagerly so `from dashboard.tcp_server import tcp` captures the
# real instance (not None).  Port is set and .start() called in _startup().
tcp = TCPServer()
