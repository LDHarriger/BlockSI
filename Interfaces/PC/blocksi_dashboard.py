#!/usr/bin/env python3
"""
BlockSI Dashboard — NiceGUI, Observer-Mode Sequences
Run with:  .venv\\Scripts\\python.exe blocksi_dashboard.py [--port 5000]

Architecture:
  - ESP32 connects to PC via TCP on port 5000
  - ESP32 owns all sequence execution (calibration, validation, etc.)
  - PC sends CMD,sequence_start/stop/confirm — observes SEQ/CAL/VAL streams
  - PC is sole authority for power_target_pct (never from telemetry)

CRITICAL: Commands are COMMA-separated  →  CMD,power_set,50\\n
          NEVER use colons (CMD,power_set:50) — silently fails on ESP32.
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import json
import math
import os
import re
import time
from collections import deque
from datetime import datetime
from typing import Any, Optional

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from nicegui import ui, app

# =============================================================================
# Configuration & Paths
# =============================================================================
DEFAULT_PORT = 5000
MAX_DATA_POINTS = 500

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, "Data")
STREAM_DIR = os.path.join(DATA_DIR, "Stream")
CALIBRATION_DIR = os.path.join(DATA_DIR, "O3PowerCalibration")
MODEL_DIR = os.path.join(BASE_DIR, "Model", "O3Power")

for _d in (DATA_DIR, STREAM_DIR, CALIBRATION_DIR, MODEL_DIR):
    os.makedirs(_d, exist_ok=True)

# Power model coefficients  (O3_max = A/F + B)
POWER_MODEL_A = 1.78
POWER_MODEL_B = 1.40
DEFAULT_FLOW_LPM = 4.0

# O3 mass-flow conversion  mg/s = %vol * LPM * K
O3_MASS_FLOW_K = 0.357
AIR_COMP_LPM = 10.0

POWER_MISMATCH_THRESHOLD = 5.0

# =============================================================================
# Prompt content mapping — rich descriptions for ESP32 prompt IDs
# =============================================================================
PROMPT_CONTENT: dict[str, dict[str, str]] = {
    "prompt_vessel": {
        "title": "Step 1 — Route to Vessel",
        "icon": "air",
        "body": (
            "<b>Turn the L-valve</b> so airflow goes through the sterilization "
            "vessel.<br><br>"
            "Verify the rotameter reads the target LPM.<br><br>"
            "Press <b>Confirm</b> when ready."
        ),
    },
    "prompt_direct": {
        "title": "Step 2 — Route Direct to Sensor",
        "icon": "sensors",
        "body": (
            "<b>Turn the L-valve</b> so airflow goes directly to the 106-H "
            "sensor, bypassing the vessel.<br><br>"
            "Adjust the needle valve until the rotameter matches the vessel "
            "route flow.<br><br>"
            "Press <b>Confirm</b> when ready."
        ),
    },
}


# =============================================================================
# Conversion helpers  (pure functions — ported from v9)
# =============================================================================
def predict_o3_from_power(power_pct: float, flow_lpm: float) -> float:
    """Piecewise model: threshold → linear ramp → saturation."""
    if power_pct <= 0 or flow_lpm <= 0:
        return 0.0
    o3_max = POWER_MODEL_A / flow_lpm + POWER_MODEL_B
    if power_pct < 20:
        scaling = (power_pct / 20) * 0.1
    elif power_pct <= 75:
        scaling = 0.1 + (power_pct - 20) / 55 * 0.9
    else:
        scaling = 1.0
    return o3_max * scaling


def predict_power_from_o3(o3_pct: float, flow_lpm: float) -> float:
    """Inverse of predict_o3_from_power."""
    if o3_pct <= 0 or flow_lpm <= 0:
        return 0.0
    o3_max = POWER_MODEL_A / flow_lpm + POWER_MODEL_B
    scaling = o3_pct / o3_max
    if scaling >= 1.0:
        return 100.0
    elif scaling <= 0.1:
        return (scaling / 0.1) * 20
    else:
        return 20 + (scaling - 0.1) / 0.9 * 55


def o3_pct_to_mg_per_s(o3_pct: float, flow_lpm: float) -> float:
    return o3_pct * flow_lpm * O3_MASS_FLOW_K


def mg_per_s_to_o3_pct(mg_per_s: float, flow_lpm: float) -> float:
    if flow_lpm <= 0:
        return 0.0
    return mg_per_s / (flow_lpm * O3_MASS_FLOW_K)


def mg_per_s_to_g_at_time(mg_per_s: float, minutes: float = 30.0) -> float:
    return mg_per_s * minutes * 60 / 1000


def g_at_time_to_mg_per_s(grams: float, minutes: float = 30.0) -> float:
    if minutes <= 0:
        return 0.0
    return grams * 1000 / (minutes * 60)


def generate_power_curve(flow_lpm: float):
    pwr = np.linspace(0, 100, 101)
    o3 = [predict_o3_from_power(p, flow_lpm) for p in pwr]
    return pwr, o3


# =============================================================================
# SystemState  (single source of truth)
# =============================================================================
class SystemState:
    def __init__(self) -> None:
        # Power — PC is sole authority for target
        self.power_target_pct: int = 0
        self.power_actual_pct: float = 0.0
        self.wiper_voltage: float = 0.0
        self.power_error: bool = False
        # Flow
        self.flow_lpm: float = DEFAULT_FLOW_LPM
        # Relays
        self.relay_o3_gen: bool = False
        self.relay_o2_conc: bool = False
        self.relay_air_comp: bool = False
        # Sensors
        self.vessel_o3_pct: float = 0.0
        self.room_o3_ppm: float = 0.0
        self.vessel_temp_c: float = -999.0
        self.cell_temp_c: float = 0.0
        self.pressure_mbar: float = 0.0
        # Timing
        self.esp_time_offset_ms: int = 0
        self.time_synced: bool = False
        self.last_update: Optional[datetime] = None
        # Connection
        self.connected: bool = False
        # ------------------------------------------------------------------
        # Sequence observer state  (ESP32 owns execution)
        # ------------------------------------------------------------------
        self.sequence_active: bool = False
        self.seq_type: str = ""            # "cal", "validate", etc.
        self.seq_phase: str = ""           # current phase name
        self.seq_progress: float = 0.0     # 0-100
        self.seq_elapsed: float = 0.0      # seconds
        self.seq_power: float = 0.0        # power ESP32 is commanding
        self.seq_air: bool = False         # air state during sequence
        # Pending prompt from ESP32
        self.pending_prompt_id: str = ""
        self.pending_prompt_text: str = ""
        # Calibration observer
        self.cal_samples: list[dict] = []
        self.cal_lpm: float = DEFAULT_FLOW_LPM
        self.cal_file: str = ""
        # Validation observer
        self.val_power: float = 75.0
        self.val_lpm: float = DEFAULT_FLOW_LPM
        self.val_samples: list[dict] = []
        self.val_result: dict = {}
        # Settings
        self.notify_level: str = "all"     # "all" | "errors" | "none"
        # Derived (kept in sync by update_derived)
        self.target_o3_pct: float = 0.0
        self.target_mg_per_s: float = 0.0
        self.target_g_30min: float = 0.0

    def update_derived(self) -> None:
        self.target_o3_pct = round(
            predict_o3_from_power(self.power_target_pct, self.flow_lpm), 4
        )
        self.target_mg_per_s = round(
            o3_pct_to_mg_per_s(self.target_o3_pct, self.flow_lpm), 4
        )
        self.target_g_30min = round(
            mg_per_s_to_g_at_time(self.target_mg_per_s, 30.0), 4
        )


S = SystemState()


# =============================================================================
# Data parsing  (verbatim port from v9)
# =============================================================================
def parse_data_line(line: str) -> Optional[dict]:
    parts = line.split(",")
    if len(parts) < 5:
        return None

    def _f(idx: int, default: float = 0.0) -> float:
        try:
            return float(parts[idx]) if idx < len(parts) and parts[idx] else default
        except (ValueError, IndexError):
            return default

    def _i(idx: int, default: int = 0) -> int:
        try:
            return int(parts[idx]) if idx < len(parts) and parts[idx] else default
        except (ValueError, IndexError):
            return default

    esp_ts_ms = _i(1)
    if S.time_synced:
        timestamp = datetime.fromtimestamp(
            (esp_ts_ms + S.esp_time_offset_ms) / 1000.0
        )
    else:
        timestamp = datetime.now()

    return {
        "timestamp": timestamp,
        "esp_ts_ms": esp_ts_ms,
        "vessel_o3_pct": _f(2),
        "cell_temp_c": _f(3),
        "pressure_mbar": _f(4),
        "sample_v": _f(5),
        "ref_v": _f(6),
        "day": _i(7),
        "month": _i(8),
        "year": _i(9),
        "hour": _i(10),
        "minute": _i(11),
        "second": _i(12),
        "room_o3_ppm": _f(13),
        "vessel_temp_c": _f(14, -999.0),
        "power_target_pct": _i(15),
        "power_actual_pct": _f(16),
        "wiper_voltage": _f(17),
    }


def apply_telemetry(sample: dict) -> None:
    """Push a parsed telemetry dict into SystemState."""
    S.power_actual_pct = sample.get("power_actual_pct", 0.0)
    S.wiper_voltage = sample.get("wiper_voltage", 0.0)
    S.vessel_o3_pct = sample.get("vessel_o3_pct", 0.0)
    S.room_o3_ppm = sample.get("room_o3_ppm", 0.0)
    S.vessel_temp_c = sample.get("vessel_temp_c", -999.0)
    S.cell_temp_c = sample.get("cell_temp_c", 0.0)
    S.pressure_mbar = sample.get("pressure_mbar", 0.0)
    S.last_update = sample.get("timestamp")
    # power_target_pct: PC is sole authority — never accept from telemetry
    S.update_derived()
    S.power_error = (
        abs(S.power_target_pct - S.power_actual_pct) > POWER_MISMATCH_THRESHOLD
    )


# =============================================================================
# Shared buffers / logging
# =============================================================================
data_buf: deque[dict] = deque(maxlen=MAX_DATA_POINTS)
debug_log: deque[tuple[str, str, str]] = deque(maxlen=200)

# Log categories and their CSS colors
LOG_CAT_COLORS = {
    "send": "#42A5F5",
    "recv": "#66BB6A",
    "error": "#EF5350",
    "warn": "#FFA726",
    "info": "#BDBDBD",
    "seq": "#CE93D8",
    "cal": "#4DD0E1",
    "val": "#AED581",
    "state": "#FFEE58",
}


def log(msg: str, cat: str = "info") -> None:
    ts = datetime.now().strftime("%H:%M:%S")
    debug_log.appendleft((ts, cat, msg))


def _notify(msg: str, level: str = "positive") -> None:
    """Show toast notification respecting user preference."""
    if S.notify_level == "none":
        return
    if S.notify_level == "errors" and level == "positive":
        return
    ui.notify(msg, type=level, position="bottom-right", timeout=3000)


# =============================================================================
# CSV logger  (stream telemetry to daily files)
# =============================================================================
class _CSVLogger:
    def __init__(self) -> None:
        self._path = os.path.join(
            STREAM_DIR, f"{datetime.now():%Y-%m-%d}_Stream.csv"
        )
        self._header = False

    def write(self, s: dict) -> None:
        try:
            if not self._header:
                with open(self._path, "w") as f:
                    f.write(
                        "timestamp,esp_ts_ms,vessel_o3_pct,cell_temp_c,"
                        "pressure_mbar,room_o3_ppm,vessel_temp_c,"
                        "power_target,power_actual,wiper_v\n"
                    )
                self._header = True
            with open(self._path, "a") as f:
                f.write(
                    f"{s['timestamp']},{s['esp_ts_ms']},"
                    f"{s['vessel_o3_pct']},{s['cell_temp_c']},"
                    f"{s['pressure_mbar']},{s['room_o3_ppm']},"
                    f"{s['vessel_temp_c']},{s.get('power_target_pct', 0)},"
                    f"{s.get('power_actual_pct', 0)},"
                    f"{s.get('wiper_voltage', 0)}\n"
                )
        except Exception:
            pass


csv_logger = _CSVLogger()


# =============================================================================
# Calibration file helpers
# =============================================================================
def list_calibration_files() -> dict[float, list[str]]:
    out: dict[float, list[str]] = {}
    if not os.path.exists(CALIBRATION_DIR):
        return out
    for fn in os.listdir(CALIBRATION_DIR):
        if fn.endswith(".csv") and "PowerO3Cal" in fn:
            m = re.search(r"_(\d+(?:\.\d+)?)Lpm", fn)
            if m:
                lpm = float(m.group(1))
                out.setdefault(lpm, []).append(
                    os.path.join(CALIBRATION_DIR, fn)
                )
    return out


def _save_cal_csv(samples: list[dict], lpm: float) -> str:
    """Save calibration samples to CSV, return filename."""
    date_str = datetime.now().strftime("%Y-%m-%d")
    lpm_s = f"{lpm:.0f}" if lpm == int(lpm) else f"{lpm:.1f}"
    fname = f"{date_str}_PowerO3Cal_{lpm_s}Lpm.csv"
    fpath = os.path.join(CALIBRATION_DIR, fname)
    if os.path.exists(fpath):
        for i in range(2, 100):
            fname = f"{date_str}_PowerO3Cal_{lpm_s}Lpm_{i}.csv"
            fpath = os.path.join(CALIBRATION_DIR, fname)
            if not os.path.exists(fpath):
                break
    if samples:
        pd.DataFrame(samples).to_csv(fpath, index=False)
        log(f"Saved {len(samples)} cal samples -> {fname}", "cal")
    return fname


# =============================================================================
# Async TCP Server — ESP32 connects to us
# =============================================================================
class TCPServer:
    def __init__(self, port: int = DEFAULT_PORT) -> None:
        self.port = port
        self._server: Optional[asyncio.AbstractServer] = None
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._response_q: asyncio.Queue[str] = asyncio.Queue(maxsize=50)
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

        # Time-sync immediately
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
                    self._dispatch(line)
        except Exception as exc:
            log(f"TCP read error: {exc}", "error")
        finally:
            log("ESP32 disconnected", "warn")
            await self._close_client()
            # Abort sequence observer state on disconnect
            if S.sequence_active:
                S.sequence_active = False
                S.pending_prompt_id = ""
                S.pending_prompt_text = ""
                _notify("Sequence aborted — ESP32 disconnected", "negative")

    # -- dispatch (routes all incoming lines) ------------------------------
    def _dispatch(self, line: str) -> None:
        prefix = line.split(",", 1)[0]
        if prefix == "DATA":
            sample = parse_data_line(line)
            if sample:
                apply_telemetry(sample)
                data_buf.append(sample)
                csv_logger.write(sample)
        elif prefix == "RSP":
            self._handle_rsp(line)
        elif prefix == "STATE":
            self._handle_state(line)
        elif prefix == "SEQ":
            self._handle_seq(line)
        elif prefix == "SEQ_DONE":
            self._handle_seq_done(line)
        elif prefix == "CAL_START":
            self._handle_cal_start(line)
        elif prefix == "CAL_DATA":
            self._handle_cal_data(line)
        elif prefix == "CAL_COMPLETE":
            self._handle_cal_complete(line)
        elif prefix == "VAL_START":
            self._handle_val_start(line)
        elif prefix == "VAL_DATA":
            self._handle_val_data(line)
        elif prefix == "VAL_RESULT":
            self._handle_val_result(line)
        else:
            log(f"Unknown line: {line[:80]}", "warn")

    # -- RSP handler -------------------------------------------------------
    def _handle_rsp(self, line: str) -> None:
        if "time_sync" in line and "esp=" in line:
            m_esp = re.search(r"esp=(\d+)", line)
            m_pc = re.search(r"pc=(\d+)", line)
            if m_esp and m_pc:
                S.esp_time_offset_ms = (
                    int(m_pc.group(1)) - int(m_esp.group(1))
                )
                S.time_synced = True
                log(f"Time synced (offset={S.esp_time_offset_ms} ms)")
        # Relay-get parsing
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
        try:
            self._response_q.put_nowait(line)
        except asyncio.QueueFull:
            pass

    # -- STATE push (on connect/reconnect) --------------------------------
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
                    S.power_target_pct = int(float(v))
                    S.update_derived()
                elif k == "flow":
                    S.flow_lpm = float(v)
            except (ValueError, IndexError):
                pass
        log("STATE sync from ESP32", "state")

    # -- SEQ status updates -----------------------------------------------
    def _handle_seq(self, line: str) -> None:
        parts = line.split(",")
        # SEQ,prompt,<id>,<text> — special case: interactive prompt
        if len(parts) >= 3 and parts[1] == "prompt":
            S.pending_prompt_id = parts[2] if len(parts) > 2 else ""
            S.pending_prompt_text = ",".join(parts[3:]) if len(parts) > 3 else ""
            log(f"Prompt requested: {S.pending_prompt_id}", "seq")
            return
        # SEQ,<phase>,<progress>,<power>,<air>,<elapsed>
        if not S.sequence_active:
            S.sequence_active = True
        if len(parts) >= 6:
            S.seq_phase = parts[1]
            try:
                S.seq_progress = float(parts[2])
            except ValueError:
                S.seq_progress = 0.0
            try:
                S.seq_power = float(parts[3])
            except ValueError:
                pass
            S.seq_air = parts[4].strip() == "1"
            try:
                S.seq_elapsed = float(parts[5])
            except ValueError:
                pass
        log(f"SEQ: {S.seq_phase} {S.seq_progress:.0f}%", "seq")

    # -- SEQ_DONE ---------------------------------------------------------
    def _handle_seq_done(self, line: str) -> None:
        parts = line.split(",")
        seq_type = parts[1] if len(parts) > 1 else S.seq_type or "?"
        result = parts[2] if len(parts) > 2 else "unknown"
        log(f"Sequence '{seq_type}' done: {result}", "seq")

        S.sequence_active = False
        S.pending_prompt_id = ""
        S.pending_prompt_text = ""

        if result == "ok":
            _notify(f"Sequence '{seq_type}' completed successfully", "positive")
        elif result == "aborted":
            _notify(f"Sequence '{seq_type}' aborted", "warning")
        else:
            _notify(f"Sequence '{seq_type}' error: {result}", "negative")

    # -- CAL messages (calibration data stream) ---------------------------
    def _handle_cal_start(self, line: str) -> None:
        # CAL_START,o2_lpm=<val>,random_count=<n>,step_pct=<n>
        log(f"CAL_START: {line}", "cal")
        S.cal_samples = []
        S.cal_file = ""
        for part in line.split(","):
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            if k.strip() == "o2_lpm":
                try:
                    S.cal_lpm = float(v)
                except ValueError:
                    pass
        _notify(f"Calibration started @ {S.cal_lpm} LPM")

    def _handle_cal_data(self, line: str) -> None:
        parts = line.split(",")
        # CAL_DATA,ts,power,actual,o3,o2_lpm,air,total_lpm,temp,phase
        if len(parts) >= 10:
            try:
                sample = {
                    "timestamp": parts[1],
                    "power_pct": float(parts[2]),
                    "actual_pct": float(parts[3]),
                    "o3_pct": float(parts[4]),
                    "o2_lpm": float(parts[5]),
                    "air_comp_on": parts[6].strip() == "1",
                    "total_lpm": float(parts[7]),
                    "cell_temp_c": float(parts[8]),
                    "phase": parts[9].strip(),
                }
                S.cal_samples.append(sample)
            except (ValueError, IndexError):
                log(f"Bad CAL_DATA: {line[:80]}", "error")

    def _handle_cal_complete(self, line: str) -> None:
        # CAL_COMPLETE,total=<n>,baseline=<n>,sweep_up=<n>,...
        log(f"CAL_COMPLETE: {line}", "cal")
        if S.cal_samples:
            S.cal_file = _save_cal_csv(S.cal_samples, S.cal_lpm)
            _notify(
                f"Calibration complete: {len(S.cal_samples)} samples saved",
                "positive",
            )
        else:
            _notify("Calibration complete (no samples)", "warning")

    # -- VAL messages (validation data stream) ----------------------------
    def _handle_val_start(self, line: str) -> None:
        # VAL_START,power=<pct>,o2_lpm=<lpm>
        log(f"VAL_START: {line}", "val")
        S.val_samples = []
        S.val_result = {}
        for part in line.split(","):
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            try:
                if k.strip() == "power":
                    S.val_power = float(v)
                elif k.strip() == "o2_lpm":
                    S.val_lpm = float(v)
            except ValueError:
                pass
        _notify(f"Validation started @ {S.val_power:.0f}% / {S.val_lpm} LPM")

    def _handle_val_data(self, line: str) -> None:
        parts = line.split(",")
        # VAL_DATA,ts,power,actual,o3,o2_lpm,temp
        if len(parts) >= 7:
            try:
                S.val_samples.append({
                    "timestamp": parts[1],
                    "power_pct": float(parts[2]),
                    "actual_pct": float(parts[3]),
                    "o3_pct": float(parts[4]),
                    "o2_lpm": float(parts[5]),
                    "cell_temp_c": float(parts[6]),
                })
            except (ValueError, IndexError):
                log(f"Bad VAL_DATA: {line[:80]}", "error")

    def _handle_val_result(self, line: str) -> None:
        # VAL_RESULT,power=<>,o2_lpm=<>,mean_o3=<>,std_o3=<>,expected_o3=<>,
        #            mean_temp=<>,samples=<>,elapsed=<>
        log(f"VAL_RESULT: {line}", "val")
        result: dict[str, Any] = {}
        for part in line.split(","):
            if "=" not in part:
                continue
            k, v = part.split("=", 1)
            k = k.strip()
            try:
                result[k] = float(v)
            except ValueError:
                result[k] = v.strip()
        # Compute deviation
        mean_o3 = result.get("mean_o3", 0.0)
        expected = result.get("expected_o3", 0.0)
        if isinstance(mean_o3, (int, float)) and isinstance(expected, (int, float)):
            if expected > 0:
                result["deviation_pct"] = abs(mean_o3 - expected) / expected * 100
            else:
                result["deviation_pct"] = 0.0
        S.val_result = result
        dev = result.get("deviation_pct", 0.0)
        if dev < 10:
            _notify(f"Validation passed - {dev:.1f}% deviation", "positive")
        elif dev < 20:
            _notify(f"Validation marginal - {dev:.1f}% deviation", "warning")
        else:
            _notify(f"Validation failed - {dev:.1f}% deviation", "negative")

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
        """Send ``CMD,<cmd>\\n`` and wait for the matching RSP."""
        if not S.connected:
            return None
        while not self._response_q.empty():
            try:
                self._response_q.get_nowait()
            except asyncio.QueueEmpty:
                break
        if not await self._send_raw(f"CMD,{cmd}\n"):
            return None
        log(f"-> {cmd}", "send")
        try:
            resp = await asyncio.wait_for(self._response_q.get(), timeout)
            return resp
        except asyncio.TimeoutError:
            log(f"timeout: {cmd}", "error")
            return None


tcp: TCPServer  # assigned in startup


# =============================================================================
# Command helpers  (all async; COMMA-separated — never colons)
# =============================================================================
async def cmd_set_power(pct: int) -> bool:
    S.power_target_pct = int(pct)
    S.update_derived()
    resp = await tcp.send_command(f"power_set,{int(pct)}")
    return resp is not None and "OK" in resp


async def cmd_set_relay(name: str, on: bool) -> bool:
    resp = await tcp.send_command(f"relay_set,{name},{1 if on else 0}")
    if resp and "OK" in resp:
        if name == "ozone_gen":
            S.relay_o3_gen = on
        elif name == "o2_conc":
            S.relay_o2_conc = on
        elif name == "air_comp":
            S.relay_air_comp = on
        return True
    return False


async def cmd_sync_relays() -> None:
    if not S.connected:
        return
    await tcp.send_command("relay_get", timeout=1.0)


async def cmd_emergency_stop() -> None:
    if S.sequence_active:
        await cmd_sequence_stop()
    await cmd_set_power(0)
    await cmd_set_relay("ozone_gen", False)


async def cmd_sequence_start(seq_type: str, *params) -> bool:
    """Send CMD,sequence_start,<type>,<params>."""
    param_str = ",".join(str(p) for p in params)
    cmd = f"sequence_start,{seq_type}"
    if param_str:
        cmd += f",{param_str}"
    S.sequence_active = True
    S.seq_type = seq_type
    S.seq_phase = "starting"
    S.seq_progress = 0.0
    S.seq_elapsed = 0.0
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    resp = await tcp.send_command(cmd)
    if resp and "OK" in resp:
        return True
    S.sequence_active = False
    return False


async def cmd_sequence_stop() -> bool:
    """Send CMD,sequence_stop. State cleared by SEQ_DONE handler."""
    resp = await tcp.send_command("sequence_stop")
    return resp is not None and "OK" in resp


async def cmd_sequence_confirm(prompt_id: str, value: str = "") -> bool:
    """Send CMD,sequence_confirm,<prompt_id>[,<value>]."""
    cmd = f"sequence_confirm,{prompt_id}"
    if value:
        cmd += f",{value}"
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    resp = await tcp.send_command(cmd)
    return resp is not None and "OK" in resp


# =============================================================================
# UI  (built once per browser session)
# =============================================================================
@ui.page("/")
async def index():
    _updating = False

    # -- derived settings sync -------------------------------------------
    def _sync_derived_to_ui() -> None:
        S.update_derived()
        if inp_o3 is not None:
            inp_o3.value = round(S.target_o3_pct, 2)
        if inp_mg is not None:
            inp_mg.value = round(S.target_mg_per_s, 2)
        if inp_g30 is not None:
            inp_g30.value = round(S.target_g_30min, 2)

    # -- power callbacks --------------------------------------------------
    async def _on_power_slide(e) -> None:
        nonlocal _updating
        if _updating or S.sequence_active:
            return
        _updating = True
        pct = int(e.value)
        await cmd_set_power(pct)
        if inp_pwr is not None:
            inp_pwr.value = pct
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    async def _on_power_input(e) -> None:
        nonlocal _updating
        if _updating or S.sequence_active:
            return
        _updating = True
        pct = int(e.value)
        await cmd_set_power(pct)
        if slider is not None:
            slider.value = pct
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    async def _on_o3_input(e) -> None:
        nonlocal _updating
        if _updating:
            return
        _updating = True
        pwr = int(predict_power_from_o3(e.value, S.flow_lpm))
        await cmd_set_power(pwr)
        if slider is not None:
            slider.value = pwr
        if inp_pwr is not None:
            inp_pwr.value = pwr
        S.update_derived()
        if inp_mg is not None:
            inp_mg.value = round(S.target_mg_per_s, 2)
        if inp_g30 is not None:
            inp_g30.value = round(S.target_g_30min, 2)
        _updating = False

    async def _on_mg_input(e) -> None:
        nonlocal _updating
        if _updating:
            return
        _updating = True
        o3 = mg_per_s_to_o3_pct(e.value, S.flow_lpm)
        pwr = int(predict_power_from_o3(o3, S.flow_lpm))
        await cmd_set_power(pwr)
        if slider is not None:
            slider.value = pwr
        if inp_pwr is not None:
            inp_pwr.value = pwr
        S.update_derived()
        if inp_o3 is not None:
            inp_o3.value = round(S.target_o3_pct, 2)
        if inp_g30 is not None:
            inp_g30.value = round(S.target_g_30min, 2)
        _updating = False

    async def _on_g30_input(e) -> None:
        nonlocal _updating
        if _updating:
            return
        _updating = True
        mg = g_at_time_to_mg_per_s(e.value, 30.0)
        o3 = mg_per_s_to_o3_pct(mg, S.flow_lpm)
        pwr = int(predict_power_from_o3(o3, S.flow_lpm))
        await cmd_set_power(pwr)
        if slider is not None:
            slider.value = pwr
        if inp_pwr is not None:
            inp_pwr.value = pwr
        S.update_derived()
        if inp_o3 is not None:
            inp_o3.value = round(S.target_o3_pct, 2)
        if inp_mg is not None:
            inp_mg.value = round(S.target_mg_per_s, 2)
        _updating = False

    async def _on_lpm_change(e) -> None:
        nonlocal _updating
        if _updating:
            return
        _updating = True
        S.flow_lpm = float(e.value)
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    async def _preset_click(pct: int) -> None:
        nonlocal _updating
        if _updating or S.sequence_active:
            return
        _updating = True
        await cmd_set_power(pct)
        if slider is not None:
            slider.value = pct
        if inp_pwr is not None:
            inp_pwr.value = pct
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    # -- relay callbacks --------------------------------------------------
    async def _toggle_air() -> None:
        await cmd_set_relay("air_comp", not S.relay_air_comp)

    async def _toggle_o2() -> None:
        await cmd_set_relay("o2_conc", not S.relay_o2_conc)

    async def _toggle_o3() -> None:
        await cmd_set_relay("ozone_gen", not S.relay_o3_gen)

    async def _estop() -> None:
        await cmd_emergency_stop()
        if slider is not None:
            slider.value = 0
        if inp_pwr is not None:
            inp_pwr.value = 0
        _sync_derived_to_ui()

    async def _send_debug_cmd() -> None:
        cmd = debug_input.value
        if not cmd:
            return
        resp = await tcp.send_command(cmd)
        debug_resp_label.text = resp or "No response"

    # -- chart update helpers ---------------------------------------------
    def _update_power_curve() -> None:
        pwr, o3 = generate_power_curve(S.flow_lpm)
        target_o3 = predict_o3_from_power(S.power_target_pct, S.flow_lpm)
        fig = _make_power_fig(
            pwr, o3,
            S.power_target_pct, target_o3,
            S.power_actual_pct, S.vessel_o3_pct,
        )
        power_plot.figure = fig
        power_plot.update()

    def _make_power_fig(pwr, o3, tgt_pct, tgt_o3, act_pct, act_o3):
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=pwr, y=o3, mode="lines",
            line=dict(color="royalblue", width=2),
            showlegend=False, name="Model",
        ))
        fig.add_trace(go.Scatter(
            x=[tgt_pct], y=[tgt_o3], mode="markers",
            marker=dict(color="rgba(0,0,0,0)", size=18,
                        line=dict(color="black", width=3)),
            showlegend=False, name="Target",
        ))
        fig.add_trace(go.Scatter(
            x=[act_pct], y=[act_o3], mode="markers",
            marker=dict(color="limegreen", size=14),
            showlegend=False, name="Actual",
        ))
        y_max = max(o3) * 1.1 if max(o3) > 0 else 1
        fig.update_layout(
            xaxis_title="Power %", yaxis_title="O3 %vol",
            height=300, margin=dict(l=50, r=20, t=10, b=50),
            xaxis=dict(range=[0, 105], dtick=10),
            yaxis=dict(range=[0, y_max]),
        )
        return fig

    # =====================================================================
    # BUILD THE PAGE
    # =====================================================================
    ui.page_title("BlockSI Control")
    dark = ui.dark_mode(True)

    # -- Custom CSS -------------------------------------------------------
    ui.add_head_html("""
    <style>
    @keyframes blink-disconnect {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.2; }
    }
    .conn-blink { animation: blink-disconnect 0.6s ease-in-out infinite; }
    .conn-steady { animation: none; }
    .sensor-card {
      min-width: 170px; padding: 8px 12px;
      border-radius: 6px; border-left: 3px solid;
    }
    .sensor-card-o3   { border-left-color: #2196F3; }
    .sensor-card-room  { border-left-color: #4CAF50; }
    .sensor-card-temp  { border-left-color: #FF9800; }
    .sensor-card-flow  { border-left-color: #9C27B0; }
    .log-send  { color: #42A5F5; }
    .log-recv  { color: #66BB6A; }
    .log-error { color: #EF5350; }
    .log-warn  { color: #FFA726; }
    .log-info  { color: #BDBDBD; }
    .log-seq   { color: #CE93D8; }
    .log-cal   { color: #4DD0E1; }
    .log-val   { color: #AED581; }
    .log-state { color: #FFEE58; }
    .power-slider .q-slider__track-container { height: 8px !important; }
    .power-slider .q-slider__thumb { width: 24px !important; height: 24px !important; }
    </style>
    """)

    # -- LEFT DRAWER (sidebar) -------------------------------------------
    with ui.left_drawer(value=True).classes(
        "bg-dark q-pa-md"
    ).style("width:240px") as drawer:
        ui.label("BlockSI v2").classes("text-h6 text-weight-bold q-mb-sm")

        conn_badge = ui.badge("Disconnected", color="red").classes(
            "q-mb-sm conn-blink"
        )
        ui.separator().classes("q-my-sm")

        # Relay toggles
        ui.label("Relays").classes("text-subtitle2 q-mt-sm")
        with ui.row().classes("q-gutter-xs"):
            btn_air = ui.button("Air", on_click=_toggle_air).props(
                "dense color=grey size=sm"
            )
            btn_o2 = ui.button("O2", on_click=_toggle_o2).props(
                "dense color=grey size=sm"
            )
            btn_o3 = ui.button("O3", on_click=_toggle_o3).props(
                "dense color=grey size=sm"
            )

        ui.separator().classes("q-my-sm")

        # O2 LPM
        ui.label("O2 LPM").classes("text-caption")
        inp_lpm_sb = ui.number(
            value=S.flow_lpm, min=1.0, max=15.0, step=0.5,
            format="%.1f", on_change=_on_lpm_change,
        ).classes("q-mb-sm")

        ui.separator().classes("q-my-sm")

        # Sensor cards
        ui.label("Readings").classes("text-subtitle2")
        with ui.card().classes("sensor-card sensor-card-flow q-mb-xs").props("flat bordered"):
            ui.label("O2 Flow").classes("text-caption text-grey")
            card_flow_val = ui.label(f"{S.flow_lpm:.1f}").classes("text-h5 text-purple")
            ui.label("LPM").classes("text-caption text-grey")
        with ui.card().classes("sensor-card sensor-card-o3 q-mb-xs").props("flat bordered"):
            ui.label("Vessel O3").classes("text-caption text-grey")
            card_o3_val = ui.label(f"{S.vessel_o3_pct:.3f}").classes("text-h5 text-blue")
            ui.label("%vol").classes("text-caption text-grey")
        with ui.card().classes("sensor-card sensor-card-room q-mb-xs").props("flat bordered"):
            ui.label("Room O3").classes("text-caption text-grey")
            card_room_val = ui.label(f"{S.room_o3_ppm:.3f}").classes("text-h5 text-green")
            ui.label("ppm").classes("text-caption text-grey")
        with ui.card().classes("sensor-card sensor-card-temp q-mb-xs").props("flat bordered"):
            ui.label("Vessel Temp").classes("text-caption text-grey")
            card_vtemp_val = ui.label("N/A").classes("text-h5 text-orange")
            ui.label("\u00b0C").classes("text-caption text-grey")
        with ui.card().classes("sensor-card sensor-card-temp q-mb-xs").props("flat bordered"):
            ui.label("Cell Temp").classes("text-caption text-grey")
            card_ctemp_val = ui.label(f"{S.cell_temp_c:.1f}").classes("text-h5 text-orange")
            ui.label("\u00b0C").classes("text-caption text-grey")

    # -- HEADER -----------------------------------------------------------
    with ui.header().classes("bg-primary items-center q-px-md"):
        ui.button(icon="menu", on_click=lambda: drawer.toggle()).props(
            "flat dense round color=white"
        )
        ui.label("BlockSI Control").classes("text-h6 q-ml-md")
        ui.space()
        ui.button(icon="dark_mode", on_click=lambda: dark.toggle()).props(
            "flat dense round color=white"
        ).tooltip("Toggle dark/light")

    # -- SEQUENCE BANNER (visible during sequences) -----------------------
    with ui.column().classes("w-full q-pa-md"):
        with ui.row().classes(
            "w-full items-center q-pa-sm q-px-md q-gutter-md"
        ).style(
            "background: #F57F17; border-radius: 4px; min-height: 44px"
        ) as seq_banner:
            ui.icon("science", size="sm").classes("text-white")
            seq_name_lbl = ui.label("").classes("text-white text-weight-bold")
            seq_phase_lbl = ui.label("").classes("text-white-50 text-caption")
            seq_progress_bar = ui.linear_progress(
                value=0, show_value=False,
            ).props("color=white track-color=amber-4").classes(
                "col-grow"
            ).style("max-width: 220px")
            seq_elapsed_lbl = ui.label("").classes("text-white text-caption")
            ui.button(
                "ABORT", icon="stop",
                on_click=lambda: asyncio.create_task(cmd_sequence_stop()),
                color="red-10",
            ).props("dense size=sm")
        seq_banner.visible = False

        # -- PROMPT DIALOG (modal — shown when ESP32 needs user action) ---
        prompt_dialog = ui.dialog().props("persistent")
        prompt_icon_ref: list = []
        prompt_title_ref: list = []
        prompt_body_ref: list = []

        with prompt_dialog, ui.card().classes("q-pa-lg").style("min-width: 420px"):
            p_icon = ui.icon("help").classes("text-h3 text-amber q-mb-sm")
            prompt_icon_ref.append(p_icon)
            p_title = ui.label("Action Required").classes("text-h5 q-mb-sm")
            prompt_title_ref.append(p_title)
            p_body = ui.html("").classes("text-body1 q-mb-lg")
            prompt_body_ref.append(p_body)

            with ui.row().classes("justify-end w-full q-gutter-sm"):
                ui.button("Abort Sequence", color="red", on_click=lambda: (
                    prompt_dialog.close(),
                    asyncio.create_task(cmd_sequence_stop()),
                )).props("flat")
                ui.button("Confirm", color="green", icon="check", on_click=lambda: (
                    prompt_dialog.close(),
                    asyncio.create_task(
                        cmd_sequence_confirm(S.pending_prompt_id)
                    ),
                )).props("unelevated")

        # -- TABS ---------------------------------------------------------
        with ui.tabs().classes("w-full") as tabs:
            tab_power = ui.tab("Power", icon="bolt")
            tab_telem = ui.tab("Telemetry", icon="show_chart")
            tab_debug = ui.tab("Debug", icon="bug_report")
            tab_settings = ui.tab("Settings", icon="settings")

        with ui.tab_panels(tabs, value=tab_power).classes("w-full"):

            # =============================================================
            # POWER TAB
            # =============================================================
            with ui.tab_panel(tab_power):

                # ---- Power Control section ------------------------------
                with ui.expansion("Power Control", icon="speed",
                                  value=True).classes("w-full q-mb-sm"):
                    with ui.row().classes("w-full items-start q-gutter-md"):
                        with ui.column().classes("col-8"):
                            ui.label("Power (%)").classes("text-subtitle2")
                            slider = ui.slider(
                                min=0, max=100, step=1,
                                value=S.power_target_pct,
                                on_change=_on_power_slide,
                            ).classes("w-full power-slider")

                            preset_btns = []
                            with ui.row().classes("q-gutter-xs q-mt-xs"):
                                for p in range(0, 110, 10):
                                    _p = p
                                    _btn = ui.button(
                                        str(_p),
                                        on_click=lambda _p=_p: _preset_click(_p),
                                    ).props("dense flat size=sm")
                                    preset_btns.append(_btn)

                            init_pwr, init_o3 = generate_power_curve(S.flow_lpm)
                            tgt_o3_init = predict_o3_from_power(
                                S.power_target_pct, S.flow_lpm
                            )
                            power_plot = ui.plotly(
                                _make_power_fig(
                                    init_pwr, init_o3,
                                    S.power_target_pct, tgt_o3_init,
                                    S.power_actual_pct, S.vessel_o3_pct,
                                )
                            ).classes("w-full")

                        with ui.column().classes("col-3"):
                            ui.label("Settings").classes("text-subtitle2 q-mb-sm")
                            ui.label("LPM").classes("text-caption")
                            inp_lpm_settings = ui.number(
                                value=S.flow_lpm, min=1.0, max=15.0,
                                step=0.5, format="%.1f",
                                on_change=_on_lpm_change,
                            )
                            ui.label("% Power").classes("text-caption q-mt-sm")
                            inp_pwr = ui.number(
                                value=S.power_target_pct,
                                min=0, max=100, step=1,
                                on_change=_on_power_input,
                            )
                            ui.label("% vol O3").classes("text-caption q-mt-sm")
                            inp_o3 = ui.number(
                                value=round(S.target_o3_pct, 2),
                                min=0.0, max=5.0, step=0.01, format="%.2f",
                                on_change=_on_o3_input,
                            )
                            ui.label("mg O3/s").classes("text-caption q-mt-sm")
                            inp_mg = ui.number(
                                value=round(S.target_mg_per_s, 2),
                                min=0.0, max=10.0, step=0.01, format="%.2f",
                                on_change=_on_mg_input,
                            )
                            ui.label("g O3 @ 30 min").classes("text-caption q-mt-sm")
                            inp_g30 = ui.number(
                                value=round(S.target_g_30min, 2),
                                min=0.0, max=20.0, step=0.1, format="%.2f",
                                on_change=_on_g30_input,
                            )

                    ui.separator().classes("q-my-sm")
                    ui.button(
                        "EMERGENCY STOP", icon="dangerous",
                        on_click=_estop, color="red",
                    ).props("size=lg")

                # ---- Calibration section --------------------------------
                with ui.expansion("Calibration", icon="tune").classes(
                    "w-full q-mb-sm"
                ):
                    ui.markdown(
                        "ESP32-owned sequence (~17 min): Baseline -> Sweep Up -> "
                        "Sweep Down -> Random Pairs"
                    ).classes("text-caption text-grey q-mb-sm")

                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        cal_lpm_input = ui.number(
                            label="O2 LPM", value=DEFAULT_FLOW_LPM,
                            min=0.5, max=15, step=0.5, format="%.1f",
                        ).classes("w-24")

                        async def _start_cal():
                            if not S.connected:
                                _notify("Not connected to ESP32", "negative")
                                return
                            lpm = float(cal_lpm_input.value or DEFAULT_FLOW_LPM)
                            await cmd_sequence_start("cal", lpm)

                        cal_start_btn = ui.button(
                            "Start", icon="play_arrow",
                            on_click=_start_cal, color="primary",
                        )
                        cal_stop_btn = ui.button(
                            "Stop", icon="stop",
                            on_click=lambda: asyncio.create_task(cmd_sequence_stop()),
                            color="grey",
                        )

                    # Phase stepper cards
                    CAL_PHASES_DEF = [
                        ("baseline", "Baseline", "30s @ 0%"),
                        ("sweep_up", "Sweep Up", "0 -> 100%"),
                        ("sweep_down", "Sweep Down", "100 -> 0%"),
                        ("random_pairs", "Random Pairs", "15 levels x Air"),
                    ]
                    cal_step_cards: dict[str, Any] = {}
                    with ui.row().classes("w-full q-gutter-xs q-mb-sm"):
                        for phase_key, name, desc in CAL_PHASES_DEF:
                            with ui.card().classes("col q-pa-xs text-center").style(
                                "opacity: 0.4; border: 1px solid #555"
                            ) as sc:
                                ui.label(name).classes("text-caption text-weight-bold")
                                ui.label(desc).classes("text-caption text-grey")
                            cal_step_cards[phase_key] = sc

                    cal_phase_lbl = ui.label("Phase: --").classes("text-body2")
                    cal_progress = ui.linear_progress(
                        value=0, show_value=False
                    ).classes("q-mb-xs")
                    cal_info_lbl = ui.label("").classes("text-caption")

                    # Live scatter (ECharts)
                    cal_chart = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "item"},
                        "legend": {"data": ["Air OFF", "Air ON"]},
                        "xAxis": {"type": "value", "name": "Power %",
                                  "min": 0, "max": 100},
                        "yAxis": {"type": "value", "name": "O3 %vol"},
                        "series": [
                            {"name": "Air OFF", "type": "scatter", "data": [],
                             "itemStyle": {"color": "#42A5F5"}},
                            {"name": "Air ON", "type": "scatter", "data": [],
                             "itemStyle": {"color": "#FFA726"}},
                        ],
                    }).classes("w-full").style("height: 280px")

                    ui.separator().classes("q-my-sm")
                    ui.label("Calibration Files").classes("text-subtitle2")
                    cal_files_container = ui.column().classes("w-full")

                    def _render_cal_files() -> None:
                        cal_files_container.clear()
                        files = list_calibration_files()
                        with cal_files_container:
                            if not files:
                                ui.label("No calibration files found").classes(
                                    "text-caption"
                                )
                            else:
                                for lpm in sorted(files):
                                    with ui.expansion(
                                        f"O2 @ {lpm} LPM ({len(files[lpm])} files)"
                                    ):
                                        for fp in sorted(files[lpm]):
                                            ui.label(os.path.basename(fp)).classes(
                                                "text-caption"
                                            )

                    _render_cal_files()

                # ---- Validation section ---------------------------------
                with ui.expansion("Validation", icon="verified").classes(
                    "w-full q-mb-sm"
                ):
                    ui.markdown(
                        "Pre-flight check: measures actual O3 output at a set "
                        "power level and compares to the predicted value. "
                        "Requires operator to route L-valve at two points."
                    ).classes("text-caption text-grey q-mb-sm")

                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        val_pwr_input = ui.number(
                            label="Power %", value=75.0,
                            min=10, max=100, step=5, format="%.0f",
                        ).classes("w-24")
                        val_lpm_input = ui.number(
                            label="O2 LPM", value=DEFAULT_FLOW_LPM,
                            min=0.5, max=15, step=0.5, format="%.1f",
                        ).classes("w-24")

                        async def _start_val():
                            if not S.connected:
                                _notify("Not connected to ESP32", "negative")
                                return
                            pwr = float(val_pwr_input.value or 75)
                            lpm = float(val_lpm_input.value or DEFAULT_FLOW_LPM)
                            await cmd_sequence_start("validate", pwr, lpm)

                        val_start_btn = ui.button(
                            "Validate", icon="play_arrow",
                            on_click=_start_val, color="blue",
                        )

                    # Validation result card (hidden until result)
                    val_result_card = ui.card().classes(
                        "w-full q-pa-md q-mt-sm"
                    )
                    val_result_card.visible = False
                    with val_result_card:
                        val_result_icon = ui.icon("check_circle").classes("text-h3")
                        val_result_title = ui.label("").classes("text-h6")
                        with ui.row().classes("q-gutter-md q-mt-sm"):
                            val_mean_lbl = ui.label("")
                            val_expected_lbl = ui.label("")
                            val_dev_lbl = ui.label("")
                            val_std_lbl = ui.label("")

                    # Live O3 chart during validation
                    val_chart = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "axis"},
                        "xAxis": {"type": "category", "name": "Sample",
                                  "data": []},
                        "yAxis": {"type": "value", "name": "O3 (%vol)"},
                        "series": [
                            {"name": "O3", "type": "line", "smooth": True,
                             "data": [], "showSymbol": False,
                             "areaStyle": {"opacity": 0.1}},
                        ],
                    }).classes("w-full").style("height: 220px")

            # =============================================================
            # TELEMETRY TAB
            # =============================================================
            with ui.tab_panel(tab_telem):
                with ui.row().classes("q-gutter-md q-mb-md"):
                    met_o3 = ui.label(
                        f"Vessel O3: {S.vessel_o3_pct:.4f} %vol"
                    ).classes("text-body1")
                    met_room = ui.label(
                        f"Room O3: {S.room_o3_ppm:.3f} ppm"
                    ).classes("text-body1")
                    met_vt = ui.label("Vessel T: N/A").classes("text-body1")
                    met_ct = ui.label(
                        f"Cell T: {S.cell_temp_c:.1f} \u00b0C"
                    ).classes("text-body1")

                telem_skeleton = ui.column().classes("w-full")
                with telem_skeleton:
                    for _ in range(2):
                        ui.skeleton(type="rect").classes(
                            "w-full q-mb-sm"
                        ).style("height: 200px")
                telem_skeleton.visible = True

                echart_o3 = ui.echart({
                    "darkMode": True,
                    "tooltip": {"trigger": "axis"},
                    "legend": {"data": ["Vessel O3 (%vol)", "Room O3 (ppm)"]},
                    "xAxis": {"type": "time", "name": "Time"},
                    "yAxis": [
                        {"type": "value", "name": "O3 %vol", "position": "left"},
                        {"type": "value", "name": "Room ppm", "position": "right"},
                    ],
                    "series": [
                        {"name": "Vessel O3 (%vol)", "type": "line", "smooth": True,
                         "yAxisIndex": 0, "data": [], "showSymbol": False,
                         "lineStyle": {"width": 2, "color": "#42A5F5"},
                         "areaStyle": {"opacity": 0.08}},
                        {"name": "Room O3 (ppm)", "type": "line", "smooth": True,
                         "yAxisIndex": 1, "data": [], "showSymbol": False,
                         "lineStyle": {"width": 2, "color": "#66BB6A"}},
                    ],
                    "dataZoom": [{"type": "inside"}, {"type": "slider"}],
                }).classes("w-full").style("height: 260px")
                echart_o3.visible = False

                echart_pwr = ui.echart({
                    "darkMode": True,
                    "tooltip": {"trigger": "axis"},
                    "legend": {"data": ["Power (%)", "Cell Temp (\u00b0C)"]},
                    "xAxis": {"type": "time", "name": "Time"},
                    "yAxis": [
                        {"type": "value", "name": "Power %", "position": "left", "max": 105},
                        {"type": "value", "name": "\u00b0C", "position": "right"},
                    ],
                    "series": [
                        {"name": "Power (%)", "type": "line", "smooth": True,
                         "yAxisIndex": 0, "data": [], "showSymbol": False,
                         "lineStyle": {"width": 2, "color": "#FFA726"}},
                        {"name": "Cell Temp (\u00b0C)", "type": "line", "smooth": True,
                         "yAxisIndex": 1, "data": [], "showSymbol": False,
                         "lineStyle": {"width": 2, "color": "#EF5350"}},
                    ],
                    "dataZoom": [{"type": "inside"}, {"type": "slider"}],
                }).classes("w-full").style("height: 260px")
                echart_pwr.visible = False

                with ui.expansion("Raw data", icon="table_chart").classes(
                    "w-full q-mt-sm"
                ):
                    async def _export_csv():
                        if not data_buf:
                            _notify("No data to export", "warning")
                            return
                        df = pd.DataFrame(list(data_buf))
                        fname = f"{datetime.now():%Y-%m-%d_%H%M%S}_Export.csv"
                        fpath = os.path.join(STREAM_DIR, fname)
                        df.to_csv(fpath, index=False)
                        _notify(f"Exported {len(df)} rows to {fname}")
                        log(f"CSV export -> {fname}")

                    ui.button(
                        "Export CSV", icon="download", on_click=_export_csv,
                    ).props("dense flat size=sm").classes("q-mb-sm")

                    raw_table = ui.table(
                        columns=[
                            {"name": "timestamp", "label": "Time",
                             "field": "timestamp", "align": "left", "sortable": True},
                            {"name": "vessel_o3_pct", "label": "O3 %",
                             "field": "vessel_o3_pct", "sortable": True},
                            {"name": "room_o3_ppm", "label": "Room ppm",
                             "field": "room_o3_ppm", "sortable": True},
                            {"name": "power_actual_pct", "label": "Power %",
                             "field": "power_actual_pct", "sortable": True},
                            {"name": "cell_temp_c", "label": "Cell \u00b0C",
                             "field": "cell_temp_c", "sortable": True},
                        ],
                        rows=[],
                        pagination={"rowsPerPage": 15},
                    ).classes("w-full")

            # =============================================================
            # DEBUG TAB
            # =============================================================
            with ui.tab_panel(tab_debug):
                ui.label("Debug Console").classes("text-h6 q-mb-sm")
                with ui.row().classes("w-full q-gutter-sm items-end"):
                    debug_input = ui.input(
                        "Command", placeholder="status"
                    ).classes("col-9")
                    ui.button("Send", on_click=_send_debug_cmd).classes("col-2")
                debug_resp_label = ui.label("").classes("text-body2 q-mt-sm")

                ui.separator().classes("q-my-md")
                ui.label("System State").classes("text-subtitle2")
                dbg_state_lbl = ui.code("").classes("w-full")

                ui.label("Log").classes("text-subtitle2 q-mt-md")
                dbg_log_html = ui.html("").classes("w-full").style(
                    "max-height: 320px; overflow-y: auto; "
                    "font-family: monospace; font-size: 0.82rem; "
                    "background: var(--q-dark-page, #1d1d1d); "
                    "border-radius: 8px; padding: 8px"
                )

            # =============================================================
            # SETTINGS TAB
            # =============================================================
            with ui.tab_panel(tab_settings):
                ui.label("Settings").classes("text-h6 q-mb-md")
                with ui.card().classes("q-pa-md"):
                    ui.label("Toast Notifications").classes("text-subtitle2 q-mb-sm")
                    ui.label(
                        "Control which notifications appear as pop-ups"
                    ).classes("text-caption text-grey q-mb-sm")
                    notify_select = ui.select(
                        options=["all", "errors", "none"],
                        value=S.notify_level,
                        label="Notification level",
                    ).classes("w-48")

                    def _on_notify_change(e):
                        S.notify_level = e.value
                        log(f"Notification level -> {e.value}")

                    notify_select.on("update:model-value", _on_notify_change)

    # =====================================================================
    # PERIODIC UI REFRESH  (1s timer)
    # =====================================================================
    _relay_ts: float = 0.0
    _echart_has_data: bool = False
    _last_prompt_id: str = ""

    async def _tick() -> None:
        nonlocal _updating, _relay_ts, _echart_has_data, _last_prompt_id

        # -- sync relays every ~10 s --------------------------------------
        if S.connected and time.time() - _relay_ts > 10:
            await cmd_sync_relays()
            _relay_ts = time.time()

        # -- sidebar status -----------------------------------------------
        if S.connected:
            conn_badge.text = "Connected"
            conn_badge.props('color="green"')
            conn_badge._classes = [
                c for c in conn_badge._classes if c != "conn-blink"
            ]
            if "conn-steady" not in conn_badge._classes:
                conn_badge._classes.append("conn-steady")
        else:
            conn_badge.text = "Disconnected"
            conn_badge.props('color="red"')
            conn_badge._classes = [
                c for c in conn_badge._classes if c != "conn-steady"
            ]
            if "conn-blink" not in conn_badge._classes:
                conn_badge._classes.append("conn-blink")
        conn_badge.update()

        btn_air.props(f'color={"green" if S.relay_air_comp else "grey"}')
        btn_o2.props(f'color={"green" if S.relay_o2_conc else "grey"}')
        btn_o3.props(f'color={"green" if S.relay_o3_gen else "grey"}')

        card_flow_val.text = f"{S.flow_lpm:.1f}"
        card_o3_val.text = f"{S.vessel_o3_pct:.3f}"
        card_room_val.text = f"{S.room_o3_ppm:.3f}"
        card_vtemp_val.text = (
            f"{S.vessel_temp_c:.1f}" if S.vessel_temp_c > -900 else "N/A"
        )
        card_ctemp_val.text = f"{S.cell_temp_c:.1f}"

        # -- sync power slider & inputs ------------------------------------
        if not _updating:
            _updating = True
            if slider.value != S.power_target_pct:
                slider.value = S.power_target_pct
            if inp_pwr.value != S.power_target_pct:
                inp_pwr.value = S.power_target_pct
            _sync_derived_to_ui()
            _updating = False

        # -- sequence banner + control lock --------------------------------
        seq_banner.visible = S.sequence_active
        if S.sequence_active:
            type_names = {"cal": "Calibration", "validate": "Validation"}
            seq_name_lbl.text = type_names.get(S.seq_type, S.seq_type)
            seq_phase_lbl.text = S.seq_phase.replace("_", " ").title()
            seq_progress_bar.value = S.seq_progress / 100
            mins = int(S.seq_elapsed) // 60
            secs = int(S.seq_elapsed) % 60
            seq_elapsed_lbl.text = f"{mins}:{secs:02d}"

        # Lock controls during sequences (power + relays + O2 input)
        lockable = [slider, inp_pwr, inp_o3, inp_mg, inp_g30,
                     btn_air, btn_o2, btn_o3, inp_lpm_sb,
                     cal_start_btn, val_start_btn]
        lockable.extend(preset_btns)
        for _el in lockable:
            if S.sequence_active:
                _el.props("disable")
            else:
                _el.props(remove="disable")

        # -- prompt dialog ------------------------------------------------
        if S.pending_prompt_id and S.pending_prompt_id != _last_prompt_id:
            _last_prompt_id = S.pending_prompt_id
            content = PROMPT_CONTENT.get(S.pending_prompt_id, {
                "title": "Action Required",
                "icon": "help",
                "body": S.pending_prompt_text or "Please confirm to continue.",
            })
            prompt_icon_ref[0].name = content.get("icon", "help")
            prompt_icon_ref[0].update()
            prompt_title_ref[0].text = content["title"]
            prompt_body_ref[0].content = content["body"]
            prompt_dialog.open()
        elif not S.pending_prompt_id:
            _last_prompt_id = ""

        # -- power curve ---------------------------------------------------
        _update_power_curve()

        # -- telemetry ECharts ---------------------------------------------
        if data_buf:
            met_o3.text = f"Vessel O3: {S.vessel_o3_pct:.4f} %vol"
            met_room.text = f"Room O3: {S.room_o3_ppm:.3f} ppm"
            met_vt.text = (
                f"Vessel T: {S.vessel_temp_c:.1f} \u00b0C"
                if S.vessel_temp_c > -900 else "Vessel T: N/A"
            )
            met_ct.text = f"Cell T: {S.cell_temp_c:.1f} \u00b0C"

            if not _echart_has_data:
                telem_skeleton.visible = False
                echart_o3.visible = True
                echart_pwr.visible = True
                _echart_has_data = True

            o3_data, room_data, pwr_data, temp_data = [], [], [], []
            for s in data_buf:
                ts_ms = int(s["timestamp"].timestamp() * 1000)
                o3_data.append([ts_ms, s["vessel_o3_pct"]])
                room_data.append([ts_ms, s["room_o3_ppm"]])
                pwr_data.append([ts_ms, s.get("power_actual_pct", 0)])
                temp_data.append([ts_ms, s.get("cell_temp_c", 0)])

            echart_o3.options["series"][0]["data"] = o3_data
            echart_o3.options["series"][1]["data"] = room_data
            echart_o3.update()
            echart_pwr.options["series"][0]["data"] = pwr_data
            echart_pwr.options["series"][1]["data"] = temp_data
            echart_pwr.update()

            rows = []
            for s in list(data_buf)[-20:]:
                rows.append({
                    "timestamp": str(s["timestamp"].strftime("%H:%M:%S")),
                    "vessel_o3_pct": f"{s['vessel_o3_pct']:.4f}",
                    "room_o3_ppm": f"{s['room_o3_ppm']:.3f}",
                    "power_actual_pct": f"{s.get('power_actual_pct', 0):.1f}",
                    "cell_temp_c": f"{s.get('cell_temp_c', 0):.1f}",
                })
            raw_table.rows = rows
            raw_table.update()

        # -- calibration observer UI --------------------------------------
        if S.sequence_active and S.seq_type == "cal":
            cal_phase_lbl.text = f"Phase: {S.seq_phase.replace('_', ' ').title()}"
            cal_progress.value = S.seq_progress / 100
            cal_info_lbl.text = (
                f"Power={S.seq_power:.0f}%  "
                f"O3={S.vessel_o3_pct:.2f}%  "
                f"Air={'ON' if S.seq_air else 'OFF'}  "
                f"Samples={len(S.cal_samples)}"
            )
            # Highlight active/completed phase cards
            phase_order = ["baseline", "sweep_up", "sweep_down", "random_pairs"]
            active_idx = phase_order.index(S.seq_phase) if S.seq_phase in phase_order else -1
            for i, pk in enumerate(phase_order):
                card = cal_step_cards.get(pk)
                if card is None:
                    continue
                if i < active_idx:
                    card.style("opacity: 0.7; border: 1px solid #66BB6A")
                elif i == active_idx:
                    card.style(
                        "opacity: 1.0; border: 2px solid #42A5F5; "
                        "box-shadow: 0 0 8px rgba(66,165,245,0.4)"
                    )
                else:
                    card.style("opacity: 0.4; border: 1px solid #555")
        elif not S.sequence_active:
            cal_phase_lbl.text = "Phase: --"
            cal_progress.value = 0
            cal_info_lbl.text = ""
            for card in cal_step_cards.values():
                card.style("opacity: 0.4; border: 1px solid #555")

        # Cal scatter chart
        if S.cal_samples:
            air_off = [[s["power_pct"], s["o3_pct"]]
                       for s in S.cal_samples if not s.get("air_comp_on")]
            air_on = [[s["power_pct"], s["o3_pct"]]
                      for s in S.cal_samples if s.get("air_comp_on")]
            cal_chart.options["series"][0]["data"] = air_off
            cal_chart.options["series"][1]["data"] = air_on
            cal_chart.update()

        # -- validation observer UI ---------------------------------------
        if S.val_samples:
            val_chart.options["xAxis"]["data"] = list(range(len(S.val_samples)))
            val_chart.options["series"][0]["data"] = [
                s["o3_pct"] for s in S.val_samples
            ]
            val_chart.update()

        if S.val_result:
            val_result_card.visible = True
            r = S.val_result
            dev = r.get("deviation_pct", 0.0)
            if isinstance(dev, (int, float)):
                if dev < 10:
                    color, icon_name = "green", "check_circle"
                elif dev < 20:
                    color, icon_name = "amber", "warning"
                else:
                    color, icon_name = "red", "error"
            else:
                color, icon_name = "grey", "help"
                dev = 0.0
            val_result_icon.name = icon_name
            val_result_icon._classes = [f"text-h3 text-{color}"]
            val_result_icon.update()
            val_result_title.text = f"Deviation: {dev:.1f}%"
            val_mean_lbl.text = f"Mean O3: {r.get('mean_o3', 0):.3f}%"
            val_expected_lbl.text = f"Expected: {r.get('expected_o3', 0):.3f}%"
            val_dev_lbl.text = f"Deviation: {dev:.1f}%"
            val_std_lbl.text = f"Std: {r.get('std_o3', 0):.4f}%"

        # -- debug state dump ---------------------------------------------
        dbg_state_lbl.content = json.dumps({
            "connected": S.connected,
            "time_synced": S.time_synced,
            "power_target": S.power_target_pct,
            "power_actual": round(S.power_actual_pct, 1),
            "power_error": S.power_error,
            "relays": {
                "o3_gen": S.relay_o3_gen,
                "o2_conc": S.relay_o2_conc,
                "air_comp": S.relay_air_comp,
            },
            "flow_lpm": S.flow_lpm,
            "vessel_o3": round(S.vessel_o3_pct, 4),
            "room_o3": round(S.room_o3_ppm, 3),
            "sequence_active": S.sequence_active,
            "seq_type": S.seq_type,
            "seq_phase": S.seq_phase,
            "seq_progress": round(S.seq_progress, 1),
            "pending_prompt": S.pending_prompt_id,
            "cal_samples": len(S.cal_samples),
            "val_samples": len(S.val_samples),
            "data_buf": len(data_buf),
            "last_update": str(S.last_update) if S.last_update else None,
        }, indent=2)

        lines_html = []
        for ts, cat, msg in list(debug_log):
            css_cls = f"log-{cat}" if cat else "log-info"
            lines_html.append(
                f'<div class="{css_cls}">'
                f'<span style="opacity:0.5">{ts}</span> {msg}</div>'
            )
        dbg_log_html.content = "\n".join(lines_html[-80:])

    ui.timer(1.0, _tick)


# =============================================================================
# Start-up
# =============================================================================
async def _startup() -> None:
    global tcp
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args, _ = parser.parse_known_args()
    tcp = TCPServer(args.port)
    await tcp.start()


app.on_startup(_startup)

ui.run(title="BlockSI Control", port=8080, reload=False)

