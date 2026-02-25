#!/usr/bin/env python3
"""
BlockSI Dashboard -- NiceGUI Version
Migrated from Streamlit v9.  Run with:  python blocksi_dashboard.py [--port 5000]

Solves four Streamlit limitations:
  1. No full-script rerun  ->  incremental DOM via WebSocket
  2. bind_value()          ->  multiple controls for the same value stay synced
  3. asyncio TCP           ->  non-blocking I/O, instant push to UI
  4. ui.timer()            ->  calibration sequences run without page flicker

DATA format from ESP32 (v2):
  DATA,esp_timestamp_ms,vessel_o3_pct,temp_c,pressure_mbar,sample_v,ref_v,
       day,month,year,hour,minute,second,room_o3_ppm,vessel_temp_c,
       power_target_pct,power_actual_pct,wiper_voltage

CRITICAL: Commands are COMMA-separated  ->  CMD,power_set,50\\n
          NEVER use colons (CMD,power_set:50) -- silently fails on ESP32.
"""
from __future__ import annotations

import argparse
import asyncio
import os
import re
import time
from collections import deque
from datetime import datetime
from typing import Optional

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from nicegui import ui, app

# =============================================================================
# Configuration
# =============================================================================
DEFAULT_PORT = 5000
MAX_DATA_POINTS = 500

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(BASE_DIR, "Data")
CALIBRATION_DIR = os.path.join(DATA_DIR, "O3PowerCalibration")
MODEL_DIR = os.path.join(BASE_DIR, "Model", "O3Power")

for _d in (DATA_DIR, CALIBRATION_DIR, MODEL_DIR):
    os.makedirs(_d, exist_ok=True)

# Power model coefficients  (O3_max = A/F + B)
POWER_MODEL_A = 1.78
POWER_MODEL_B = 1.40
DEFAULT_FLOW_LPM = 4.0

# O3 mass-flow conversion  mg/s = %vol * LPM * K
O3_MASS_FLOW_K = 0.357

# Calibration timing
CAL_STEP_DURATION_S = 2.0
CAL_BASELINE_DURATION_S = 30.0
CAL_AIR_DWELL_TIME_S = 20.0
CAL_NUM_RANDOM_POINTS = 15
AIR_COMP_LPM = 10.0

POWER_MISMATCH_THRESHOLD = 5.0


# =============================================================================
# Conversion helpers  (pure functions -- ported verbatim from v9)
# =============================================================================
def predict_o3_from_power(power_pct: float, flow_lpm: float) -> float:
    """Piecewise model: threshold -> linear ramp -> saturation."""
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
# SystemState  (single source of truth -- same fields as v9)
# =============================================================================
class SystemState:
    """Mutable singleton; survives the whole process -- no cache eviction."""

    def __init__(self) -> None:
        # Power
        self.power_target_pct: int = 0
        self.power_actual_pct: float = 0.0
        self.wiper_voltage: float = 0.0
        self.power_error: bool = False
        # Flow (user-entered from analog meter)
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
        # Calibration
        self.cal_active: bool = False
        self.cal_phase: Optional[str] = None
        self.cal_phase_progress: float = 0.0
        self.cal_current_power: int = 0
        self.cal_air_state: bool = False
        self.cal_start_time: float = 0.0
        self.cal_step_start: float = 0.0
        self.cal_data: list[dict] = []
        self.cal_random_powers: list[int] = []
        self.cal_random_idx: int = 0
        self.cal_o2_lpm: float = DEFAULT_FLOW_LPM
        # Connection
        self.connected: bool = False
        # Sequence tracking (visual indicator + authority model)
        self.sequence_active: bool = False
        self.sequence_name: str = ""
        self.sequence_description: str = ""
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


S = SystemState()  # singleton


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
    # power_target_pct: PC is sole authority -- never accept from
    # telemetry.  Avoids stale-DATA feedback loops where old ESP32
    # readings overwrite the value the user just commanded.
    S.update_derived()
    S.power_error = (
        abs(S.power_target_pct - S.power_actual_pct) > POWER_MISMATCH_THRESHOLD
    )


# =============================================================================
# Async TCP Server
# =============================================================================
class TCPServer:
    """asyncio TCP server -- ESP32 connects to us on *port*."""

    def __init__(self, port: int = DEFAULT_PORT) -> None:
        self.port = port
        self._server: Optional[asyncio.AbstractServer] = None
        self._reader: Optional[asyncio.StreamReader] = None
        self._writer: Optional[asyncio.StreamWriter] = None
        self._response_q: asyncio.Queue[str] = asyncio.Queue(maxsize=50)
        self._running = False

    # -- lifecycle ---------------------------------------------------------
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

    # -- connection handler ------------------------------------------------
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
                    if line.startswith("DATA,"):
                        sample = parse_data_line(line)
                        if sample:
                            apply_telemetry(sample)
                            data_buf.append(sample)
                            csv_logger.write(sample)
                    elif line.startswith("RSP,"):
                        self._handle_response(line)
        except Exception as exc:
            log(f"TCP read error: {exc}")
        finally:
            log("ESP32 disconnected")
            await self._close_client()

    def _handle_response(self, line: str) -> None:
        # Time-sync parsing
        if "time_sync" in line and "esp=" in line:
            m_esp = re.search(r"esp=(\d+)", line)
            m_pc = re.search(r"pc=(\d+)", line)
            if m_esp and m_pc:
                S.esp_time_offset_ms = (
                    int(m_pc.group(1)) - int(m_esp.group(1))
                )
                S.time_synced = True
                log(f"Time synced (offset={S.esp_time_offset_ms} ms)")
        try:
            self._response_q.put_nowait(line)
        except asyncio.QueueFull:
            pass

    # -- sending -----------------------------------------------------------
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
        # flush stale responses
        while not self._response_q.empty():
            try:
                self._response_q.get_nowait()
            except asyncio.QueueEmpty:
                break
        if not await self._send_raw(f"CMD,{cmd}\n"):
            return None
        log(f"-> {cmd}")
        try:
            resp = await asyncio.wait_for(self._response_q.get(), timeout)
            log(f"<- {resp}")
            return resp
        except asyncio.TimeoutError:
            log(f"timeout: {cmd}")
            return None


tcp = TCPServer()


# =============================================================================
# Command helpers  (all async; COMMA-separated -- never colons)
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
    resp = await tcp.send_command("relay_get", timeout=1.0)
    if not resp or "OK" not in resp:
        return
    for part in resp.split(","):
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


async def cmd_emergency_stop() -> None:
    await cmd_set_power(0)
    await cmd_set_relay("ozone_gen", False)


# =============================================================================
# CSV logger
# =============================================================================
class _CSVLogger:
    def __init__(self) -> None:
        self._path = os.path.join(
            DATA_DIR, f"{datetime.now():%Y%m%d_%H%M%S}_Stream.csv"
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
                    f"{s['vessel_temp_c']},{s.get('power_target_pct',0)},"
                    f"{s.get('power_actual_pct',0)},"
                    f"{s.get('wiper_voltage',0)}\n"
                )
        except Exception:
            pass


csv_logger = _CSVLogger()


# =============================================================================
# Calibration runner  (async, same state machine as v9)
# =============================================================================
class CalibrationRunner:

    async def start(self) -> None:
        S.cal_active = True
        S.sequence_active = True
        S.sequence_name = "Power-O3 Calibration"
        S.sequence_description = "Characterizing O3 output vs power"
        S.cal_phase = "baseline"
        S.cal_phase_progress = 0.0
        S.cal_current_power = 0
        S.cal_air_state = False
        S.cal_start_time = time.time()
        S.cal_step_start = time.time()
        S.cal_data = []
        S.cal_o2_lpm = S.flow_lpm
        np.random.seed(int(time.time()))
        S.cal_random_powers = sorted(
            np.random.randint(5, 96, CAL_NUM_RANDOM_POINTS).tolist()
        )
        S.cal_random_idx = 0
        await tcp.send_command("relay_set,air_comp,0")
        S.cal_air_state = False
        S.power_target_pct = 0
        await tcp.send_command("power_set,0")
        S.update_derived()
        log("Calibration started")

    async def stop(self) -> None:
        S.cal_active = False
        S.sequence_active = False
        S.sequence_name = ""
        S.sequence_description = ""
        S.power_target_pct = 0
        await tcp.send_command("power_set,0")
        await tcp.send_command("relay_set,air_comp,0")
        S.update_derived()
        if S.cal_data:
            path = self._save()
            log(f"Calibration saved -> {path}")
        S.cal_phase = None
        S.cal_data = []
        log("Calibration stopped")

    async def step(self) -> None:
        """Called every ~1 s from UI timer."""
        if not S.cal_active:
            return
        # Abort if connection lost mid-sequence
        if not S.connected:
            log("Connection lost during calibration -- aborting")
            await self.stop()
            return
        # Keep slider/UI in sync with calibration power target
        S.power_target_pct = S.cal_current_power
        S.update_derived()
        elapsed = time.time() - S.cal_step_start

        if S.cal_phase == "baseline":
            S.cal_phase_progress = min(
                100.0, elapsed / CAL_BASELINE_DURATION_S * 100
            )
            if elapsed >= CAL_BASELINE_DURATION_S:
                self._record()
                S.cal_phase = "sweep_up"
                S.cal_current_power = 0
                S.cal_step_start = time.time()
                await tcp.send_command("power_set,0")
            elif elapsed >= CAL_STEP_DURATION_S:
                self._record()
                S.cal_step_start = time.time()

        elif S.cal_phase == "sweep_up":
            S.cal_phase_progress = S.cal_current_power
            if elapsed >= CAL_STEP_DURATION_S:
                self._record()
                if S.cal_current_power < 100:
                    S.cal_current_power += 1
                    await tcp.send_command(
                        f"power_set,{S.cal_current_power}"
                    )
                else:
                    S.cal_phase = "sweep_down"
                    S.cal_current_power = 100
                S.cal_step_start = time.time()

        elif S.cal_phase == "sweep_down":
            S.cal_phase_progress = 100 - S.cal_current_power
            if elapsed >= CAL_STEP_DURATION_S:
                self._record()
                if S.cal_current_power > 0:
                    S.cal_current_power -= 1
                    await tcp.send_command(
                        f"power_set,{S.cal_current_power}"
                    )
                else:
                    S.cal_phase = "random_pairs"
                    S.cal_random_idx = 0
                    S.cal_air_state = False
                    if S.cal_random_powers:
                        S.cal_current_power = S.cal_random_powers[0]
                        await tcp.send_command(
                            f"power_set,{S.cal_current_power}"
                        )
                S.cal_step_start = time.time()

        elif S.cal_phase == "random_pairs":
            total = CAL_NUM_RANDOM_POINTS * 2
            cur = S.cal_random_idx * 2 + (1 if S.cal_air_state else 0)
            S.cal_phase_progress = cur / total * 100
            if elapsed >= CAL_AIR_DWELL_TIME_S:
                self._record()
                if not S.cal_air_state:
                    S.cal_air_state = True
                    await tcp.send_command("relay_set,air_comp,1")
                else:
                    S.cal_air_state = False
                    await tcp.send_command("relay_set,air_comp,0")
                    S.cal_random_idx += 1
                    if S.cal_random_idx >= len(S.cal_random_powers):
                        await self.stop()
                        return
                    S.cal_current_power = S.cal_random_powers[
                        S.cal_random_idx
                    ]
                    await tcp.send_command(
                        f"power_set,{S.cal_current_power}"
                    )
                S.cal_step_start = time.time()

    # -- helpers -----------------------------------------------------------
    def _record(self) -> None:
        total_lpm = S.cal_o2_lpm + (
            AIR_COMP_LPM if S.cal_air_state else 0
        )
        o2_frac = (
            S.cal_o2_lpm * 0.93
            + (AIR_COMP_LPM * 0.21 if S.cal_air_state else 0)
        ) / total_lpm
        S.cal_data.append(
            {
                "timestamp": datetime.now().isoformat(),
                "power_pct": S.cal_current_power,
                "o3_pct": S.vessel_o3_pct,
                "o2_lpm": S.cal_o2_lpm,
                "air_comp_on": S.cal_air_state,
                "total_lpm": total_lpm,
                "o2_concentration_pct": o2_frac * 100,
                "cell_temp_c": S.cell_temp_c,
                "phase": S.cal_phase,
            }
        )

    def _save(self) -> Optional[str]:
        if not S.cal_data:
            return None
        date_str = datetime.now().strftime("%Y-%m-%d")
        lpm_s = (
            f"{S.cal_o2_lpm:.0f}"
            if S.cal_o2_lpm == int(S.cal_o2_lpm)
            else f"{S.cal_o2_lpm:.1f}"
        )
        fname = f"{date_str}_PowerO3Cal_{lpm_s}Lpm.csv"
        fpath = os.path.join(CALIBRATION_DIR, fname)
        if os.path.exists(fpath):
            for i in range(2, 100):
                fname = f"{date_str}_PowerO3Cal_{lpm_s}Lpm_{i}.csv"
                fpath = os.path.join(CALIBRATION_DIR, fname)
                if not os.path.exists(fpath):
                    break
        pd.DataFrame(S.cal_data).to_csv(fpath, index=False)
        return fname


cal = CalibrationRunner()


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


# =============================================================================
# Shared buffers / logging
# =============================================================================
data_buf: deque[dict] = deque(maxlen=MAX_DATA_POINTS)
debug_log: deque[str] = deque(maxlen=200)


def log(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S")
    debug_log.appendleft(f"{ts}  {msg}")


# =============================================================================
# UI  (built once per browser session)
# =============================================================================
@ui.page("/")
async def index():
    # Guard against circular update cascades when we programmatically
    # update bound inputs after a user-initiated change.
    _updating = False

    # -- helper to recalc derived settings and push to UI ------------------
    def _sync_derived_to_ui() -> None:
        """Recompute derived values from power_target_pct and push them."""
        S.update_derived()
        if inp_o3 is not None:
            inp_o3.value = round(S.target_o3_pct, 2)
        if inp_mg is not None:
            inp_mg.value = round(S.target_mg_per_s, 2)
        if inp_g30 is not None:
            inp_g30.value = round(S.target_g_30min, 2)

    # -- callbacks ---------------------------------------------------------
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

    # -- relay callbacks ---------------------------------------------------
    async def _toggle_air() -> None:
        new = not S.relay_air_comp
        if await cmd_set_relay("air_comp", new):
            btn_air.props(
                f'color={"green" if S.relay_air_comp else "grey"}'
            )

    async def _toggle_o2() -> None:
        new = not S.relay_o2_conc
        if await cmd_set_relay("o2_conc", new):
            btn_o2.props(
                f'color={"green" if S.relay_o2_conc else "grey"}'
            )

    async def _toggle_o3() -> None:
        new = not S.relay_o3_gen
        if await cmd_set_relay("ozone_gen", new):
            btn_o3.props(
                f'color={"green" if S.relay_o3_gen else "grey"}'
            )

    async def _estop() -> None:
        if S.sequence_active:
            await cal.stop()
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

    # -- calibration callbacks ---------------------------------------------
    async def _cal_start() -> None:
        if not S.connected:
            ui.notify("Not connected to ESP32", type="negative")
            return
        if not S.relay_o2_conc:
            ui.notify(
                "O2 Concentrator should be ON first", type="warning"
            )
        await cal.start()

    async def _cal_stop() -> None:
        await cal.stop()
        ui.notify("Calibration stopped", type="info")

    # -- chart update helpers ----------------------------------------------
    def _update_power_curve() -> None:
        pwr, o3 = generate_power_curve(S.flow_lpm)
        target_o3 = predict_o3_from_power(
            S.power_target_pct, S.flow_lpm
        )
        fig = _make_power_fig(
            pwr, o3,
            S.power_target_pct, target_o3,
            S.power_actual_pct, S.vessel_o3_pct,
        )
        power_plot.figure = fig
        power_plot.update()

    def _make_power_fig(pwr, o3, tgt_pct, tgt_o3, act_pct, act_o3):
        fig = go.Figure()
        fig.add_trace(
            go.Scatter(
                x=pwr, y=o3, mode="lines",
                line=dict(color="royalblue", width=2),
                showlegend=False, name="Model",
            )
        )
        fig.add_trace(
            go.Scatter(
                x=[tgt_pct], y=[tgt_o3], mode="markers",
                marker=dict(
                    color="rgba(0,0,0,0)", size=18,
                    line=dict(color="black", width=3),
                ),
                showlegend=False, name="Target",
            )
        )
        fig.add_trace(
            go.Scatter(
                x=[act_pct], y=[act_o3], mode="markers",
                marker=dict(color="limegreen", size=14),
                showlegend=False, name="Actual",
            )
        )
        y_max = max(o3) * 1.1 if max(o3) > 0 else 1
        fig.update_layout(
            xaxis_title="Power %",
            yaxis_title="O3 %vol",
            height=340,
            margin=dict(l=50, r=20, t=10, b=50),
            xaxis=dict(range=[0, 105], dtick=10),
            yaxis=dict(range=[0, y_max]),
        )
        return fig

    # =====================================================================
    # BUILD THE PAGE
    # =====================================================================
    ui.page_title("BlockSI Control")
    ui.dark_mode(True)

    # -- LEFT DRAWER (sidebar) --------------------------------------------
    with ui.left_drawer(value=True).classes(
        "bg-dark q-pa-md"
    ).style("width:220px") as drawer:
        ui.label("BlockSI v2").classes("text-h6 text-weight-bold q-mb-sm")
        conn_icon = ui.icon("circle", color="red").classes("q-mr-xs")
        conn_label = ui.label("Disconnected").classes("text-caption")
        ui.separator().classes("q-my-sm")

        # Relay toggles
        ui.label("Relays").classes("text-subtitle2 q-mt-sm")
        with ui.row().classes("q-gutter-xs"):
            btn_air = ui.button(
                "Air", on_click=_toggle_air
            ).props("dense color=grey size=sm")
            btn_o2 = ui.button(
                "O2", on_click=_toggle_o2
            ).props("dense color=grey size=sm")
            btn_o3 = ui.button(
                "O3", on_click=_toggle_o3
            ).props("dense color=grey size=sm")

        ui.separator().classes("q-my-sm")

        # O2 LPM (manual meter reading)
        ui.label("O2 LPM").classes("text-caption")
        inp_lpm_sb = ui.number(
            value=S.flow_lpm, min=1.0, max=15.0, step=0.5,
            format="%.1f", on_change=_on_lpm_change,
        ).classes("q-mb-sm")

        ui.separator().classes("q-my-sm")

        # Sensor readings
        ui.label("Readings").classes("text-subtitle2")
        lbl_o2_read = ui.label(
            f"O2: {S.flow_lpm:.1f} LPM"
        ).classes("text-caption")
        lbl_o3_read = ui.label(
            f"O3: {S.vessel_o3_pct:.3f} %vol"
        ).classes("text-caption")
        lbl_tgt_o3 = ui.label(
            f"Target: {S.target_o3_pct:.2f} %vol"
        ).classes("text-caption")
        lbl_room_o3 = ui.label(
            f"Room: {S.room_o3_ppm:.3f} ppm"
        ).classes("text-caption")
        lbl_vtemp = ui.label("Vessel T: N/A").classes("text-caption")
        lbl_ctemp = ui.label(
            f"Cell T: {S.cell_temp_c:.1f} C"
        ).classes("text-caption")

    # -- HEADER ------------------------------------------------------------
    with ui.header().classes("bg-primary items-center q-px-md"):
        ui.button(
            icon="menu", on_click=lambda: drawer.toggle()
        ).props("flat dense round color=white")
        ui.label("BlockSI Control").classes("text-h6 q-ml-md")

    # -- TABS --------------------------------------------------------------
    with ui.column().classes("w-full q-pa-md"):
        # -- SEQUENCE BANNER (visible during automated sequences) ------
        with ui.row().classes(
            "w-full items-center q-pa-sm q-px-md q-gutter-md"
        ).style(
            "background: #F57F17; border-radius: 4px;"
            " min-height: 44px"
        ) as seq_banner:
            ui.icon("science", size="sm").classes("text-white")
            seq_name_lbl = ui.label("").classes(
                "text-white text-weight-bold"
            )
            seq_phase_lbl = ui.label("").classes(
                "text-white-50 text-caption"
            )
            seq_progress_bar = ui.linear_progress(
                value=0, show_value=False,
            ).props(
                "color=white track-color=amber-4"
            ).classes("col-grow").style("max-width: 220px")
            ui.button(
                "ABORT", icon="stop",
                on_click=_cal_stop, color="red-10",
            ).props("dense size=sm")
        seq_banner.visible = False

        with ui.tabs().classes("w-full") as tabs:
            tab_power = ui.tab("Power", icon="bolt")
            tab_telem = ui.tab("Telemetry", icon="show_chart")
            tab_cal = ui.tab("Calibration", icon="tune")
            tab_debug = ui.tab("Debug", icon="bug_report")

        with ui.tab_panels(tabs, value=tab_power).classes("w-full"):

            # =============================================================
            # POWER TAB
            # =============================================================
            with ui.tab_panel(tab_power):
                with ui.row().classes("w-full items-start q-gutter-md"):
                    # -- left: slider + presets + curve --------------------
                    with ui.column().classes("col-8"):
                        ui.label("Power (%)").classes("text-subtitle2")
                        slider = ui.slider(
                            min=0, max=100, step=1,
                            value=S.power_target_pct,
                            on_change=_on_power_slide,
                        ).classes("w-full")

                        # Graduated preset buttons
                        preset_btns = []
                        with ui.row().classes("q-gutter-xs q-mt-xs"):
                            for p in range(0, 110, 10):
                                _p = p  # capture
                                _btn = ui.button(
                                    str(_p),
                                    on_click=lambda _p=_p: _preset_click(
                                        _p
                                    ),
                                ).props("dense flat size=sm")
                                preset_btns.append(_btn)

                        # Power-O3 curve
                        init_pwr, init_o3 = generate_power_curve(
                            S.flow_lpm
                        )
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

                    # -- right: linked settings boxes ----------------------
                    with ui.column().classes("col-3"):
                        ui.label("Settings").classes(
                            "text-subtitle2 q-mb-sm"
                        )

                        ui.label("LPM").classes("text-caption")
                        inp_lpm_settings = ui.number(
                            value=S.flow_lpm,
                            min=1.0, max=15.0, step=0.5,
                            format="%.1f",
                            on_change=_on_lpm_change,
                        )

                        ui.label("% Power").classes(
                            "text-caption q-mt-sm"
                        )
                        inp_pwr = ui.number(
                            value=S.power_target_pct,
                            min=0, max=100, step=1,
                            on_change=_on_power_input,
                        )

                        ui.label("% vol O3").classes(
                            "text-caption q-mt-sm"
                        )
                        inp_o3 = ui.number(
                            value=round(S.target_o3_pct, 2),
                            min=0.0, max=5.0, step=0.01,
                            format="%.2f",
                            on_change=_on_o3_input,
                        )

                        ui.label("mg O3/s").classes(
                            "text-caption q-mt-sm"
                        )
                        inp_mg = ui.number(
                            value=round(S.target_mg_per_s, 2),
                            min=0.0, max=10.0, step=0.01,
                            format="%.2f",
                            on_change=_on_mg_input,
                        )

                        ui.label("g O3 @ 30 min").classes(
                            "text-caption q-mt-sm"
                        )
                        inp_g30 = ui.number(
                            value=round(S.target_g_30min, 2),
                            min=0.0, max=20.0, step=0.1,
                            format="%.2f",
                            on_change=_on_g30_input,
                        )

                # E-stop
                ui.separator().classes("q-my-md")
                ui.button(
                    "EMERGENCY STOP", icon="dangerous",
                    on_click=_estop, color="red",
                ).classes("q-mt-sm").props("size=lg")

            # =============================================================
            # TELEMETRY TAB
            # =============================================================
            with ui.tab_panel(tab_telem):
                # Metrics row
                with ui.row().classes("q-gutter-md q-mb-md"):
                    met_o3 = ui.label(
                        f"Vessel O3: {S.vessel_o3_pct:.4f} %vol"
                    ).classes("text-body1")
                    met_room = ui.label(
                        f"Room O3: {S.room_o3_ppm:.3f} ppm"
                    ).classes("text-body1")
                    met_vt = ui.label("Vessel T: N/A").classes(
                        "text-body1"
                    )
                    met_ct = ui.label(
                        f"Cell T: {S.cell_temp_c:.1f} C"
                    ).classes("text-body1")

                telem_plot = ui.plotly(go.Figure()).classes("w-full")

                with ui.expansion(
                    "Raw data", icon="table_chart"
                ).classes("w-full q-mt-sm"):
                    raw_table = ui.table(
                        columns=[
                            {
                                "name": "timestamp",
                                "label": "Time",
                                "field": "timestamp",
                                "align": "left",
                            },
                            {
                                "name": "vessel_o3_pct",
                                "label": "O3 %",
                                "field": "vessel_o3_pct",
                            },
                            {
                                "name": "room_o3_ppm",
                                "label": "Room ppm",
                                "field": "room_o3_ppm",
                            },
                            {
                                "name": "power_actual_pct",
                                "label": "Power %",
                                "field": "power_actual_pct",
                            },
                            {
                                "name": "cell_temp_c",
                                "label": "Cell C",
                                "field": "cell_temp_c",
                            },
                        ],
                        rows=[],
                        pagination={"rowsPerPage": 15},
                    ).classes("w-full")

            # =============================================================
            # CALIBRATION TAB
            # =============================================================
            with ui.tab_panel(tab_cal):
                ui.label("Power -- O3 Calibration").classes(
                    "text-h6 q-mb-sm"
                )
                ui.markdown(
                    "**Sequence** (~17 min): "
                    "1. Baseline 30 s @ 0 %  "
                    "2. Sweep Up 0->100 %  "
                    "3. Sweep Down 100->0 %  "
                    "4. 15 random levels x "
                    "(Air OFF 20 s + Air ON 20 s)"
                )

                with ui.row().classes("q-gutter-sm q-my-md"):
                    cal_start_btn = ui.button(
                        "Start", icon="play_arrow",
                        on_click=_cal_start, color="primary",
                    )
                    cal_stop_btn = ui.button(
                        "Stop", icon="stop",
                        on_click=_cal_stop, color="grey",
                    )

                cal_phase_lbl = ui.label("Phase: --").classes(
                    "text-body2"
                )
                cal_progress = ui.linear_progress(
                    value=0, show_value=False
                ).classes("q-mb-sm")
                cal_info_lbl = ui.label("").classes("text-caption")

                # Live scatter
                cal_plot = ui.plotly(go.Figure()).classes(
                    "w-full q-mt-sm"
                )

                ui.separator().classes("q-my-md")
                ui.label("Calibration Files").classes("text-subtitle2")
                cal_files_container = ui.column().classes("w-full")

                def _render_cal_files() -> None:
                    cal_files_container.clear()
                    files = list_calibration_files()
                    with cal_files_container:
                        if not files:
                            ui.label(
                                "No calibration files found"
                            ).classes("text-caption")
                        else:
                            for lpm in sorted(files):
                                with ui.expansion(
                                    f"O2 @ {lpm} LPM  "
                                    f"({len(files[lpm])} files)"
                                ):
                                    for fp in sorted(files[lpm]):
                                        ui.label(
                                            os.path.basename(fp)
                                        ).classes("text-caption")

                _render_cal_files()

            # =============================================================
            # DEBUG TAB
            # =============================================================
            with ui.tab_panel(tab_debug):
                ui.label("Debug Console").classes("text-h6 q-mb-sm")
                with ui.row().classes(
                    "w-full q-gutter-sm items-end"
                ):
                    debug_input = ui.input(
                        "Command", placeholder="status"
                    ).classes("col-9")
                    ui.button(
                        "Send", on_click=_send_debug_cmd
                    ).classes("col-2")
                debug_resp_label = ui.label("").classes(
                    "text-body2 q-mt-sm"
                )

                ui.separator().classes("q-my-md")
                ui.label("System State").classes("text-subtitle2")
                dbg_state_lbl = ui.code("").classes("w-full")

                ui.label("Log").classes("text-subtitle2 q-mt-md")
                dbg_log_area = ui.log(max_lines=100).classes(
                    "w-full h-64"
                )

    # =====================================================================
    # PERIODIC UI REFRESH  (runs every 1 s via NiceGUI timer)
    # =====================================================================
    _relay_ts: float = 0.0

    async def _tick() -> None:
        nonlocal _updating, _relay_ts

        # -- run calibration step ------------------------------------------
        if S.cal_active:
            await cal.step()

        # -- sync relays every ~10 s ---------------------------------------
        if S.connected and time.time() - _relay_ts > 10:
            await cmd_sync_relays()
            _relay_ts = time.time()

        # -- update sidebar status -----------------------------------------
        conn_icon.props(
            f'color={"green" if S.connected else "red"}'
        )
        conn_label.text = (
            "Connected" if S.connected else "Disconnected"
        )
        btn_air.props(
            f'color={"green" if S.relay_air_comp else "grey"}'
        )
        btn_o2.props(
            f'color={"green" if S.relay_o2_conc else "grey"}'
        )
        btn_o3.props(
            f'color={"green" if S.relay_o3_gen else "grey"}'
        )

        lbl_o2_read.text = f"O2: {S.flow_lpm:.1f} LPM"
        lbl_o3_read.text = f"O3: {S.vessel_o3_pct:.3f} %vol"
        lbl_tgt_o3.text = f"Target: {S.target_o3_pct:.2f} %vol"
        lbl_room_o3.text = f"Room: {S.room_o3_ppm:.3f} ppm"
        lbl_vtemp.text = (
            f"Vessel T: {S.vessel_temp_c:.1f} C"
            if S.vessel_temp_c > -900
            else "Vessel T: N/A"
        )
        lbl_ctemp.text = f"Cell T: {S.cell_temp_c:.1f} C"

        # -- sync power slider & inputs to SystemState -----------------
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
            seq_name_lbl.text = S.sequence_name
            _p_desc = {
                "baseline": "Baseline (0%)",
                "sweep_up": (
                    f"Sweep Up ({S.cal_current_power}%)"
                ),
                "sweep_down": (
                    f"Sweep Down ({S.cal_current_power}%)"
                ),
                "random_pairs": (
                    f"Random Pairs"
                    f" ({S.cal_random_idx + 1}"
                    f"/{CAL_NUM_RANDOM_POINTS})"
                ),
            }
            seq_phase_lbl.text = _p_desc.get(
                S.cal_phase, S.cal_phase or ""
            )
            seq_progress_bar.value = (
                S.cal_phase_progress / 100
            )
        for _el in [slider, inp_pwr, inp_o3, inp_mg, inp_g30]:
            if S.sequence_active:
                _el.props("disable")
            else:
                _el.props(remove="disable")
        for _btn in preset_btns:
            if S.sequence_active:
                _btn.props("disable")
            else:
                _btn.props(remove="disable")

        # -- power curve (target + actual markers) -------------------------
        _update_power_curve()

        # -- telemetry charts ----------------------------------------------
        if data_buf:
            df = pd.DataFrame(list(data_buf))
            met_o3.text = f"Vessel O3: {S.vessel_o3_pct:.4f} %vol"
            met_room.text = f"Room O3: {S.room_o3_ppm:.3f} ppm"
            met_vt.text = (
                f"Vessel T: {S.vessel_temp_c:.1f} C"
                if S.vessel_temp_c > -900
                else "Vessel T: N/A"
            )
            met_ct.text = f"Cell T: {S.cell_temp_c:.1f} C"

            fig = make_subplots(
                rows=2, cols=1,
                subplot_titles=(
                    "Ozone Concentration", "Power & Temperature"
                ),
                vertical_spacing=0.15,
                specs=[
                    [{"secondary_y": True}],
                    [{"secondary_y": True}],
                ],
            )
            ts = df["timestamp"]
            fig.add_trace(
                go.Scatter(
                    x=ts, y=df["vessel_o3_pct"],
                    name="Vessel O3 (%vol)",
                    line=dict(color="royalblue", width=2),
                ),
                row=1, col=1, secondary_y=False,
            )
            fig.add_trace(
                go.Scatter(
                    x=ts, y=df["room_o3_ppm"],
                    name="Room O3 (ppm)",
                    line=dict(color="limegreen", width=2),
                ),
                row=1, col=1, secondary_y=True,
            )
            if "power_actual_pct" in df.columns:
                fig.add_trace(
                    go.Scatter(
                        x=ts, y=df["power_actual_pct"],
                        name="Power (%)",
                        line=dict(color="orange", width=2),
                    ),
                    row=2, col=1, secondary_y=False,
                )
            fig.add_trace(
                go.Scatter(
                    x=ts, y=df["cell_temp_c"],
                    name="Cell Temp (C)",
                    line=dict(color="red", width=2),
                ),
                row=2, col=1, secondary_y=True,
            )
            fig.update_yaxes(
                title_text="O3 %vol", row=1, col=1, secondary_y=False
            )
            fig.update_yaxes(
                title_text="Room ppm", row=1, col=1, secondary_y=True
            )
            fig.update_yaxes(
                title_text="Power %", row=2, col=1, secondary_y=False
            )
            fig.update_yaxes(
                title_text="C", row=2, col=1, secondary_y=True
            )
            fig.update_layout(
                height=500, showlegend=True,
                margin=dict(l=50, r=30, t=30, b=30),
            )
            telem_plot.figure = fig
            telem_plot.update()

            # raw table (latest 20)
            rows = []
            for s in list(data_buf)[-20:]:
                rows.append(
                    {
                        "timestamp": str(
                            s["timestamp"].strftime("%H:%M:%S")
                        ),
                        "vessel_o3_pct": f"{s['vessel_o3_pct']:.4f}",
                        "room_o3_ppm": f"{s['room_o3_ppm']:.3f}",
                        "power_actual_pct": (
                            f"{s.get('power_actual_pct', 0):.1f}"
                        ),
                        "cell_temp_c": (
                            f"{s.get('cell_temp_c', 0):.1f}"
                        ),
                    }
                )
            raw_table.rows = rows
            raw_table.update()

        # -- calibration UI refresh ----------------------------------------
        if S.cal_active:
            phase_labels = {
                "baseline": "Baseline (0 %)",
                "sweep_up": "Sweep Up (0->100 %)",
                "sweep_down": "Sweep Down (100->0 %)",
                "random_pairs": "Random Pairs",
            }
            cal_phase_lbl.text = (
                f"Phase: "
                f"{phase_labels.get(S.cal_phase, S.cal_phase or '--')}"
            )
            cal_progress.value = S.cal_phase_progress / 100
            info = (
                f"Power={S.cal_current_power}%  "
                f"O3={S.vessel_o3_pct:.2f}%  "
                f"Air={'ON' if S.cal_air_state else 'OFF'}  "
                f"Samples={len(S.cal_data)}"
            )
            if S.cal_phase == "random_pairs" and S.cal_random_powers:
                idx = min(
                    S.cal_random_idx,
                    len(S.cal_random_powers) - 1,
                )
                info += (
                    f"  (point {S.cal_random_idx+1}"
                    f"/{CAL_NUM_RANDOM_POINTS}: "
                    f"{S.cal_random_powers[idx]}%)"
                )
            cal_info_lbl.text = info
        else:
            cal_phase_lbl.text = "Phase: --"
            cal_progress.value = 0
            cal_info_lbl.text = ""

        # Live calibration scatter
        if S.cal_data:
            cdf = pd.DataFrame(S.cal_data)
            cfig = go.Figure()
            off = cdf[~cdf["air_comp_on"]]
            if len(off):
                cfig.add_trace(
                    go.Scatter(
                        x=off["power_pct"], y=off["o3_pct"],
                        mode="markers", name="Air OFF",
                        marker=dict(color="royalblue", size=6),
                    )
                )
            on = cdf[cdf["air_comp_on"]]
            if len(on):
                cfig.add_trace(
                    go.Scatter(
                        x=on["power_pct"], y=on["o3_pct"],
                        mode="markers", name="Air ON",
                        marker=dict(color="orange", size=6),
                    )
                )
            cfig.update_layout(
                xaxis_title="Power %",
                yaxis_title="O3 %vol",
                height=340, showlegend=True,
                margin=dict(l=50, r=20, t=10, b=50),
            )
            cal_plot.figure = cfig
            cal_plot.update()

        # -- debug state dump ----------------------------------------------
        dbg_state_lbl.content = (
            f"connected={S.connected}  "
            f"time_synced={S.time_synced}\n"
            f"power_target={S.power_target_pct}%  "
            f"power_actual={S.power_actual_pct:.1f}%  "
            f"error={S.power_error}\n"
            f"relays: o3_gen={S.relay_o3_gen}  "
            f"o2_conc={S.relay_o2_conc}  "
            f"air_comp={S.relay_air_comp}\n"
            f"flow_lpm={S.flow_lpm}  "
            f"vessel_o3={S.vessel_o3_pct:.4f}  "
            f"room_o3={S.room_o3_ppm:.3f}\n"
            f"sequence={S.sequence_name or 'none'}  "
            f"cal_active={S.cal_active}  "
            f"cal_phase={S.cal_phase}  "
            f"samples={len(S.cal_data)}\n"
            f"data_buf={len(data_buf)}  "
            f"last_update={S.last_update}"
        )
        # push recent log lines
        for entry in list(debug_log)[:5]:
            dbg_log_area.push(entry)

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

