#!/usr/bin/env python3
"""
BlockSI Dashboard — NiceGUI, Recipe-Based Sequence Protocol
Run with:  .venv\\Scripts\\python.exe blocksi_dashboard.py [--port 5000]

Architecture ("ESP32 = Arms, PC = Brains"):
  - ESP32 connects to PC via TCP on port 5000
  - PC generates recipes (step lists, prompts) and sends to ESP32
  - ESP32 executes blindly: set power, count samples, stream SEQ messages
  - PC does ALL analysis: statistics, pass/fail, model fitting, CSV saving
  - PC is sole authority for power_target_pct (never from telemetry)

Recipe protocol:
  PC → CMD,sequence_start,<type>,<params>
  PC → CMD,seq_step,<idx>,<pwr>,<hold>,<phase>  (× N)
  PC → CMD,seq_prompt,<before>,<id>,<text>       (× M)
  PC → CMD,seq_run
  ESP32 → SEQ,<type>,STARTED/STEP/SAMPLE/PROMPT/COMPLETE/ABORTED

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
import random
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
TELEMETRY_DIR = os.path.join(DATA_DIR, "Telemetry")
CALIBRATION_DIR = os.path.join(DATA_DIR, "Calibration")
VALIDATION_DIR = os.path.join(DATA_DIR, "Validation")
MODEL_DIR = os.path.join(BASE_DIR, "Models", "O3Power")

for _d in (DATA_DIR, TELEMETRY_DIR, CALIBRATION_DIR, VALIDATION_DIR, MODEL_DIR):
    os.makedirs(_d, exist_ok=True)

# Power model coefficients  (O3_max = A/F + B)
POWER_MODEL_A = 1.78
POWER_MODEL_B = 1.40
DEFAULT_FLOW_LPM = 4.0

# O3 mass-flow conversion  mg/s = %vol * LPM * K
O3_MASS_FLOW_K = 0.357
AIR_COMP_LPM = 10.0

# O2 concentration constants (for weighted-average calculation)
O2_CONC_PCT = 95      # O2 concentrator output purity
AIR_COMP_O2_PCT = 21  # Atmospheric O2 from air compressor

POWER_MISMATCH_THRESHOLD = 5.0

# =============================================================================
# Prompt content mapping — rich descriptions for ESP32 prompt IDs
# =============================================================================
PROMPT_CONTENT: dict[str, dict[str, str]] = {
    "check_flow": {
        "title": "Verify O2 Flow",
        "icon": "air",
        "body": (
            "<b>Check the rotameter</b> and verify the O2 flow matches the "
            "target LPM setting.<br><br>"
            "Ensure the gas route is connected to the 106-H sensor.<br><br>"
            "Press <b>Confirm</b> when the flow is correct."
        ),
    },
    "check_route": {
        "title": "Confirm Gas Route",
        "icon": "sensors",
        "body": (
            "<b>Confirm the gas route</b> is correctly plumbed to the 106-H "
            "ozone sensor for measurement.<br><br>"
            "Verify there are no leaks and all fittings are tight.<br><br>"
            "Press <b>Confirm</b> when ready to proceed."
        ),
    },
    # Legacy prompt IDs (kept for backward compatibility)
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
        # Sequence observer state  (ESP32 executes PC-generated recipes)
        # ------------------------------------------------------------------
        self.sequence_active: bool = False
        self.seq_type: str = ""            # "calibrate", "validate", etc.
        self.seq_phase: str = ""           # current phase name from step
        self.seq_progress: float = 0.0     # 0-100
        self.seq_elapsed: float = 0.0      # seconds
        self.seq_power: float = 0.0        # power target at current step
        self.seq_step_idx: int = 0         # current step index
        self.seq_step_total: int = 0       # total steps in recipe
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
            TELEMETRY_DIR, f"{datetime.now():%Y-%m-%d}_Stream.csv"
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
# O2 concentration helpers
# =============================================================================
def compute_effective_o2_pct(flow_lpm: float, air_comp_on: bool) -> int:
    """Weighted-average O2% from O2 concentrator + optional air compressor.

    O2 concentrator: *flow_lpm* at ~95% O2.
    Air compressor:  ~10 LPM at ~21% O2 (only when *air_comp_on*).
    Returns integer O2% rounded to nearest whole number.
    """
    if air_comp_on:
        total = flow_lpm + AIR_COMP_LPM
        if total > 0:
            return round((flow_lpm * O2_CONC_PCT + AIR_COMP_LPM * AIR_COMP_O2_PCT) / total)
    return O2_CONC_PCT


# =============================================================================
# Calibration file helpers
# =============================================================================
def list_calibration_files() -> dict[tuple[float, int], list[str]]:
    """List calibration CSVs grouped by (LPM, O2%).

    Parses filenames like ``2026-02-25_143022_PowerO3Cal_4.0Lpm_95O2.csv``.
    Falls back to O2_CONC_PCT when no O2 tag is present (legacy files).
    """
    out: dict[tuple[float, int], list[str]] = {}
    if not os.path.exists(CALIBRATION_DIR):
        return out
    for fn in os.listdir(CALIBRATION_DIR):
        if fn.endswith(".csv") and "PowerO3Cal" in fn:
            m_lpm = re.search(r"_(\d+(?:\.\d+)?)Lpm", fn)
            if m_lpm:
                lpm = float(m_lpm.group(1))
                m_o2 = re.search(r"_(\d+)O2", fn)
                o2 = int(m_o2.group(1)) if m_o2 else O2_CONC_PCT
                out.setdefault((lpm, o2), []).append(
                    os.path.join(CALIBRATION_DIR, fn)
                )
    return out


def _save_cal_csv(samples: list[dict], lpm: float, o2_pct: int) -> str:
    """Save calibration samples to CSV, return filename.

    Naming: ``{YYYY-MM-DD}_{HHMMSS}_PowerO3Cal_{LPM}Lpm_{O2}O2.csv``
    """
    now = datetime.now()
    lpm_s = f"{lpm:.0f}" if lpm == int(lpm) else f"{lpm:.1f}"
    fname = f"{now:%Y-%m-%d_%H%M%S}_PowerO3Cal_{lpm_s}Lpm_{o2_pct}O2.csv"
    fpath = os.path.join(CALIBRATION_DIR, fname)
    if samples:
        pd.DataFrame(samples).to_csv(fpath, index=False)
        log(f"Saved {len(samples)} cal samples -> {fname}", "cal")
    return fname


# =============================================================================
# Recipe generation  (PC = Brains — generates step lists for ESP32)
# =============================================================================
def generate_cal_recipe(flow_lpm: float) -> tuple[list[tuple], list[tuple]]:
    """Generate calibration recipe: baseline + sweep up + sweep down + random.

    Returns (steps, prompts).
    Each step = (idx, power_pct, hold_samples, phase, air_comp).
    Each prompt = (before_step_idx, prompt_id, text).
    """
    steps: list[tuple] = []

    # Phase 1 — Baseline: 0% for 15 samples (~37s)
    steps.append((len(steps), 0, 15, "baseline", 0))

    # Phase 2 — Sweep Up: 0→100% in 1% steps, 2 samples each
    for pwr in range(0, 101):
        steps.append((len(steps), pwr, 2, "sweep_up", 0))

    # Phase 3 — Sweep Down: 100→0% in 1% steps, 2 samples each
    for pwr in range(100, -1, -1):
        steps.append((len(steps), pwr, 2, "sweep_down", 0))

    # Phase 4 — Random spot checks: 15 random levels, 5 samples each
    for _ in range(15):
        pwr = random.randint(0, 100)
        steps.append((len(steps), pwr, 5, "random", 0))

    prompts: list[tuple] = [
        (0, "check_flow", "Verify O2 flow and gas route before calibration"),
    ]
    return steps, prompts


def generate_val_recipe(
    power_pct: float, flow_lpm: float
) -> tuple[list[tuple], list[tuple]]:
    """Generate validation recipe: baseline + spots + target + cooldown.

    Returns (steps, prompts).
    """
    spot1 = max(10, int(power_pct * 0.33))
    spot2 = max(10, int(power_pct * 0.66))
    steps: list[tuple] = [
        (0, 0,              15, "baseline",  0),  # 0% for ~37s
        (1, spot1,           5, "spot_low",  0),  # ~33% for ~12s
        (2, spot2,           5, "spot_high", 0),  # ~66% for ~12s
        (3, int(power_pct), 15, "target",    0),  # Target power for ~37s
        (4, 0,               5, "cooldown",  0),  # 0% for ~12s
    ]
    prompts: list[tuple] = [
        (0, "check_flow", "Verify O2 flow matches rotameter"),
        (1, "check_route", "Confirm gas route to 106-H sensor"),
    ]
    return steps, prompts


def _analyze_validation(samples: list[dict], power_pct: float,
                        flow_lpm: float) -> dict:
    """Analyze validation samples and determine pass/fail.

    Criteria from interface_contract.md:
    - Baseline: mean O3 < 0.02 %vol
    - Spot correlation: each within 0.15 %vol or 15% relative of model
    - Target accuracy: mean within 10% relative of prediction
    - Target stability: CV < 5%
    """
    result: dict[str, Any] = {
        "power": power_pct,
        "flow": flow_lpm,
        "total_samples": len(samples),
    }

    # Group by phase
    by_phase: dict[str, list[dict]] = {}
    for s in samples:
        phase = s.get("phase", "unknown")
        by_phase.setdefault(phase, []).append(s)

    # Baseline check
    baseline = by_phase.get("baseline", [])
    if baseline:
        bl_mean = float(np.mean([s["o3_pct"] for s in baseline]))
        result["baseline_mean"] = bl_mean
        result["baseline_ok"] = bl_mean < 0.02
    else:
        result["baseline_mean"] = 0.0
        result["baseline_ok"] = True

    # Spot correlation checks
    spot_checks = []
    for phase_name in ("spot_low", "spot_high"):
        phase_data = by_phase.get(phase_name, [])
        if not phase_data:
            continue
        spot_mean = float(np.mean([s["o3_pct"] for s in phase_data]))
        spot_power = phase_data[0].get("power_target", 0)
        expected = predict_o3_from_power(spot_power, flow_lpm)
        abs_err = abs(spot_mean - expected)
        rel_err = (abs_err / expected * 100) if expected > 0 else 0
        spot_ok = abs_err < 0.15 or rel_err < 15
        spot_checks.append({
            "phase": phase_name, "mean_o3": spot_mean,
            "expected_o3": expected, "abs_err": abs_err,
            "rel_err": rel_err, "ok": spot_ok,
        })
    result["spot_checks"] = spot_checks
    result["spots_ok"] = all(sc["ok"] for sc in spot_checks)

    # Target analysis
    target = by_phase.get("target", [])
    if target:
        t_o3 = [s["o3_pct"] for s in target]
        mean_o3 = float(np.mean(t_o3))
        std_o3 = float(np.std(t_o3))
        mean_temp = float(np.mean([s["temp_c"] for s in target]))
        expected_o3 = predict_o3_from_power(power_pct, flow_lpm)
        deviation_pct = (
            abs(mean_o3 - expected_o3) / expected_o3 * 100
            if expected_o3 > 0 else 0.0
        )
        cv = (std_o3 / mean_o3 * 100) if mean_o3 > 0 else 0.0

        result.update({
            "mean_o3": mean_o3,
            "std_o3": std_o3,
            "expected_o3": expected_o3,
            "deviation_pct": deviation_pct,
            "cv_pct": cv,
            "mean_temp": mean_temp,
            "target_samples": len(target),
            "target_ok": deviation_pct < 10,
            "stable": cv < 5.0,
        })
    else:
        result.update({
            "mean_o3": 0.0, "std_o3": 0.0, "expected_o3": 0.0,
            "deviation_pct": 0.0, "cv_pct": 0.0, "mean_temp": 0.0,
            "target_samples": 0, "target_ok": False, "stable": False,
        })

    # Overall pass/fail
    result["passed"] = (
        result.get("baseline_ok", False)
        and result.get("spots_ok", True)
        and result.get("target_ok", False)
        and result.get("stable", False)
    )
    return result


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

    # -- SEQ message handler (generic recipe protocol) ---------------------
    def _handle_seq(self, line: str) -> None:
        """Handle all SEQ,<type>,<action>,... messages from recipe executor."""
        parts = line.split(",")
        if len(parts) < 3:
            log(f"Malformed SEQ: {line[:80]}", "warn")
            return

        seq_type = parts[1]   # e.g., "calibrate", "validate"
        action = parts[2]     # STARTED, STEP, SAMPLE, PROMPT, COMPLETE, ABORTED

        if action == "STARTED":
            # SEQ,calibrate,STARTED,steps=203,flow=4.0
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
                        elif seq_type == "validate":
                            S.val_lpm = float(v)
                except ValueError:
                    pass
            log(
                f"Sequence '{seq_type}' started ({S.seq_step_total} steps)",
                "seq",
            )
            _notify(f"{seq_type.title()} sequence started")

        elif action == "RELAY":
            # SEQ,calibrate,RELAY,<relay_name>,<0|1>
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
            # SEQ,calibrate,STATUS,relay_stabilizing
            status = parts[3].strip() if len(parts) > 3 else ""
            if status == "relay_stabilizing":
                S.seq_phase = "stabilizing"
            log(f"SEQ status: {status}", "seq")

        elif action == "STEP":
            # SEQ,calibrate,STEP,45,22,sweep_up,0
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
                # Parse optional air_comp field
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
            # SEQ,calibrate,SAMPLE,step_idx,sample_num,o3_pct,temp_c,power_actual[,air_comp]
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
                    elif seq_type == "validate":
                        S.val_samples.append(sample)
                except (ValueError, IndexError):
                    log(f"Bad SAMPLE: {line[:80]}", "error")

        elif action == "PROMPT":
            # SEQ,calibrate,PROMPT,check_flow,Verify O2 flow
            S.pending_prompt_id = parts[3] if len(parts) > 3 else ""
            S.pending_prompt_text = (
                ",".join(parts[4:]) if len(parts) > 4 else ""
            )
            log(f"Prompt requested: {S.pending_prompt_id}", "seq")

        elif action == "COMPLETE":
            # SEQ,calibrate,COMPLETE,1020
            elapsed = 0.0
            if len(parts) > 3:
                try:
                    elapsed = float(parts[3])
                except ValueError:
                    pass
            S.seq_elapsed = elapsed
            log(
                f"Sequence '{seq_type}' complete ({elapsed:.0f}s)", "seq"
            )
            # Post-sequence analysis
            if seq_type == "calibrate":
                if S.cal_samples:
                    o2 = compute_effective_o2_pct(S.cal_lpm, S.relay_air_comp)
                    S.cal_file = _save_cal_csv(S.cal_samples, S.cal_lpm, o2)
                    _notify(
                        f"Calibration: {len(S.cal_samples)} samples saved",
                        "positive",
                    )
                else:
                    _notify("Calibration complete (no samples)", "warning")
            elif seq_type == "validate":
                S.val_result = _analyze_validation(
                    S.val_samples, S.val_power, S.val_lpm
                )
                dev = S.val_result.get("deviation_pct", 0.0)
                if S.val_result.get("passed"):
                    _notify(
                        f"Validation PASSED — {dev:.1f}% deviation",
                        "positive",
                    )
                elif dev < 20:
                    _notify(
                        f"Validation marginal — {dev:.1f}% deviation",
                        "warning",
                    )
                else:
                    _notify(
                        f"Validation FAILED — {dev:.1f}% deviation",
                        "negative",
                    )
            else:
                _notify(
                    f"{seq_type.title()} completed ({elapsed:.0f}s)",
                    "positive",
                )
            S.sequence_active = False
            S.pending_prompt_id = ""
            S.pending_prompt_text = ""

        elif action == "ABORTED":
            # SEQ,calibrate,ABORTED,user_request
            reason = ",".join(parts[3:]) if len(parts) > 3 else "unknown"
            log(f"Sequence '{seq_type}' aborted: {reason}", "seq")
            S.sequence_active = False
            S.pending_prompt_id = ""
            S.pending_prompt_text = ""
            _notify(f"{seq_type.title()} aborted: {reason}", "warning")

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
        await cmd_sequence_abort()
    await cmd_set_power(0)
    await cmd_set_relay("ozone_gen", False)


async def cmd_sequence_start(seq_type: str, **kwargs) -> bool:
    """Generate a recipe and send it to ESP32.

    Protocol: sequence_start → seq_step × N → seq_prompt × M → seq_run.
    Steps are sent via raw TCP (no individual response wait) for speed.
    """
    if not S.connected:
        _notify("Not connected to ESP32", "negative")
        return False

    # Generate recipe based on type
    # Relay prereqs: executor applies these at seq_run time
    relay_params = ",relay_o2=1,relay_o3=1,relay_air=0"
    if seq_type == "calibrate":
        flow = kwargs.get("flow", DEFAULT_FLOW_LPM)
        steps, prompts = generate_cal_recipe(flow)
        param_str = f"flow={flow}{relay_params}"
    elif seq_type == "validate":
        power = kwargs.get("power", 75.0)
        flow = kwargs.get("flow", DEFAULT_FLOW_LPM)
        steps, prompts = generate_val_recipe(power, flow)
        param_str = f"power={power},flow={flow}{relay_params}"
        S.val_power = power
        S.val_lpm = flow
    else:
        log(f"Unknown sequence type: {seq_type}", "error")
        return False

    # Pre-flight: belt-and-suspenders relay enable (executor also applies)
    if not S.relay_o2_conc:
        log("Pre-enabling O2 concentrator relay", "seq")
        await cmd_set_relay("o2_conc", True)
    if not S.relay_o3_gen:
        log("Pre-enabling ozone generator relay", "seq")
        await cmd_set_relay("ozone_gen", True)
    if S.relay_air_comp:
        log("Pre-disabling air compressor relay", "seq")
        await cmd_set_relay("air_comp", False)

    # Enter loading state
    S.sequence_active = True
    S.seq_type = seq_type
    S.seq_phase = "loading"
    S.seq_progress = 0.0
    S.seq_elapsed = 0.0
    S.seq_step_idx = 0
    S.seq_step_total = len(steps)
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    if seq_type == "calibrate":
        S.cal_samples = []
        S.cal_file = ""
        S.cal_lpm = kwargs.get("flow", DEFAULT_FLOW_LPM)
    elif seq_type == "validate":
        S.val_samples = []
        S.val_result = {}

    # 1. Send sequence_start (wait for RSP)
    resp = await tcp.send_command(f"sequence_start,{seq_type},{param_str}")
    if not resp or "OK" not in resp:
        log(f"sequence_start failed: {resp}", "error")
        S.sequence_active = False
        return False

    # 2. Send all steps via raw TCP (fire-and-forget for speed)
    log(f"Sending {len(steps)} recipe steps...", "seq")
    for idx, pwr, hold, phase, air in steps:
        await tcp._send_raw(f"CMD,seq_step,{idx},{pwr},{hold},{phase},{air}\n")
        await asyncio.sleep(0.005)  # brief yield to avoid buffer overflow

    # 3. Send prompts via raw TCP
    for before, pid, text in prompts:
        await tcp._send_raw(f"CMD,seq_prompt,{before},{pid},{text}\n")
        await asyncio.sleep(0.005)

    # Brief pause for ESP32 to process buffered commands
    await asyncio.sleep(0.1)

    # 4. Send seq_run (wait for RSP — validates recipe loaded OK)
    resp = await tcp.send_command("seq_run")
    if resp and "OK" in resp:
        log(f"Recipe running: {len(steps)} steps", "seq")
        return True

    log(f"seq_run failed: {resp}", "error")
    S.sequence_active = False
    return False


async def cmd_sequence_abort(reason: str = "") -> bool:
    """Send CMD,sequence_abort[,reason]."""
    cmd = "sequence_abort"
    if reason:
        cmd += f",{reason}"
    resp = await tcp.send_command(cmd)
    return resp is not None and "OK" in resp


async def cmd_sequence_stop() -> bool:
    """Legacy alias — sends sequence_abort."""
    return await cmd_sequence_abort()


async def cmd_sequence_confirm() -> bool:
    """Send CMD,sequence_confirm (no args — unblocks pending prompt)."""
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    resp = await tcp.send_command("sequence_confirm")
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
                    asyncio.create_task(cmd_sequence_abort()),
                )).props("flat")
                ui.button("Confirm", color="green", icon="check", on_click=lambda: (
                    prompt_dialog.close(),
                    asyncio.create_task(cmd_sequence_confirm()),
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
                        "PC-generated recipe (~17 min): Baseline → Sweep Up "
                        "(0→100%) → Sweep Down (100→0%) → Random Spots. "
                        "Air compressor must be OFF."
                    ).classes("text-caption text-grey q-mb-sm")

                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        cal_lpm_input = ui.number(
                            label="O2 LPM", value=DEFAULT_FLOW_LPM,
                            min=0.5, max=15, step=0.5, format="%.1f",
                        ).classes("w-24")

                        async def _start_cal():
                            lpm = float(cal_lpm_input.value or DEFAULT_FLOW_LPM)
                            await cmd_sequence_start(
                                "calibrate", flow=lpm,
                            )

                        cal_start_btn = ui.button(
                            "Start", icon="play_arrow",
                            on_click=_start_cal, color="primary",
                        )
                        cal_stop_btn = ui.button(
                            "Stop", icon="stop",
                            on_click=lambda: asyncio.create_task(
                                cmd_sequence_abort()
                            ),
                            color="grey",
                        )

                    # Phase stepper cards
                    CAL_PHASES_DEF = [
                        ("baseline", "Baseline", "~37s @ 0%"),
                        ("sweep_up", "Sweep Up", "0 → 100%"),
                        ("sweep_down", "Sweep Down", "100 → 0%"),
                        ("random", "Random Spots", "15 random levels"),
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

                    # Live scatter (ECharts) — colored by phase
                    cal_chart = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "item"},
                        "legend": {"data": ["Sweep Up", "Sweep Down", "Random"]},
                        "xAxis": {"type": "value", "name": "Power %",
                                  "min": 0, "max": 100},
                        "yAxis": {"type": "value", "name": "O3 %vol"},
                        "series": [
                            {"name": "Sweep Up", "type": "scatter", "data": [],
                             "itemStyle": {"color": "#42A5F5"}},
                            {"name": "Sweep Down", "type": "scatter", "data": [],
                             "itemStyle": {"color": "#FFA726"}},
                            {"name": "Random", "type": "scatter", "data": [],
                             "itemStyle": {"color": "#66BB6A"}},
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
                                for (lpm, o2) in sorted(files):
                                    flist = files[(lpm, o2)]
                                    with ui.expansion(
                                        f"{lpm} LPM / {o2}% O2 ({len(flist)} files)"
                                    ):
                                        for fp in sorted(flist):
                                            ui.label(os.path.basename(fp)).classes(
                                                "text-caption"
                                            )

                    _render_cal_files()

                # ---- Validation section ---------------------------------
                with ui.expansion("Validation", icon="verified").classes(
                    "w-full q-mb-sm"
                ):
                    ui.markdown(
                        "Pre-flight check: PC generates a recipe with baseline, "
                        "spot checks, and target power hold. Compares measured "
                        "O3 to model prediction. Air compressor must be OFF."
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
                            pwr = float(val_pwr_input.value or 75)
                            lpm = float(val_lpm_input.value or DEFAULT_FLOW_LPM)
                            await cmd_sequence_start(
                                "validate", power=pwr, flow=lpm,
                            )

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
                        fpath = os.path.join(TELEMETRY_DIR, fname)
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
            type_names = {
                "calibrate": "CALIBRATE",
                "validate": "VALIDATE",
            }
            display_name = type_names.get(S.seq_type, S.seq_type.upper())
            # Phase-aware banner text
            phase = S.seq_phase
            if phase == "loading":
                seq_name_lbl.text = f"{display_name} — Loading recipe..."
                seq_phase_lbl.text = f"{S.seq_step_total} steps"
            elif phase == "relay_setup":
                seq_name_lbl.text = f"{display_name} — Enabling relays..."
                seq_phase_lbl.text = ""
            elif phase == "stabilizing":
                seq_name_lbl.text = f"{display_name} — Equipment stabilizing..."
                seq_phase_lbl.text = "~3s warm-up"
            elif phase == "started":
                seq_name_lbl.text = f"{display_name} — Starting..."
                seq_phase_lbl.text = ""
            else:
                seq_name_lbl.text = (
                    f"{display_name} — Step {S.seq_step_idx}/{S.seq_step_total}"
                )
                seq_phase_lbl.text = phase.replace("_", " ").title()
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
        if S.sequence_active and S.seq_type == "calibrate":
            cal_phase_lbl.text = (
                f"Phase: {S.seq_phase.replace('_', ' ').title()}  "
                f"Step {S.seq_step_idx}/{S.seq_step_total}"
            )
            cal_progress.value = S.seq_progress / 100
            cal_info_lbl.text = (
                f"Power={S.seq_power:.0f}%  "
                f"O3={S.vessel_o3_pct:.2f}%  "
                f"Samples={len(S.cal_samples)}"
            )
            # Highlight active/completed phase cards
            phase_order = ["baseline", "sweep_up", "sweep_down", "random"]
            active_idx = (
                phase_order.index(S.seq_phase)
                if S.seq_phase in phase_order else -1
            )
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

        # Cal scatter chart — by phase (air compressor OFF during cal)
        if S.cal_samples:
            sweep_up = [
                [s["power_actual"], s["o3_pct"]]
                for s in S.cal_samples if s.get("phase") == "sweep_up"
            ]
            sweep_down = [
                [s["power_actual"], s["o3_pct"]]
                for s in S.cal_samples if s.get("phase") == "sweep_down"
            ]
            rand_pts = [
                [s["power_actual"], s["o3_pct"]]
                for s in S.cal_samples if s.get("phase") == "random"
            ]
            cal_chart.options["series"][0]["data"] = sweep_up
            cal_chart.options["series"][1]["data"] = sweep_down
            cal_chart.options["series"][2]["data"] = rand_pts
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
            passed = r.get("passed", False)
            dev = r.get("deviation_pct", 0.0)
            cv = r.get("cv_pct", 0.0)
            if passed:
                color, icon_name = "green", "check_circle"
            elif isinstance(dev, (int, float)) and dev < 20:
                color, icon_name = "amber", "warning"
            else:
                color, icon_name = "red", "error"
            val_result_icon.name = icon_name
            val_result_icon._classes = [f"text-h3 text-{color}"]
            val_result_icon.update()
            status = "PASSED" if passed else "FAILED"
            val_result_title.text = f"{status} — {dev:.1f}% deviation"
            val_mean_lbl.text = f"Mean O3: {r.get('mean_o3', 0):.3f}%"
            val_expected_lbl.text = f"Expected: {r.get('expected_o3', 0):.3f}%"
            val_dev_lbl.text = f"CV: {cv:.1f}%"
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
            "seq_step": f"{S.seq_step_idx}/{S.seq_step_total}",
            "seq_progress": round(S.seq_progress, 1),
            "pending_prompt": S.pending_prompt_id,
            "cal_samples": len(S.cal_samples),
            "val_samples": len(S.val_samples),
            "val_passed": S.val_result.get("passed", None),
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

