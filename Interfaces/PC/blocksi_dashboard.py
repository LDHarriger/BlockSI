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
import re
import time
import traceback
from collections import deque
from datetime import datetime
from typing import Any, Optional

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from nicegui import ui, app

# Analysis module — model fitting and prediction
from analysis import (
    PowerO3Model,
    load_model_for_condition,
    list_models as list_saved_models,
    fit_sigmoid_model,
    save_model as save_model_json,
    predict_o3 as _model_predict_o3,
    predict_power as _model_predict_power,
    generate_curve as _model_generate_curve,
    aggregate_calibration_data,
    # CSTR model (with decay)
    CSTRModel,
    FillModel,  # backward-compatible alias
    fit_cstr_model,
    load_cstr_model_from_dir,
    save_cstr_model as save_cstr_model_json,
    list_cstr_models as list_saved_cstr_models,
)

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
CSTR_DATA_DIR = os.path.join(DATA_DIR, "CSTR")
MODEL_DIR = os.path.join(BASE_DIR, "Models", "O3Power")
CSTR_MODEL_DIR = os.path.join(BASE_DIR, "Models", "CSTR")

for _d in (DATA_DIR, TELEMETRY_DIR, CALIBRATION_DIR, VALIDATION_DIR,
           CSTR_DATA_DIR, MODEL_DIR, CSTR_MODEL_DIR):
    os.makedirs(_d, exist_ok=True)

# Power model coefficients  (legacy fallback — O3_max = A/F + B)
POWER_MODEL_A = 1.78
POWER_MODEL_B = 1.40
DEFAULT_FLOW_LPM = 4.0

# O3 mass-flow conversion
#   K = M_O3 / (V_m × 60 × 100)
#   V_m = 24.04 L/mol at 20°C, 1 atm (rotameter reference temperature)
#   M_O3 = 48.0 g/mol
#   Factor 60 converts L/min → L/s;  factor 100 converts %vol → fraction
#   Result: mg/s = %vol × LPM × K
O3_MASS_FLOW_K = 0.3327      # 48000 / (24.04 × 60 × 100)
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
    """Predict O3 %vol from power %, using active model if available."""
    return _model_predict_o3(power_pct, flow_lpm, S.active_model)


def predict_power_from_o3(o3_pct: float, flow_lpm: float) -> float:
    """Predict power % from O3 %vol, using active model if available."""
    return _model_predict_power(o3_pct, flow_lpm, S.active_model)


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
    """Generate power/O3 curve arrays for plotting, using active model."""
    return _model_generate_curve(flow_lpm, S.active_model)


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
        self.last_esp_ts_ms: int = 0  # raw ESP32 ms timestamp from most recent DATA frame
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
        self.seq_start_time: float = 0.0   # time.time() when sequence confirmed
        self.seq_confirmed: bool = False   # True after ESP32 confirmed start
        self.seq_cleanup_pending: bool = False  # Set by COMPLETE/ABORTED, consumed by _tick
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
        self.val_file: str = ""
        # Settings
        self.notify_level: str = "all"     # "all" | "errors" | "none"
        # Backfill state (during BACKFILL_START..BACKFILL_END from ESP32)
        self.backfill_active: bool = False
        self.backfill_expected: int = 0
        self.backfill_received: int = 0
        self.backfill_start_time: float = 0.0
        # Derived (kept in sync by update_derived)
        self.target_o3_pct: float = 0.0
        self.target_mg_per_s: float = 0.0
        self.target_g_30min: float = 0.0
        # Active model (loaded from Models/O3Power/)
        self.active_model: Optional[PowerO3Model] = None
        self.model_status: str = "No model"  # "Fitted", "Fallback", "No model"
        # CSTR model (loaded from Models/CSTR/)
        self.active_cstr_model: Optional[CSTRModel] = None
        self.cstr_model_status: str = "No model"
        # Fill/Evac sequence state
        self.fill_active: bool = False
        self.fill_phase: str = ""        # "baseline", "fill", "transition", "evac", ""
        self.cstr_samples: list[dict] = []   # all phases in one list
        self.cstr_csv_path: str = ""
        self.fill_target_o3: float = 0.0   # validated max O3 at 100% power
        self.fill_lpm: float = DEFAULT_FLOW_LPM

    def load_model_for_current_condition(self) -> None:
        """Try to load a fitted model matching current flow/O2%."""
        o2 = compute_effective_o2_pct(self.flow_lpm, self.relay_air_comp)
        model = load_model_for_condition(MODEL_DIR, self.flow_lpm, o2)
        if model is not None and model.is_valid:
            self.active_model = model
            self.model_status = (
                f"Sigmoid (R²={model.r_squared:.3f})"
            )
            log(
                f"Model loaded: {model.flow_lpm} LPM / {model.o2_pct}% O2 "
                f"R²={model.r_squared:.4f}",
                "info",
            )
        else:
            self.active_model = None
            self.model_status = "Fallback (piecewise)"

    def load_cstr_model(self) -> None:
        """Try to load the universal CSTR model."""
        model = load_cstr_model_from_dir(CSTR_MODEL_DIR)
        if model is not None and model.is_valid:
            self.active_cstr_model = model
            self.cstr_model_status = (
                f"V={model.system_volume_L:.1f}L, "
                f"k_d={model.decay_rate_per_s:.5f}/s "
                f"(R²={model.r_squared_fill:.3f}/{model.r_squared_evac:.3f})"
            )
            log(
                f"CSTR model loaded: V={model.system_volume_L:.2f}L, "
                f"k_d={model.decay_rate_per_s:.6f}/s, "
                f"cal@{model.calibration_flow_lpm}LPM",
                "info",
            )
        else:
            self.active_cstr_model = None
            self.cstr_model_status = "No model"

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
    S.last_esp_ts_ms = sample.get("esp_ts_ms", S.last_esp_ts_ms)
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


_notify_queue: deque[tuple[str, str]] = deque()


def _notify(msg: str, level: str = "positive") -> None:
    """Show toast notification respecting user preference.

    Safe to call from background tasks (asyncio.create_task context): if
    NiceGUI's slot context is unavailable, the message is queued and the
    periodic _tick loop will flush it with the correct context.
    """
    if S.notify_level == "none":
        return
    if S.notify_level == "errors" and level == "positive":
        return
    try:
        ui.notify(msg, type=level, position="bottom-right", timeout=3000)
    except RuntimeError:
        _notify_queue.append((msg, level))


# =============================================================================
# CSV logger  (stream telemetry — new file per connection session)
# =============================================================================
CSV_HEADER = (
    "timestamp,esp_ts_ms,vessel_o3_pct,cell_temp_c,"
    "pressure_mbar,room_o3_ppm,vessel_temp_c,"
    "power_target,power_actual,wiper_v\n"
)


class _CSVLogger:
    def __init__(self) -> None:
        self._path: Optional[str] = None
        self._header = False

    def reset(self, connect_time: Optional[datetime] = None) -> None:
        """Start a new CSV file.  Called on each ESP32 (re)connection."""
        ts = connect_time or datetime.now()
        self._path = os.path.join(
            TELEMETRY_DIR, f"{ts:%Y-%m-%d_%H%M%S}_Stream.csv"
        )
        self._header = False
        log(f"CSV → {os.path.basename(self._path)}")

    def write(self, s: dict) -> None:
        if self._path is None:
            return  # no active session yet
        try:
            if not self._header:
                with open(self._path, "w") as f:
                    f.write(CSV_HEADER)
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
        except Exception as exc:
            log(f"CSV write error: {exc}", "error")


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


def _save_val_csv(samples: list[dict], power: float, lpm: float,
                  passed: bool) -> str:
    """Save validation samples to CSV, return filename.

    Naming: ``{YYYY-MM-DD}_{HHMMSS}_Validation_{pwr}pct_{LPM}Lpm_PASS.csv``
            ``{YYYY-MM-DD}_{HHMMSS}_Validation_{pwr}pct_{LPM}Lpm_FAIL.csv``
    """
    now = datetime.now()
    lpm_s = f"{lpm:.0f}" if lpm == int(lpm) else f"{lpm:.1f}"
    result_tag = "PASS" if passed else "FAIL"
    fname = (
        f"{now:%Y-%m-%d_%H%M%S}_Validation_"
        f"{int(power)}pct_{lpm_s}Lpm_{result_tag}.csv"
    )
    fpath = os.path.join(VALIDATION_DIR, fname)
    if samples:
        pd.DataFrame(samples).to_csv(fpath, index=False)
        log(f"Saved {len(samples)} val samples -> {fname}", "seq")
    return fname


def _find_valid_cert(power_pct: float, flow_lpm: float,
                     max_age_h: float = 24.0) -> "str | None":
    """Return path to the most-recent PASS cert for (power_pct, flow_lpm).

    Searches VALIDATION_DIR for files matching the naming convention::

        {YYYY-MM-DD}_{HHMMSS}_Validation_{pwr}pct_{LPM}Lpm_PASS.csv

    Returns the path of the newest matching file that is younger than
    *max_age_h* hours, or ``None`` if no such file exists.
    """
    lpm_s = f"{flow_lpm:.0f}" if flow_lpm == int(flow_lpm) else f"{flow_lpm:.1f}"
    pattern = re.compile(
        rf"^(\d{{4}}-\d{{2}}-\d{{2}}_\d{{6}})_Validation_"
        rf"{int(power_pct)}pct_{re.escape(lpm_s)}Lpm_PASS\.csv$"
    )
    cutoff = datetime.now().timestamp() - max_age_h * 3600
    best: tuple[float, str] | None = None
    try:
        entries = os.listdir(VALIDATION_DIR)
    except FileNotFoundError:
        return None
    for name in entries:
        m = pattern.match(name)
        if not m:
            continue
        try:
            dt = datetime.strptime(m.group(1), "%Y-%m-%d_%H%M%S")
        except ValueError:
            continue
        ts = dt.timestamp()
        if ts >= cutoff:
            if best is None or ts > best[0]:
                best = (ts, os.path.join(VALIDATION_DIR, name))
    return best[1] if best else None


# =============================================================================
# Recipe generation  (PC sends step lists for recipe-protocol sequences)
# =============================================================================
# NOTE: Calibration no longer uses the recipe protocol — ESP32 has the sweep
# pattern on-board via CMD,calibrate,flow=<lpm>.  generate_cal_recipe() removed.


# generate_val_recipe() removed — ESP32 now owns the validation sweep
# pattern on-board via CMD,validate,power=<pct>,flow=<lpm>.


def _analyze_validation(samples: list[dict], power_pct: float,
                        flow_lpm: float) -> dict:
    """Analyze validation samples and determine pass/fail.

    Each non-baseline phase discards the first VAL_TRANSIENT_SKIP samples
    to account for motor pot settling + gas transit to the 106-H sensor.
    Statistics are computed only on the remaining stable-state tail.

    Criteria:
    - Baseline: mean O3 < 0.02 %vol (no skip — trivially stable at 0%)
    - Spot correlation: stable mean within 0.15 %vol or 15% relative of model
    - Target accuracy: stable mean within 10% relative of prediction
    - Target stability: CV of stable window < 5%
    """
    skip = VAL_TRANSIENT_SKIP
    result: dict[str, Any] = {
        "power": power_pct,
        "flow": flow_lpm,
        "total_samples": len(samples),
        "transient_skip": skip,
    }

    # Group by phase
    by_phase: dict[str, list[dict]] = {}
    for s in samples:
        phase = s.get("phase", "unknown")
        by_phase.setdefault(phase, []).append(s)

    # Baseline check (no transient skip — stable at 0%)
    baseline = by_phase.get("baseline", [])
    if baseline:
        bl_mean = float(np.mean([s["o3_pct"] for s in baseline]))
        result["baseline_mean"] = bl_mean
        result["baseline_ok"] = bl_mean <= 0.02
    else:
        result["baseline_mean"] = 0.0
        result["baseline_ok"] = True

    # Spot correlation checks (use only stable tail)
    spot_checks = []
    for phase_name in ("spot_low", "spot_high"):
        phase_data = by_phase.get(phase_name, [])
        if not phase_data:
            continue
        stable = phase_data[skip:]
        spot_power = phase_data[0].get("power_target", 0)
        expected = predict_o3_from_power(spot_power, flow_lpm)
        if len(stable) < VAL_MIN_STABLE:
            # Not enough stable data — inconclusive, don't gate pass/fail
            spot_checks.append({
                "phase": phase_name, "mean_o3": 0.0,
                "expected_o3": expected, "abs_err": 0.0,
                "rel_err": 0.0, "ok": None,
                "stable_samples": len(stable),
                "skip_reason": "insufficient_stable_data",
            })
            continue
        spot_mean = float(np.mean([s["o3_pct"] for s in stable]))
        abs_err = abs(spot_mean - expected)
        rel_err = (abs_err / expected * 100) if expected > 0 else 0
        spot_ok = abs_err < 0.15 or rel_err < 15
        spot_checks.append({
            "phase": phase_name, "mean_o3": spot_mean,
            "expected_o3": expected, "abs_err": abs_err,
            "rel_err": rel_err, "ok": spot_ok,
            "stable_samples": len(stable),
        })
    result["spot_checks"] = spot_checks
    # Inconclusive spots (ok=None) don't gate pass/fail
    result["spots_ok"] = all(
        sc["ok"] for sc in spot_checks if sc["ok"] is not None
    )

    # Target analysis (use only stable tail)
    target = by_phase.get("target", [])
    target_stable = target[skip:]
    if len(target_stable) >= VAL_MIN_STABLE:
        t_o3 = [s["o3_pct"] for s in target_stable]
        mean_o3 = float(np.mean(t_o3))
        std_o3 = float(np.std(t_o3))
        mean_temp = float(np.mean([s["temp_c"] for s in target_stable]))
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
            "stable_samples": len(target_stable),
            "target_ok": deviation_pct < 10,
            "stable": cv < 5.0,
        })
    else:
        result.update({
            "mean_o3": 0.0, "std_o3": 0.0, "expected_o3": 0.0,
            "deviation_pct": 0.0, "cv_pct": 0.0, "mean_temp": 0.0,
            "target_samples": len(target), "stable_samples": 0,
            "target_ok": False, "stable": False,
        })

    # Overall pass/fail (baseline is advisory only — shown as warning dialog)
    result["passed"] = (
        result.get("spots_ok", True)
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
        # Per-command response matching: {cmd_name: (Event, response_str)}
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
        S.time_synced = False          # force re-sync on next connection

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

        # Start a new CSV file for this connection session
        csv_logger.reset(datetime.now())

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
                    await self._dispatch(line)
        except Exception as exc:
            log(f"TCP read error: {exc}", "error")
        finally:
            # Only close if this coroutine's writer is still the active one.
            # A newer _on_connect may have already replaced self._writer;
            # closing it would kill the *new* connection (reconnect race).
            if self._writer is writer:
                log("ESP32 disconnected", "warn")
                await self._close_client()
                # Abort sequence observer state on disconnect
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

    # -- dispatch (routes all incoming lines) ------------------------------
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
                        # Historical data — write to CSV + graph buffer, but
                        # do NOT update live sensor displays.
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

    # -- Backfill handlers -------------------------------------------------
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

    # -- RSP handler -------------------------------------------------------
    def _handle_rsp(self, line: str) -> None:
        # Self-handled RSPs: parse inline, mark matched to suppress warning
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
            matched = True  # time_sync sent via _send_raw, no waiter
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
        # Route RSP to the matching send_command waiter by command name
        # RSP format: RSP,OK|ERR,<cmd_name>,<response_data>
        # Note: _pending_responses access is safe without lock here because
        # _handle_rsp runs synchronously in the asyncio event loop (single-
        # threaded), and send_command only modifies the dict at await points.
        parts = line.split(",")
        rsp_cmd = parts[2].strip() if len(parts) >= 3 else ""
        if rsp_cmd and rsp_cmd in self._pending_responses:
            event, result_box = self._pending_responses[rsp_cmd]
            result_box.append(line)
            event.set()
            matched = True
        if not matched:
            log(f"Unmatched RSP (no waiter for '{rsp_cmd}'): {line[:60]}", "warn")

    # -- STATE push (on connect/reconnect) --------------------------------
    # IMPORTANT: PC is sole authority for power_target_pct — never
    # overwrite it from ESP32.  The STATE "power" field is the ADC-read
    # motor position (o3_power_get()), NOT the commanded target.  Writing
    # it to power_target_pct causes the tick's slider sync to schedule an
    # async _on_power_slide callback (NiceGUI schedules async handlers
    # via background_tasks.create, so they run AFTER _updating is cleared),
    # which sends cmd_set_power(0) and drives the motor back to zero.
    # See also: decisions_log.md 2026-03-09 "Motor-to-0 root cause".
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
                    # Informational only — do NOT set power_target_pct
                    S.power_actual_pct = float(v)
                elif k == "flow":
                    S.flow_lpm = float(v)
            except (ValueError, IndexError):
                pass
        log("STATE sync from ESP32", "state")

    # -- DIAG message handler (firmware diagnostics) -----------------------
    def _handle_diag(self, line: str) -> None:
        """Handle DIAG,<event>,key=val,... messages from firmware validator."""
        log(f"DIAG: {line}", "warn")
        # Show toast so operator sees it immediately
        _notify(f"FW DIAG: {line}", "warning")
        # Append to CSTR debug log if a sequence is running
        if _cstr_debug_file:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            try:
                with open(_cstr_debug_file, "a") as _f:
                    _f.write(f"[{ts}] {line}\n")
            except OSError:
                pass

    # -- SEQ message handler (generic recipe protocol) ---------------------
    async def _handle_seq(self, line: str) -> None:
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
            # Show "saving" phase while we save/analyze
            S.seq_phase = "saving"
            # Post-sequence analysis — wrapped in try/finally to
            # guarantee S.sequence_active is cleared even on errors.
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
                elif seq_type == "validate":
                    # Analyze first so we know pass/fail for the filename
                    S.val_result = _analyze_validation(
                        S.val_samples, S.val_power, S.val_lpm
                    )
                    S.val_file = _save_val_csv(
                        S.val_samples, S.val_power, S.val_lpm,
                        passed=S.val_result.get("passed", False),
                    )
                    dev = S.val_result.get("deviation_pct", 0.0)
                    n = len(S.val_samples)
                    if S.val_result.get("passed"):
                        _notify(
                            f"Validation PASSED — {dev:.1f}% deviation"
                            f" ({n} samples saved)",
                            "positive",
                        )
                    elif dev < 20:
                        _notify(
                            f"Validation marginal — {dev:.1f}% deviation"
                            f" ({n} samples saved)",
                            "warning",
                        )
                    else:
                        _notify(
                            f"Validation FAILED — {dev:.1f}% deviation"
                            f" ({n} samples saved)",
                            "negative",
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
                # ALWAYS release UI. Relay/power cleanup is deferred to
                # _tick (next 1s cycle) because sending commands from
                # inside _handle_seq deadlocks the TCP read loop.
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
            # SEQ,calibrate,ABORTED,user_request
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
        """Send ``CMD,<cmd>\\n`` and wait for the matching RSP by command name.

        Uses per-command Event+result so concurrent callers don't steal
        each other's responses (fixes the relay_get vs calibrate race).
        """
        if not S.connected:
            return None
        # Extract command name (first token before any comma args)
        cmd_name = cmd.split(",", 1)[0]
        # Set up a per-command waiter
        event = asyncio.Event()
        result_box: list[str] = []
        async with self._pending_lock:
            # If there's already a pending waiter for this command, clear it
            if cmd_name in self._pending_responses:
                old_event, _ = self._pending_responses[cmd_name]
                old_event.set()  # unblock any stale waiter
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


tcp: TCPServer  # assigned in startup


# =============================================================================
# Command helpers  (all async; COMMA-separated — never colons)
# =============================================================================
async def cmd_set_power(pct: int, timeout: float = 2.0) -> bool:
    old = S.power_target_pct
    S.power_target_pct = int(pct)
    S.update_derived()
    resp = await tcp.send_command(f"power_set,{int(pct)}", timeout=timeout)
    ok = resp is not None and "OK" in resp
    if not ok:
        # Roll back — ESP32 never ACK'd, keep UI consistent
        S.power_target_pct = old
        S.update_derived()
    return ok


async def cmd_set_relay(name: str, on: bool) -> bool:
    resp = await tcp.send_command(f"relay_set,{name},{1 if on else 0}")
    if resp and "OK" in resp:
        if name == "ozone_gen":
            S.relay_o3_gen = on
        elif name == "o2_conc":
            S.relay_o2_conc = on
        elif name == "air_comp":
            S.relay_air_comp = on
            # Air comp changes effective O2% → may need different model
            S.load_model_for_current_condition()
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
    await cmd_set_relay("air_comp", False)
    await cmd_set_relay("ozone_gen", False)
    await cmd_set_relay("o2_conc", False)


async def cmd_sequence_start(seq_type: str, **kwargs) -> bool:
    """Start a sequence on the ESP32.

    For calibrate: single CMD,calibrate,flow=<lpm> — ESP32 has the sweep
    pattern on-board (baseline + sweep up 0→100% + sweep down 100→0%).

    For validate: recipe protocol (sequence_start → seq_step × N → seq_run).
    """
    if not S.connected:
        _notify("Not connected to ESP32", "negative")
        return False

    if seq_type == "calibrate":
        return await _start_calibration(**kwargs)
    elif seq_type == "validate":
        return await _start_validation(**kwargs)
    elif seq_type == "cstr_cal":
        # PC-driven — launch as background task so UI remains responsive
        asyncio.create_task(_start_fill_evac(**kwargs))
        return True
    else:
        log(f"Unknown sequence type: {seq_type}", "error")
        return False


async def _start_calibration(**kwargs) -> bool:
    """Single-command calibration: CMD,calibrate,flow=<lpm>[,air_comp=1][,random=p1,p2,...].

    ESP32 builds the sweep internally and appends random hold steps if provided.
    Relay prereqs (O2 ON if flow>0, O3 ON, Air per toggle) handled by executor.
    """
    import random as _rng

    flow = kwargs.get("flow", DEFAULT_FLOW_LPM)
    num_random = int(kwargs.get("num_random", 0))
    air_comp = bool(kwargs.get("air_comp", False))

    # Pre-flight: ensure power starts at 0%
    log("Pre-flight: setting power to 0%", "seq")
    if not await cmd_set_power(0):
        _notify("Calibration aborted — failed to set power to 0%", "negative")
        return False

    # Generate stratified random power levels (ascending then descending)
    random_powers: list[int] = []
    if num_random > 0:
        # Divide [0, 100] into N windows, sample one from each
        window = 100.0 / num_random
        levels = sorted(set(
            max(0, min(100, int(_rng.uniform(i * window, (i + 1) * window))))
            for i in range(num_random)
        ))
        # Ascending then descending (mountain shape, minimises pot travel)
        random_powers = levels + levels[::-1]

    # Step count: 1 baseline + 101 up + 101 down + len(random_powers) random
    total_steps = 203 + len(random_powers)

    # Prepare observer state (but do NOT set sequence_active yet)
    S.seq_type = "calibrate"
    S.seq_phase = "starting"
    S.seq_progress = 0.0
    S.seq_elapsed = 0.0
    S.seq_start_time = 0.0
    S.seq_confirmed = False
    S.seq_step_idx = 0
    S.seq_step_total = total_steps
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    S.cal_samples = []
    S.cal_file = ""
    S.cal_lpm = flow

    # Build command
    cmd = f"calibrate,flow={flow}"
    if air_comp:
        cmd += ",air_comp=1"
    else:
        cmd += ",air_comp=0"
    if random_powers:
        pwr_str = ",".join(str(p) for p in random_powers)
        cmd += f",random={pwr_str}"

    resp = await tcp.send_command(cmd, timeout=5.0)
    if resp and "OK" in resp:
        S.sequence_active = True
        S.seq_confirmed = True
        S.seq_start_time = time.time()
        log(f"Calibration started: flow={flow} LPM, air={air_comp}, "
            f"{num_random} rnd levels, {total_steps} steps", "seq")
        _notify("Calibration started", "positive")
        return True

    log(f"Calibration failed: {resp}", "error")
    _notify(f"Calibration failed to start: {resp}", "negative")
    return False


async def _start_validation(**kwargs) -> bool:
    """Single-command validation: CMD,validate,power=<pct>,flow=<lpm>.

    ESP32 owns the full sweep pattern on-board.
    """
    power = int(kwargs.get("power", 75))
    flow = kwargs.get("flow", DEFAULT_FLOW_LPM)
    S.val_power = power
    S.val_lpm = flow

    # Pre-flight: ensure power starts at 0%
    log("Pre-flight: setting power to 0%", "seq")
    if not await cmd_set_power(0):
        _notify("Validation aborted — failed to set power to 0%", "negative")
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

    # Pre-flight: warn if baseline O3 reading is elevated
    if S.vessel_o3_pct > 0.01:
        proceed = asyncio.Event()
        cancelled = [False]

        with ui.dialog().props("persistent") as bl_dlg:
            with ui.card().classes("q-pa-lg").style("min-width: 420px"):
                ui.icon("warning").classes("text-h3 text-amber q-mb-sm")
                ui.label("Baseline may not be stable").classes(
                    "text-h5 q-mb-sm"
                )
                ui.label(
                    f"Current O\u2083 reading: {S.vessel_o3_pct:.3f} %vol. "
                    "The sensor may not have fully zeroed. "
                    "Proceeding may affect data accuracy."
                ).classes("text-body1 q-mb-lg")
                with ui.row().classes("justify-end w-full q-gutter-sm"):
                    def _cancel():
                        cancelled[0] = True
                        bl_dlg.close()
                        proceed.set()

                    def _proceed():
                        bl_dlg.close()
                        proceed.set()

                    ui.button("Cancel", color="red",
                              on_click=_cancel).props("flat")
                    ui.button("Proceed anyway", color="green",
                              on_click=_proceed).props("unelevated")
        bl_dlg.open()
        await proceed.wait()
        if cancelled[0]:
            log("Validation cancelled — elevated baseline", "seq")
            _notify("Validation cancelled by user", "warning")
            return False

    # Prepare observer state (do NOT set sequence_active yet)
    S.seq_type = "validate"
    S.seq_phase = "starting"
    S.seq_progress = 0.0
    S.seq_elapsed = 0.0
    S.seq_start_time = 0.0
    S.seq_confirmed = False
    S.seq_step_idx = 0
    S.seq_step_total = 5
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    S.val_samples = []
    S.val_result = {}
    S.val_file = ""

    # Single command — ESP32 builds recipe internally
    resp = await tcp.send_command(
        f"validate,power={power},flow={flow}", timeout=5.0
    )
    if resp and "OK" in resp:
        S.sequence_active = True
        S.seq_confirmed = True
        S.seq_start_time = time.time()
        log(f"Validation running: power={power}%, flow={flow} LPM", "seq")
        _notify("Validation started", "positive")
        return True

    log(f"validate failed: {resp}", "error")
    _notify(f"Validation failed to start: {resp}", "negative")
    return False


async def cmd_sequence_abort(reason: str = "") -> bool:
    """Send CMD,sequence_abort[,reason]."""
    cmd = "sequence_abort"
    if reason:
        cmd += f",{reason}"
    resp = await tcp.send_command(cmd)
    return resp is not None and "OK" in resp


async def cmd_sequence_stop() -> bool:
    """Abort sequence + full cleanup (power=0, all relays off)."""
    result = await cmd_sequence_abort()
    await _sequence_cleanup("stop")
    S.seq_phase = "aborted"
    S.sequence_active = False
    return result


async def _safe_standby() -> None:
    """Unconditionally set power=0% and all relays off (safe state).

    Called at end of any sequence or whenever the system must return to
    a known-safe idle configuration.  No guards — always executes.
    """
    log("Safe standby: power=0, all relays off", "seq")
    await cmd_set_power(0)
    await cmd_set_relay("ozone_gen", False)
    await cmd_set_relay("o2_conc", False)
    await cmd_set_relay("air_comp", False)


async def _sequence_cleanup(source: str = "unknown") -> None:
    """End-of-sequence cleanup — always returns to safe standby."""
    log(f"Sequence cleanup ({source})", "seq")
    S.seq_confirmed = False
    await _safe_standby()


async def cmd_sequence_confirm() -> bool:
    """Send CMD,sequence_confirm (no args — unblocks pending prompt)."""
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    resp = await tcp.send_command("sequence_confirm")
    return resp is not None and "OK" in resp


# =============================================================================
# Fill / Evacuation calibration sequence  (PC-driven)
# =============================================================================
# Validation transient skip — leading samples discarded per phase to account
# for motor pot settling + gas transit to 106-H sensor (~17.5 s at ~2.5 s/sample).
VAL_TRANSIENT_SKIP = 7
VAL_MIN_STABLE = 3          # Minimum stable samples required for valid statistics

# Constants for fill/evac steady-state detection
FILL_STEADY_COUNT = 45      # Samples in stability window (~112 s at 2.5 s/sample)
FILL_STEADY_RANGE = 0.08    # Max range (%vol) across steady window (sanity bound)
# Slope threshold derived from measured evac asymptotic behaviour (2026-03-10
# CSTR run).  Evac 45-sample windows show |slope| ≈ 0.0003 at convergence
# (O3 0.02→0.01%), 6× below the fill's premature-stop slope of 0.00184.
FILL_STEADY_SLOPE = 0.0003  # Max abs linear slope (%vol/sample) — from evac data
EVAC_THRESHOLD_PCT = 0.01   # O3 %vol below which vessel is considered evacuated
EVAC_STEADY_COUNT = 5       # Consecutive samples below threshold to declare evacuated
FILL_BASELINE_SAMPLES = 15  # Number of baseline samples at 0% power
FILL_SAMPLE_INTERVAL = 2.5  # Approximate seconds between DATA samples
FILL_MAX_SAMPLES = 600      # Hard limit: abort fill if steady-state not reached in N samples (~25 min)
EVAC_MAX_SAMPLES = 400      # Hard limit: abort evac if not cleared in N samples (~17 min)
CSTR_STALE_THRESHOLD = 8.0  # Seconds: warn if telemetry hasn't updated since last sample
CSTR_CMD_RETRIES = 3        # Retries for relay_set commands before aborting
CSTR_CMD_RETRY_DELAY = 1.5  # Seconds between relay command retries
CSTR_CHECKPOINT_INTERVAL = 10  # Write intermediate CSV every N samples during fill/evac
# power_set timeout: motor pot physically ramps to target — can take 10-20 s at 100%.
# Using a single long-timeout attempt avoids sending multiple conflicting CMD,power_set
# commands to the ESP32 while it is still executing the first one.
CSTR_POWER_SET_TIMEOUT = 25.0   # Seconds to wait for power_set RSP before fallback
CSTR_POWER_TOLERANCE = 5.0      # % — fallback: accept if actual power within this of target
FILL_POWER_MIN_PCT = 80.0       # Minimum pwr_act at fill steady-state; lower means power was cut mid-fill
FILL_POWER_GRACE_SAMPLES = 5    # Skip power watchdog for first N fill samples (motor settling)
FILL_POWER_RESEND_MAX = 3       # Max power re-sends before aborting fill


def _make_cstr_csv_path(lpm: float) -> str:
    """Generate CSTR calibration CSV filename.

    Format: ``{YYYY-MM-DD}_{HHMMSS}_CSTR_{LPM}Lpm.csv``
    Saved to ``Data/CSTR/``.
    """
    now = datetime.now()
    lpm_s = f"{lpm:.0f}" if lpm == int(lpm) else f"{lpm:.1f}"
    fname = f"{now:%Y-%m-%d_%H%M%S}_CSTR_{lpm_s}Lpm.csv"
    return os.path.join(CSTR_DATA_DIR, fname)


def _write_cstr_csv(path: str, samples: list[dict]) -> None:
    """Write combined CSTR calibration samples (baseline+fill+evac) to CSV."""
    if not samples:
        return
    header = "timestamp,esp_ts_ms,elapsed_s,vessel_o3_pct,cell_temp_c,vessel_temp_c,power_pct,power_actual_pct,phase\n"
    with open(path, "w") as f:
        f.write(header)
        t0 = samples[0].get("esp_ts_ms", 0)
        for s in samples:
            elapsed = (s.get("esp_ts_ms", 0) - t0) / 1000.0
            f.write(
                f"{s['timestamp']},{s['esp_ts_ms']},{elapsed:.2f},"
                f"{s['vessel_o3_pct']},{s['cell_temp_c']},"
                f"{s.get('vessel_temp_c', -999)},"
                f"{s.get('power_pct', 0)},{s.get('power_actual_pct', 0.0)},"
                f"{s.get('phase', '')}\n"
            )
    log(f"Saved {len(samples)} samples -> {os.path.basename(path)}", "seq")


# ---------------------------------------------------------------------------
# CSTR sequence: file-based debug logger
# ---------------------------------------------------------------------------
_cstr_debug_file: Optional[str] = None   # set at sequence start


def _cstr_flog(msg: str) -> None:
    """Write a timestamped line to the CSTR debug log file AND in-memory log.

    The file persists across restarts — essential for post-mortem debugging
    when the app crashes or the connection drops mid-sequence.
    """
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log(msg, "seq")
    if _cstr_debug_file:
        try:
            with open(_cstr_debug_file, "a") as _f:
                _f.write(f"[{ts}] {msg}\n")
        except OSError:
            pass  # don't cascade file errors during an already-failing sequence


def _cstr_flog_state(label: str) -> None:
    """Snapshot key SystemState fields into the debug log — call at each phase boundary."""
    _cstr_flog(
        f"STATE@{label}: connected={S.connected} "
        f"O3={S.vessel_o3_pct:.4f}% pwr_tgt={S.power_target_pct}% "
        f"pwr_act={S.power_actual_pct:.1f}% "
        f"last_update={S.last_update.strftime('%H:%M:%S') if S.last_update else 'None'} "
        f"esp_ts={S.last_esp_ts_ms} "
        f"relay(o3={S.relay_o3_gen} o2={S.relay_o2_conc} air={S.relay_air_comp}) "
        f"seq_cleanup_pending={S.seq_cleanup_pending}"
    )


async def _cstr_set_power(pct: int) -> None:
    """Send power_set with a long timeout; raises RuntimeError if unconfirmed.

    The ESP32 motor pot takes up to ~20 s to physically ramp to a new level
    before it sends RSP,power_set,OK.  Using the default 2 s timeout causes
    spurious failures even though the command was received and is executing.

    Strategy:
      1. Single attempt with CSTR_POWER_SET_TIMEOUT (25 s).
      2. If RSP still doesn't arrive within that window, check telemetry:
         if pwr_act is within CSTR_POWER_TOLERANCE of the target the ESP32
         clearly executed the command — accept with a warning rather than
         aborting the entire calibration run.
      3. Only raise RuntimeError if both the RSP and telemetry checks fail.

    Never retry: sending multiple CMD,power_set while the motor is still
    ramping creates conflicting in-flight commands and corrupts the ESP32
    command queue.
    """
    _cstr_flog(
        f"cmd_set_power({pct}%) — single attempt, timeout={CSTR_POWER_SET_TIMEOUT}s"
    )
    ok = await cmd_set_power(pct, timeout=CSTR_POWER_SET_TIMEOUT)
    if ok:
        _cstr_flog(f"cmd_set_power({pct}%) -> RSP OK (pwr_act={S.power_actual_pct:.1f}%)")
        return

    # RSP timed out — check telemetry as fallback
    _cstr_flog(
        f"cmd_set_power({pct}%) RSP timeout/failed "
        f"(connected={S.connected}, pwr_act={S.power_actual_pct:.1f}%)"
    )
    if abs(S.power_actual_pct - pct) <= CSTR_POWER_TOLERANCE:
        _cstr_flog(
            f"WARN cmd_set_power({pct}%) RSP not received but "
            f"pwr_act={S.power_actual_pct:.1f}% is within {CSTR_POWER_TOLERANCE}% "
            f"of target — treating as success (telemetry fallback)"
        )
        # Force state consistent (cmd_set_power rolled back on failure)
        S.power_target_pct = int(pct)
        S.update_derived()
        return

    raise RuntimeError(
        f"Failed to set power to {pct}%: "
        f"RSP timed out and pwr_act={S.power_actual_pct:.1f}% is not near target"
    )


async def _cstr_set_relay(name: str, on: bool) -> None:
    """Set relay with retries; raises RuntimeError if all retries exhausted."""
    for attempt in range(1, CSTR_CMD_RETRIES + 1):
        _cstr_flog(f"cmd_set_relay({name},{on}) attempt {attempt}/{CSTR_CMD_RETRIES}")
        ok = await cmd_set_relay(name, on)
        if ok:
            _cstr_flog(f"cmd_set_relay({name},{on}) -> OK")
            return
        _cstr_flog(f"cmd_set_relay({name},{on}) attempt {attempt} FAILED")
        if attempt < CSTR_CMD_RETRIES:
            await asyncio.sleep(CSTR_CMD_RETRY_DELAY)
    raise RuntimeError(
        f"Failed to set relay {name}={on} after {CSTR_CMD_RETRIES} attempts"
    )


async def _start_fill_evac(**kwargs) -> bool:
    """PC-driven fill/evacuation calibration sequence.

    Unlike calibration/validation (ESP32-driven), this sequence is orchestrated
    entirely by the PC.  The PC sends power_set/relay_set commands and monitors
    incoming DATA telemetry for steady-state detection.

    Calibration is performed WITHOUT air compressor for maximum k_d
    sensitivity.  The fitted parameters (V, k_d, V_dead) generalise to
    any flow rate and air compressor configuration.

    Phases:
      1. Pre-checks: validated model must exist, connection active
      2. Relay setup: O3 gen ON, O2 conc ON (if flow>0), air comp OFF
      3. Baseline: 15 samples at 0% power — establishes zero reference
      4. Fill: 100% power, monitor until 30 consecutive samples stable
      5. Transition: prepare evac
      6. Evac: 0% power, monitor until 5 consecutive samples < 0.01% vol
      7. Cleanup: save combined CSV, safe standby, offer model fitting

    Debugging: a persistent log file is written to Data/CSTR/ alongside the
    CSV so post-mortem analysis is possible even after an app restart.
    """
    global _cstr_debug_file

    if not S.connected:
        _notify("Not connected to ESP32", "negative")
        return False

    if S.sequence_active or S.fill_active:
        _notify("Another sequence is already running", "negative")
        return False

    flow = kwargs.get("flow", DEFAULT_FLOW_LPM)

    # Determine target O3 from the most recent 100% validation PASS cert
    # (measured, held at steady-state, direct-to-sensor — no model needed).
    # Fall back to sigmoid prediction only if no validation data exists.
    target_o3 = 0.0
    val_cert = _find_valid_cert(100, flow, max_age_h=720)  # ~30 days
    if val_cert:
        try:
            vdf = pd.read_csv(val_cert)
            vdf.columns = [c.strip() for c in vdf.columns]
            vt = vdf[vdf["phase"] == "target"].iloc[VAL_TRANSIENT_SKIP:]
            if len(vt) >= VAL_MIN_STABLE:
                target_o3 = float(vt["o3_pct"].mean())
                log(f"CSTR fill target from validation: {target_o3:.4f}% "
                    f"({os.path.basename(val_cert)})", "info")
        except Exception as exc:
            log(f"Failed to read validation cert: {exc}", "warning")
    if target_o3 <= 0.01:
        target_o3 = predict_o3_from_power(100, flow)
        log(f"CSTR fill target from sigmoid model: {target_o3:.4f}% "
            f"(no validation cert found)", "info")
    if target_o3 <= 0.01:
        _notify(
            "Cannot determine fill target — run a validation first",
            "negative",
        )
        return False

    # -- Initialize state --------------------------------------------------
    S.fill_active = True
    S.fill_phase = "setup"
    S.cstr_samples = []
    S.cstr_csv_path = ""
    S.fill_target_o3 = target_o3
    S.fill_lpm = flow
    S.sequence_active = True
    S.seq_type = "cstr_cal"
    S.seq_phase = "setup"
    S.seq_progress = 0.0
    S.seq_start_time = time.time()
    # Cancel any deferred cleanup pending from a prior ESP32-driven sequence
    # (calibrate/validate).  If seq_cleanup_pending is True when the first
    # _tick fires, _sequence_cleanup() → _safe_standby() would cut power and
    # turn off all relays mid-fill — the root cause of the spurious power-cut bug.
    S.seq_cleanup_pending = False

    # Open persistent debug log for this run
    now_tag = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    _cstr_debug_file = os.path.join(
        CSTR_DATA_DIR, f"{now_tag}_CSTR_debug.log"
    )

    _cstr_flog(
        f"CSTR calibration starting: flow={flow} LPM, "
        f"target C_in={target_o3:.3f} %vol (no air comp)"
    )
    _cstr_flog_state("start")

    # Helper: collect one telemetry snapshot with freshness check
    def _snap(phase: str) -> dict:
        now = datetime.now()
        staleness = (now - S.last_update).total_seconds() if S.last_update else 999.0
        if staleness > CSTR_STALE_THRESHOLD:
            _cstr_flog(
                f"WARN stale telemetry in {phase}: last_update "
                f"{staleness:.1f}s ago (>{CSTR_STALE_THRESHOLD}s) — "
                f"O3={S.vessel_o3_pct:.4f}% may be stale"
            )
        return {
            "timestamp": now,
            "esp_ts_ms": S.last_esp_ts_ms,
            "vessel_o3_pct": S.vessel_o3_pct,
            "cell_temp_c": S.cell_temp_c,
            "vessel_temp_c": S.vessel_temp_c,
            "power_pct": S.power_target_pct,
            "power_actual_pct": S.power_actual_pct,
            "phase": phase,
            "_staleness_s": round(staleness, 2),
        }

    # Helper: write intermediate checkpoint CSV (preserves data on mid-run abort)
    def _checkpoint(label: str) -> None:
        if not S.cstr_samples:
            return
        try:
            ckpt_path = os.path.join(
                CSTR_DATA_DIR,
                f"{now_tag}_CSTR_checkpoint_{label}.csv",
            )
            _write_cstr_csv(ckpt_path, S.cstr_samples)
            _cstr_flog(
                f"Checkpoint saved: {os.path.basename(ckpt_path)} "
                f"({len(S.cstr_samples)} samples)"
            )
        except OSError as ckpt_err:
            _cstr_flog(f"WARN checkpoint write failed: {ckpt_err}")

    try:
        # -- Phase 1: Relay setup ------------------------------------------
        S.fill_phase = "relay_setup"
        S.seq_phase = "relay_setup"
        _cstr_flog("Phase 1: relay setup")

        # Power to 0% first (safe starting point)
        await _cstr_set_power(0)

        # Enable O3 generator (required — fail hard if relay doesn't respond)
        if not S.relay_o3_gen:
            await _cstr_set_relay("ozone_gen", True)
        else:
            _cstr_flog("ozone_gen already ON — skipping relay_set")
        # Enable O2 concentrator if flow > 0
        if flow > 0 and not S.relay_o2_conc:
            await _cstr_set_relay("o2_conc", True)
        elif flow > 0:
            _cstr_flog("o2_conc already ON — skipping relay_set")
        # Air compressor OFF for calibration (best k_d sensitivity)
        if S.relay_air_comp:
            await _cstr_set_relay("air_comp", False)

        # Allow relays to stabilize
        _cstr_flog("Relay setup complete — waiting 3 s for stabilization")
        await asyncio.sleep(3.0)
        _cstr_flog_state("post_relay_setup")

        # -- Phase 2: Baseline (15 samples at 0% power) -------------------
        S.fill_phase = "baseline"
        S.seq_phase = "baseline"
        _cstr_flog(
            f"Phase 2: baseline — {FILL_BASELINE_SAMPLES} samples at 0% power"
        )

        baseline_collected = 0

        while baseline_collected < FILL_BASELINE_SAMPLES:
            if not S.fill_active or not S.connected:
                raise RuntimeError(
                    f"CSTR sequence aborted or disconnected "
                    f"(fill_active={S.fill_active} connected={S.connected})"
                )

            await asyncio.sleep(FILL_SAMPLE_INTERVAL)

            sample = _snap("baseline")
            S.cstr_samples.append(sample)
            baseline_collected += 1
            S.seq_progress = (baseline_collected / FILL_BASELINE_SAMPLES) * 10
            _cstr_flog(
                f"Baseline {baseline_collected}/{FILL_BASELINE_SAMPLES}: "
                f"O3={sample['vessel_o3_pct']:.4f}% "
                f"esp_ts={sample['esp_ts_ms']} stale={sample['_staleness_s']}s"
            )

        _cstr_flog_state("baseline_complete")
        _checkpoint("baseline")

        # -- Phase 3: Fill (100% power, wait for steady-state) -------------
        S.fill_phase = "fill"
        S.seq_phase = "fill"
        _cstr_flog(f"Phase 3: fill — ramping to 100%, target={target_o3:.3f}% vol")
        _notify("Fill phase — ramping to 100% power", "info")

        await _cstr_set_power(100)
        _cstr_flog_state("fill_power_set")

        fill_sample_idx = 0
        power_resend_count = 0
        # Ring buffer of recent O3 readings for range-based steady-state check
        recent_o3: list[float] = []

        while True:
            if not S.fill_active or not S.connected:
                raise RuntimeError(
                    f"CSTR sequence aborted or disconnected during fill "
                    f"(fill_active={S.fill_active} connected={S.connected})"
                )

            await asyncio.sleep(FILL_SAMPLE_INTERVAL)

            sample = _snap("fill")
            S.cstr_samples.append(sample)
            fill_sample_idx += 1

            # -- Power watchdog: detect motor pot drift and re-send --------
            if fill_sample_idx > FILL_POWER_GRACE_SAMPLES:
                if S.power_actual_pct < FILL_POWER_MIN_PCT:
                    power_resend_count += 1
                    if power_resend_count > FILL_POWER_RESEND_MAX:
                        raise RuntimeError(
                            f"Power dropped below {FILL_POWER_MIN_PCT}% "
                            f"{power_resend_count} times during fill — motor pot "
                            f"cannot hold position. pwr_act={S.power_actual_pct:.1f}%"
                        )
                    _cstr_flog(
                        f"WARN power drift: pwr_act={S.power_actual_pct:.1f}% "
                        f"< {FILL_POWER_MIN_PCT}% at fill #{fill_sample_idx} — "
                        f"re-sending power command "
                        f"(resend {power_resend_count}/{FILL_POWER_RESEND_MAX})"
                    )
                    _notify(
                        f"Power drift detected ({S.power_actual_pct:.0f}%) "
                        f"— re-sending (#{power_resend_count})",
                        "warning",
                    )
                    await _cstr_set_power(100)
                    _cstr_flog_state(f"fill_power_resend_{power_resend_count}")
                    # Reset O3 ring buffer — readings during drift are invalid
                    recent_o3.clear()

            # Track recent readings for stability check
            recent_o3.append(sample["vessel_o3_pct"])
            if len(recent_o3) > FILL_STEADY_COUNT:
                recent_o3.pop(0)

            # Progress bar: map O3 level to % of target
            fill_frac = (
                min(1.0, sample["vessel_o3_pct"] / target_o3) if target_o3 > 0 else 0
            )
            S.seq_progress = 10 + fill_frac * 40  # 10-50% of total

            # Check for steady-state: slope + range over last N samples.
            # The 106-H sensor has 0.01% resolution, so range alone can
            # falsely trigger on a slowly rising signal.  Linear slope
            # catches monotonic trends that range misses.
            rng = (max(recent_o3) - min(recent_o3)) if len(recent_o3) >= 2 else 999.0
            slope = 999.0
            if len(recent_o3) >= FILL_STEADY_COUNT:
                x = np.arange(len(recent_o3), dtype=float)
                y = np.array(recent_o3, dtype=float)
                slope = float(np.polyfit(x, y, 1)[0])  # %vol per sample
            is_stable = (
                len(recent_o3) >= FILL_STEADY_COUNT
                and rng < FILL_STEADY_RANGE
                and abs(slope) < FILL_STEADY_SLOPE
            )

            if fill_sample_idx % 10 == 0 or is_stable:
                _cstr_flog(
                    f"Fill #{fill_sample_idx}: O3={sample['vessel_o3_pct']:.4f}% "
                    f"({fill_frac*100:.1f}% of target) range={rng:.4f}% "
                    f"slope={slope:.6f} "
                    f"window={len(recent_o3)}/{FILL_STEADY_COUNT} "
                    f"stale={sample['_staleness_s']}s "
                    f"pwr_act={S.power_actual_pct:.1f}%"
                )

            # Intermediate checkpoint every N samples
            if fill_sample_idx % CSTR_CHECKPOINT_INTERVAL == 0:
                _checkpoint(f"fill_{fill_sample_idx}")

            if is_stable:
                _cstr_flog(
                    f"Fill steady-state reached: O3={sample['vessel_o3_pct']:.4f}%, "
                    f"range={rng:.4f}%, slope={slope:.6f} "
                    f"over {FILL_STEADY_COUNT} samples"
                )
                # Power sanity check: steady-state at 0% power means the fill ran with
                # no ozone input — data is invalid and continuing to evac would be wrong.
                if S.power_actual_pct < FILL_POWER_MIN_PCT:
                    raise RuntimeError(
                        f"Fill steady-state declared but pwr_act={S.power_actual_pct:.1f}% "
                        f"is below minimum {FILL_POWER_MIN_PCT}% — power was cut during fill; "
                        f"data is invalid. Check for a deferred cleanup race or manual abort."
                    )
                break

            # Hard limit: avoid infinite loop if steady-state never arrives
            if fill_sample_idx >= FILL_MAX_SAMPLES:
                _cstr_flog(
                    f"WARN fill hard limit reached ({FILL_MAX_SAMPLES} samples). "
                    f"Steady-state NOT detected (range={rng:.4f}% > {FILL_STEADY_RANGE}%). "
                    f"Proceeding to evac with available data."
                )
                _notify(
                    f"Fill limit reached ({FILL_MAX_SAMPLES} samples) — "
                    f"steady-state not achieved, proceeding anyway",
                    "warning",
                )
                break

        _cstr_flog_state("fill_complete")
        _checkpoint(f"fill_final_{fill_sample_idx}")

        # -- Phase 4: Transition (prepare evac) ----------------------------
        S.fill_phase = "transition"
        S.seq_phase = "transition"
        S.seq_progress = 50.0

        _notify(
            f"Fill complete — {fill_sample_idx} samples, "
            f"O3={S.vessel_o3_pct:.3f}%",
            "positive",
        )

        # -- Phase 5: Evacuation (0% power, wait for O3 to clear) ----------
        S.fill_phase = "evac"
        S.seq_phase = "evac"
        _cstr_flog("Phase 5: evacuation — setting power to 0%")
        _notify("Evac phase — power off, purging vessel", "info")

        await _cstr_set_power(0)
        _cstr_flog_state("evac_power_set")

        evac_start_o3 = S.vessel_o3_pct
        steady_count = 0
        evac_sample_idx = 0
        _cstr_flog(f"Evac start O3={evac_start_o3:.4f}%, threshold={EVAC_THRESHOLD_PCT}%")

        while True:
            if not S.fill_active or not S.connected:
                raise RuntimeError(
                    f"CSTR sequence aborted or disconnected during evac "
                    f"(fill_active={S.fill_active} connected={S.connected})"
                )

            await asyncio.sleep(FILL_SAMPLE_INTERVAL)

            sample = _snap("evac")
            S.cstr_samples.append(sample)
            evac_sample_idx += 1

            # Progress: map O3 decay to 50-90%
            if evac_start_o3 > 0:
                evac_frac = 1.0 - min(1.0, sample["vessel_o3_pct"] / evac_start_o3)
            else:
                evac_frac = 1.0
            S.seq_progress = 50 + evac_frac * 40  # 50-90% of total

            # Check for evacuation complete
            if sample["vessel_o3_pct"] < EVAC_THRESHOLD_PCT:
                steady_count += 1
            else:
                steady_count = 0

            if evac_sample_idx % 10 == 0 or steady_count > 0:
                _cstr_flog(
                    f"Evac #{evac_sample_idx}: O3={sample['vessel_o3_pct']:.4f}% "
                    f"steady={steady_count}/{EVAC_STEADY_COUNT} "
                    f"stale={sample['_staleness_s']}s"
                )

            # Intermediate checkpoint every N samples
            if evac_sample_idx % CSTR_CHECKPOINT_INTERVAL == 0:
                _checkpoint(f"evac_{evac_sample_idx}")

            if steady_count >= EVAC_STEADY_COUNT:
                _cstr_flog(
                    f"Evacuation complete: O3={sample['vessel_o3_pct']:.4f}% < "
                    f"{EVAC_THRESHOLD_PCT}% for {EVAC_STEADY_COUNT} consecutive"
                )
                break

            # Hard limit: avoid infinite loop if vessel never clears
            if evac_sample_idx >= EVAC_MAX_SAMPLES:
                _cstr_flog(
                    f"WARN evac hard limit reached ({EVAC_MAX_SAMPLES} samples). "
                    f"O3={sample['vessel_o3_pct']:.4f}% still above threshold. "
                    f"Proceeding to save with available data."
                )
                _notify(
                    f"Evac limit reached ({EVAC_MAX_SAMPLES} samples) — "
                    f"vessel not fully cleared, proceeding to save",
                    "warning",
                )
                break

        _cstr_flog_state("evac_complete")

        # -- Phase 6: Save combined CSV and cleanup ------------------------
        S.fill_phase = "saving"
        S.seq_phase = "saving"
        S.seq_progress = 90.0

        csv_path = _make_cstr_csv_path(flow)
        S.cstr_csv_path = csv_path
        _write_cstr_csv(csv_path, S.cstr_samples)
        _cstr_flog(f"Final CSV saved: {os.path.basename(csv_path)}")

        # Cleanup — safe standby
        await _safe_standby()
        S.seq_progress = 100.0
        S.fill_phase = "complete"
        S.seq_phase = "complete"
        S.fill_active = False
        S.sequence_active = False

        n_fill = sum(1 for s in S.cstr_samples if s.get("phase") == "fill")
        n_evac = sum(1 for s in S.cstr_samples if s.get("phase") == "evac")
        _cstr_flog(
            f"CSTR calibration COMPLETE: {len(S.cstr_samples)} total samples "
            f"({FILL_BASELINE_SAMPLES} baseline + {n_fill} fill + {n_evac} evac)"
        )
        _notify(
            f"CSTR calibration complete — {n_fill} fill + {n_evac} evac samples",
            "positive",
        )
        return True

    except RuntimeError as exc:
        tb = traceback.format_exc()
        _cstr_flog(f"ERROR (RuntimeError): {exc}\n{tb}")
        _cstr_flog_state("error")
        _checkpoint("error")
        log(f"CSTR calibration error: {exc}", "error")
        _notify(f"CSTR calibration failed: {exc}", "negative")
        await _safe_standby()
        S.fill_phase = "error"
        S.seq_phase = "error"
        S.fill_active = False
        S.sequence_active = False
        return False

    except Exception as exc:
        tb = traceback.format_exc()
        _cstr_flog(f"ERROR (unexpected): {exc}\n{tb}")
        _cstr_flog_state("error")
        _checkpoint("error")
        log(f"CSTR calibration unexpected error: {exc}", "error")
        _notify(f"CSTR calibration error: {exc}", "negative")
        await _safe_standby()
        S.fill_phase = "error"
        S.seq_phase = "error"
        S.fill_active = False
        S.sequence_active = False
        return False


async def _fit_and_save_cstr_model() -> Optional[CSTRModel]:
    """Fit CSTR model from the most recent calibration CSV and save."""
    if not S.cstr_csv_path:
        _notify("No CSTR calibration CSV to fit", "negative")
        return None

    if not os.path.exists(S.cstr_csv_path):
        _notify("CSTR calibration CSV not found", "negative")
        return None

    try:
        # C_in from power model at 100% power, same flow used in calibration
        c_in = predict_o3_from_power(100, S.fill_lpm)
        model = fit_cstr_model(
            csv_path=S.cstr_csv_path,
            flow_lpm=S.fill_lpm,
            c_in_pct=c_in,
        )
        path = save_cstr_model_json(model, CSTR_MODEL_DIR)
        log(
            f"CSTR model fitted: V={model.system_volume_L:.2f}L, "
            f"k_d={model.decay_rate_per_s:.6f}/s, "
            f"C_ss={model.c_ss_fitted_pct:.3f}% (C_in={c_in:.3f}%), "
            f"R²={model.r_squared_fill:.4f}/{model.r_squared_evac:.4f} "
            f"-> {os.path.basename(path)}",
            "seq",
        )
        _notify(
            f"CSTR model: V={model.system_volume_L:.1f}L, "
            f"k_d={model.decay_rate_per_s:.5f}/s, "
            f"R²={model.r_squared_fill:.3f}/{model.r_squared_evac:.3f}",
            "positive",
        )
        S.load_cstr_model()
        return model
    except Exception as exc:
        log(f"CSTR model fitting failed: {exc}", "error")
        _notify(f"CSTR model fitting failed: {exc}", "negative")
        return None


# =============================================================================
# UI  (built once per browser session)
# =============================================================================
@ui.page("/")
async def index():
    _updating = False

    # -- derived settings sync -------------------------------------------
    def _sync_derived_to_ui() -> None:
        S.update_derived()
        # Use _props + update() to avoid triggering on_change callbacks.
        if inp_o3 is not None:
            inp_o3._props['model-value'] = round(S.target_o3_pct, 2)
            inp_o3.update()
        if inp_mg is not None:
            inp_mg._props['model-value'] = round(S.target_mg_per_s, 2)
            inp_mg.update()
        if inp_g30 is not None:
            inp_g30._props['model-value'] = round(S.target_g_30min, 2)
            inp_g30.update()

    # -- power callbacks --------------------------------------------------
    async def _on_power_slide(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating or S.sequence_active:
            return
        _updating = True
        pct = int(e.value)
        # Guard: NiceGUI schedules async handlers as background tasks,
        # so this can fire AFTER _updating is cleared by the tick.
        # Skip if value already matches target (tick-echo scenario).
        if pct == S.power_target_pct:
            _updating = False
            return
        await cmd_set_power(pct)
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pct
            inp_pwr.update()
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    async def _on_power_input(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating or S.sequence_active:
            return
        if e.value is None:   # field cleared — ignore
            return
        _updating = True
        pct = int(e.value)
        if pct == S.power_target_pct:
            _updating = False
            return
        await cmd_set_power(pct)
        if slider is not None:
            slider._props['model-value'] = pct
            slider.update()
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    async def _on_o3_input(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating:
            return
        _updating = True
        pwr = int(predict_power_from_o3(e.value, S.flow_lpm))
        await cmd_set_power(pwr)
        if slider is not None:
            slider._props['model-value'] = pwr
            slider.update()
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pwr
            inp_pwr.update()
        S.update_derived()
        if inp_mg is not None:
            inp_mg._props['model-value'] = round(S.target_mg_per_s, 2)
            inp_mg.update()
        if inp_g30 is not None:
            inp_g30._props['model-value'] = round(S.target_g_30min, 2)
            inp_g30.update()
        _updating = False

    async def _on_mg_input(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating:
            return
        _updating = True
        o3 = mg_per_s_to_o3_pct(e.value, S.flow_lpm)
        pwr = int(predict_power_from_o3(o3, S.flow_lpm))
        await cmd_set_power(pwr)
        if slider is not None:
            slider._props['model-value'] = pwr
            slider.update()
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pwr
            inp_pwr.update()
        S.update_derived()
        if inp_o3 is not None:
            inp_o3._props['model-value'] = round(S.target_o3_pct, 2)
            inp_o3.update()
        if inp_g30 is not None:
            inp_g30._props['model-value'] = round(S.target_g_30min, 2)
            inp_g30.update()
        _updating = False

    async def _on_g30_input(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating:
            return
        _updating = True
        mg = g_at_time_to_mg_per_s(e.value, 30.0)
        o3 = mg_per_s_to_o3_pct(mg, S.flow_lpm)
        pwr = int(predict_power_from_o3(o3, S.flow_lpm))
        await cmd_set_power(pwr)
        if slider is not None:
            slider._props['model-value'] = pwr
            slider.update()
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pwr
            inp_pwr.update()
        S.update_derived()
        if inp_o3 is not None:
            inp_o3._props['model-value'] = round(S.target_o3_pct, 2)
            inp_o3.update()
        if inp_mg is not None:
            inp_mg._props['model-value'] = round(S.target_mg_per_s, 2)
            inp_mg.update()
        _updating = False

    async def _on_lpm_change(e) -> None:
        nonlocal _updating
        if _updating:
            return
        _updating = True
        S.flow_lpm = float(e.value)
        S.load_model_for_current_condition()
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
            slider._props['model-value'] = pct
            slider.update()
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pct
            inp_pwr.update()
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
        """Full rebuild — call when model, LPM, or CI band changes."""
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
        """Build Plotly figure with FIXED 4-trace order:
        [0] model curve, [1] CI band (or invisible placeholder),
        [2] target ring, [3] actual dot."""
        fig = go.Figure()
        # Trace 0 — model curve
        fig.add_trace(go.Scatter(
            x=pwr, y=o3, mode="lines",
            line=dict(color="royalblue", width=2),
            showlegend=False, name="Model",
        ))
        # Trace 1 — ±1σ empirical measurement spread (or invisible placeholder)
        mdl = S.active_model
        if mdl and mdl.spread_sigma and mdl.spread_power and any(s > 0 for s in mdl.spread_sigma):
            sp_pwr = np.array(mdl.spread_power)
            sp_o3 = np.array([mdl.predict(p) for p in sp_pwr])
            sp_s = np.array(mdl.spread_sigma)
            fig.add_trace(go.Scatter(
                x=list(sp_pwr) + list(sp_pwr[::-1]),
                y=list(sp_o3 + sp_s) + list((sp_o3 - sp_s)[::-1]),
                fill="toself",
                fillcolor="rgba(65,105,225,0.20)",
                line=dict(color="rgba(65,105,225,0.45)", width=1),
                showlegend=False, name="±1σ spread",
                hoverinfo="skip",
            ))
        else:
            fig.add_trace(go.Scatter(
                x=[], y=[], mode="lines",
                showlegend=False, visible=False,
            ))
        # Trace 2 — target ring
        fig.add_trace(go.Scatter(
            x=[tgt_pct], y=[tgt_o3], mode="markers",
            marker=dict(color="rgba(0,0,0,0)", size=18,
                        line=dict(color="black", width=3)),
            showlegend=False, name="Target",
        ))
        # Trace 3 — actual dot
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

    def _restyle_markers() -> None:
        """Lightweight update of ONLY the target ring (trace 2) and actual
        dot (trace 3) via Plotly.restyle — avoids full figure rebuild.

        Silently skips if the NiceGUI client context is gone (browser
        disconnected / tab closed).  Matches the same guard pattern used
        by _notify() to prevent cascading RuntimeErrors that can kill the
        timer permanently.
        """
        target_o3 = predict_o3_from_power(S.power_target_pct, S.flow_lpm)
        js = (
            f"const el = getElement({power_plot.id});"
            f"if(el && el.$el){{"
            f"Plotly.restyle(el.$el,"
            f"{{x:[[{S.power_target_pct}],[{S.power_actual_pct}]],"
            f"y:[[{target_o3:.6f}],[{S.vessel_o3_pct:.6f}]]}},[2,3]);"
            f"}}"
        )
        try:
            ui.run_javascript(js)
        except RuntimeError:
            pass  # client disconnected — skip silently, tick will retry next second

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

        # Sensor readings — compact single card
        ui.label("Readings").classes("text-subtitle2")
        with ui.card().classes("w-full q-pa-sm").props("flat bordered"):
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("O2 Flow").classes("text-caption text-grey")
                card_flow_val = ui.label(f"{S.flow_lpm:.1f}").classes("text-subtitle1 text-weight-medium text-purple")
                ui.label("LPM").classes("text-caption text-grey")
            ui.separator().classes("q-my-xs")
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Vessel O3").classes("text-caption text-grey")
                card_o3_val = ui.label(f"{S.vessel_o3_pct:.3f}").classes("text-subtitle1 text-weight-medium text-blue")
                ui.label("%vol").classes("text-caption text-grey")
            ui.separator().classes("q-my-xs")
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Room O3").classes("text-caption text-grey")
                card_room_val = ui.label(f"{S.room_o3_ppm:.3f}").classes("text-subtitle1 text-weight-medium text-green")
                ui.label("ppm").classes("text-caption text-grey")
            ui.separator().classes("q-my-xs")
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Vessel Temp").classes("text-caption text-grey")
                card_vtemp_val = ui.label("N/A").classes("text-subtitle1 text-weight-medium text-orange")
                ui.label("\u00b0C").classes("text-caption text-grey")
            ui.separator().classes("q-my-xs")
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Cell Temp").classes("text-caption text-grey")
                card_ctemp_val = ui.label(f"{S.cell_temp_c:.1f}").classes("text-subtitle1 text-weight-medium text-orange")
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
                        "Calibration: Baseline → Sweep Up (0→100%) → "
                        "Sweep Down (100→0%) → Random hold (~20 samples each). "
                        "ESP32 runs sweep autonomously."
                    ).classes("text-caption text-grey q-mb-sm")

                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        cal_lpm_input = ui.number(
                            label="O2 LPM", value=DEFAULT_FLOW_LPM,
                            min=0.0, max=15, step=0.5, format="%.1f",
                        ).classes("w-24")
                        cal_rnd_input = ui.number(
                            label="# Rnd Lvls", value=15,
                            min=0, max=50, step=1, format="%.0f",
                        ).classes("w-24")
                        cal_air_toggle = ui.switch("Air ON").classes(
                            "q-ml-sm"
                        )

                        async def _start_cal():
                            lpm = float(cal_lpm_input.value or 0.0)
                            num_rnd = int(cal_rnd_input.value or 0)
                            air_on = bool(cal_air_toggle.value)
                            # Validate: must have O2 flow and/or air compressor
                            if lpm <= 0.0 and not air_on:
                                _notify(
                                    "Error: O2 LPM must be > 0 or Air "
                                    "Compressor must be enabled",
                                    "negative",
                                )
                                return
                            await cmd_sequence_start(
                                "calibrate", flow=lpm,
                                num_random=num_rnd, air_comp=air_on,
                            )

                        cal_start_btn = ui.button(
                            "Start", icon="play_arrow",
                            on_click=_start_cal, color="primary",
                        )
                        cal_stop_btn = ui.button(
                            "Stop", icon="stop",
                            on_click=lambda: asyncio.create_task(
                                cmd_sequence_stop()
                            ),
                            color="grey",
                        )

                    # Phase stepper cards
                    CAL_PHASES_DEF = [
                        ("baseline", "Baseline", "~37s @ 0%"),
                        ("sweep_up", "Sweep Up", "0 → 100%"),
                        ("sweep_down", "Sweep Down", "100 → 0%"),
                        ("random", "Random", "Hold ~20 samp"),
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

                    ui.separator().classes("q-my-sm")

                    # ---- Model Fitting ----------------------------------
                    ui.label("Model Fitting").classes("text-subtitle2")
                    ui.markdown(
                        "Fit a 4-parameter sigmoid model to calibration data. "
                        "All files for the selected condition are auto-aggregated."
                    ).classes("text-caption text-grey q-mb-sm")

                    model_status_card = ui.card().classes(
                        "w-full q-pa-sm q-mb-sm"
                    ).props("flat bordered")
                    with model_status_card:
                        with ui.row().classes("items-center q-gutter-sm"):
                            model_icon = ui.icon("model_training", size="sm")
                            model_status_lbl = ui.label(S.model_status).classes(
                                "text-body2"
                            )

                    model_fit_container = ui.column().classes("w-full")

                    def _render_model_fitting() -> None:
                        """Render model fitting controls for each condition."""
                        model_fit_container.clear()
                        files = list_calibration_files()
                        existing_models = list_saved_models(MODEL_DIR)
                        model_map = {
                            (m.flow_lpm, m.o2_pct): m for m in existing_models
                        }
                        with model_fit_container:
                            if not files:
                                ui.label(
                                    "Run a calibration first"
                                ).classes("text-caption text-grey")
                                return
                            for (lpm, o2) in sorted(files):
                                flist = files[(lpm, o2)]
                                existing = model_map.get((lpm, o2))
                                with ui.card().classes(
                                    "w-full q-pa-sm q-mb-xs"
                                ).props("flat bordered"):
                                    with ui.row().classes(
                                        "items-center q-gutter-sm"
                                    ):
                                        ui.label(
                                            f"{lpm} LPM / {o2}% O2"
                                        ).classes("text-weight-bold")
                                        ui.label(
                                            f"({len(flist)} file{'s' if len(flist) != 1 else ''})"
                                        ).classes("text-caption text-grey")
                                        if existing and existing.is_valid:
                                            ui.badge(
                                                f"R²={existing.r_squared:.3f}",
                                                color="green",
                                            )
                                        else:
                                            ui.badge(
                                                "No model",
                                                color="grey",
                                            )
                                        ui.space()

                                        async def _do_fit(
                                            _lpm=lpm, _o2=o2, _files=flist,
                                        ):
                                            await _fit_model_for_condition(
                                                _files, _lpm, _o2,
                                            )

                                        ui.button(
                                            "Fit Model",
                                            icon="analytics",
                                            on_click=_do_fit,
                                            color="primary",
                                        ).props("dense size=sm")

                                    if existing and existing.is_valid:
                                        ui.label(
                                            f"L={existing.L:.3f}  k={existing.k:.3f}  "
                                            f"P0={existing.P0:.1f}  b={existing.b:.3f}  "
                                            f"RMSE={existing.rmse:.4f}  n={existing.n_points}"
                                        ).classes(
                                            "text-caption text-grey q-ml-md"
                                        )

                    _render_model_fitting()

                    # Model fit result notification area
                    fit_result_container = ui.column().classes("w-full")

                    async def _fit_model_for_condition(
                        filepaths: list[str],
                        lpm: float,
                        o2: int,
                    ) -> None:
                        """Fit sigmoid model and save to Models/O3Power/."""
                        try:
                            model = fit_sigmoid_model(
                                filepaths, flow_lpm=lpm, o2_pct=o2,
                            )
                            path = save_model_json(model, MODEL_DIR)
                            log(
                                f"Model fitted: {model.summary()} -> "
                                f"{os.path.basename(path)}",
                                "cal",
                            )
                            _notify(
                                f"Model fitted: R²={model.r_squared:.4f}, "
                                f"RMSE={model.rmse:.4f}",
                                "positive",
                            )
                            # Reload active model if this condition matches
                            S.load_model_for_current_condition()
                            _update_model_status()
                            _render_model_fitting()
                            _update_power_curve()
                            _sync_derived_to_ui()
                        except Exception as exc:
                            log(f"Model fit failed: {exc}", "error")
                            _notify(f"Model fit failed: {exc}", "negative")

                    def _update_model_status() -> None:
                        """Update the model status label."""
                        if S.active_model and S.active_model.is_valid:
                            model_icon.name = "check_circle"
                            model_icon.classes("text-green")
                            model_status_lbl.text = (
                                f"Active: {S.active_model.flow_lpm} LPM / "
                                f"{S.active_model.o2_pct}% O2 — "
                                f"{S.model_status}"
                            )
                        else:
                            model_icon.name = "info"
                            model_icon.classes("text-orange")
                            model_status_lbl.text = S.model_status

                    _update_model_status()

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
                        with ui.row().classes(
                            "w-full items-center justify-between"
                        ):
                            with ui.row().classes("items-center q-gutter-sm"):
                                val_result_icon = ui.icon(
                                    "check_circle"
                                ).classes("text-h3")
                                val_result_title = ui.label("").classes(
                                    "text-h6"
                                )

                            def _dismiss_val_result():
                                S.val_result = {}
                                val_result_card.visible = False

                            ui.button(
                                icon="close",
                                on_click=_dismiss_val_result,
                                color="grey",
                            ).props("flat dense round size=sm")
                        with ui.row().classes("q-gutter-md q-mt-sm"):
                            val_mean_lbl = ui.label("")
                            val_expected_lbl = ui.label("")
                            val_dev_lbl = ui.label("")
                            val_std_lbl = ui.label("")
                        val_file_lbl = ui.label("").classes(
                            "text-caption text-grey q-mt-xs"
                        )

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

                # ---- Fill / Evacuation Calibration section ---------------
                with ui.expansion(
                    "CSTR Calibration", icon="air"
                ).classes("w-full q-mb-sm"):
                    ui.markdown(
                        "Fill the vessel at 100% power until steady-state, "
                        "then evacuate at 0% power until O3 clears. "
                        "Fits a decay-aware CSTR model to extract system volume, "
                        "O3 decay rate, and dead volume. Air compressor is OFF "
                        "during calibration for best decay sensitivity. "
                        "Parameters generalise to any flow rate and air config."
                    ).classes("text-caption text-grey q-mb-sm")

                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        fill_lpm_input = ui.number(
                            label="O2 LPM", value=DEFAULT_FLOW_LPM,
                            min=0.5, max=5, step=0.5, format="%.1f",
                        ).classes("w-24")

                    # Target O3 readout (from validated model at 100% power)
                    fill_target_lbl = ui.label("").classes(
                        "text-caption text-grey q-mb-xs"
                    )

                    def _update_fill_target_label() -> None:
                        lpm = float(fill_lpm_input.value or DEFAULT_FLOW_LPM)
                        tgt = predict_o3_from_power(100, lpm)
                        if tgt > 0.01:
                            fill_target_lbl.text = (
                                f"C_in at 100% power: {tgt:.3f} %vol "
                                f"(steady-state: range < {FILL_STEADY_RANGE}% "
                                f"over {FILL_STEADY_COUNT} samples)"
                            )
                        else:
                            fill_target_lbl.text = (
                                "No power model — fit a model first"
                            )

                    _update_fill_target_label()
                    fill_lpm_input.on("update:model-value", lambda _: _update_fill_target_label())

                    with ui.row().classes("q-gutter-sm items-center q-mb-sm"):
                        async def _start_fill_seq():
                            lpm = float(fill_lpm_input.value or DEFAULT_FLOW_LPM)
                            cert = _find_valid_cert(100, lpm)
                            if cert is None:
                                with ui.dialog() as _cert_dlg, ui.card().classes("q-pa-md"):
                                    ui.label(
                                        "No valid validation certificate found"
                                    ).classes("text-subtitle1 text-bold q-mb-sm")
                                    ui.label(
                                        "A 100% power validation at the selected flow rate "
                                        "must pass within the last 24 hours before running "
                                        "CSTR calibration. This ensures the C_in used for "
                                        "model fitting is accurate."
                                    ).classes("text-body2 q-mb-md")
                                    with ui.row().classes("q-gutter-sm justify-end"):
                                        ui.button(
                                            "Cancel", on_click=_cert_dlg.close,
                                        ).props("flat")

                                        async def _run_val_from_dlg():
                                            _cert_dlg.close()
                                            val_pwr_input.value = 100
                                            val_lpm_input.value = lpm
                                            await cmd_sequence_start(
                                                "validate", power=100, flow=lpm,
                                            )

                                        ui.button(
                                            "Run Validation", icon="verified",
                                            on_click=_run_val_from_dlg,
                                            color="blue",
                                        )
                                _cert_dlg.open()
                                return
                            await cmd_sequence_start(
                                "cstr_cal",
                                flow=lpm,
                            )

                        fill_start_btn = ui.button(
                            "Start CSTR Calibration", icon="play_arrow",
                            on_click=_start_fill_seq, color="teal",
                        )

                        async def _stop_fill_seq():
                            S.fill_active = False
                            await cmd_sequence_stop()

                        fill_stop_btn = ui.button(
                            "Stop", icon="stop",
                            on_click=_stop_fill_seq, color="red",
                        ).props("flat")

                    # Progress bar for fill/evac
                    fill_progress = ui.linear_progress(
                        value=0, show_value=False,
                    ).classes("w-full q-mb-xs")
                    fill_phase_lbl = ui.label("").classes(
                        "text-caption text-grey"
                    )

                    # CSTR model status
                    with ui.row().classes("items-center q-gutter-xs q-mt-sm"):
                        cstr_model_icon = ui.icon("info").classes("text-orange")
                        cstr_model_lbl = ui.label(
                            S.cstr_model_status
                        ).classes("text-caption")

                    def _update_cstr_model_status() -> None:
                        if S.active_cstr_model and S.active_cstr_model.is_valid:
                            cstr_model_icon.name = "check_circle"
                            cstr_model_icon.classes("text-green", remove="text-orange")
                            cstr_model_lbl.text = (
                                f"Active: {S.cstr_model_status}"
                            )
                        else:
                            cstr_model_icon.name = "info"
                            cstr_model_icon.classes("text-orange", remove="text-green")
                            cstr_model_lbl.text = S.cstr_model_status

                    _update_cstr_model_status()

                    # Fit model button (enabled after CSTR calibration completes)
                    async def _fit_cstr_model_click():
                        model = await _fit_and_save_cstr_model()
                        if model:
                            _update_cstr_model_status()

                    fill_fit_btn = ui.button(
                        "Fit CSTR Model", icon="show_chart",
                        on_click=_fit_cstr_model_click, color="purple",
                    ).props("flat")

                    # Fill/Evac live chart
                    fill_chart = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "axis"},
                        "xAxis": {"type": "category", "name": "Sample",
                                  "data": []},
                        "yAxis": {"type": "value", "name": "O3 (%vol)"},
                        "series": [
                            {"name": "O3", "type": "line", "smooth": True,
                             "data": [], "showSymbol": False,
                             "lineStyle": {"color": "#26A69A"},
                             "areaStyle": {"opacity": 0.1, "color": "#26A69A"}},
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
    _tick_running: bool = False

    async def _tick() -> None:
        nonlocal _updating, _relay_ts, _echart_has_data, _last_prompt_id, _tick_running

        # Prevent overlapping _tick invocations
        if _tick_running:
            return
        _tick_running = True
        try:
            await _tick_inner()
        finally:
            _tick_running = False

    async def _tick_inner() -> None:
        nonlocal _updating, _relay_ts, _echart_has_data, _last_prompt_id

        # -- flush notifications queued from background tasks ----------------
        while _notify_queue:
            _msg, _lvl = _notify_queue.popleft()
            _notify(_msg, _lvl)

        # -- deferred sequence cleanup (set by COMPLETE/ABORTED handler) ---
        if S.seq_cleanup_pending and S.connected and not S.fill_active:
            S.seq_cleanup_pending = False
            log("Running deferred sequence cleanup from _tick", "seq")
            await _sequence_cleanup("deferred")

        # -- backfill safety timeout (30 s) --------------------------------
        if S.backfill_active and time.time() - S.backfill_start_time > 30:
            log("Backfill timeout (30s) — forcing backfill_active=False", "warn")
            S.backfill_active = False
            S.backfill_expected = 0
            S.backfill_received = 0

        # -- sync relays every ~10 s (SKIP during active sequences) --------
        if S.connected and not S.sequence_active and time.time() - _relay_ts > 10:
            await cmd_sync_relays()
            _relay_ts = time.time()

        # -- sidebar status -----------------------------------------------
        if S.connected:
            conn_badge.text = "Connected"
            conn_badge.props('color="green"')
            conn_badge.classes(remove="conn-blink")
            conn_badge.classes(add="conn-steady")
        else:
            conn_badge.text = "Disconnected"
            conn_badge.props('color="red"')
            conn_badge.classes(remove="conn-steady")
            conn_badge.classes(add="conn-blink")
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
        # Use _props + update() instead of .value = X to avoid triggering
        # on_change handlers (which NiceGUI schedules as background tasks;
        # clearing _updating synchronously before the task runs causes the
        # handler to re-fire cmd_set_power with stale values).
        if slider._props.get('model-value') != S.power_target_pct:
            slider._props['model-value'] = S.power_target_pct
            slider.update()
        if inp_pwr._props.get('model-value') != S.power_target_pct:
            inp_pwr._props['model-value'] = S.power_target_pct
            inp_pwr.update()
        _sync_derived_to_ui()

        # -- sequence banner + control lock --------------------------------
        seq_banner.visible = S.sequence_active
        if S.sequence_active:
            type_names = {
                "calibrate": "CALIBRATE",
                "validate": "VALIDATE",
                "cstr_cal": "CSTR CAL",
            }
            display_name = type_names.get(S.seq_type, S.seq_type.upper())

            # Phase-aware descriptive labels
            phase = S.seq_phase
            _PHASE_DESCS = {
                "loading": "Preparing...",
                "starting": f"Starting @ {S.cal_lpm:.1f} LPM",
                "started": "Initializing...",
                "relay_setup": "Enabling relays",
                "stabilizing": "Equipment warm-up (~3s)",
                "baseline": f"Initial baseline (0% power)",
                "sweep_up": f"Sweep up (0→100%)",
                "sweep_down": f"Sweep down (100→0%)",
                "random": f"Random hold @ {S.seq_power:.0f}%",
                "spot_low": f"Spot check low ({S.seq_power:.0f}%)",
                "spot_high": f"Spot check high ({S.seq_power:.0f}%)",
                "target": f"Target hold ({S.seq_power:.0f}%)",
                "cooldown": "Cooldown (0%)",
                "fill": f"Filling @ 100% — O3={S.vessel_o3_pct:.3f}%",
                "transition": "Transitioning to evac...",
                "evac": f"Evacuating @ 0% — O3={S.vessel_o3_pct:.4f}%",
                "setup": "Initializing CSTR calibration...",
                "saving": "Saving file...",
                "complete": "Complete!",
                "error": "Error — stopped",
            }
            phase_desc = _PHASE_DESCS.get(phase, phase.replace("_", " ").title())

            if phase in ("loading", "starting", "started"):
                seq_name_lbl.text = f"{display_name} — {phase_desc}"
                seq_phase_lbl.text = f"{S.seq_step_total} steps"
            elif phase == "relay_setup":
                seq_name_lbl.text = f"{display_name} — {phase_desc}"
                seq_phase_lbl.text = ""
            elif phase == "stabilizing":
                seq_name_lbl.text = f"{display_name} — {phase_desc}"
                seq_phase_lbl.text = "~3s warm-up"
            elif phase in ("saving", "complete"):
                seq_name_lbl.text = f"{display_name} — {phase_desc}"
                if S.seq_type == "cstr_cal":
                    seq_phase_lbl.text = f"{len(S.cstr_samples)} samples"
                else:
                    seq_phase_lbl.text = f"{len(S.cal_samples)} samples"
            else:
                if S.seq_type == "cstr_cal":
                    seq_name_lbl.text = f"{display_name} — {phase_desc}"
                    seq_phase_lbl.text = f"{len(S.cstr_samples)} samples"
                else:
                    seq_name_lbl.text = (
                        f"{display_name} — Step {S.seq_step_idx}/{S.seq_step_total}"
                    )
                    seq_phase_lbl.text = phase_desc
            seq_progress_bar.value = S.seq_progress / 100
            # Live elapsed time from start timestamp
            if S.seq_start_time > 0:
                S.seq_elapsed = time.time() - S.seq_start_time
            mins = int(S.seq_elapsed) // 60
            secs = int(S.seq_elapsed) % 60
            seq_elapsed_lbl.text = f"{mins}:{secs:02d}"

        # Lock controls during sequences (power + relays + O2 input)
        lockable = [slider, inp_pwr, inp_o3, inp_mg, inp_g30,
                     btn_air, btn_o2, btn_o3, inp_lpm_sb,
                     cal_start_btn, cal_lpm_input, cal_rnd_input,
                     cal_air_toggle, val_start_btn, fill_start_btn]
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

        # -- power curve markers ----------------------------------------
        try:
            _restyle_markers()
        except RuntimeError:
            pass  # Page refresh — slot parent deleted, timer fires once more

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
            _CAL_TAB_LABELS = {
                "loading": "Preparing calibration",
                "starting": f"Starting calibration @ {S.cal_lpm:.1f} LPM",
                "started": "Initializing",
                "relay_setup": "Enabling relays",
                "stabilizing": "Equipment warm-up",
                "baseline": "Initial baseline (0% power)",
                "sweep_up": f"Sweep up (0→100%) @ {S.seq_power:.0f}%",
                "sweep_down": f"Sweep down (100→0%) @ {S.seq_power:.0f}%",
                "random": f"Random hold @ {S.seq_power:.0f}%",
                "saving": "Saving calibration file",
                "complete": "Calibration complete",
            }
            phase_text = _CAL_TAB_LABELS.get(
                S.seq_phase,
                S.seq_phase.replace("_", " ").title()
            )
            cal_phase_lbl.text = (
                f"Phase: {phase_text}  "
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
            # "saving" and "complete" mean all phases are done
            if S.seq_phase in ("saving", "complete"):
                active_idx = len(phase_order)  # all complete
            else:
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

        # Cal scatter chart — by phase
        if S.cal_samples:
            sweep_up = [
                [s["power_actual"], s["o3_pct"]]
                for s in S.cal_samples if s.get("phase") == "sweep_up"
            ]
            sweep_down = [
                [s["power_actual"], s["o3_pct"]]
                for s in S.cal_samples if s.get("phase") == "sweep_down"
            ]
            random_pts = [
                [s["power_actual"], s["o3_pct"]]
                for s in S.cal_samples if s.get("phase") == "random"
            ]
            cal_chart.options["series"][0]["data"] = sweep_up
            cal_chart.options["series"][1]["data"] = sweep_down
            cal_chart.options["series"][2]["data"] = random_pts
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
            val_file_lbl.text = (
                f"Saved: {S.val_file}" if S.val_file else ""
            )

        # -- fill/evac observer UI ----------------------------------------
        if S.fill_active or S.fill_phase in ("complete", "error"):
            # Update progress bar and phase label
            fill_progress.value = S.seq_progress / 100
            n_fill = sum(1 for s in S.cstr_samples if s.get("phase") == "fill")
            n_evac = sum(1 for s in S.cstr_samples if s.get("phase") == "evac")
            _FILL_PHASE_DESCS = {
                "setup": "Initializing...",
                "relay_setup": "Enabling relays",
                "baseline": f"Baseline (0% power) — {len(S.cstr_samples)} samples",
                "fill": (
                    f"Filling @ 100% — O3={S.vessel_o3_pct:.3f}% "
                    f"/ C_in={S.fill_target_o3:.3f}%"
                ),
                "transition": "Transitioning to evac...",
                "evac": (
                    f"Evacuating @ 0% — O3={S.vessel_o3_pct:.4f}%"
                ),
                "saving": "Saving data...",
                "complete": (
                    f"Complete — {n_fill} fill + {n_evac} evac samples"
                ),
                "error": "Error — sequence stopped",
            }
            fill_phase_lbl.text = _FILL_PHASE_DESCS.get(
                S.fill_phase, S.fill_phase.replace("_", " ").title()
            )

            # Update live chart with all CSTR samples
            if S.cstr_samples:
                x_labels = list(range(len(S.cstr_samples)))
                y_data = [s.get("vessel_o3_pct", 0) for s in S.cstr_samples]
                fill_chart.options["xAxis"]["data"] = x_labels
                fill_chart.options["series"][0]["data"] = y_data
                fill_chart.update()

        if not S.fill_active and S.fill_phase not in ("complete", "error"):
            fill_progress.value = 0
            fill_phase_lbl.text = ""

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
            "fill_active": S.fill_active,
            "fill_phase": S.fill_phase,
            "cstr_samples": len(S.cstr_samples),
            "fill_target_o3": round(S.fill_target_o3, 4),
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
    # Load fitted model for current operating condition
    S.load_model_for_current_condition()
    S.load_cstr_model()
    tcp = TCPServer(args.port)
    await tcp.start()


app.on_startup(_startup)

ui.run(title="BlockSI Control", port=8080, reload=False)

