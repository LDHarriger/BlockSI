"""
Dashboard shared state — SystemState singleton, constants, logging, notifications.

This module is the leaf of the dependency graph: everything else imports from
here, but this module imports only from ``analysis/`` (external).
"""
from __future__ import annotations

import os
import re
import sys
from collections import deque
from datetime import datetime
from typing import Any, Optional

import numpy as np

# ---------------------------------------------------------------------------
# Ensure the parent ``Interfaces/PC/`` directory is on sys.path so that
# ``from analysis import ...`` works regardless of how the dashboard is
# launched (direct execution, ``-m``, or import from another location).
# ---------------------------------------------------------------------------
# _PC_DIR = Interfaces/PC/ (parent of dashboard/)
_PC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PC_DIR not in sys.path:
    sys.path.insert(0, _PC_DIR)

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
    CSTRModel,
    FillModel,
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

# BASE_DIR = Interfaces/ (3 levels up from dashboard/state.py)
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA_DIR = os.path.join(BASE_DIR, "Data")
TELEMETRY_DIR = os.path.join(DATA_DIR, "Telemetry")
CALIBRATION_DIR = os.path.join(DATA_DIR, "Power-O3_cal")
VALIDATION_DIR = os.path.join(DATA_DIR, "Validation")
CSTR_DATA_DIR = os.path.join(DATA_DIR, "k_d_cal")
MODEL_DIR = os.path.join(BASE_DIR, "Models", "O3Power")
CSTR_MODEL_DIR = os.path.join(BASE_DIR, "Models", "cstr_k_d")
K_ABS_DATA_DIR = os.path.join(DATA_DIR, "k_abs_cal")
K_ABS_MODEL_DIR = os.path.join(BASE_DIR, "Models", "cstr_k_abs")
BATCH_DATA_DIR = os.path.join(DATA_DIR, "Batch")
DIAGNOSTICS_DIR = os.path.join(DATA_DIR, "Diagnostics")

for _d in (DATA_DIR, TELEMETRY_DIR, CALIBRATION_DIR, VALIDATION_DIR,
           CSTR_DATA_DIR, MODEL_DIR, CSTR_MODEL_DIR,
           K_ABS_DATA_DIR, K_ABS_MODEL_DIR, BATCH_DATA_DIR, DIAGNOSTICS_DIR):
    os.makedirs(_d, exist_ok=True)

# Power model coefficients  (legacy fallback — O3_max = A/F + B)
POWER_MODEL_A = 1.78
POWER_MODEL_B = 1.40
DEFAULT_FLOW_LPM = 4.0

# O3 mass-flow conversion
O3_MASS_FLOW_K = 0.3327      # 48000 / (24.04 × 60 × 100)
AIR_COMP_LPM = 10.0

# O2 concentration constants
O2_CONC_PCT = 95
AIR_COMP_O2_PCT = 21

POWER_MISMATCH_THRESHOLD = 5.0

# Prompt content mapping
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
    "batch_vessel_cool": {
        "title": "Vessel Cool-Down Check",
        "icon": "thermostat",
        "body": (
            "Sterilization complete. Verify the vessel has cooled to a safe "
            "temperature and O3 is below 0.01% vol before opening.<br><br>"
            "Press <b>Confirm</b> when safe to proceed with inoculation."
        ),
    },
    "batch_add_inoculant": {
        "title": "Add Inoculant",
        "icon": "science",
        "body": (
            "Open the vessel, add the prepared inoculant to the substrate, "
            "and close the vessel securely.<br><br>"
            "Press <b>Confirm</b> when done."
        ),
    },
    "batch_distribute": {
        "title": "Distribute & Label",
        "icon": "inventory_2",
        "body": (
            "Mix the inoculated substrate thoroughly, distribute into bags "
            "or containers, and label each with the Batch ID shown in the "
            "dashboard.<br><br>"
            "Press <b>Confirm</b> when distribution is complete."
        ),
    },
}


# =============================================================================
# Conversion helpers
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
# O2 concentration helpers
# =============================================================================
def compute_effective_o2_pct(flow_lpm: float, air_comp_on: bool) -> int:
    if air_comp_on:
        total = flow_lpm + AIR_COMP_LPM
        if total > 0:
            return round((flow_lpm * O2_CONC_PCT + AIR_COMP_LPM * AIR_COMP_O2_PCT) / total)
    return O2_CONC_PCT


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
        self.last_esp_ts_ms: int = 0
        # Connection
        self.connected: bool = False
        # Sequence observer state
        self.sequence_active: bool = False
        self.seq_type: str = ""
        self.seq_phase: str = ""
        self.seq_progress: float = 0.0
        self.seq_elapsed: float = 0.0
        self.seq_start_time: float = 0.0
        self.seq_confirmed: bool = False
        self.seq_cleanup_pending: bool = False
        self.seq_power: float = 0.0
        self.seq_step_idx: int = 0
        self.seq_step_total: int = 0
        # Pending prompt from ESP32
        self.pending_prompt_id: str = ""
        self.pending_prompt_text: str = ""
        # Calibration observer
        self.cal_samples: list[dict] = []
        self.cal_lpm: float = DEFAULT_FLOW_LPM
        self.cal_file: str = ""
        # Verification observer (measurement of C_in)
        self.verify_samples: list[dict] = []
        self.verify_result: Optional[object] = None  # VerificationResult
        self.verify_file: str = ""
        # Settings
        self.notify_level: str = "all"
        # Backfill state
        self.backfill_active: bool = False
        self.backfill_expected: int = 0
        self.backfill_received: int = 0
        self.backfill_start_time: float = 0.0
        # Derived
        self.target_o3_pct: float = 0.0
        self.target_mg_per_s: float = 0.0
        self.target_g_30min: float = 0.0
        # Active model
        self.active_model: Optional[PowerO3Model] = None
        self.model_status: str = "No model"
        # CSTR model
        self.active_cstr_model: Optional[CSTRModel] = None
        self.cstr_model_status: str = "No model"
        # Fill/Evac sequence state
        self.fill_active: bool = False
        self.fill_phase: str = ""
        self.cstr_samples: list[dict] = []
        self.cstr_csv_path: str = ""
        self.fill_target_o3: float = 0.0
        self.fill_lpm: float = DEFAULT_FLOW_LPM
        # Batch (process_batch) sequence state
        self.batch_samples: list[dict] = []
        self.batch_dose_running: float = 0.0
        self.batch_dose_target: float = 0.0
        self.batch_schedule: object = None   # DoseSchedule, if active

    def load_model_for_current_condition(self) -> None:
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
# Data parsing
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

    Safe to call from background tasks: if NiceGUI's slot context is
    unavailable, the message is queued and the periodic _tick loop will
    flush it with the correct context.
    """
    if S.notify_level == "none":
        return
    if S.notify_level == "errors" and level == "positive":
        return
    try:
        from nicegui import ui
        ui.notify(msg, type=level, position="bottom-right", timeout=3000)
    except (RuntimeError, ImportError):
        _notify_queue.append((msg, level))
