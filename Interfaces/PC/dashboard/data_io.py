"""
Data I/O — CSV logger, calibration/validation file helpers.
"""
from __future__ import annotations

import os
import re
from datetime import datetime
from typing import Optional

import pandas as pd

from dashboard.state import (
    S, log,
    TELEMETRY_DIR, CALIBRATION_DIR, VALIDATION_DIR, MODEL_DIR,
    CSTR_DATA_DIR, CSTR_MODEL_DIR,
    O2_CONC_PCT,
)

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
            return
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
# Calibration file helpers
# =============================================================================
def list_calibration_files() -> dict[tuple[float, int], list[str]]:
    """List calibration CSVs grouped by (LPM, O2%)."""
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
    """Save calibration samples to CSV, return filename."""
    now = datetime.now()
    lpm_s = f"{lpm:.2f}"
    fname = f"{now:%Y-%m-%d_%H%M%S}_PowerO3Cal_{lpm_s}Lpm_{o2_pct}O2.csv"
    fpath = os.path.join(CALIBRATION_DIR, fname)
    if samples:
        pd.DataFrame(samples).to_csv(fpath, index=False)
        log(f"Saved {len(samples)} cal samples -> {fname}", "cal")
    return fname


def _save_val_csv(samples: list[dict], power: float, lpm: float,
                  passed: bool) -> str:
    """Save validation samples to CSV, return filename."""
    now = datetime.now()
    lpm_s = f"{lpm:.2f}"
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


def _find_valid_cert(flow_lpm: float,
                     max_age_h: float = 24.0) -> "str | None":
    """Return path to the most-recent PASS verification cert for *flow_lpm*.

    All verification runs include a 100% full_power phase (Step 1), so any
    PASS cert at the correct flow rate confirms C_in at 100%.  The hold-power
    level encoded in the filename is irrelevant for this check.

    Matches both legacy Validation and new Verification filenames:
      - *_Validation_<pwr>pct_<lpm>Lpm_PASS.csv
      - *_Verification_<hold>hold_<lpm>Lpm_PASS.csv

    Flow rate is compared numerically (tolerance 0.05 LPM) so both old
    formats (``4Lpm``) and new (``3.50Lpm``) match correctly.
    """
    pattern = re.compile(
        r"^(\d{4}-\d{2}-\d{2}_\d{6})_"
        r"(?:Validation_\d+pct|Verification_\d+hold)_"
        r"([\d.]+)Lpm_PASS\.csv$"
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
        # Numeric flow-rate filter (handles 4, 4.0, 4.00 formats)
        try:
            file_lpm = float(m.group(2))
        except ValueError:
            continue
        if abs(file_lpm - flow_lpm) > 0.05:
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
# WP-6: Calibration enforcement helpers
# =============================================================================
def _find_valid_calibration(
    flow_lpm: float,
    max_age_h: float = 336.0,  # 2 weeks default
    lpm_tolerance: float = 0.25,
) -> "str | None":
    """Return path to the most-recent power-O3 calibration CSV matching flow_lpm.

    Scans Data/Power-O3_cal/ for calibration CSVs within ±lpm_tolerance of the
    requested flow rate, newer than max_age_h.
    """
    cutoff = datetime.now().timestamp() - max_age_h * 3600
    best: tuple[float, str] | None = None
    try:
        entries = os.listdir(CALIBRATION_DIR)
    except FileNotFoundError:
        return None
    for name in entries:
        if not name.endswith(".csv") or "PowerO3Cal" not in name:
            continue
        m_lpm = re.search(r"_(\d+(?:\.\d+)?)Lpm", name)
        if not m_lpm:
            continue
        cal_lpm = float(m_lpm.group(1))
        if abs(cal_lpm - flow_lpm) > lpm_tolerance:
            continue
        m_ts = re.match(r"^(\d{4}-\d{2}-\d{2}_\d{6})", name)
        if not m_ts:
            continue
        try:
            dt = datetime.strptime(m_ts.group(1), "%Y-%m-%d_%H%M%S")
        except ValueError:
            continue
        ts = dt.timestamp()
        if ts >= cutoff:
            if best is None or ts > best[0]:
                best = (ts, os.path.join(CALIBRATION_DIR, name))
    return best[1] if best else None


def _find_valid_calibration_model(
    flow_lpm: float,
    max_age_h: float = 336.0,
    lpm_tolerance: float = 0.25,
) -> "str | None":
    """Return path to the most-recent power-O3 model JSON matching flow_lpm.

    Scans Models/O3Power/ for model JSONs within ±lpm_tolerance.
    """
    import json as _json
    cutoff = datetime.now().timestamp() - max_age_h * 3600
    best: tuple[float, str] | None = None
    try:
        entries = os.listdir(MODEL_DIR)
    except FileNotFoundError:
        return None
    for name in sorted(entries, reverse=True):
        if not name.endswith(".json"):
            continue
        fpath = os.path.join(MODEL_DIR, name)
        try:
            with open(fpath) as f:
                data = _json.load(f)
            model_lpm = data.get("flow_lpm", 0)
            if abs(model_lpm - flow_lpm) > lpm_tolerance:
                continue
            fitted_at = data.get("fitted_at", "")
            if fitted_at:
                dt = datetime.fromisoformat(fitted_at)
                ts = dt.timestamp()
                if ts >= cutoff:
                    if best is None or ts > best[0]:
                        best = (ts, fpath)
        except Exception:
            continue
    return best[1] if best else None


# =============================================================================
# WP-7: Flow rate helpers
# =============================================================================
def list_calibrated_flow_rates() -> list[float]:
    """Return sorted list of flow rates that have valid power-O3 models.

    Scans Models/O3Power/ for model JSONs and extracts their flow rate values.
    """
    import json as _json
    rates: set[float] = set()
    try:
        entries = os.listdir(MODEL_DIR)
    except FileNotFoundError:
        return []
    for name in entries:
        if not name.endswith(".json"):
            continue
        fpath = os.path.join(MODEL_DIR, name)
        try:
            with open(fpath) as f:
                data = _json.load(f)
            lpm = data.get("flow_lpm")
            if lpm is not None and lpm > 0:
                rates.add(float(lpm))
        except Exception:
            continue
    return sorted(rates)
