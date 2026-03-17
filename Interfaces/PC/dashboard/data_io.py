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
    TELEMETRY_DIR, CALIBRATION_DIR, VALIDATION_DIR,
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
    lpm_s = f"{lpm:.0f}" if lpm == int(lpm) else f"{lpm:.1f}"
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
    """Return path to the most-recent PASS cert for (power_pct, flow_lpm)."""
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
