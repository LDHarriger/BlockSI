"""
Measurement verification — measure C_in at operating power levels before a batch.

Replaces legacy validation (spot-checking model at random points) with direct
measurement of inlet O3 concentration at 100% power and at the predicted
power_hold.  These measured values feed into the dosimetry solver for improved
dose schedule accuracy.

Safety constraint: process time is FIXED and must NEVER be adjusted to
compensate for concentration discrepancies (Chick-Watson kinetics — log-linear,
not time-linear).

Recipe structure:
    Step 0: power=0,           phase=baseline,    hold=15      (verify no residual O3)
    Step 1: power=100,         phase=full_power,  hold=120     (measure C_in @ 100%)
    Step 2: power=power_hold,  phase=hold_power,  hold=120     (measure C_in @ hold)
    Step 3: power=0,           phase=cooldown,    hold=5       (safe shutdown)

Stability criteria (per phase):
    - Rolling window of 45 samples
    - Stable if EITHER: absolute slope < 0.0003 %vol/sample
      OR relative slope (|slope/mean|) < 0.0005 when mean > 0.1 %vol
    - C_in = mean of stable window
"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

import numpy as np


# =============================================================================
# Constants
# =============================================================================
STABILITY_WINDOW = 45
STABILITY_SLOPE_THRESHOLD = 0.0003   # absolute: %vol per sample
STABILITY_REL_SLOPE_THRESHOLD = 0.0005  # relative: |slope/mean| per sample
FULL_POWER_HOLD_SAMPLES = 120       # ~5 min at 2.5 s/sample
HOLD_POWER_HOLD_SAMPLES = 120
BASELINE_HOLD_SAMPLES = 15
COOLDOWN_HOLD_SAMPLES = 5
TRANSIENT_SKIP = 15                 # skip first N samples per phase (settling)

# Legacy constants used by k_d_cal cert check for trimming validation CSV data
VAL_TRANSIENT_SKIP = 7
VAL_MIN_STABLE = 3


# =============================================================================
# Result dataclass
# =============================================================================
@dataclass
class VerificationResult:
    """Measured C_in values and stability diagnostics."""
    c_in_100pct: float = 0.0          # Measured O3 %vol at 100% power
    c_in_hold: float = 0.0            # Measured O3 %vol at power_hold
    power_hold_pct: float = 0.0       # Power level used for hold measurement
    flow_lpm: float = 0.0

    full_power_stable: bool = False
    full_power_slope: float = 0.0
    full_power_n_stable: int = 0

    hold_power_stable: bool = False
    hold_power_slope: float = 0.0
    hold_power_n_stable: int = 0

    baseline_mean: float = 0.0
    baseline_ok: bool = True

    passed: bool = False
    timestamp: str = ""


# =============================================================================
# Recipe generator
# =============================================================================
def generate_verify_recipe(
    power_hold_pct: float,
) -> list[tuple[int, str, int]]:
    """Return a verification recipe as [(power, phase_label, hold_samples), ...].

    Two measurement phases with generous holds for stability detection.
    """
    return [
        (0,                     "baseline",    BASELINE_HOLD_SAMPLES),
        (100,                   "full_power",  FULL_POWER_HOLD_SAMPLES),
        (int(round(power_hold_pct)), "hold_power",  HOLD_POWER_HOLD_SAMPLES),
        (0,                     "cooldown",    COOLDOWN_HOLD_SAMPLES),
    ]


# =============================================================================
# Stability analysis
# =============================================================================
def _check_stability(
    values: list[float],
    window: int = STABILITY_WINDOW,
    slope_threshold: float = STABILITY_SLOPE_THRESHOLD,
    rel_slope_threshold: float = STABILITY_REL_SLOPE_THRESHOLD,
) -> tuple[bool, float, float]:
    """Check for a stable window in a series of O3 readings.

    Scans from the END of the series backward to find the latest stable window.
    A window is stable if EITHER the absolute slope is below *slope_threshold*
    OR the relative slope (|slope/mean|) is below *rel_slope_threshold* when
    the mean is > 0.1 %vol.  The relative criterion handles the fact that
    higher concentrations naturally have larger absolute drift.

    Returns:
        (is_stable, slope_of_best_window, mean_of_best_window)
    """
    if len(values) < window:
        return False, 0.0, 0.0

    arr = np.asarray(values, dtype=np.float64)
    x = np.arange(window, dtype=np.float64)

    # Scan from end (most representative of steady-state)
    for start in range(len(arr) - window, -1, -1):
        segment = arr[start : start + window]
        # Least-squares slope
        x_mean = (window - 1) / 2.0
        y_mean = np.mean(segment)
        slope = float(
            np.sum((x - x_mean) * (segment - y_mean))
            / np.sum((x - x_mean) ** 2)
        )
        abs_slope = abs(slope)
        # Stable if absolute slope is small enough
        if abs_slope < slope_threshold:
            return True, slope, float(y_mean)
        # OR if relative slope is small enough (for higher concentrations)
        if y_mean > 0.1 and abs_slope / y_mean < rel_slope_threshold:
            return True, slope, float(y_mean)

    # No stable window found — return stats from last window  
    last_seg = arr[-window:]
    x_mean = (window - 1) / 2.0
    y_mean = float(np.mean(last_seg))
    slope = float(
        np.sum((x - x_mean) * (last_seg - y_mean))
        / np.sum((x - x_mean) ** 2)
    )
    return False, slope, y_mean


# =============================================================================
# Post-run analysis
# =============================================================================
def analyze_verification(
    samples: list[dict],
    power_hold_pct: float,
    flow_lpm: float,
) -> VerificationResult:
    """Analyze verification samples and extract measured C_in values.

    Groups samples by phase, skips initial transients, then checks for
    stability in each measurement phase.
    """
    result = VerificationResult(
        power_hold_pct=power_hold_pct,
        flow_lpm=flow_lpm,
        timestamp=datetime.now().isoformat(),
    )

    # Group by phase
    by_phase: dict[str, list[dict]] = {}
    for s in samples:
        phase = s.get("phase", "unknown")
        by_phase.setdefault(phase, []).append(s)

    # Baseline check
    baseline = by_phase.get("baseline", [])
    if baseline:
        bl_vals = [s["o3_pct"] for s in baseline]
        result.baseline_mean = float(np.mean(bl_vals))
        result.baseline_ok = result.baseline_mean <= 0.02

    # Full power (100%) measurement
    fp_data = by_phase.get("full_power", [])
    if fp_data:
        fp_stable_data = fp_data[TRANSIENT_SKIP:]
        if fp_stable_data:
            fp_vals = [s["o3_pct"] for s in fp_stable_data]
            stable, slope, mean_val = _check_stability(fp_vals)
            result.full_power_stable = stable
            result.full_power_slope = slope
            result.c_in_100pct = mean_val
            result.full_power_n_stable = min(len(fp_vals), STABILITY_WINDOW)

    # Hold power measurement
    hp_data = by_phase.get("hold_power", [])
    if hp_data:
        hp_stable_data = hp_data[TRANSIENT_SKIP:]
        if hp_stable_data:
            hp_vals = [s["o3_pct"] for s in hp_stable_data]
            stable, slope, mean_val = _check_stability(hp_vals)
            result.hold_power_stable = stable
            result.hold_power_slope = slope
            result.c_in_hold = mean_val
            result.hold_power_n_stable = min(len(hp_vals), STABILITY_WINDOW)

    # Overall pass: both phases must be stable
    result.passed = (
        result.baseline_ok
        and result.full_power_stable
        and result.hold_power_stable
    )
    return result
