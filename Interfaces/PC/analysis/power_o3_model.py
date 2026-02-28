"""
Power→O3 sigmoid model: fitting, persistence, and prediction.

Model:  O3 = L / (1 + exp(-k * (P - P0))) + b

Parameters:
    L   – maximum O3 concentration (asymptote height above baseline)
    k   – steepness (slope) of the transition
    P0  – inflection point (power % at half-max)
    b   – baseline offset (O3 at 0% power, typically ~0)

Each model applies to a specific (flow_lpm, o2_pct) operating condition.
Models are saved as JSON in ``Models/O3Power/``.
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Optional

import numpy as np
import pandas as pd

# Try importing scipy — graceful degradation if not installed
try:
    from scipy.optimize import curve_fit
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# =============================================================================
# Data classes
# =============================================================================
@dataclass
class PowerO3Model:
    """A fitted 4-parameter sigmoid model for a given operating condition."""

    # Sigmoid params
    L: float = 0.0      # asymptote
    k: float = 0.0      # steepness
    P0: float = 0.0     # inflection point (power %)
    b: float = 0.0      # baseline offset

    # Operating condition
    flow_lpm: float = 4.0
    o2_pct: int = 95

    # Fit quality
    r_squared: float = 0.0
    rmse: float = 0.0
    n_points: int = 0

    # Provenance
    source_files: list[str] = field(default_factory=list)
    excluded_files: list[str] = field(default_factory=list)
    fitted_at: str = ""
    notes: str = ""

    def predict(self, power_pct: float) -> float:
        """Predict O3 %vol from power %."""
        if power_pct <= 0:
            return max(0.0, self.b)
        return _sigmoid(power_pct, self.L, self.k, self.P0, self.b)

    def predict_inverse(self, o3_pct: float) -> float:
        """Predict power % from O3 %vol (inverse sigmoid)."""
        if o3_pct <= self.b:
            return 0.0
        if self.L <= 0 or self.k == 0:
            return 0.0
        ratio = (o3_pct - self.b) / self.L
        if ratio >= 1.0:
            return 100.0
        if ratio <= 0.0:
            return 0.0
        return self.P0 - np.log(1.0 / ratio - 1.0) / self.k

    def generate_curve(self, n_points: int = 101) -> tuple[np.ndarray, np.ndarray]:
        """Generate smooth power/O3 arrays for plotting."""
        pwr = np.linspace(0, 100, n_points)
        o3 = np.array([self.predict(p) for p in pwr])
        return pwr, o3

    @property
    def is_valid(self) -> bool:
        """True if the model has been fitted (L > 0 and finite params)."""
        return (
            self.L > 0
            and np.isfinite(self.L)
            and np.isfinite(self.k)
            and np.isfinite(self.P0)
            and np.isfinite(self.b)
            and self.r_squared > 0
        )

    @property
    def condition_key(self) -> tuple[float, int]:
        """(flow_lpm, o2_pct) tuple for matching."""
        return (self.flow_lpm, self.o2_pct)

    def summary(self) -> str:
        """One-line human-readable summary."""
        return (
            f"{self.flow_lpm} LPM / {self.o2_pct}% O2: "
            f"L={self.L:.3f}, k={self.k:.3f}, P0={self.P0:.1f}, b={self.b:.3f} "
            f"(R²={self.r_squared:.4f}, RMSE={self.rmse:.4f}, n={self.n_points})"
        )


# =============================================================================
# Sigmoid function
# =============================================================================
def _sigmoid(P: float | np.ndarray, L: float, k: float, P0: float, b: float):
    """4-parameter sigmoid: O3 = L / (1 + exp(-k*(P - P0))) + b."""
    return L / (1.0 + np.exp(-k * (P - P0))) + b


# =============================================================================
# CSV loading
# =============================================================================
def load_calibration_csv(filepath: str) -> pd.DataFrame:
    """Load a single calibration CSV and return a cleaned DataFrame.

    Expected columns: step_idx, sample_num, o3_pct, temp_c,
                      power_actual, air_comp, phase, power_target

    Returns DataFrame with columns:
        power_target, power_actual, o3_pct, temp_c, air_comp, phase, source_file
    """
    df = pd.read_csv(filepath)
    # Normalise column names (strip whitespace)
    df.columns = [c.strip() for c in df.columns]
    df["source_file"] = os.path.basename(filepath)
    return df


def aggregate_calibration_data(
    filepaths: list[str],
    exclude_files: Optional[list[str]] = None,
    phases: Optional[list[str]] = None,
) -> pd.DataFrame:
    """Load and aggregate multiple calibration CSVs.

    Args:
        filepaths: Paths to calibration CSVs.
        exclude_files: Basenames of files to skip (user exclusions).
        phases: Phases to include. Default: ``sweep_up`` and ``sweep_down``
                (excludes ``baseline``).

    Returns:
        Combined DataFrame, averaged per (power_target, phase) to collapse
        multi-sample steps into single points.
    """
    if phases is None:
        phases = ["sweep_up", "sweep_down"]
    exclude_set = set(exclude_files or [])

    frames: list[pd.DataFrame] = []
    for fp in filepaths:
        if os.path.basename(fp) in exclude_set:
            continue
        try:
            df = load_calibration_csv(fp)
            frames.append(df)
        except Exception as exc:
            print(f"[analysis] Skipping {fp}: {exc}")

    if not frames:
        return pd.DataFrame()

    combined = pd.concat(frames, ignore_index=True)

    # Filter to requested phases
    if phases:
        combined = combined[combined["phase"].isin(phases)]

    if combined.empty:
        return combined

    # Average o3_pct per (power_target, phase, source_file) — collapses
    # multi-sample steps while keeping per-file identity for diagnostics.
    grouped = (
        combined.groupby(["power_target", "phase", "source_file"])
        .agg(
            o3_pct_mean=("o3_pct", "mean"),
            o3_pct_std=("o3_pct", "std"),
            temp_c_mean=("temp_c", "mean"),
            power_actual_mean=("power_actual", "mean"),
            n_samples=("o3_pct", "count"),
        )
        .reset_index()
    )

    return grouped


def _compute_fit_points(aggregated: pd.DataFrame) -> tuple[np.ndarray, np.ndarray]:
    """From aggregated data, produce (power, o3) arrays for curve fitting.

    Further averages across files so each power_target has a single O3 value.
    """
    if aggregated.empty:
        return np.array([]), np.array([])

    final = (
        aggregated.groupby("power_target")
        .agg(o3=("o3_pct_mean", "mean"))
        .reset_index()
        .sort_values("power_target")
    )
    return final["power_target"].values.astype(float), final["o3"].values.astype(float)


# =============================================================================
# Fitting
# =============================================================================
def fit_sigmoid_model(
    filepaths: list[str],
    flow_lpm: float,
    o2_pct: int,
    exclude_files: Optional[list[str]] = None,
    phases: Optional[list[str]] = None,
) -> PowerO3Model:
    """Fit a 4-parameter sigmoid to calibration data.

    Args:
        filepaths: Calibration CSV paths for this (flow, O2%) condition.
        flow_lpm: O2 concentrator flow in LPM.
        o2_pct: Effective O2 percentage (weighted average incl. air comp).
        exclude_files: Basenames to exclude from fitting.
        phases: Sweep phases to include (default: sweep_up + sweep_down).

    Returns:
        Fitted ``PowerO3Model``. Check ``.is_valid`` to confirm success.

    Raises:
        RuntimeError: If scipy is not installed.
        ValueError: If no data points remain after aggregation.
    """
    if not HAS_SCIPY:
        raise RuntimeError(
            "scipy is required for model fitting. "
            "Install with: pip install scipy"
        )

    agg = aggregate_calibration_data(filepaths, exclude_files, phases)
    power, o3 = _compute_fit_points(agg)

    if len(power) < 4:
        raise ValueError(
            f"Need at least 4 data points for sigmoid fit, got {len(power)}"
        )

    # Initial guesses from data
    o3_max = float(np.max(o3))
    o3_min = float(np.min(o3))
    L_guess = o3_max - o3_min if o3_max > o3_min else 0.5
    b_guess = o3_min
    P0_guess = float(power[np.argmin(np.abs(o3 - (o3_max + o3_min) / 2))])
    k_guess = 0.1

    try:
        popt, pcov = curve_fit(
            _sigmoid,
            power,
            o3,
            p0=[L_guess, k_guess, P0_guess, b_guess],
            bounds=(
                [0, 0.001, 0, -0.5],         # lower bounds
                [10.0, 1.0, 100.0, 0.5],      # upper bounds
            ),
            maxfev=10000,
        )
    except Exception as exc:
        raise ValueError(f"Curve fitting failed: {exc}") from exc

    L_fit, k_fit, P0_fit, b_fit = popt

    # Goodness of fit
    o3_pred = _sigmoid(power, *popt)
    ss_res = float(np.sum((o3 - o3_pred) ** 2))
    ss_tot = float(np.sum((o3 - np.mean(o3)) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(ss_res / len(o3)))

    # Build source file list (only non-excluded)
    exclude_set = set(exclude_files or [])
    used_files = [
        os.path.basename(fp)
        for fp in filepaths
        if os.path.basename(fp) not in exclude_set
    ]

    return PowerO3Model(
        L=round(float(L_fit), 6),
        k=round(float(k_fit), 6),
        P0=round(float(P0_fit), 6),
        b=round(float(b_fit), 6),
        flow_lpm=flow_lpm,
        o2_pct=o2_pct,
        r_squared=round(r_squared, 6),
        rmse=round(rmse, 6),
        n_points=len(power),
        source_files=used_files,
        excluded_files=list(exclude_set),
        fitted_at=datetime.now().isoformat(timespec="seconds"),
    )


# =============================================================================
# Persistence  (JSON in Models/O3Power/)
# =============================================================================
def _model_filename(flow_lpm: float, o2_pct: int) -> str:
    """Canonical model filename: ``power_o3_{LPM}lpm_{O2}o2.json``."""
    lpm_s = f"{flow_lpm:.0f}" if flow_lpm == int(flow_lpm) else f"{flow_lpm:.1f}"
    return f"power_o3_{lpm_s}lpm_{o2_pct}o2.json"


def save_model(model: PowerO3Model, model_dir: str) -> str:
    """Save model to JSON file in *model_dir*. Returns the full path."""
    os.makedirs(model_dir, exist_ok=True)
    fname = _model_filename(model.flow_lpm, model.o2_pct)
    fpath = os.path.join(model_dir, fname)

    data = asdict(model)
    data["type"] = "sigmoid_4p"
    data["version"] = 1

    with open(fpath, "w") as f:
        json.dump(data, f, indent=2)

    return fpath


def load_model(filepath: str) -> PowerO3Model:
    """Load a PowerO3Model from a JSON file."""
    with open(filepath) as f:
        data = json.load(f)

    # Strip non-dataclass keys
    data.pop("type", None)
    data.pop("version", None)

    return PowerO3Model(**data)


def load_model_for_condition(
    model_dir: str, flow_lpm: float, o2_pct: int
) -> Optional[PowerO3Model]:
    """Load the model matching (flow_lpm, o2_pct), or None if not found."""
    fname = _model_filename(flow_lpm, o2_pct)
    fpath = os.path.join(model_dir, fname)
    if os.path.exists(fpath):
        try:
            return load_model(fpath)
        except Exception:
            return None
    return None


def list_models(model_dir: str) -> list[PowerO3Model]:
    """List all saved models in model_dir."""
    models: list[PowerO3Model] = []
    if not os.path.isdir(model_dir):
        return models
    for fn in sorted(os.listdir(model_dir)):
        if fn.endswith(".json") and fn.startswith("power_o3_"):
            try:
                models.append(load_model(os.path.join(model_dir, fn)))
            except Exception:
                pass
    return models


# =============================================================================
# Prediction dispatch (model-aware, with hardcoded fallback)
# =============================================================================
# Fallback constants (original piecewise model)
_FALLBACK_A = 1.78
_FALLBACK_B = 1.40


def predict_o3(
    power_pct: float,
    flow_lpm: float,
    model: Optional[PowerO3Model] = None,
) -> float:
    """Predict O3 from power, using fitted model if available.

    Falls back to the original piecewise model (POWER_MODEL_A/B) if no
    fitted model is provided or the model is invalid.
    """
    if model is not None and model.is_valid:
        return model.predict(power_pct)
    return _fallback_predict(power_pct, flow_lpm)


def predict_power(
    o3_pct: float,
    flow_lpm: float,
    model: Optional[PowerO3Model] = None,
) -> float:
    """Predict power from O3, using fitted model if available."""
    if model is not None and model.is_valid:
        return model.predict_inverse(o3_pct)
    return _fallback_predict_inverse(o3_pct, flow_lpm)


def generate_curve(
    flow_lpm: float,
    model: Optional[PowerO3Model] = None,
    n_points: int = 101,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate power/O3 curve arrays for plotting."""
    if model is not None and model.is_valid:
        return model.generate_curve(n_points)
    pwr = np.linspace(0, 100, n_points)
    o3 = np.array([_fallback_predict(p, flow_lpm) for p in pwr])
    return pwr, o3


def _fallback_predict(power_pct: float, flow_lpm: float) -> float:
    """Original piecewise model: threshold → linear ramp → saturation."""
    if power_pct <= 0 or flow_lpm <= 0:
        return 0.0
    o3_max = _FALLBACK_A / flow_lpm + _FALLBACK_B
    if power_pct < 20:
        scaling = (power_pct / 20) * 0.1
    elif power_pct <= 75:
        scaling = 0.1 + (power_pct - 20) / 55 * 0.9
    else:
        scaling = 1.0
    return o3_max * scaling


def _fallback_predict_inverse(o3_pct: float, flow_lpm: float) -> float:
    """Inverse of the piecewise fallback model."""
    if o3_pct <= 0 or flow_lpm <= 0:
        return 0.0
    o3_max = _FALLBACK_A / flow_lpm + _FALLBACK_B
    scaling = o3_pct / o3_max
    if scaling >= 1.0:
        return 100.0
    elif scaling <= 0.1:
        return (scaling / 0.1) * 20
    else:
        return 20 + (scaling - 0.1) / 0.9 * 55
