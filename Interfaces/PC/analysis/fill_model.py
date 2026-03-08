"""
Fill / Evacuation CSTR model: fitting, persistence, and prediction.

Models the ozone concentration dynamics in the vessel as a CSTR
(Continuously Stirred Tank Reactor):

  Fill:  C_out(t) = C_in × (1 - exp(-(t - t_d) / τ))   for t > t_d
  Evac:  C_out(t) = C_ss × exp(-(t - t_d) / τ)          for t > t_d

Parameters:
    τ    – mixing time constant (s), equals V_system / F  (L / (L/s))
    t_d  – transport delay (s), dead volume in plumbing
    C_in – validated inlet O3 concentration (%vol)
    C_ss – steady-state concentration at evacuation start (%vol)

The model determines:
    V_system = τ × F        total system volume (vessel + plumbing)
    t_transit = t_d          mean transport delay through plumbing

Each model applies to a specific (flow_lpm, o2_pct) operating condition
with noted air compressor states for fill and evacuation phases.
Models are saved as JSON in ``Models/Fill/``.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, asdict
from datetime import datetime
from typing import Optional

import numpy as np
import pandas as pd

try:
    from scipy.optimize import curve_fit

    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# =============================================================================
# CSTR curve functions
# =============================================================================
def _cstr_fill(
    t: np.ndarray, c_in: float, tau: float, t_delay: float
) -> np.ndarray:
    """CSTR fill response: C_out(t) = C_in × (1 − exp(−(t − t_d)/τ)).

    Returns 0 for t ≤ t_delay.
    """
    result = np.zeros_like(t, dtype=float)
    mask = t > t_delay
    result[mask] = c_in * (1.0 - np.exp(-(t[mask] - t_delay) / tau))
    return result


def _cstr_evac(
    t: np.ndarray, c_ss: float, tau: float, t_delay: float
) -> np.ndarray:
    """CSTR evacuation (washout): C_out(t) = C_ss × exp(−(t − t_d)/τ).

    Returns C_ss for t ≤ t_delay.
    """
    result = np.full_like(t, c_ss, dtype=float)
    mask = t > t_delay
    result[mask] = c_ss * np.exp(-(t[mask] - t_delay) / tau)
    return result


# Wrappers for curve_fit (c_in / c_ss fixed externally)
def _fill_for_fit(t: np.ndarray, tau: float, t_delay: float, c_in: float = 1.0):
    return _cstr_fill(t, c_in, tau, t_delay)


def _evac_for_fit(t: np.ndarray, tau: float, t_delay: float, c_ss: float = 1.0):
    return _cstr_evac(t, c_ss, tau, t_delay)


# =============================================================================
# Data class
# =============================================================================
@dataclass
class FillModel:
    """Fitted CSTR fill/evacuation model for a given operating condition."""

    # CSTR parameters
    system_volume_L: float = 0.0
    tau_fill_s: float = 0.0
    tau_evac_s: float = 0.0
    transport_delay_fill_s: float = 0.0
    transport_delay_evac_s: float = 0.0

    # Inlet / steady-state concentrations used for fitting
    c_in_pct: float = 0.0           # validated inlet O3 (%vol)
    c_ss_pct: float = 0.0           # measured steady-state O3 (%vol)

    # Operating condition
    flow_lpm: float = 4.0
    o2_pct: int = 95
    air_comp_fill: bool = False
    air_comp_evac: bool = True

    # Fit quality
    r_squared_fill: float = 0.0
    r_squared_evac: float = 0.0
    rmse_fill: float = 0.0
    rmse_evac: float = 0.0
    n_points_fill: int = 0
    n_points_evac: int = 0

    # Provenance
    fill_data_file: str = ""
    evac_data_file: str = ""
    fitted_at: str = ""
    notes: str = ""

    # -- predictions -------------------------------------------------------
    def predict_fill(self, t: float | np.ndarray) -> float | np.ndarray:
        """Predict outlet O3 (%vol) during fill at time *t* (seconds)."""
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        result = _cstr_fill(t_arr, self.c_in_pct, self.tau_fill_s,
                            self.transport_delay_fill_s)
        return float(result[0]) if np.ndim(t) == 0 else result

    def predict_evac(self, t: float | np.ndarray) -> float | np.ndarray:
        """Predict outlet O3 (%vol) during evacuation at time *t* (seconds)."""
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        result = _cstr_evac(t_arr, self.c_ss_pct, self.tau_evac_s,
                            self.transport_delay_evac_s)
        return float(result[0]) if np.ndim(t) == 0 else result

    @property
    def is_valid(self) -> bool:
        return (
            self.tau_fill_s > 0
            and self.tau_evac_s > 0
            and np.isfinite(self.tau_fill_s)
            and np.isfinite(self.tau_evac_s)
            and self.r_squared_fill > 0
            and self.r_squared_evac > 0
        )

    @property
    def condition_key(self) -> tuple[float, int]:
        return (self.flow_lpm, self.o2_pct)

    @property
    def mean_tau_s(self) -> float:
        """Average of fill and evacuation τ values."""
        return (self.tau_fill_s + self.tau_evac_s) / 2.0

    def summary(self) -> str:
        return (
            f"{self.flow_lpm} LPM / {self.o2_pct}% O2: "
            f"V={self.system_volume_L:.2f} L, "
            f"τ_fill={self.tau_fill_s:.1f}s, τ_evac={self.tau_evac_s:.1f}s, "
            f"t_d_fill={self.transport_delay_fill_s:.1f}s, "
            f"t_d_evac={self.transport_delay_evac_s:.1f}s "
            f"(R²_fill={self.r_squared_fill:.4f}, "
            f"R²_evac={self.r_squared_evac:.4f})"
        )


# =============================================================================
# CSV loading
# =============================================================================
def load_fill_csv(filepath: str) -> pd.DataFrame:
    """Load a fill or evacuation phase CSV.

    Expected columns (same as telemetry stream):
        timestamp, esp_ts_ms, vessel_o3_pct, cell_temp_c, pressure_mbar,
        room_o3_ppm, vessel_temp_c, power_target, power_actual, wiper_v

    Returns DataFrame with an added ``elapsed_s`` column (seconds from
    the first sample).
    """
    df = pd.read_csv(filepath)
    df.columns = [c.strip() for c in df.columns]

    # Compute elapsed time from esp_ts_ms (monotonic, ms since boot)
    if "esp_ts_ms" in df.columns:
        t0 = df["esp_ts_ms"].iloc[0]
        df["elapsed_s"] = (df["esp_ts_ms"] - t0) / 1000.0
    else:
        # Fallback: sequential samples at ~2 s
        df["elapsed_s"] = np.arange(len(df)) * 2.0

    df["source_file"] = os.path.basename(filepath)
    return df


# =============================================================================
# Fitting
# =============================================================================
def fit_fill_curve(
    elapsed_s: np.ndarray,
    o3_pct: np.ndarray,
    c_in_pct: float,
    tau_guess: float = 60.0,
) -> tuple[float, float, float, float]:
    """Fit CSTR fill curve to measured data.

    Args:
        elapsed_s: Time array (seconds from start of fill phase).
        o3_pct: Measured O3 concentration (%vol).
        c_in_pct: Validated inlet concentration (%vol) — held constant.
        tau_guess: Initial guess for τ (seconds).

    Returns:
        (tau, t_delay, r_squared, rmse)
    """
    if not HAS_SCIPY:
        raise RuntimeError("scipy required for model fitting")

    if len(elapsed_s) < 3:
        raise ValueError(f"Need ≥3 points for fill fit, got {len(elapsed_s)}")

    # Wrapper that fixes c_in
    def _fit_fn(t, tau, t_delay):
        return _cstr_fill(t, c_in_pct, tau, t_delay)

    t = np.asarray(elapsed_s, dtype=float)
    y = np.asarray(o3_pct, dtype=float)

    popt, _ = curve_fit(
        _fit_fn, t, y,
        p0=[tau_guess, 2.0],
        bounds=([1.0, 0.0], [600.0, 60.0]),
        maxfev=10000,
    )
    tau, t_delay = popt

    y_pred = _fit_fn(t, tau, t_delay)
    ss_res = float(np.sum((y - y_pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r_sq = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(ss_res / len(y)))

    return float(tau), float(t_delay), r_sq, rmse


def fit_evac_curve(
    elapsed_s: np.ndarray,
    o3_pct: np.ndarray,
    c_ss_pct: float,
    tau_guess: float = 60.0,
) -> tuple[float, float, float, float]:
    """Fit CSTR evacuation (washout) curve to measured data.

    Args:
        elapsed_s: Time array (seconds from start of evac phase).
        o3_pct: Measured O3 concentration (%vol).
        c_ss_pct: Steady-state concentration at start of evacuation (%vol).
        tau_guess: Initial guess for τ (seconds).

    Returns:
        (tau, t_delay, r_squared, rmse)
    """
    if not HAS_SCIPY:
        raise RuntimeError("scipy required for model fitting")

    if len(elapsed_s) < 3:
        raise ValueError(f"Need ≥3 points for evac fit, got {len(elapsed_s)}")

    def _fit_fn(t, tau, t_delay):
        return _cstr_evac(t, c_ss_pct, tau, t_delay)

    t = np.asarray(elapsed_s, dtype=float)
    y = np.asarray(o3_pct, dtype=float)

    popt, _ = curve_fit(
        _fit_fn, t, y,
        p0=[tau_guess, 2.0],
        bounds=([1.0, 0.0], [600.0, 60.0]),
        maxfev=10000,
    )
    tau, t_delay = popt

    y_pred = _fit_fn(t, tau, t_delay)
    ss_res = float(np.sum((y - y_pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r_sq = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(ss_res / len(y)))

    return float(tau), float(t_delay), r_sq, rmse


def fit_fill_model(
    fill_csv: str,
    evac_csv: str,
    flow_lpm: float,
    o2_pct: int,
    c_in_pct: float,
    air_comp_fill: bool = False,
    air_comp_evac: bool = True,
    baseline_samples: int = 15,
) -> FillModel:
    """Fit a complete CSTR model from fill and evacuation CSV files.

    Args:
        fill_csv: Path to fill-phase CSV.
        evac_csv: Path to evacuation-phase CSV.
        flow_lpm: O2 flow rate during the sequence.
        o2_pct: Effective O2 percentage.
        c_in_pct: Validated inlet O3 concentration (%vol) at 100% power.
        air_comp_fill: Air compressor state during fill.
        air_comp_evac: Air compressor state during evacuation.
        baseline_samples: Number of baseline samples to skip at start of
            fill CSV (collected at 0% power before fill begins).

    Returns:
        Fitted ``FillModel``.  Check ``.is_valid`` to confirm success.
    """
    # --- Load and prepare fill data ---
    df_fill = load_fill_csv(fill_csv)
    # Skip baseline samples (0% power phase at start)
    df_fill = df_fill.iloc[baseline_samples:].reset_index(drop=True)
    # Re-zero elapsed time
    if len(df_fill) > 0 and "elapsed_s" in df_fill.columns:
        t0 = df_fill["elapsed_s"].iloc[0]
        df_fill["elapsed_s"] = df_fill["elapsed_s"] - t0

    # Guess τ from flow and a ~10L system
    tau_guess = 10.0 / (flow_lpm / 60.0)

    tau_f, td_f, r2_f, rmse_f = fit_fill_curve(
        df_fill["elapsed_s"].values,
        df_fill["vessel_o3_pct"].values,
        c_in_pct,
        tau_guess=tau_guess,
    )

    # --- Load and prepare evac data ---
    df_evac = load_fill_csv(evac_csv)
    # Steady-state = mean of first 5 samples (should be at C_ss)
    c_ss = float(df_evac["vessel_o3_pct"].iloc[:5].mean())

    tau_e, td_e, r2_e, rmse_e = fit_evac_curve(
        df_evac["elapsed_s"].values,
        df_evac["vessel_o3_pct"].values,
        c_ss,
        tau_guess=tau_guess,
    )

    # System volume from mean τ
    mean_tau = (tau_f + tau_e) / 2.0
    flow_Ls = flow_lpm / 60.0
    system_vol = mean_tau * flow_Ls

    return FillModel(
        system_volume_L=round(system_vol, 3),
        tau_fill_s=round(tau_f, 3),
        tau_evac_s=round(tau_e, 3),
        transport_delay_fill_s=round(td_f, 3),
        transport_delay_evac_s=round(td_e, 3),
        c_in_pct=round(c_in_pct, 6),
        c_ss_pct=round(c_ss, 6),
        flow_lpm=flow_lpm,
        o2_pct=o2_pct,
        air_comp_fill=air_comp_fill,
        air_comp_evac=air_comp_evac,
        r_squared_fill=round(r2_f, 6),
        r_squared_evac=round(r2_e, 6),
        rmse_fill=round(rmse_f, 6),
        rmse_evac=round(rmse_e, 6),
        n_points_fill=len(df_fill),
        n_points_evac=len(df_evac),
        fill_data_file=os.path.basename(fill_csv),
        evac_data_file=os.path.basename(evac_csv),
        fitted_at=datetime.now().isoformat(timespec="seconds"),
    )


# =============================================================================
# Persistence  (JSON in Models/Fill/)
# =============================================================================
def _fill_model_filename(flow_lpm: float, o2_pct: int) -> str:
    lpm_s = f"{flow_lpm:.0f}" if flow_lpm == int(flow_lpm) else f"{flow_lpm:.1f}"
    return f"fill_model_{lpm_s}lpm_{o2_pct}o2.json"


def save_fill_model(model: FillModel, model_dir: str) -> str:
    """Save fill model to JSON. Returns full path."""
    os.makedirs(model_dir, exist_ok=True)
    fname = _fill_model_filename(model.flow_lpm, model.o2_pct)
    fpath = os.path.join(model_dir, fname)

    data = asdict(model)
    data["type"] = "cstr_fill_evac"
    data["version"] = 1

    with open(fpath, "w") as f:
        json.dump(data, f, indent=2)

    return fpath


def load_fill_model(filepath: str) -> FillModel:
    """Load a FillModel from JSON."""
    with open(filepath) as f:
        data = json.load(f)

    data.pop("type", None)
    data.pop("version", None)

    return FillModel(**data)


def load_fill_model_for_condition(
    model_dir: str, flow_lpm: float, o2_pct: int
) -> Optional[FillModel]:
    """Load the fill model matching (flow_lpm, o2_pct), or None."""
    fname = _fill_model_filename(flow_lpm, o2_pct)
    fpath = os.path.join(model_dir, fname)
    if os.path.exists(fpath):
        try:
            return load_fill_model(fpath)
        except Exception:
            return None
    return None


def list_fill_models(model_dir: str) -> list[FillModel]:
    """List all saved fill models in model_dir."""
    models: list[FillModel] = []
    if not os.path.isdir(model_dir):
        return models
    for fn in sorted(os.listdir(model_dir)):
        if fn.endswith(".json") and fn.startswith("fill_model_"):
            try:
                models.append(load_fill_model(os.path.join(model_dir, fn)))
            except Exception:
                pass
    return models
