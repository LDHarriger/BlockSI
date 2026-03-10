"""
CSTR model with first-order O3 decay: fitting, persistence, and prediction.

Models the ozone concentration dynamics in the vessel as a CSTR
(Continuously Stirred Tank Reactor) with a first-order decay term:

  dC/dt = (C_in - C) / τ  -  k_d · C

Fill solution:
  C(t) = C_ss · (1 - exp(-(t - t_d) / τ_eff))       for t > t_d

  where  C_ss   = C_in / (1 + k_d · τ)               (decay-suppressed plateau)
         τ_eff  = τ / (1 + k_d · τ)                   (effective time constant)

Evac solution (C_in = 0, flow continues):
  C(t) = C_ss · exp(-(t - t_d) / τ_drain)            for t > t_d

  where  τ_drain = τ_evac / (1 + k_d · τ_evac)

Fundamental physical parameters (flow-rate independent):
  V          – system gas volume (L)
  k_d        – first-order O3 decay rate (s⁻¹)
  V_dead     – dead volume in plumbing (L)

Derived per-condition:
  τ          = V / Q
  τ_eff      = τ / (1 + k_d · τ)
  C_ss       = C_in / (1 + k_d · τ)
  t_d        = V_dead / Q

One calibration run (without air compressor, for best k_d sensitivity)
extracts V, k_d, and V_dead.  These generalise to any flow rate and air
compressor configuration via the CSTR equations above.

Model saved as JSON in ``Models/CSTR/cstr_model.json``.
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
# CSTR curve functions (decay-aware)
# =============================================================================
def _cstr_fill(
    t: np.ndarray, c_ss: float, tau_eff: float, t_delay: float
) -> np.ndarray:
    """CSTR fill with decay: C(t) = C_ss · (1 − exp(−(t − t_d)/τ_eff)).

    Returns 0 for t ≤ t_delay.
    """
    result = np.zeros_like(t, dtype=float)
    mask = t > t_delay
    result[mask] = c_ss * (1.0 - np.exp(-(t[mask] - t_delay) / tau_eff))
    return result


def _cstr_evac(
    t: np.ndarray, c_ss: float, tau_drain: float, t_delay: float
) -> np.ndarray:
    """CSTR evacuation (washout): C(t) = C_ss · exp(−(t − t_d)/τ_drain).

    Returns C_ss for t ≤ t_delay.
    """
    result = np.full_like(t, c_ss, dtype=float)
    mask = t > t_delay
    result[mask] = c_ss * np.exp(-(t[mask] - t_delay) / tau_drain)
    return result


# =============================================================================
# Data class
# =============================================================================
@dataclass
class CSTRModel:
    """Fitted CSTR model with first-order O3 decay.

    Fundamental parameters (flow-rate independent):
        system_volume_L  – vessel gas volume
        decay_rate_per_s – first-order O3 decay constant k_d
        dead_volume_L    – plumbing dead volume between generator and vessel

    These generalise to any flow rate / air compressor configuration.
    """

    # Fundamental physical parameters (flow-independent)
    system_volume_L: float = 0.0
    decay_rate_per_s: float = 0.0
    dead_volume_L: float = 0.0

    # Fitted curve parameters (at calibration flow rate)
    c_ss_fitted_pct: float = 0.0        # fitted fill asymptote (%vol)
    tau_eff_fill_s: float = 0.0         # effective fill time constant
    tau_physical_fill_s: float = 0.0    # physical τ = V/Q (no decay)
    transport_delay_fill_s: float = 0.0
    tau_drain_evac_s: float = 0.0       # fitted evac time constant
    transport_delay_evac_s: float = 0.0

    # Inlet concentration used for fitting (from power model at 100%)
    c_in_pct: float = 0.0

    # Cross-check: k_d derived independently from evac curve
    decay_rate_evac_per_s: float = 0.0

    # Calibration condition
    calibration_flow_lpm: float = 4.0
    calibration_vessel_temp_c: float = -999.0

    # Fit quality
    r_squared_fill: float = 0.0
    r_squared_evac: float = 0.0
    rmse_fill: float = 0.0
    rmse_evac: float = 0.0
    n_points_fill: int = 0
    n_points_evac: int = 0

    # Provenance
    data_file: str = ""
    fitted_at: str = ""
    notes: str = ""

    # -- predictions -------------------------------------------------------
    def predict_fill(
        self, t: float | np.ndarray, flow_lpm: float | None = None,
        c_in_pct: float | None = None,
    ) -> float | np.ndarray:
        """Predict O3 (%vol) during fill at time *t* (seconds).

        If flow_lpm / c_in_pct are given, uses fundamental parameters to
        predict at a different flow rate.  Otherwise uses calibration values.
        """
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        if flow_lpm is not None:
            q = flow_lpm / 60.0
            tau = self.system_volume_L / q if q > 0 else 9999.0
            t_d = self.dead_volume_L / q if q > 0 else 0.0
            c_in = c_in_pct if c_in_pct is not None else self.c_in_pct
            c_ss = c_in / (1.0 + self.decay_rate_per_s * tau)
            tau_eff = tau / (1.0 + self.decay_rate_per_s * tau)
        else:
            c_ss = self.c_ss_fitted_pct
            tau_eff = self.tau_eff_fill_s
            t_d = self.transport_delay_fill_s
        result = _cstr_fill(t_arr, c_ss, tau_eff, t_d)
        return float(result[0]) if np.ndim(t) == 0 else result

    def predict_evac(
        self, t: float | np.ndarray, flow_lpm: float | None = None,
        c_ss_pct: float | None = None,
    ) -> float | np.ndarray:
        """Predict O3 (%vol) during evacuation at time *t* (seconds).

        If flow_lpm is given, computes τ_drain for that flow rate (e.g. with
        air compressor).  Otherwise uses calibration evac τ_drain.
        """
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        c_ss = c_ss_pct if c_ss_pct is not None else self.c_ss_fitted_pct
        if flow_lpm is not None:
            q = flow_lpm / 60.0
            tau_evac = self.system_volume_L / q if q > 0 else 9999.0
            tau_drain = tau_evac / (1.0 + self.decay_rate_per_s * tau_evac)
            t_d = self.dead_volume_L / q if q > 0 else 0.0
        else:
            tau_drain = self.tau_drain_evac_s
            t_d = self.transport_delay_evac_s
        result = _cstr_evac(t_arr, c_ss, tau_drain, t_d)
        return float(result[0]) if np.ndim(t) == 0 else result

    @property
    def is_valid(self) -> bool:
        return (
            self.system_volume_L > 0
            and self.tau_eff_fill_s > 0
            and np.isfinite(self.system_volume_L)
            and np.isfinite(self.decay_rate_per_s)
            and self.r_squared_fill > 0
            and self.r_squared_evac > 0
        )

    def summary(self) -> str:
        return (
            f"CSTR: V={self.system_volume_L:.2f}L, "
            f"k_d={self.decay_rate_per_s:.6f}/s, "
            f"V_dead={self.dead_volume_L:.3f}L, "
            f"C_ss={self.c_ss_fitted_pct:.3f}% (C_in={self.c_in_pct:.3f}%), "
            f"cal@{self.calibration_flow_lpm}LPM "
            f"(R²={self.r_squared_fill:.4f}/{self.r_squared_evac:.4f})"
        )


# Backward-compatible alias
FillModel = CSTRModel


# =============================================================================
# CSV loading
# =============================================================================
def load_cstr_csv(filepath: str) -> pd.DataFrame:
    """Load a CSTR calibration CSV (combined baseline+fill+evac phases).

    Expected columns:
        timestamp, esp_ts_ms, elapsed_s, vessel_o3_pct, cell_temp_c,
        vessel_temp_c, power_pct, phase

    Returns DataFrame with ``elapsed_s`` and ``phase`` columns.
    """
    df = pd.read_csv(filepath)
    df.columns = [c.strip() for c in df.columns]

    # Compute elapsed time from esp_ts_ms if not present
    if "elapsed_s" not in df.columns and "esp_ts_ms" in df.columns:
        t0 = df["esp_ts_ms"].iloc[0]
        df["elapsed_s"] = (df["esp_ts_ms"] - t0) / 1000.0

    df["source_file"] = os.path.basename(filepath)
    return df


# Keep old name as alias
load_fill_csv = load_cstr_csv


# =============================================================================
# Fitting
# =============================================================================
def fit_fill_curve(
    elapsed_s: np.ndarray,
    o3_pct: np.ndarray,
    c_in_pct: float,
    tau_guess: float = 60.0,
) -> tuple[float, float, float, float, float]:
    """Fit decay-aware CSTR fill curve: C(t) = C_ss · (1 − exp(−(t−t_d)/τ_eff)).

    Free parameters: (C_ss, τ_eff, t_d).  c_in_pct is used post-fit to
    derive k_d and τ_physical.

    Returns:
        (c_ss, tau_eff, t_delay, r_squared, rmse)
    """
    if not HAS_SCIPY:
        raise RuntimeError("scipy required for model fitting")

    if len(elapsed_s) < 5:
        raise ValueError(f"Need ≥5 points for fill fit, got {len(elapsed_s)}")

    t = np.asarray(elapsed_s, dtype=float)
    y = np.asarray(o3_pct, dtype=float)

    # Initial guesses: C_ss near max observed, τ_eff from tau_guess
    c_ss_guess = min(float(np.max(y)) * 1.02, c_in_pct)

    popt, _ = curve_fit(
        _cstr_fill, t, y,
        p0=[c_ss_guess, tau_guess, 2.0],
        bounds=([0.01, 1.0, 0.0], [c_in_pct * 1.1, 3600.0, 60.0]),
        maxfev=10000,
    )
    c_ss, tau_eff, t_delay = popt

    y_pred = _cstr_fill(t, c_ss, tau_eff, t_delay)
    ss_res = float(np.sum((y - y_pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r_sq = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(ss_res / len(y)))

    return float(c_ss), float(tau_eff), float(t_delay), r_sq, rmse


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
        tau_guess: Initial guess for τ_drain (seconds).

    Returns:
        (tau_drain, t_delay, r_squared, rmse)
    """
    if not HAS_SCIPY:
        raise RuntimeError("scipy required for model fitting")

    if len(elapsed_s) < 3:
        raise ValueError(f"Need ≥3 points for evac fit, got {len(elapsed_s)}")

    def _fit_fn(t, tau_drain, t_delay):
        return _cstr_evac(t, c_ss_pct, tau_drain, t_delay)

    t = np.asarray(elapsed_s, dtype=float)
    y = np.asarray(o3_pct, dtype=float)

    popt, _ = curve_fit(
        _fit_fn, t, y,
        p0=[tau_guess, 2.0],
        bounds=([1.0, 0.0], [3600.0, 60.0]),
        maxfev=10000,
    )
    tau_drain, t_delay = popt

    y_pred = _fit_fn(t, tau_drain, t_delay)
    ss_res = float(np.sum((y - y_pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r_sq = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(ss_res / len(y)))

    return float(tau_drain), float(t_delay), r_sq, rmse


def fit_cstr_model(
    csv_path: str,
    flow_lpm: float,
    c_in_pct: float,
    baseline_samples: int = 15,
) -> CSTRModel:
    """Fit a complete CSTR model from a single combined CSV file.

    The CSV must have a ``phase`` column with values: baseline, fill, evac.

    Args:
        csv_path: Path to combined CSTR calibration CSV.
        flow_lpm: O2 flow rate during calibration (no air compressor).
        c_in_pct: Validated inlet O3 concentration (%vol) at 100% power.
        baseline_samples: Number of baseline samples to skip from fill start.

    Returns:
        Fitted ``CSTRModel``.  Check ``.is_valid`` to confirm success.
    """
    df = load_cstr_csv(csv_path)
    flow_Ls = flow_lpm / 60.0

    # --- Split phases ---
    df_fill = df[df["phase"] == "fill"].copy().reset_index(drop=True)
    df_evac = df[df["phase"] == "evac"].copy().reset_index(drop=True)

    if len(df_fill) < 5:
        raise ValueError(f"Need ≥5 fill samples, got {len(df_fill)}")
    if len(df_evac) < 3:
        raise ValueError(f"Need ≥3 evac samples, got {len(df_evac)}")

    # Re-zero elapsed time for each phase
    t0_fill = df_fill["elapsed_s"].iloc[0]
    df_fill["elapsed_s"] = df_fill["elapsed_s"] - t0_fill

    t0_evac = df_evac["elapsed_s"].iloc[0]
    df_evac["elapsed_s"] = df_evac["elapsed_s"] - t0_evac

    # Mean vessel temperature during fill (for metadata)
    vessel_temp = -999.0
    if "vessel_temp_c" in df_fill.columns:
        temps = df_fill["vessel_temp_c"]
        valid_temps = temps[temps > -900]
        if len(valid_temps) > 0:
            vessel_temp = float(valid_temps.mean())

    # Guess τ from flow and a ~10L system
    tau_guess = 10.0 / flow_Ls if flow_Ls > 0 else 150.0

    # --- Fit fill curve (free params: C_ss, τ_eff, t_d) ---
    c_ss, tau_eff_f, td_f, r2_f, rmse_f = fit_fill_curve(
        df_fill["elapsed_s"].values,
        df_fill["vessel_o3_pct"].values,
        c_in_pct,
        tau_guess=tau_guess,
    )

    # Derive fundamental parameters from fill fit
    # τ_physical = τ_eff × (C_in / C_ss)
    ratio = c_in_pct / c_ss if c_ss > 0.001 else 1.0
    tau_physical = tau_eff_f * ratio
    # k_d = (C_in - C_ss) / (C_in × τ_eff)
    k_d = (c_in_pct - c_ss) / (c_in_pct * tau_eff_f) if c_in_pct > 0.001 and tau_eff_f > 0 else 0.0
    # V = τ_physical × Q
    volume = tau_physical * flow_Ls
    # V_dead = t_d × Q
    dead_vol = td_f * flow_Ls

    # --- Fit evac curve ---
    # Use fitted C_ss as the starting concentration
    tau_drain_e, td_e, r2_e, rmse_e = fit_evac_curve(
        df_evac["elapsed_s"].values,
        df_evac["vessel_o3_pct"].values,
        c_ss,
        tau_guess=tau_guess,
    )

    # Cross-check: k_d from evac
    # τ_drain = τ_evac_physical / (1 + k_d × τ_evac_physical)
    # At same flow rate: τ_evac_physical = τ_physical = V/Q
    # k_d_evac = 1/τ_drain - Q/V = 1/τ_drain - 1/τ_physical
    k_d_evac = 0.0
    if tau_drain_e > 0 and tau_physical > 0:
        k_d_evac = 1.0 / tau_drain_e - 1.0 / tau_physical

    return CSTRModel(
        system_volume_L=round(volume, 3),
        decay_rate_per_s=round(k_d, 8),
        dead_volume_L=round(dead_vol, 4),
        c_ss_fitted_pct=round(c_ss, 6),
        tau_eff_fill_s=round(tau_eff_f, 3),
        tau_physical_fill_s=round(tau_physical, 3),
        transport_delay_fill_s=round(td_f, 3),
        tau_drain_evac_s=round(tau_drain_e, 3),
        transport_delay_evac_s=round(td_e, 3),
        c_in_pct=round(c_in_pct, 6),
        decay_rate_evac_per_s=round(k_d_evac, 8),
        calibration_flow_lpm=flow_lpm,
        calibration_vessel_temp_c=round(vessel_temp, 1),
        r_squared_fill=round(r2_f, 6),
        r_squared_evac=round(r2_e, 6),
        rmse_fill=round(rmse_f, 6),
        rmse_evac=round(rmse_e, 6),
        n_points_fill=len(df_fill),
        n_points_evac=len(df_evac),
        data_file=os.path.basename(csv_path),
        fitted_at=datetime.now().isoformat(timespec="seconds"),
    )


# Backward-compatible alias
fit_fill_model = fit_cstr_model


# =============================================================================
# Persistence  (JSON in Models/CSTR/)
# =============================================================================
CSTR_MODEL_FILENAME = "cstr_model.json"


def save_cstr_model(model: CSTRModel, model_dir: str) -> str:
    """Save CSTR model to JSON. Returns full path."""
    os.makedirs(model_dir, exist_ok=True)
    fpath = os.path.join(model_dir, CSTR_MODEL_FILENAME)

    data = asdict(model)
    data["type"] = "cstr_decay"
    data["version"] = 2

    with open(fpath, "w") as f:
        json.dump(data, f, indent=2)

    return fpath


# Backward-compatible alias
save_fill_model = save_cstr_model


def load_cstr_model(filepath: str) -> CSTRModel:
    """Load a CSTRModel from JSON."""
    with open(filepath) as f:
        data = json.load(f)

    data.pop("type", None)
    data.pop("version", None)

    # Handle old-format models (version 1) gracefully
    known_fields = {f.name for f in CSTRModel.__dataclass_fields__.values()}
    filtered = {k: v for k, v in data.items() if k in known_fields}

    return CSTRModel(**filtered)


# Backward-compatible alias
load_fill_model = load_cstr_model


def load_cstr_model_from_dir(model_dir: str) -> Optional[CSTRModel]:
    """Load the CSTR model from a directory, or None if not found."""
    fpath = os.path.join(model_dir, CSTR_MODEL_FILENAME)
    if os.path.exists(fpath):
        try:
            return load_cstr_model(fpath)
        except Exception:
            return None
    return None


# Backward-compatible aliases
load_fill_model_for_condition = lambda model_dir, flow_lpm=None, o2_pct=None: load_cstr_model_from_dir(model_dir)


def list_cstr_models(model_dir: str) -> list[CSTRModel]:
    """List all CSTR models in model_dir."""
    models: list[CSTRModel] = []
    if not os.path.isdir(model_dir):
        return models
    for fn in sorted(os.listdir(model_dir)):
        if fn.endswith(".json"):
            try:
                models.append(load_cstr_model(os.path.join(model_dir, fn)))
            except Exception:
                pass
    return models


# Backward-compatible alias
list_fill_models = list_cstr_models
