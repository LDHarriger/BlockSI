"""
k_abs calibration model — CSTR with loaded substrate, first-order O3 absorption.

Models ozone concentration dynamics in a vessel loaded with sterilization
substrate (e.g. mushroom spawn, grain jars).  Two mechanisms consume O3:

  1. k_d  – first-order wall/gas-phase decay (already calibrated, fixed input)
  2. k_abs – first-order absorption into substrate (NEW, fitted here)

The loaded vessel has a smaller free gas volume than the empty vessel:
  V_residual < V_vessel    (substrate displaces some gas volume)

CSTR dynamics for fill (C_in supplied at flow Q):
  k_total  = k_d + k_abs
  τ_loaded = V_residual / Q
  C_ss     = C_in / (1 + k_total · τ_loaded)
  τ_eff    = τ_loaded / (1 + k_total · τ_loaded)
  C(t)     = C_ss · (1 − exp(−t / τ_eff))

Two models are compared:

  1-param  Single k_abs (homogeneous substrate, V_residual also fitted →
           effectively 2 degrees of freedom against fill-curve shape).
           Reported as "1-param" for user clarity.

  2-param biphasic  Fast surface absorption decays quickly; bulk absorption
           continues at k_abs_slow.  Fraction α ∈ (0,1) is fast component:
           k_abs_eff(t) = α·k_abs_fast + (1-α)·k_abs_slow
           Implemented as superposition of two CSTR responses:
           C(t) = α·C_fast(t) + (1-α)·C_slow(t)
           Each component uses its own k_total and τ_eff.

Model selection uses AIC / BIC.  The biphasic model is preferred only if
ΔAIC > 4 and ΔBIC > 0 (conservative threshold to avoid over-fitting).

Fixed constants (imported from cstr_k_d_model):
  V_vessel = 9.27 L   (water displacement)
  V_dead   = 0.020 L  (plumbing dead volume)

Free parameters — 1-param fit:
  V_residual ∈ [0.1, V_vessel]  (L)
  k_abs      ∈ [0, 0.1]         (s⁻¹)

Free parameters — 2-param (biphasic) fit:
  V_residual, k_abs_fast, k_abs_slow, fraction_fast (all bounded)

Model saved as timestamped JSON in Models/cstr_k_abs/.
"""
from __future__ import annotations

import json
import logging
import math
import os
import re
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Optional, Tuple

import numpy as np
import pandas as pd

try:
    from scipy.optimize import minimize, differential_evolution
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

# Import fixed geometry constants from the k_d module
from analysis.cstr_k_d_model import V_VESSEL_L, V_DEAD_L

logger = logging.getLogger(__name__)


# =============================================================================
# AIC / BIC helpers
# =============================================================================
def _aic(n: int, k: int, sse: float) -> float:
    """Akaike Information Criterion (assuming Gaussian errors).

    AIC = n·ln(SSE/n) + 2k
    """
    if n <= 0 or sse <= 0:
        return float("inf")
    return n * math.log(sse / n) + 2 * k


def _bic(n: int, k: int, sse: float) -> float:
    """Bayesian Information Criterion (assuming Gaussian errors).

    BIC = n·ln(SSE/n) + k·ln(n)
    """
    if n <= 0 or sse <= 0:
        return float("inf")
    return n * math.log(sse / n) + k * math.log(n)


def _r_squared(y: np.ndarray, y_pred: np.ndarray) -> float:
    ss_res = float(np.sum((y - y_pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0


def _rmse(y: np.ndarray, y_pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y - y_pred) ** 2)))


# =============================================================================
# CSTR curve functions (loaded vessel)
# =============================================================================
def _fill_loaded(
    t: np.ndarray,
    v_residual: float,
    k_total: float,
    c_in: float,
    flow_lpm: float,
) -> np.ndarray:
    """Single-component CSTR fill for a loaded vessel.

    Args:
        t: Time array (seconds, measured from fill start).
        v_residual: Free gas volume (L) when substrate is loaded.
        k_total: k_d + k_abs (s⁻¹).
        c_in: Inlet O3 concentration (%vol).
        flow_lpm: Volumetric flow rate (L/min).

    Returns:
        Predicted O3 concentration array (%vol).
    """
    q = flow_lpm / 60.0
    tau = v_residual / q if q > 0 else 9999.0
    denom = 1.0 + k_total * tau
    c_ss = c_in / denom
    tau_eff = tau / denom
    if tau_eff <= 0:
        return np.zeros_like(t, dtype=float)
    return c_ss * (1.0 - np.exp(-t / tau_eff))


def _evac_loaded(
    t: np.ndarray,
    v_residual: float,
    k_total: float,
    c_ss: float,
    flow_lpm: float,
) -> np.ndarray:
    """Single-component CSTR evacuation (washout) for a loaded vessel.

    C_in = 0, flow continues.

    Args:
        t: Time array (seconds, measured from evac start).
        v_residual: Free gas volume (L) when substrate is loaded.
        k_total: k_d + k_abs (s⁻¹).
        c_ss: Concentration at evac start (%vol).
        flow_lpm: Volumetric flow rate (L/min).

    Returns:
        Predicted O3 concentration array (%vol).
    """
    q = flow_lpm / 60.0
    tau = v_residual / q if q > 0 else 9999.0
    denom = 1.0 + k_total * tau
    tau_drain = tau / denom
    if tau_drain <= 0:
        return np.zeros_like(t, dtype=float)
    return c_ss * np.exp(-t / tau_drain)


def _fill_biphasic(
    t: np.ndarray,
    v_residual: float,
    k_d: float,
    k_abs_fast: float,
    k_abs_slow: float,
    fraction_fast: float,
    c_in: float,
    flow_lpm: float,
) -> np.ndarray:
    """Biphasic fill: superposition of fast and slow absorption components.

    C(t) = α·C_fast(t) + (1-α)·C_slow(t)

    where C_fast uses k_total_fast = k_d + k_abs_fast and C_slow uses
    k_total_slow = k_d + k_abs_slow.  The effective inlet concentration is
    split proportionally so that C(∞) converges correctly.
    """
    alpha = max(0.0, min(1.0, fraction_fast))
    c_fast = _fill_loaded(t, v_residual, k_d + k_abs_fast, c_in, flow_lpm)
    c_slow = _fill_loaded(t, v_residual, k_d + k_abs_slow, c_in, flow_lpm)
    return alpha * c_fast + (1.0 - alpha) * c_slow


def _evac_biphasic(
    t: np.ndarray,
    v_residual: float,
    k_d: float,
    k_abs_fast: float,
    k_abs_slow: float,
    fraction_fast: float,
    c_ss: float,
    flow_lpm: float,
) -> np.ndarray:
    """Biphasic evac: superposition of fast and slow drainage components."""
    alpha = max(0.0, min(1.0, fraction_fast))
    c_fast = _evac_loaded(t, v_residual, k_d + k_abs_fast, c_ss, flow_lpm)
    c_slow = _evac_loaded(t, v_residual, k_d + k_abs_slow, c_ss, flow_lpm)
    return alpha * c_fast + (1.0 - alpha) * c_slow


# =============================================================================
# Data class
# =============================================================================
@dataclass
class KAbsModel:
    """Fitted k_abs model for a loaded CSTR vessel.

    Fixed inputs (from prior calibrations):
        k_d_empty_per_s  – loaded from the most-recent cstr_k_d JSON
        c_in_pct         – from power-O3 model at 100% power
        calibration_flow_lpm – measured flow used during this calibration run

    Fitted parameters (1-param model):
        V_residual_L     – free gas volume with substrate loaded (L)
        k_abs_per_s      – first-order O3 absorption rate into substrate (s⁻¹)

    Derived:
        loaded_material_density_kg_per_L – kg_substrate / (V_vessel − V_residual)
        c_ss_loaded_pct  – steady-state O3 at calibration conditions
        tau_eff_loaded_s – effective CSTR time constant (fill)
        tau_physical_loaded_s – V_residual / Q (physical residence time)
        t_fill_99        – time to reach 99% of C_ss (−τ_eff · ln(0.01)) (s)

    Optional biphasic (2-param) model:
        k_abs_fast, k_abs_slow, fraction_fast
        aic_2param, bic_2param
        preferred_model  – "1param" or "2param"
    """

    # ---------- fixed geometry (always set) -----------------------------------
    v_vessel_L: float = V_VESSEL_L
    v_dead_L: float = V_DEAD_L

    # ---------- provenance inputs ---------------------------------------------
    k_d_empty_per_s: float = 0.0
    c_in_pct: float = 0.0
    calibration_flow_lpm: float = 4.0
    kg_substrate: float = 0.0

    # ---------- 1-param fitted parameters ------------------------------------
    V_residual_L: float = 0.0
    k_abs_per_s: float = 0.0

    # ---------- derived / display fields -------------------------------------
    loaded_material_density_kg_per_L: float = 0.0
    c_ss_loaded_pct: float = 0.0
    tau_eff_loaded_s: float = 0.0
    tau_physical_loaded_s: float = 0.0
    t_fill_99: float = 0.0          # time to 99% C_ss (seconds)

    # ---------- fit quality (1-param) ----------------------------------------
    r_squared: float = 0.0
    rmse: float = 0.0
    n_points: int = 0
    aic_1param: float = 0.0
    bic_1param: float = 0.0

    # ---------- 2-param biphasic (optional, None if not fitted) ---------------
    k_abs_fast: Optional[float] = None
    k_abs_slow: Optional[float] = None
    fraction_fast: Optional[float] = None
    aic_2param: Optional[float] = None
    bic_2param: Optional[float] = None
    preferred_model: str = "1param"  # "1param" or "2param"

    # ---------- provenance ----------------------------------------------------
    data_file: str = ""
    fitted_at: str = ""
    notes: str = ""

    # ==========================================================================
    # Derived property
    # ==========================================================================
    @property
    def is_valid(self) -> bool:
        """True if the model was successfully fitted with reasonable parameters."""
        return (
            self.V_residual_L > 0.0
            and self.k_abs_per_s >= 0.0
            and self.tau_eff_loaded_s > 0.0
            and np.isfinite(self.V_residual_L)
            and np.isfinite(self.k_abs_per_s)
            and self.r_squared > 0.0
            and self.n_points >= 5
        )

    # ==========================================================================
    # Prediction methods
    # ==========================================================================
    def predict_fill(
        self,
        t: float | np.ndarray,
        flow_lpm: float | None = None,
        c_in_pct: float | None = None,
        use_biphasic: bool | None = None,
    ) -> float | np.ndarray:
        """Predict O3 (%vol) during fill at time *t* (seconds from fill start).

        Args:
            t: Time scalar or array (seconds).
            flow_lpm: Override flow rate; uses calibration_flow_lpm if None.
            c_in_pct: Override inlet concentration; uses fitted c_in_pct if None.
            use_biphasic: Force biphasic model (True/False); uses preferred_model
                if None.

        Returns:
            Scalar or array matching shape of *t*.
        """
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        q_lpm = flow_lpm if flow_lpm is not None else self.calibration_flow_lpm
        c_in = c_in_pct if c_in_pct is not None else self.c_in_pct

        biphasic = use_biphasic if use_biphasic is not None else (
            self.preferred_model == "2param"
        )

        if (
            biphasic
            and self.k_abs_fast is not None
            and self.k_abs_slow is not None
            and self.fraction_fast is not None
        ):
            result = _fill_biphasic(
                t_arr, self.V_residual_L, self.k_d_empty_per_s,
                self.k_abs_fast, self.k_abs_slow, self.fraction_fast,
                c_in, q_lpm,
            )
        else:
            k_total = self.k_d_empty_per_s + self.k_abs_per_s
            result = _fill_loaded(t_arr, self.V_residual_L, k_total, c_in, q_lpm)

        return float(result[0]) if np.ndim(t) == 0 else result

    def predict_evac(
        self,
        t: float | np.ndarray,
        flow_lpm: float | None = None,
        c_ss_pct: float | None = None,
        use_biphasic: bool | None = None,
    ) -> float | np.ndarray:
        """Predict O3 (%vol) during evacuation at time *t* (seconds from evac start).

        Args:
            t: Time scalar or array (seconds).
            flow_lpm: Override flow rate; uses calibration_flow_lpm if None.
            c_ss_pct: Starting concentration; uses fitted c_ss_loaded_pct if None.
            use_biphasic: Force biphasic model; uses preferred_model if None.

        Returns:
            Scalar or array matching shape of *t*.
        """
        t_arr = np.atleast_1d(np.asarray(t, dtype=float))
        q_lpm = flow_lpm if flow_lpm is not None else self.calibration_flow_lpm
        c_ss = c_ss_pct if c_ss_pct is not None else self.c_ss_loaded_pct

        biphasic = use_biphasic if use_biphasic is not None else (
            self.preferred_model == "2param"
        )

        if (
            biphasic
            and self.k_abs_fast is not None
            and self.k_abs_slow is not None
            and self.fraction_fast is not None
        ):
            result = _evac_biphasic(
                t_arr, self.V_residual_L, self.k_d_empty_per_s,
                self.k_abs_fast, self.k_abs_slow, self.fraction_fast,
                c_ss, q_lpm,
            )
        else:
            k_total = self.k_d_empty_per_s + self.k_abs_per_s
            result = _evac_loaded(t_arr, self.V_residual_L, k_total, c_ss, q_lpm)

        return float(result[0]) if np.ndim(t) == 0 else result

    def summary(self) -> str:
        """Human-readable one-line summary."""
        return (
            f"k_abs: V_res={self.V_residual_L:.3f}L, "
            f"k_abs={self.k_abs_per_s:.4e}/s, "
            f"k_d={self.k_d_empty_per_s:.4e}/s (fixed), "
            f"ρ_load={self.loaded_material_density_kg_per_L:.3f}kg/L, "
            f"C_ss={self.c_ss_loaded_pct:.3f}%, "
            f"τ_eff={self.tau_eff_loaded_s:.1f}s, "
            f"t_99={self.t_fill_99:.0f}s "
            f"(R²={self.r_squared:.4f}, model={self.preferred_model})"
        )


# =============================================================================
# CSV loading
# =============================================================================
def load_k_abs_csv(filepath: str) -> pd.DataFrame:
    """Load a k_abs calibration CSV (loaded-vessel fill/evac run).

    Expected columns (same format as k_d calibration):
        timestamp, esp_ts_ms, elapsed_s, vessel_o3_pct, cell_temp_c,
        vessel_temp_c, power_pct, phase

    The 'phase' column must contain at least "fill" rows.  "evac" rows are
    optional but improve parameter identifiability.

    Returns:
        DataFrame with ``elapsed_s``, ``vessel_o3_pct``, and ``phase`` columns.

    Raises:
        FileNotFoundError: If *filepath* does not exist.
        ValueError: If required columns are missing or the file is empty.
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"k_abs CSV not found: {filepath}")

    df = pd.read_csv(filepath)
    df.columns = [c.strip() for c in df.columns]

    # Synthesise elapsed_s from esp_ts_ms if not present
    if "elapsed_s" not in df.columns:
        if "esp_ts_ms" in df.columns:
            t0 = df["esp_ts_ms"].iloc[0]
            df["elapsed_s"] = (df["esp_ts_ms"] - t0) / 1000.0
        else:
            raise ValueError(
                f"CSV '{filepath}' has neither 'elapsed_s' nor 'esp_ts_ms' column."
            )

    required = {"elapsed_s", "vessel_o3_pct", "phase"}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(
            f"CSV '{filepath}' is missing required columns: {missing}"
        )

    if df.empty:
        raise ValueError(f"CSV '{filepath}' is empty.")

    df["source_file"] = os.path.basename(filepath)
    return df


# =============================================================================
# Internal fitting helpers
# =============================================================================
def _extract_segments(
    df: pd.DataFrame,
) -> Tuple[Tuple[np.ndarray, np.ndarray], Optional[Tuple[np.ndarray, np.ndarray]]]:
    """Split DataFrame into fill and (optionally) evac time/O3 arrays.

    Returns:
        (fill_segment, evac_segment)  where each segment is (t, y) or None.

    Raises:
        ValueError: If there are fewer than 5 fill points.
    """
    df_fill = df[df["phase"] == "fill"].copy().reset_index(drop=True)
    df_evac = df[df["phase"] == "evac"].copy().reset_index(drop=True)

    if len(df_fill) < 5:
        raise ValueError(
            f"Need ≥5 fill data points for k_abs fitting, got {len(df_fill)}."
        )

    t0_fill = df_fill["elapsed_s"].iloc[0]
    t_fill = (df_fill["elapsed_s"] - t0_fill).values.astype(float)
    y_fill = df_fill["vessel_o3_pct"].values.astype(float)

    evac_seg = None
    if len(df_evac) >= 3:
        t0_evac = df_evac["elapsed_s"].iloc[0]
        t_evac = (df_evac["elapsed_s"] - t0_evac).values.astype(float)
        y_evac = df_evac["vessel_o3_pct"].values.astype(float)
        evac_seg = (t_evac, y_evac)

    return (t_fill, y_fill), evac_seg


def _fit_1param(
    t_fill: np.ndarray,
    y_fill: np.ndarray,
    k_d: float,
    c_in: float,
    flow_lpm: float,
    evac_seg: Optional[Tuple[np.ndarray, np.ndarray]] = None,
    c_ss_from_fill: float | None = None,
) -> Tuple[float, float, float, float, float, float, float]:
    """Fit V_residual and k_abs from fill curve (1-param model).

    Uses scipy.optimize.minimize with L-BFGS-B and a global search using
    differential_evolution as a warm start to avoid local minima.

    Args:
        t_fill: Fill time array (s, starting from 0).
        y_fill: Fill O3 array (%vol).
        k_d: Fixed k_d value from prior calibration (s⁻¹).
        c_in: Inlet concentration (%vol).
        flow_lpm: Flow rate (L/min).
        evac_seg: Optional (t_evac, y_evac) tuple to include in cost.
        c_ss_from_fill: C_ss estimate for evac (computed from fill tail if None).

    Returns:
        (V_residual, k_abs, sse, r_squared, rmse, aic, bic)
    """
    n = len(t_fill)

    def _cost(params: np.ndarray) -> float:
        v_res, k_abs = params
        if v_res <= 0 or k_abs < 0:
            return 1e12
        k_tot = k_d + k_abs
        y_pred = _fill_loaded(t_fill, v_res, k_tot, c_in, flow_lpm)
        cost = float(np.sum((y_fill - y_pred) ** 2))
        if evac_seg is not None:
            t_e, y_e = evac_seg
            c_ss_e = c_ss_from_fill if c_ss_from_fill is not None else float(y_e[0])
            y_pred_e = _evac_loaded(t_e, v_res, k_tot, c_ss_e, flow_lpm)
            cost += float(np.sum((y_e - y_pred_e) ** 2))
        return cost

    bounds_de = [(0.1, V_VESSEL_L), (0.0, 0.1)]

    # Global search first (coarse, fast)
    de_result = differential_evolution(
        _cost,
        bounds=bounds_de,
        seed=42,
        maxiter=300,
        tol=1e-8,
        mutation=(0.5, 1.5),
        recombination=0.9,
        popsize=12,
        polish=False,
    )
    x0 = de_result.x

    # Local refinement
    result = minimize(
        _cost,
        x0=x0,
        method="L-BFGS-B",
        bounds=bounds_de,
        options={"ftol": 1e-14, "gtol": 1e-10, "maxiter": 2000},
    )

    v_res, k_abs = float(result.x[0]), float(result.x[1])
    k_tot = k_d + k_abs
    y_pred = _fill_loaded(t_fill, v_res, k_tot, c_in, flow_lpm)
    sse = float(np.sum((y_fill - y_pred) ** 2))
    r2 = _r_squared(y_fill, y_pred)
    rmse = _rmse(y_fill, y_pred)
    # 2 free parameters: V_residual and k_abs
    aic = _aic(n, 2, sse)
    bic = _bic(n, 2, sse)

    return v_res, k_abs, sse, r2, rmse, aic, bic


def _fit_2param_biphasic(
    t_fill: np.ndarray,
    y_fill: np.ndarray,
    k_d: float,
    c_in: float,
    flow_lpm: float,
    evac_seg: Optional[Tuple[np.ndarray, np.ndarray]] = None,
    c_ss_from_fill: float | None = None,
) -> Tuple[float, float, float, float, float, float, float, float, float]:
    """Fit V_residual, k_abs_fast, k_abs_slow, fraction_fast (biphasic model).

    Returns:
        (V_residual, k_abs_fast, k_abs_slow, fraction_fast,
         sse, r_squared, rmse, aic, bic)
    """
    n = len(t_fill)

    def _cost(params: np.ndarray) -> float:
        v_res, k_fast, k_slow, alpha = params
        if v_res <= 0 or k_fast < 0 or k_slow < 0 or not (0 < alpha < 1):
            return 1e12
        if k_fast < k_slow:   # enforce k_fast ≥ k_slow
            return 1e12
        y_pred = _fill_biphasic(
            t_fill, v_res, k_d, k_fast, k_slow, alpha, c_in, flow_lpm
        )
        cost = float(np.sum((y_fill - y_pred) ** 2))
        if evac_seg is not None:
            t_e, y_e = evac_seg
            c_ss_e = c_ss_from_fill if c_ss_from_fill is not None else float(y_e[0])
            y_pred_e = _evac_biphasic(
                t_e, v_res, k_d, k_fast, k_slow, alpha, c_ss_e, flow_lpm
            )
            cost += float(np.sum((y_e - y_pred_e) ** 2))
        return cost

    bounds_de = [
        (0.1, V_VESSEL_L),  # v_res
        (1e-5, 0.1),         # k_fast
        (0.0, 0.05),         # k_slow
        (0.01, 0.99),        # alpha
    ]

    de_result = differential_evolution(
        _cost,
        bounds=bounds_de,
        seed=42,
        maxiter=500,
        tol=1e-9,
        mutation=(0.5, 1.5),
        recombination=0.9,
        popsize=15,
        polish=False,
    )
    x0 = de_result.x

    result = minimize(
        _cost,
        x0=x0,
        method="L-BFGS-B",
        bounds=bounds_de,
        options={"ftol": 1e-14, "gtol": 1e-10, "maxiter": 3000},
    )

    v_res, k_fast, k_slow, alpha = result.x.tolist()
    v_res, k_fast, k_slow, alpha = (
        float(v_res), float(k_fast), float(k_slow), float(alpha)
    )

    # Enforce k_fast ≥ k_slow (may swap if optimiser crosses the boundary)
    if k_fast < k_slow:
        k_fast, k_slow = k_slow, k_fast

    y_pred = _fill_biphasic(
        t_fill, v_res, k_d, k_fast, k_slow, alpha, c_in, flow_lpm
    )
    sse = float(np.sum((y_fill - y_pred) ** 2))
    r2 = _r_squared(y_fill, y_pred)
    rmse = _rmse(y_fill, y_pred)
    # 4 free parameters: V_residual, k_fast, k_slow, fraction_fast
    aic = _aic(n, 4, sse)
    bic = _bic(n, 4, sse)

    return v_res, k_fast, k_slow, alpha, sse, r2, rmse, aic, bic


# =============================================================================
# Public fitting API
# =============================================================================
def fit_k_abs_model(
    csv_path: str,
    flow_lpm: float,
    c_in_pct: float,
    k_d_empty: float,
    kg_substrate: float,
    delta_aic_threshold: float = 4.0,
    require_delta_bic_positive: bool = True,
) -> KAbsModel:
    """Fit k_abs (and V_residual) from a loaded-vessel calibration run.

    Both a 1-parameter (single k_abs) and 2-parameter (biphasic k_abs_fast /
    k_abs_slow) model are fitted and compared via AIC / BIC.  The simpler
    1-param model is preferred unless the biphasic model achieves a reduction
    in AIC > *delta_aic_threshold* **and** a lower BIC.

    Args:
        csv_path: Path to calibration CSV (fill phase required, evac optional).
        flow_lpm: Measured volumetric flow rate (L/min) during the run.
        c_in_pct: Inlet O3 concentration (%vol) at 100% generator power.
            Obtained from the validated power-O3 model.
        k_d_empty: First-order O3 decay constant in the empty vessel (s⁻¹).
            Loaded from the most-recent cstr_k_d JSON before calling this.
        kg_substrate: Mass of loaded substrate (kg).  Used to compute the
            volumetric packing density (kg/L of displaced gas).
        delta_aic_threshold: Minimum ΔAIC required to prefer the biphasic
            model.  Default 4.0 (substantial evidence threshold per Burnham &
            Anderson 2002).
        require_delta_bic_positive: If True (default), also require that the
            biphasic BIC is lower than the 1-param BIC before switching.

    Returns:
        Fitted KAbsModel.  Check ``.is_valid`` before use.

    Raises:
        RuntimeError: If scipy is not installed.
        FileNotFoundError: If *csv_path* does not exist.
        ValueError: If the CSV lacks required columns or has insufficient data.
    """
    if not HAS_SCIPY:
        raise RuntimeError(
            "scipy is required for k_abs model fitting. "
            "Install with: pip install scipy"
        )

    logger.info("Fitting k_abs model from '%s'", csv_path)

    df = load_k_abs_csv(csv_path)
    (t_fill, y_fill), evac_seg = _extract_segments(df)
    n_fill = len(t_fill)

    # Estimate C_ss from the tail of the fill segment for evac c_ss
    c_ss_estimate = float(y_fill[-min(10, n_fill // 5 or 1):].mean())

    # ------------------------------------------------------------------
    # 1-param fit
    # ------------------------------------------------------------------
    logger.debug("Fitting 1-param (single k_abs) model …")
    v_res_1, k_abs_1, sse_1, r2_1, rmse_1, aic_1, bic_1 = _fit_1param(
        t_fill, y_fill, k_d_empty, c_in_pct, flow_lpm,
        evac_seg=evac_seg, c_ss_from_fill=c_ss_estimate,
    )

    # ------------------------------------------------------------------
    # 2-param (biphasic) fit
    # ------------------------------------------------------------------
    logger.debug("Fitting 2-param (biphasic) model …")
    try:
        (
            v_res_2, k_fast_2, k_slow_2, alpha_2,
            sse_2, r2_2, rmse_2, aic_2, bic_2,
        ) = _fit_2param_biphasic(
            t_fill, y_fill, k_d_empty, c_in_pct, flow_lpm,
            evac_seg=evac_seg, c_ss_from_fill=c_ss_estimate,
        )
    except Exception as exc:
        logger.warning("Biphasic fit failed (%s); falling back to 1-param.", exc)
        v_res_2 = v_res_1
        k_fast_2 = k_abs_1
        k_slow_2 = k_abs_1
        alpha_2 = 0.5
        sse_2 = sse_1
        r2_2 = r2_1
        rmse_2 = rmse_1
        aic_2 = aic_1 + 10.0   # penalise failed fit
        bic_2 = bic_1 + 10.0

    # ------------------------------------------------------------------
    # Model selection (conservative: prefer simpler 1-param)
    # ------------------------------------------------------------------
    delta_aic = aic_1 - aic_2   # positive means biphasic is better
    delta_bic = bic_1 - bic_2

    prefer_biphasic = (
        delta_aic > delta_aic_threshold
        and (not require_delta_bic_positive or delta_bic > 0)
    )
    preferred_model = "2param" if prefer_biphasic else "1param"

    logger.info(
        "Model selection: ΔAIC=%.2f, ΔBIC=%.2f → preferred=%s",
        delta_aic, delta_bic, preferred_model,
    )

    # ------------------------------------------------------------------
    # Derive display / physical quantities from the 1-param model
    # (always computed so the model is self-contained regardless of
    #  preferred_model)
    # ------------------------------------------------------------------
    q = flow_lpm / 60.0
    k_total = k_d_empty + k_abs_1
    tau_phys = v_res_1 / q if q > 0 else float("inf")
    denom = 1.0 + k_total * tau_phys
    c_ss = c_in_pct / denom
    tau_eff = tau_phys / denom
    t_fill_99 = -tau_eff * math.log(0.01) if tau_eff > 0 else float("inf")

    v_displaced = V_VESSEL_L - v_res_1
    if v_displaced > 0 and kg_substrate > 0:
        density = kg_substrate / v_displaced
    elif v_displaced <= 0:
        logger.warning(
            "V_residual (%.3f L) ≥ V_vessel (%.3f L); density undefined.",
            v_res_1, V_VESSEL_L,
        )
        density = 0.0
    else:
        density = 0.0   # substrate mass not provided or zero

    # ------------------------------------------------------------------
    # Build result
    # ------------------------------------------------------------------
    notes = (
        f"Fixed V_vessel={V_VESSEL_L}L, V_dead={V_DEAD_L}L, "
        f"k_d={k_d_empty:.4e}/s, C_in={c_in_pct:.4f}%, Q={flow_lpm:.2f}LPM. "
        f"1-param: V_res={v_res_1:.4f}L, k_abs={k_abs_1:.4e}/s "
        f"(R²={r2_1:.4f}, AIC={aic_1:.2f}, BIC={bic_1:.2f}). "
        f"2-param: k_fast={k_fast_2:.4e}/s, k_slow={k_slow_2:.4e}/s, "
        f"α={alpha_2:.3f} (AIC={aic_2:.2f}, BIC={bic_2:.2f}). "
        f"ΔAIC={delta_aic:.2f}, ΔBIC={delta_bic:.2f} → {preferred_model}."
    )

    model = KAbsModel(
        v_vessel_L=V_VESSEL_L,
        v_dead_L=V_DEAD_L,
        k_d_empty_per_s=round(k_d_empty, 10),
        c_in_pct=round(c_in_pct, 6),
        calibration_flow_lpm=float(flow_lpm),
        kg_substrate=float(kg_substrate),
        V_residual_L=round(v_res_1, 6),
        k_abs_per_s=round(k_abs_1, 10),
        loaded_material_density_kg_per_L=round(density, 4),
        c_ss_loaded_pct=round(c_ss, 6),
        tau_eff_loaded_s=round(tau_eff, 3),
        tau_physical_loaded_s=round(tau_phys, 3),
        t_fill_99=round(t_fill_99, 1),
        r_squared=round(r2_1, 6),
        rmse=round(rmse_1, 6),
        n_points=n_fill,
        aic_1param=round(aic_1, 4),
        bic_1param=round(bic_1, 4),
        k_abs_fast=round(k_fast_2, 10),
        k_abs_slow=round(k_slow_2, 10),
        fraction_fast=round(alpha_2, 6),
        aic_2param=round(aic_2, 4),
        bic_2param=round(bic_2, 4),
        preferred_model=preferred_model,
        data_file=os.path.basename(csv_path),
        fitted_at=datetime.now().isoformat(timespec="seconds"),
        notes=notes,
    )

    logger.info("k_abs fit complete: %s", model.summary())
    return model


# =============================================================================
# Persistence
# =============================================================================
def save_k_abs_model(model: KAbsModel, model_dir: str) -> str:
    """Save a KAbsModel to a timestamped JSON file.

    File name format: ``YYYYMMDD_HHMMSS_cstr_k_abs.json``

    Args:
        model: The fitted KAbsModel to persist.
        model_dir: Directory to write the file into (created if absent).

    Returns:
        Absolute path to the saved file.
    """
    os.makedirs(model_dir, exist_ok=True)
    now = datetime.now()
    fname = f"{now:%Y%m%d_%H%M%S}_cstr_k_abs.json"
    fpath = os.path.join(model_dir, fname)

    data = asdict(model)
    data["type"] = "cstr_k_abs"
    data["version"] = 1

    with open(fpath, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2)

    logger.info("Saved k_abs model → %s", fpath)
    return fpath


def load_k_abs_model(filepath: str) -> KAbsModel:
    """Load a KAbsModel from a JSON file.

    Args:
        filepath: Full path to a ``*_cstr_k_abs.json`` file.

    Returns:
        Reconstructed KAbsModel.

    Raises:
        FileNotFoundError: If *filepath* does not exist.
        ValueError: If the JSON is missing expected fields or is malformed.
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"k_abs model file not found: {filepath}")

    with open(filepath, encoding="utf-8") as fh:
        data = json.load(fh)

    data.pop("type", None)
    data.pop("version", None)

    known_fields = {f.name for f in KAbsModel.__dataclass_fields__.values()}
    filtered = {k: v for k, v in data.items() if k in known_fields}

    return KAbsModel(**filtered)


def load_k_abs_model_from_dir(model_dir: str) -> Optional[KAbsModel]:
    """Load the most recent KAbsModel from a directory.

    Scans *model_dir* for files matching the pattern
    ``YYYYMMDD_HHMMSS_cstr_k_abs.json`` and loads the newest one.

    Args:
        model_dir: Directory containing ``*_cstr_k_abs.json`` files.

    Returns:
        The most recent KAbsModel, or None if the directory is empty or
        does not exist.
    """
    if not os.path.isdir(model_dir):
        logger.warning("k_abs model directory not found: %s", model_dir)
        return None

    candidates = sorted(
        (
            fn for fn in os.listdir(model_dir)
            if fn.endswith("_cstr_k_abs.json")
            and re.match(r"\d{8}_\d{6}", fn)
        ),
        reverse=True,  # newest timestamp first
    )

    for fname in candidates:
        fpath = os.path.join(model_dir, fname)
        try:
            model = load_k_abs_model(fpath)
            logger.debug("Loaded k_abs model from %s", fpath)
            return model
        except Exception as exc:
            logger.warning("Could not load '%s': %s", fpath, exc)

    logger.info("No valid k_abs model found in '%s'.", model_dir)
    return None


def list_k_abs_models(model_dir: str) -> list[KAbsModel]:
    """Return all KAbsModel instances in *model_dir*, newest first.

    Silently skips files that cannot be parsed.
    """
    models: list[KAbsModel] = []
    if not os.path.isdir(model_dir):
        return models

    for fn in sorted(os.listdir(model_dir), reverse=True):
        if fn.endswith(".json"):
            try:
                models.append(load_k_abs_model(os.path.join(model_dir, fn)))
            except Exception:
                pass
    return models
