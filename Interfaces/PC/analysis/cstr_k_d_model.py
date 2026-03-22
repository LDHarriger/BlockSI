"""
k_d calibration model — CSTR with first-order O3 decay, fixed vessel geometry.

Models ozone concentration dynamics in the vessel as a CSTR with decay:

  dC/dt = (C_in − C) / τ  −  k_d · C

Fixed constants (measured):
  V_vessel = 9.27 L    (water displacement measurement)
  V_dead   = 0.020 L   (plumbing dead volume)

Single free parameter:
  k_d  – first-order O3 decay rate (s⁻¹)

Fill solution:
  C_ss   = C_in / (1 + k_d · τ)
  τ_eff  = τ / (1 + k_d · τ)
  C(t)   = C_ss · (1 − exp(−(t − t_d) / τ_eff))   for t > t_d

Evac solution (C_in = 0, flow continues):
  τ_drain = τ / (1 + k_d · τ)
  C(t)    = C_ss · exp(−(t − t_d) / τ_drain)        for t > t_d

Multi-file fitting: accepts multiple calibration CSVs (potentially at
different flow rates). Since k_d is flow-rate-independent, multiple flow
rates constrain the same parameter. Datasets are weighted by 1/τ (lower
flow → higher τ → more sensitivity to k_d → higher weight).

Model saved as timestamped JSON in Models/cstr_k_d/.
"""
from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass, asdict
from datetime import datetime
from typing import Optional

import numpy as np
import pandas as pd

try:
    from scipy.optimize import minimize_scalar, curve_fit
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


# =============================================================================
# Fixed constants
# =============================================================================
V_VESSEL_L = 9.27       # measured by water displacement
V_DEAD_L = 0.020        # measured plumbing volume


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


def _cstr_fill_k_d(
    t: np.ndarray, k_d: float, c_in: float, flow_lpm: float,
) -> np.ndarray:
    """CSTR fill curve parameterized by k_d only (V and V_dead fixed)."""
    q = flow_lpm / 60.0
    tau = V_VESSEL_L / q if q > 0 else 9999.0
    t_d = V_DEAD_L / q if q > 0 else 0.0
    c_ss = c_in / (1.0 + k_d * tau)
    tau_eff = tau / (1.0 + k_d * tau)
    return _cstr_fill(t, c_ss, tau_eff, t_d)


def _cstr_evac_k_d(
    t: np.ndarray, k_d: float, c_ss: float, flow_lpm: float,
) -> np.ndarray:
    """CSTR evac curve parameterized by k_d only (V and V_dead fixed)."""
    q = flow_lpm / 60.0
    tau = V_VESSEL_L / q if q > 0 else 9999.0
    t_d = V_DEAD_L / q if q > 0 else 0.0
    tau_drain = tau / (1.0 + k_d * tau)
    return _cstr_evac(t, c_ss, tau_drain, t_d)


# =============================================================================
# Data class
# =============================================================================
@dataclass
class CSTRModel:
    """Fitted CSTR k_d model with fixed vessel geometry.

    Fixed constants:
        system_volume_L  – 9.27 L (always)
        dead_volume_L    – 0.020 L (always)

    Fitted parameter:
        decay_rate_per_s – first-order O3 decay constant k_d (s⁻¹)
    """

    # Fixed physical parameters
    system_volume_L: float = V_VESSEL_L
    decay_rate_per_s: float = 0.0
    dead_volume_L: float = V_DEAD_L

    # Fitted curve parameters (at calibration flow rate — for display)
    c_ss_fitted_pct: float = 0.0
    tau_eff_fill_s: float = 0.0
    tau_physical_fill_s: float = 0.0
    transport_delay_fill_s: float = 0.0
    tau_drain_evac_s: float = 0.0
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

    # Multi-file fitting info
    data_files: list[str] | None = None
    flow_rates_lpm: list[float] | None = None

    # Provenance
    data_file: str = ""
    fitted_at: str = ""
    notes: str = ""

    # -- predictions -------------------------------------------------------
    def predict_fill(
        self, t: float | np.ndarray, flow_lpm: float | None = None,
        c_in_pct: float | None = None,
    ) -> float | np.ndarray:
        """Predict O3 (%vol) during fill at time *t* (seconds)."""
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
        """Predict O3 (%vol) during evacuation at time *t* (seconds)."""
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
            f"k_d: V={self.system_volume_L:.2f}L (fixed), "
            f"k_d={self.decay_rate_per_s:.6f}/s, "
            f"V_dead={self.dead_volume_L:.3f}L (fixed), "
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
    """Load a k_d calibration CSV (combined baseline+fill+evac phases).

    Expected columns:
        timestamp, esp_ts_ms, elapsed_s, vessel_o3_pct, cell_temp_c,
        vessel_temp_c, power_pct, phase

    Returns DataFrame with ``elapsed_s`` and ``phase`` columns.
    """
    df = pd.read_csv(filepath)
    df.columns = [c.strip() for c in df.columns]

    if "elapsed_s" not in df.columns and "esp_ts_ms" in df.columns:
        t0 = df["esp_ts_ms"].iloc[0]
        df["elapsed_s"] = (df["esp_ts_ms"] - t0) / 1000.0

    df["source_file"] = os.path.basename(filepath)
    return df


load_fill_csv = load_cstr_csv


def _extract_flow_from_filename(filepath: str) -> float | None:
    """Extract flow rate from calibration CSV filename (e.g. '4Lpm')."""
    m = re.search(r"_(\d+(?:\.\d+)?)Lpm", os.path.basename(filepath))
    return float(m.group(1)) if m else None


# =============================================================================
# Fitting — single free parameter: k_d
# =============================================================================
def fit_fill_curve(
    elapsed_s: np.ndarray,
    o3_pct: np.ndarray,
    c_in_pct: float,
    flow_lpm: float,
) -> tuple[float, float, float, float, float]:
    """Fit k_d from fill curve with V and V_dead fixed.

    Returns:
        (k_d, c_ss, tau_eff, r_squared, rmse)
    """
    if not HAS_SCIPY:
        raise RuntimeError("scipy required for model fitting")

    if len(elapsed_s) < 5:
        raise ValueError(f"Need ≥5 points for fill fit, got {len(elapsed_s)}")

    t = np.asarray(elapsed_s, dtype=float)
    y = np.asarray(o3_pct, dtype=float)

    def _residual_sum(k_d_val):
        y_pred = _cstr_fill_k_d(t, k_d_val, c_in_pct, flow_lpm)
        return float(np.sum((y - y_pred) ** 2))

    result = minimize_scalar(
        _residual_sum,
        bounds=(0.0, 1e-2),
        method="bounded",
        options={"xatol": 1e-10, "maxiter": 1000},
    )
    k_d = float(result.x)

    q = flow_lpm / 60.0
    tau = V_VESSEL_L / q if q > 0 else 9999.0
    c_ss = c_in_pct / (1.0 + k_d * tau)
    tau_eff = tau / (1.0 + k_d * tau)

    y_pred = _cstr_fill_k_d(t, k_d, c_in_pct, flow_lpm)
    ss_res = float(np.sum((y - y_pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r_sq = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(ss_res / len(y)))

    return k_d, float(c_ss), float(tau_eff), r_sq, rmse


def fit_evac_curve(
    elapsed_s: np.ndarray,
    o3_pct: np.ndarray,
    c_ss_pct: float,
    flow_lpm: float,
) -> tuple[float, float, float, float]:
    """Fit k_d from evac curve with V and V_dead fixed.

    Returns:
        (k_d, tau_drain, r_squared, rmse)
    """
    if not HAS_SCIPY:
        raise RuntimeError("scipy required for model fitting")

    if len(elapsed_s) < 3:
        raise ValueError(f"Need ≥3 points for evac fit, got {len(elapsed_s)}")

    t = np.asarray(elapsed_s, dtype=float)
    y = np.asarray(o3_pct, dtype=float)

    def _residual_sum(k_d_val):
        y_pred = _cstr_evac_k_d(t, k_d_val, c_ss_pct, flow_lpm)
        return float(np.sum((y - y_pred) ** 2))

    result = minimize_scalar(
        _residual_sum,
        bounds=(0.0, 1e-2),
        method="bounded",
        options={"xatol": 1e-10, "maxiter": 1000},
    )
    k_d = float(result.x)

    q = flow_lpm / 60.0
    tau = V_VESSEL_L / q if q > 0 else 9999.0
    tau_drain = tau / (1.0 + k_d * tau)

    y_pred = _cstr_evac_k_d(t, k_d, c_ss_pct, flow_lpm)
    ss_res = float(np.sum((y - y_pred) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r_sq = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(ss_res / len(y)))

    return k_d, float(tau_drain), r_sq, rmse


def fit_cstr_model(
    csv_path: str | list[str],
    flow_lpm: float | list[float] | None = None,
    c_in_pct: float | list[float] | None = None,
    baseline_samples: int = 15,
) -> CSTRModel:
    """Fit k_d from one or more calibration CSVs (V and V_dead fixed).

    Multi-file fitting: if csv_path is a list, each file may be at a
    different flow rate. k_d is the single shared parameter. Each dataset
    is weighted by 1/τ (lower flow → higher τ → more sensitivity to k_d).

    Args:
        csv_path: Path(s) to calibration CSV(s).
        flow_lpm: Flow rate(s) for each CSV. If None, extracted from filename.
        c_in_pct: Inlet O3 concentration(s) at 100% power for each flow rate.
        baseline_samples: Number of baseline samples (metadata only).

    Returns:
        Fitted CSTRModel. Check .is_valid to confirm success.
    """
    # Normalize inputs to lists
    if isinstance(csv_path, str):
        csv_paths = [csv_path]
    else:
        csv_paths = list(csv_path)

    if flow_lpm is None:
        flows = []
        for p in csv_paths:
            f = _extract_flow_from_filename(p)
            if f is None:
                raise ValueError(f"Cannot extract flow rate from filename: {p}")
            flows.append(f)
    elif isinstance(flow_lpm, (int, float)):
        flows = [float(flow_lpm)] * len(csv_paths)
    else:
        flows = [float(f) for f in flow_lpm]

    if c_in_pct is None:
        raise ValueError("c_in_pct is required (from power-O3 model at 100%)")
    if isinstance(c_in_pct, (int, float)):
        c_ins = [float(c_in_pct)] * len(csv_paths)
    else:
        c_ins = [float(c) for c in c_in_pct]

    if len(flows) != len(csv_paths) or len(c_ins) != len(csv_paths):
        raise ValueError("csv_path, flow_lpm, and c_in_pct must have the same length")

    # Load and split all datasets
    all_fill_segments: list[tuple[np.ndarray, np.ndarray, float, float, float]] = []
    all_evac_segments: list[tuple[np.ndarray, np.ndarray, float, float, float]] = []
    total_fill_pts = 0
    total_evac_pts = 0
    vessel_temps: list[float] = []

    for path, flow, c_in in zip(csv_paths, flows, c_ins):
        df = load_cstr_csv(path)
        q = flow / 60.0
        tau = V_VESSEL_L / q if q > 0 else 9999.0
        weight = 1.0 / tau  # lower flow → higher τ → more weight

        df_fill = df[df["phase"] == "fill"].copy().reset_index(drop=True)
        df_evac = df[df["phase"] == "evac"].copy().reset_index(drop=True)

        if len(df_fill) >= 5:
            t0 = df_fill["elapsed_s"].iloc[0]
            t_fill = (df_fill["elapsed_s"] - t0).values
            y_fill = df_fill["vessel_o3_pct"].values
            all_fill_segments.append((t_fill, y_fill, flow, c_in, weight))
            total_fill_pts += len(df_fill)

        if len(df_evac) >= 3:
            t0 = df_evac["elapsed_s"].iloc[0]
            t_evac = (df_evac["elapsed_s"] - t0).values
            y_evac = df_evac["vessel_o3_pct"].values
            # c_ss for evac: use last few fill samples or max fill reading
            if len(df_fill) >= 5:
                c_ss_evac = float(df_fill["vessel_o3_pct"].iloc[-10:].mean())
            else:
                c_ss_evac = float(df_evac["vessel_o3_pct"].iloc[0])
            all_evac_segments.append((t_evac, y_evac, flow, c_ss_evac, weight))
            total_evac_pts += len(df_evac)

        if "vessel_temp_c" in df.columns:
            valid = df["vessel_temp_c"][df["vessel_temp_c"] > -900]
            if len(valid) > 0:
                vessel_temps.append(float(valid.mean()))

    if not all_fill_segments:
        raise ValueError("No fill data found in any CSV")
    if not all_evac_segments:
        raise ValueError("No evac data found in any CSV")

    # --- Combined k_d fit across all datasets ---
    def _combined_residual(k_d_val):
        total = 0.0
        for t_f, y_f, flow_f, c_in_f, w_f in all_fill_segments:
            y_pred = _cstr_fill_k_d(t_f, k_d_val, c_in_f, flow_f)
            total += w_f * float(np.sum((y_f - y_pred) ** 2))
        for t_e, y_e, flow_e, c_ss_e, w_e in all_evac_segments:
            y_pred = _cstr_evac_k_d(t_e, k_d_val, c_ss_e, flow_e)
            total += w_e * float(np.sum((y_e - y_pred) ** 2))
        return total

    result = minimize_scalar(
        _combined_residual,
        bounds=(0.0, 1e-2),
        method="bounded",
        options={"xatol": 1e-12, "maxiter": 2000},
    )
    k_d = float(result.x)

    # --- Compute per-segment statistics using the primary (first) dataset ---
    primary_flow = flows[0]
    primary_c_in = c_ins[0]
    q_p = primary_flow / 60.0
    tau_p = V_VESSEL_L / q_p if q_p > 0 else 9999.0

    c_ss = primary_c_in / (1.0 + k_d * tau_p)
    tau_eff = tau_p / (1.0 + k_d * tau_p)
    t_d = V_DEAD_L / q_p if q_p > 0 else 0.0
    tau_drain = tau_p / (1.0 + k_d * tau_p)

    # Fill R² and RMSE (across ALL fill segments combined)
    all_y_fill = np.concatenate([s[1] for s in all_fill_segments])
    all_y_fill_pred = np.concatenate([
        _cstr_fill_k_d(s[0], k_d, s[3], s[2]) for s in all_fill_segments
    ])
    ss_res_f = float(np.sum((all_y_fill - all_y_fill_pred) ** 2))
    ss_tot_f = float(np.sum((all_y_fill - np.mean(all_y_fill)) ** 2))
    r2_f = 1.0 - ss_res_f / ss_tot_f if ss_tot_f > 0 else 0.0
    rmse_f = float(np.sqrt(ss_res_f / len(all_y_fill)))

    # Evac R² and RMSE
    all_y_evac = np.concatenate([s[1] for s in all_evac_segments])
    all_y_evac_pred = np.concatenate([
        _cstr_evac_k_d(s[0], k_d, s[3], s[2]) for s in all_evac_segments
    ])
    ss_res_e = float(np.sum((all_y_evac - all_y_evac_pred) ** 2))
    ss_tot_e = float(np.sum((all_y_evac - np.mean(all_y_evac)) ** 2))
    r2_e = 1.0 - ss_res_e / ss_tot_e if ss_tot_e > 0 else 0.0
    rmse_e = float(np.sqrt(ss_res_e / len(all_y_evac)))

    # Independent k_d from evac only (cross-check)
    def _evac_only_residual(k_d_val):
        total = 0.0
        for t_e, y_e, flow_e, c_ss_e, w_e in all_evac_segments:
            y_pred = _cstr_evac_k_d(t_e, k_d_val, c_ss_e, flow_e)
            total += w_e * float(np.sum((y_e - y_pred) ** 2))
        return total

    evac_result = minimize_scalar(
        _evac_only_residual,
        bounds=(0.0, 1e-2),
        method="bounded",
        options={"xatol": 1e-12, "maxiter": 2000},
    )
    k_d_evac = float(evac_result.x)

    vessel_temp = float(np.mean(vessel_temps)) if vessel_temps else -999.0

    return CSTRModel(
        system_volume_L=V_VESSEL_L,
        decay_rate_per_s=round(k_d, 10),
        dead_volume_L=V_DEAD_L,
        c_ss_fitted_pct=round(c_ss, 6),
        tau_eff_fill_s=round(tau_eff, 3),
        tau_physical_fill_s=round(tau_p, 3),
        transport_delay_fill_s=round(t_d, 3),
        tau_drain_evac_s=round(tau_drain, 3),
        transport_delay_evac_s=round(t_d, 3),
        c_in_pct=round(primary_c_in, 6),
        decay_rate_evac_per_s=round(k_d_evac, 10),
        calibration_flow_lpm=primary_flow,
        calibration_vessel_temp_c=round(vessel_temp, 1),
        r_squared_fill=round(r2_f, 6),
        r_squared_evac=round(r2_e, 6),
        rmse_fill=round(rmse_f, 6),
        rmse_evac=round(rmse_e, 6),
        n_points_fill=total_fill_pts,
        n_points_evac=total_evac_pts,
        data_files=[os.path.basename(p) for p in csv_paths],
        flow_rates_lpm=flows,
        data_file=os.path.basename(csv_paths[0]),
        fitted_at=datetime.now().isoformat(timespec="seconds"),
        notes=(
            f"Fixed V={V_VESSEL_L}L, V_dead={V_DEAD_L}L. "
            f"Fitted k_d from {len(csv_paths)} file(s) at flow(s) "
            f"{', '.join(f'{f:.1f}' for f in flows)} LPM. "
            f"k_d(combined)={k_d:.2e}/s, k_d(evac only)={k_d_evac:.2e}/s."
        ),
    )


# Backward-compatible alias
fit_fill_model = fit_cstr_model


# =============================================================================
# Persistence — timestamped JSONs in Models/cstr_k_d/
# =============================================================================
def save_cstr_model(model: CSTRModel, model_dir: str) -> str:
    """Save CSTR k_d model to timestamped JSON. Returns full path."""
    os.makedirs(model_dir, exist_ok=True)
    now = datetime.now()
    fname = f"{now:%Y%m%d_%H%M%S}_cstr_k_d.json"
    fpath = os.path.join(model_dir, fname)

    data = asdict(model)
    data["type"] = "cstr_k_d"
    data["version"] = 3

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

    known_fields = {f.name for f in CSTRModel.__dataclass_fields__.values()}
    filtered = {k: v for k, v in data.items() if k in known_fields}

    return CSTRModel(**filtered)


load_fill_model = load_cstr_model


def load_cstr_model_from_dir(model_dir: str) -> Optional[CSTRModel]:
    """Load the most recent CSTR k_d model from a directory.

    Looks for timestamped files (YYYYMMDD_HHMMSS_cstr_k_d.json) first,
    falls back to legacy cstr_model.json for backward compatibility.
    """
    if not os.path.isdir(model_dir):
        return None

    # Find all timestamped model files, sorted newest first
    timestamped = sorted(
        (fn for fn in os.listdir(model_dir)
         if fn.endswith("_cstr_k_d.json") and re.match(r"\d{8}_\d{6}", fn)),
        reverse=True,
    )

    if timestamped:
        try:
            return load_cstr_model(os.path.join(model_dir, timestamped[0]))
        except Exception:
            pass

    # Legacy fallback: single cstr_model.json
    legacy = os.path.join(model_dir, "cstr_model.json")
    if os.path.exists(legacy):
        try:
            return load_cstr_model(legacy)
        except Exception:
            pass

    return None


load_fill_model_for_condition = lambda model_dir, flow_lpm=None, o2_pct=None: load_cstr_model_from_dir(model_dir)


def list_cstr_models(model_dir: str) -> list[CSTRModel]:
    """List all CSTR k_d models in model_dir, newest first."""
    models: list[CSTRModel] = []
    if not os.path.isdir(model_dir):
        return models
    for fn in sorted(os.listdir(model_dir), reverse=True):
        if fn.endswith(".json"):
            try:
                models.append(load_cstr_model(os.path.join(model_dir, fn)))
            except Exception:
                pass
    return models


list_fill_models = list_cstr_models
