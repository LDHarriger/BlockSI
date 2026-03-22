"""
Dosing schedule solver and per-sample dosimetry for BlockSI ozone sterilization.

The system is a CSTR ozone sterilization vessel operating in three phases:

  1. Ramp-up : 100% power until vessel reaches ramp_switch_fraction * C_target,
               then switch to power_hold.
  2. Hold    : Constant power_hold, maintaining C_target for remaining hold
               duration.
  3. Evacuation : 0% power, O2 + (optional) air compressor until O3 ≈ 0.

Dosimetry conversion factor K
------------------------------
  V_m(T) = 24.04 × (T_K / 293.15)   [L/mol, molar volume at temperature T]
  K      = 48000 / (V_m × 100)       [mg / (%vol × L)]
  K_min  = K / 60                    [mg·min / (%vol × L)]

  Per-sample dose contribution:
    mg_O3 = C_pct_vol × Q_Ls × K × dt_s
    where Q_Ls = Q_lpm / 60

Temperature priority for K: vessel_temp_c → lab_temperature_c → 20°C

CSTR dynamics (used inside the solver)
---------------------------------------
  dC/dt = (C_in − C) / τ  −  k_d · C
  τ      = V_vessel / Q
  C_ss   = C_in / (1 + k_d·τ)
  C(t)   = C_ss · (1 − exp(−t / τ_eff))   during fill (t measured from 0)
  C(t)   = C_0 · exp(−t / τ_drain)         during evac

All accumulators use float64.  Never hardcode sample intervals — always use dt.
"""
from __future__ import annotations

import json
import os
import sys
from dataclasses import dataclass, asdict
from typing import Callable, Optional

import numpy as np

# ---------------------------------------------------------------------------
# Ensure Interfaces/PC/ is on sys.path so ``from analysis import ...`` works
# regardless of invocation style.
# ---------------------------------------------------------------------------
_PC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PC_DIR not in sys.path:
    sys.path.insert(0, _PC_DIR)

from analysis.cstr_k_d_model import V_VESSEL_L, V_DEAD_L


# =============================================================================
# Type aliases
# =============================================================================
# predict_o3_fn(power_pct: float, flow_lpm: float) -> float
PredictO3Fn = Callable[[float, float], float]
# predict_power_fn(o3_pct: float, flow_lpm: float) -> float
PredictPowerFn = Callable[[float, float], float]


# =============================================================================
# Temperature helpers
# =============================================================================

def get_temperature_K(
    vessel_temp_c: Optional[float],
    lab_temperature_c: Optional[float] = None,
) -> float:
    """Return the best available temperature in Kelvin.

    Priority chain:
      1. vessel_temp_c   — if provided and finite (not a sentinel like -999)
      2. lab_temperature_c — if provided and finite
      3. 20°C default

    Args:
        vessel_temp_c: Temperature measured inside the vessel (°C).  Pass
            ``None`` or a sentinel value <= −100 to skip.
        lab_temperature_c: Ambient lab temperature (°C).  Pass ``None`` to
            skip.

    Returns:
        Temperature in Kelvin (float64).
    """
    def _valid(t: Optional[float]) -> bool:
        return t is not None and np.isfinite(float(t)) and float(t) > -100.0

    if _valid(vessel_temp_c):
        return float(vessel_temp_c) + 273.15  # type: ignore[arg-type]
    if _valid(lab_temperature_c):
        return float(lab_temperature_c) + 273.15  # type: ignore[arg-type]
    return 293.15  # 20°C


# =============================================================================
# Core dosimetry constants
# =============================================================================

def compute_K(temperature_c: float = 20.0) -> float:
    """Return the dosimetry conversion factor K in mg / (%vol × L).

    K = 48000 / (V_m × 100)
    where V_m = 24.04 × (T_K / 293.15)  [L/mol, ideal-gas molar volume]

    At 20°C: K ≈ 19.967 mg / (%vol × L)
    K_min  = K / 60 ≈ 0.3328 mg·min / (%vol × L)

    Args:
        temperature_c: Gas temperature in °C (default 20.0).

    Returns:
        K as float64.
    """
    T_K = float(temperature_c) + 273.15
    V_m = 24.04 * (T_K / 293.15)   # L/mol
    return np.float64(48000.0 / (V_m * 100.0))


# =============================================================================
# DoseSchedule dataclass
# =============================================================================

@dataclass
class DoseSchedule:
    """Computed dosing schedule for one batch run.

    All times are in minutes; concentrations in %vol; power in %; dose in
    mg/kg; masses in mg.

    Fields
    ------
    C_target         : Target steady-state O3 concentration (%vol) that
                       delivers the desired dose.
    power_hold       : Generator power (%) to maintain C_target.
    achievable       : True if power_hold <= 100%.

    t_switch_predicted : Predicted time (min) from ramp start until vessel
                         concentration reaches ramp_switch_fraction × C_target
                         (i.e., when we switch from 100% to power_hold).
    t_hold           : Hold duration (min, from switch point to evac start).
    t_evac_predicted : Predicted time (min) for O3 to reach evac_threshold
                       (%vol) after the hold phase ends.

    mg_O3_ramp       : O3 mass delivered to substrate during ramp (mg).
    mg_O3_hold       : O3 mass delivered to substrate during hold (mg).
    mg_O3_evac       : O3 mass delivered to substrate during evac (mg).
    mg_O3_total      : Total O3 mass delivered (mg).
    dose_predicted   : Predicted dose (mg/kg).

    Solver inputs (stored for traceability)
    ----------------------------------------
    target_dose_mg_per_kg : Requested dose (mg/kg).
    kg_substrate          : Substrate mass (kg).
    process_time_min      : Total process time budget (min).
    flow_lpm              : O2 concentrator flow rate (LPM).
    k_d_empty             : CSTR decay rate in empty vessel (s⁻¹).
    k_abs                 : Fractional O3 absorption rate by substrate (s⁻¹).
    V_residual            : Headspace volume occupied by substrate (L, < V_vessel).
    c_in_100pct           : Inlet O3 concentration at 100% power (%vol).
    ramp_switch_fraction  : Fraction of C_target at which ramp→hold switch
                            occurs (default 0.85).
    lab_temperature_c     : Lab temperature used for K computation (°C).
    evac_threshold_pct    : O3 concentration considered 'evacuated' (%vol).
    notes                 : Free-form solver diagnostics.
    """

    # --- Schedule outputs ---------------------------------------------------
    C_target: float = 0.0
    power_hold: float = 0.0
    achievable: bool = False

    t_switch_predicted: float = 0.0   # min
    t_hold: float = 0.0               # min
    t_evac_predicted: float = 0.0     # min

    mg_O3_ramp: float = 0.0
    mg_O3_hold: float = 0.0
    mg_O3_evac: float = 0.0
    mg_O3_total: float = 0.0
    dose_predicted: float = 0.0       # mg/kg

    # --- Solver inputs (traceability) ----------------------------------------
    target_dose_mg_per_kg: float = 0.0
    kg_substrate: float = 0.0
    process_time_min: float = 0.0
    flow_lpm: float = 4.0
    k_d_empty: float = 0.0
    k_abs: float = 0.0
    V_residual: float = 0.0
    c_in_100pct: float = 0.0
    ramp_switch_fraction: float = 0.85
    lab_temperature_c: float = 20.0
    evac_threshold_pct: float = 0.01
    notes: str = ""

    # -------------------------------------------------------------------------
    def to_json(self) -> str:
        """Serialize to a JSON string."""
        return json.dumps(asdict(self), indent=2)

    @classmethod
    def from_json(cls, json_str: str) -> "DoseSchedule":
        """Deserialize from a JSON string produced by ``to_json()``."""
        data = json.loads(json_str)
        known = {f for f in cls.__dataclass_fields__}
        filtered = {k: v for k, v in data.items() if k in known}
        return cls(**filtered)

    def summary(self) -> str:
        """One-line human-readable summary."""
        status = "OK" if self.achievable else "OVER-POWER"
        return (
            f"[{status}] C_target={self.C_target:.4f}%vol "
            f"power_hold={self.power_hold:.1f}% "
            f"t_switch={self.t_switch_predicted:.1f}min "
            f"t_hold={self.t_hold:.1f}min "
            f"t_evac={self.t_evac_predicted:.1f}min "
            f"dose={self.dose_predicted:.1f}mg/kg "
            f"(target={self.target_dose_mg_per_kg:.1f}mg/kg)"
        )


# =============================================================================
# Internal CSTR simulation helpers
# =============================================================================

def _cstr_params(
    flow_lpm: float,
    k_d: float,
    V_vessel: float = V_VESSEL_L,
    V_dead: float = V_DEAD_L,
) -> tuple[float, float, float, float]:
    """Compute CSTR timing parameters.

    Returns:
        (tau_s, tau_eff_s, tau_drain_s, t_dead_s)
        All in seconds.
    """
    q = max(flow_lpm / 60.0, 1e-9)
    tau_s = V_vessel / q
    t_dead_s = V_dead / q
    tau_eff_s = tau_s / (1.0 + k_d * tau_s)
    tau_drain_s = tau_eff_s  # same expression for evac
    return tau_s, tau_eff_s, tau_drain_s, t_dead_s


def _ramp_switch_time_s(
    C_switch: float,
    C_ss_ramp: float,
    tau_eff_s: float,
) -> float:
    """Time (s) for CSTR fill to reach C_switch (from t=0 after dead-time).

    C(t) = C_ss_ramp * (1 - exp(-t / tau_eff_s))
    Solve for t: t = -tau_eff_s * ln(1 - C_switch / C_ss_ramp)

    Returns inf if C_switch >= C_ss_ramp (never reached).
    """
    if C_ss_ramp <= 0.0:
        return float("inf")
    ratio = C_switch / C_ss_ramp
    if ratio >= 1.0:
        return float("inf")
    return -tau_eff_s * np.log(1.0 - ratio)


def _evac_time_s(
    C_start: float,
    C_threshold: float,
    tau_drain_s: float,
) -> float:
    """Time (s) for CSTR evac to decay from C_start to C_threshold.

    C(t) = C_start * exp(-t / tau_drain_s)
    Solve for t: t = tau_drain_s * ln(C_start / C_threshold)

    Returns 0 if C_start <= C_threshold.
    """
    if C_start <= C_threshold or tau_drain_s <= 0.0:
        return 0.0
    return tau_drain_s * np.log(C_start / C_threshold)


def _integrate_dose_ramp(
    C_ss_ramp: float,
    tau_eff_s: float,
    t_switch_s: float,
    flow_lpm: float,
    K: float,
    k_abs: float,
    V_vessel: float = V_VESSEL_L,
) -> float:
    """Integrate mg_O3 absorbed by substrate during ramp phase (numerical).

    During ramp: C(t) = C_ss_ramp * (1 - exp(-t / tau_eff_s))
    The absorption rate into substrate: dM/dt = k_abs * C(t) * V_vessel * K_vol
    where K_vol converts %vol → mg/L.

    Simplified: the exhaust gas carries away:
      dM_evac/dt = C(t) * Q_Ls * K   [mg/s]

    We approximate the substrate dose as the fraction k_abs/(flow-through rate)
    of total O3 present.  However the caller (`solve_dosing_schedule`) uses the
    same integrate_dose_* functions to compute the *total* mg_O3 passing
    through the vessel — substrate absorption is accounted for in the effective
    k_d passed to the solver (k_d_empty + k_abs).

    This function returns the total mg_O3 that exits the vessel (exhaust) +
    absorbed during ramp.  The dose to substrate is derived separately by the
    accumulator using per-sample k_abs integration.

    For schedule prediction we approximate the absorbed fraction numerically
    over n_steps = 1000.
    """
    if t_switch_s <= 0.0:
        return 0.0
    n = 1000
    dt = t_switch_s / n
    t_arr = np.linspace(0.0, t_switch_s, n + 1, dtype=np.float64)
    C_arr = C_ss_ramp * (1.0 - np.exp(-t_arr / max(tau_eff_s, 1e-9)))
    Q_Ls = flow_lpm / 60.0
    # mg absorbed by substrate per step: k_abs * C(t) * V_vessel * (K/100) * dt_s
    # We store %vol * L → mg using K directly (K is in mg/(%vol·L))
    mg_abs = float(np.trapz(k_abs * C_arr * V_vessel * K, t_arr))
    return mg_abs


def _integrate_dose_hold(
    C_ss_hold: float,
    t_hold_s: float,
    k_abs: float,
    V_vessel: float,
    K: float,
) -> float:
    """mg_O3 absorbed by substrate during constant-concentration hold phase."""
    if t_hold_s <= 0.0:
        return 0.0
    return float(k_abs * C_ss_hold * V_vessel * K * t_hold_s)


def _integrate_dose_evac(
    C_start: float,
    tau_drain_s: float,
    t_evac_s: float,
    k_abs: float,
    V_vessel: float,
    K: float,
) -> float:
    """mg_O3 absorbed by substrate during evacuation phase (decaying C)."""
    if t_evac_s <= 0.0 or C_start <= 0.0:
        return 0.0
    n = 500
    dt = t_evac_s / n
    t_arr = np.linspace(0.0, t_evac_s, n + 1, dtype=np.float64)
    C_arr = C_start * np.exp(-t_arr / max(tau_drain_s, 1e-9))
    return float(np.trapz(k_abs * C_arr * V_vessel * K, t_arr))


# =============================================================================
# Dosing schedule solver
# =============================================================================

def solve_dosing_schedule(
    target_dose_mg_per_kg: float,
    kg_substrate: float,
    process_time_min: float,
    flow_lpm: float,
    k_d_empty: float,
    k_abs: float,
    V_residual: float,
    c_in_100pct: float,
    predict_o3_fn: PredictO3Fn,
    predict_power_fn: PredictPowerFn,
    air_comp_evac: bool = True,
    air_comp_lpm: float = 7.5,
    ramp_switch_fraction: float = 0.85,
    lab_temperature_c: float = 20.0,
    evac_threshold_pct: float = 0.01,
    solver_n_grid: int = 200,
) -> DoseSchedule:
    """Find the dosing schedule that delivers target_dose to the substrate.

    The solver searches for C_target (steady-state O3 %vol) such that the
    integrated absorbed O3 dose equals target_dose * kg_substrate.

    Three-phase model
    -----------------
    Ramp-up:
      Generator runs at 100%.  Inlet O3 = c_in_100pct.  The vessel fills
      as a CSTR with effective decay k_d = k_d_empty + k_abs (substrate
      absorbs and the absorption is irreversible at this level of
      approximation).  Switch to power_hold when C reaches
      ramp_switch_fraction * C_target.

    Hold:
      Generator runs at power_hold.  The vessel is at approximately C_target
      (C_ss at power_hold with flow_lpm).  Duration = process_time_min −
      t_switch − t_evac.

    Evacuation:
      Generator off.  Optional air compressor supplements the O2 flow.
      Total evacuation flow = flow_lpm + (air_comp_lpm if air_comp_evac else 0).
      Vessel decays from C_target → 0.

    Args:
        target_dose_mg_per_kg : Desired dose in mg of O3 per kg of substrate.
        kg_substrate          : Mass of substrate in the vessel (kg).
        process_time_min      : Total scheduled process time (min).
        flow_lpm              : O2 concentrator flow rate during hold (LPM).
        k_d_empty             : First-order O3 decay in empty vessel (s⁻¹).
        k_abs                 : O3 absorption rate by substrate (s⁻¹).
                                Effective k_d = k_d_empty + k_abs.
        V_residual            : Volume displaced by substrate (L).
                                Effective V = V_VESSEL_L − V_residual.
        c_in_100pct           : Inlet O3 at 100% power, current flow (%vol).
        predict_o3_fn         : (power_pct, flow_lpm) → O3 %vol.
        predict_power_fn      : (o3_pct, flow_lpm) → power %.
        air_comp_evac         : Use air compressor during evacuation.
        air_comp_lpm          : Air compressor flow rate (LPM).
        ramp_switch_fraction  : Switch from 100% → power_hold when vessel
                                reaches this fraction of C_target (0–1).
        lab_temperature_c     : Lab temperature for K computation (°C).
        evac_threshold_pct    : O3 level considered 'evacuated' (%vol).
        solver_n_grid         : Number of C_target grid points to scan.

    Returns:
        DoseSchedule with all computed parameters.

    Raises:
        ValueError: If arguments are out of range.
    """
    # --- Input validation ---------------------------------------------------
    if target_dose_mg_per_kg <= 0.0:
        raise ValueError(f"target_dose_mg_per_kg must be > 0, got {target_dose_mg_per_kg}")
    if kg_substrate <= 0.0:
        raise ValueError(f"kg_substrate must be > 0, got {kg_substrate}")
    if process_time_min <= 0.0:
        raise ValueError(f"process_time_min must be > 0, got {process_time_min}")
    if flow_lpm <= 0.0:
        raise ValueError(f"flow_lpm must be > 0, got {flow_lpm}")
    if k_d_empty < 0.0:
        raise ValueError(f"k_d_empty must be >= 0, got {k_d_empty}")
    if k_abs < 0.0:
        raise ValueError(f"k_abs must be >= 0, got {k_abs}")
    if V_residual < 0.0 or V_residual >= V_VESSEL_L:
        raise ValueError(
            f"V_residual must be in [0, {V_VESSEL_L}), got {V_residual}"
        )
    if c_in_100pct <= 0.0:
        raise ValueError(f"c_in_100pct must be > 0, got {c_in_100pct}")
    if not (0.0 < ramp_switch_fraction < 1.0):
        raise ValueError(
            f"ramp_switch_fraction must be in (0, 1), got {ramp_switch_fraction}"
        )

    # --- Derived geometry ---------------------------------------------------
    V_eff = V_VESSEL_L - V_residual          # effective headspace volume (L)
    k_d_eff = k_d_empty + k_abs             # effective decay rate (s⁻¹)

    # Evacuation flow (may include air compressor)
    evac_flow_lpm = flow_lpm + (air_comp_lpm if air_comp_evac else 0.0)

    # CSTR timing parameters during hold (O2 flow, effective V)
    tau_s, tau_eff_s, tau_drain_s, t_dead_s = _cstr_params(
        flow_lpm, k_d_eff, V_eff, V_DEAD_L
    )
    # CSTR timing during evac (higher flow, same decay — no generator)
    _, _, tau_drain_evac_s, _ = _cstr_params(
        evac_flow_lpm, k_d_eff, V_eff, V_DEAD_L
    )

    # CSTR steady-state at 100% power (ramp phase)
    C_ss_100pct = c_in_100pct / (1.0 + k_d_eff * tau_s)

    # Dosimetry constant K at lab temperature
    K = float(compute_K(lab_temperature_c))

    # --- Target dose in mg --------------------------------------------------
    target_mg = target_dose_mg_per_kg * kg_substrate

    # --- Grid search for C_target -------------------------------------------
    # C_target must be in (0, C_ss_100pct] — can't exceed what 100% delivers.
    C_max = C_ss_100pct
    if C_max <= 0.0:
        return DoseSchedule(
            target_dose_mg_per_kg=target_dose_mg_per_kg,
            kg_substrate=kg_substrate,
            process_time_min=process_time_min,
            flow_lpm=flow_lpm,
            k_d_empty=k_d_empty,
            k_abs=k_abs,
            V_residual=V_residual,
            c_in_100pct=c_in_100pct,
            ramp_switch_fraction=ramp_switch_fraction,
            lab_temperature_c=lab_temperature_c,
            evac_threshold_pct=evac_threshold_pct,
            achievable=False,
            notes="c_in_100pct is zero; no O3 can be generated.",
        )

    def _predict_total_dose(C_target: float) -> float:
        """Predict total absorbed dose (mg) for a given C_target."""
        # power_hold → C_target (via predict_power_fn)
        # But for dose computation C_target is the hold steady-state.
        C_switch = ramp_switch_fraction * C_target

        # Ramp: 100% power until C reaches C_switch
        t_sw_s = _ramp_switch_time_s(C_switch, C_ss_100pct, tau_eff_s)
        # Add dead time (gas transit from generator to sensor/vessel)
        t_sw_s = t_sw_s + t_dead_s

        # Hold duration (process_time_min is total; subtract ramp and evac)
        t_evac_s = _evac_time_s(C_target, evac_threshold_pct, tau_drain_evac_s)
        t_ramp_min = t_sw_s / 60.0
        t_evac_min = t_evac_s / 60.0
        t_hold_min = process_time_min - t_ramp_min - t_evac_min

        if t_hold_min < 0.0:
            # Process time budget too tight for this C_target
            return 0.0

        t_hold_s = t_hold_min * 60.0

        # Ramp dose: from t=0 (after dead time) to t_switch
        ramp_fill_s = max(t_sw_s - t_dead_s, 0.0)
        mg_ramp = _integrate_dose_ramp(
            C_ss_100pct, tau_eff_s, ramp_fill_s, flow_lpm, K, k_abs, V_eff
        )

        # Hold dose: constant C_target
        mg_hold = _integrate_dose_hold(C_target, t_hold_s, k_abs, V_eff, K)

        # Evac dose: C decays from C_target
        mg_evac = _integrate_dose_evac(
            C_target, tau_drain_evac_s, t_evac_s, k_abs, V_eff, K
        )

        return mg_ramp + mg_hold + mg_evac

    # Scan C_target from a small epsilon up to C_max
    C_grid = np.linspace(C_max / solver_n_grid, C_max, solver_n_grid,
                         dtype=np.float64)
    dose_grid = np.array([_predict_total_dose(c) for c in C_grid],
                         dtype=np.float64)

    # Find the C_target that delivers the target dose (interpolate)
    achievable = True
    notes_list: list[str] = []
    C_target_solved: float

    if dose_grid[-1] < target_mg:
        # Even at C_ss_100pct the dose is insufficient
        C_target_solved = float(C_grid[-1])
        achievable = False  # will also be False if power_hold > 100
        notes_list.append(
            f"Maximum achievable dose at C_ss_100pct={C_ss_100pct:.4f}%vol is "
            f"{dose_grid[-1]:.1f} mg < target {target_mg:.1f} mg.  "
            "Consider longer process_time or higher flow."
        )
    elif dose_grid[0] >= target_mg:
        # Target is met even at minimum C — use minimum C to save ozone
        C_target_solved = float(C_grid[0])
        notes_list.append("Target dose met at very low C_target; consider reducing dose.")
    else:
        # Linear interpolation between the two bounding grid points
        idx = int(np.searchsorted(dose_grid, target_mg))
        idx = max(1, min(idx, len(C_grid) - 1))
        C_lo, C_hi = float(C_grid[idx - 1]), float(C_grid[idx])
        D_lo, D_hi = float(dose_grid[idx - 1]), float(dose_grid[idx])
        if D_hi > D_lo:
            frac = (target_mg - D_lo) / (D_hi - D_lo)
            C_target_solved = C_lo + frac * (C_hi - C_lo)
        else:
            C_target_solved = C_lo

    # --- Derive power_hold from C_target ------------------------------------
    power_hold = float(predict_power_fn(C_target_solved, flow_lpm))
    power_hold = max(0.0, min(power_hold, 200.0))  # allow >100 for over-power flag
    if power_hold > 100.0:
        achievable = False
        notes_list.append(
            f"power_hold={power_hold:.1f}% exceeds 100% — target concentration "
            f"C_target={C_target_solved:.4f}%vol is not achievable at "
            f"{flow_lpm:.1f} LPM."
        )

    # --- Re-compute final schedule with solved C_target ---------------------
    C_switch = ramp_switch_fraction * C_target_solved
    t_sw_s = _ramp_switch_time_s(C_switch, C_ss_100pct, tau_eff_s) + t_dead_s
    t_evac_s = _evac_time_s(C_target_solved, evac_threshold_pct, tau_drain_evac_s)

    t_ramp_min = t_sw_s / 60.0
    t_evac_min = t_evac_s / 60.0
    t_hold_min = max(0.0, process_time_min - t_ramp_min - t_evac_min)
    t_hold_s = t_hold_min * 60.0

    ramp_fill_s = max(t_sw_s - t_dead_s, 0.0)
    mg_ramp = _integrate_dose_ramp(
        C_ss_100pct, tau_eff_s, ramp_fill_s, flow_lpm, K, k_abs, V_eff
    )
    mg_hold = _integrate_dose_hold(C_target_solved, t_hold_s, k_abs, V_eff, K)
    mg_evac = _integrate_dose_evac(
        C_target_solved, tau_drain_evac_s, t_evac_s, k_abs, V_eff, K
    )
    mg_total = mg_ramp + mg_hold + mg_evac
    dose_pred = mg_total / kg_substrate if kg_substrate > 0.0 else 0.0

    return DoseSchedule(
        # Schedule outputs
        C_target=float(C_target_solved),
        power_hold=float(min(power_hold, 200.0)),
        achievable=achievable,
        t_switch_predicted=float(t_ramp_min),
        t_hold=float(t_hold_min),
        t_evac_predicted=float(t_evac_min),
        mg_O3_ramp=float(mg_ramp),
        mg_O3_hold=float(mg_hold),
        mg_O3_evac=float(mg_evac),
        mg_O3_total=float(mg_total),
        dose_predicted=float(dose_pred),
        # Solver inputs
        target_dose_mg_per_kg=float(target_dose_mg_per_kg),
        kg_substrate=float(kg_substrate),
        process_time_min=float(process_time_min),
        flow_lpm=float(flow_lpm),
        k_d_empty=float(k_d_empty),
        k_abs=float(k_abs),
        V_residual=float(V_residual),
        c_in_100pct=float(c_in_100pct),
        ramp_switch_fraction=float(ramp_switch_fraction),
        lab_temperature_c=float(lab_temperature_c),
        evac_threshold_pct=float(evac_threshold_pct),
        notes="; ".join(notes_list) if notes_list else "OK",
    )


# =============================================================================
# DosimetryAccumulator — real-time per-sample tracking
# =============================================================================

class DosimetryAccumulator:
    """Accumulates real-time dosimetry during a batch run.

    All internal accumulators are float64.  Call ``update()`` once per
    telemetry sample with the current outlet O3 concentration, actual power,
    and elapsed time since the last sample.

    The total O3 balance per time step (dt_s):
      - Produced in vessel : C_in(power) × Q_Ls × K × dt_s
                             where C_in = predict_o3_fn(power, flow_lpm)
      - Evacuated (exhaust): C_out × Q_Ls × K × dt_s
      - Decayed (gas-phase) : k_d_empty × C_vessel × V_vessel × K × dt_s
      - Absorbed (substrate): k_abs × C_vessel × V_eff × K × dt_s
                              → this is the dose contribution

    C_vessel is approximated as C_out (well-mixed CSTR assumption — the outlet
    concentration equals the vessel concentration).

    The dose to substrate per step:
      dose_step = k_abs × C_out × V_eff × K × dt_s / kg_substrate  [mg/kg]

    Args:
        flow_lpm       : O2 concentrator flow during this batch (LPM).
        k_d_empty      : First-order O3 decay in empty vessel (s⁻¹).
        V_residual     : Volume displaced by substrate (L).
        kg_substrate   : Substrate mass (kg).
        predict_o3_fn  : (power_pct, flow_lpm) → inlet O3 %vol.
        k_abs          : O3 absorption rate by substrate (s⁻¹).
    """

    def __init__(
        self,
        flow_lpm: float,
        k_d_empty: float,
        V_residual: float,
        kg_substrate: float,
        predict_o3_fn: PredictO3Fn,
        k_abs: float = 0.0,
    ) -> None:
        if flow_lpm <= 0.0:
            raise ValueError(f"flow_lpm must be > 0, got {flow_lpm}")
        if k_d_empty < 0.0:
            raise ValueError(f"k_d_empty must be >= 0, got {k_d_empty}")
        if V_residual < 0.0 or V_residual >= V_VESSEL_L:
            raise ValueError(
                f"V_residual must be in [0, {V_VESSEL_L}), got {V_residual}"
            )
        if kg_substrate <= 0.0:
            raise ValueError(f"kg_substrate must be > 0, got {kg_substrate}")
        if k_abs < 0.0:
            raise ValueError(f"k_abs must be >= 0, got {k_abs}")

        self._flow_lpm = float(flow_lpm)
        self._k_d_empty = np.float64(k_d_empty)
        self._k_abs = np.float64(k_abs)
        self._V_eff = np.float64(V_VESSEL_L - V_residual)
        self._kg_substrate = np.float64(kg_substrate)
        self._predict_o3_fn = predict_o3_fn

        self._Q_Ls: np.float64 = np.float64(flow_lpm / 60.0)

        # Float64 accumulators
        self._mg_O3_produced: np.float64 = np.float64(0.0)
        self._mg_O3_evacuated: np.float64 = np.float64(0.0)
        self._mg_O3_decayed: np.float64 = np.float64(0.0)
        self._mg_O3_absorbed: np.float64 = np.float64(0.0)
        self._dose_running: np.float64 = np.float64(0.0)   # mg/kg

        self._n_samples: int = 0
        self._elapsed_s: np.float64 = np.float64(0.0)

    # -------------------------------------------------------------------------
    # Public interface
    # -------------------------------------------------------------------------

    def update(
        self,
        C_out_pct: float,
        power_actual_pct: float,
        dt_s: float,
        temperature_c: float = 20.0,
        vessel_temp_c: Optional[float] = None,
    ) -> None:
        """Accumulate one telemetry sample.

        Args:
            C_out_pct        : Outlet (vessel) O3 concentration (%vol).
            power_actual_pct : Actual generator power reading (%).
            dt_s             : Time since last sample (s).  Must be > 0.
            temperature_c    : Gas temperature for K computation (°C).
                               Used as lab_temperature_c fallback.
            vessel_temp_c    : Vessel temperature (°C), if available.
                               Priority: vessel_temp_c → temperature_c.
        """
        if dt_s <= 0.0:
            return  # skip zero-length or negative intervals

        # Temperature-corrected K
        T_K = get_temperature_K(vessel_temp_c, temperature_c)
        temp_c_eff = T_K - 273.15
        K = np.float64(compute_K(temp_c_eff))

        C_out = np.float64(max(0.0, C_out_pct))
        pwr = np.float64(max(0.0, min(100.0, power_actual_pct)))
        dt = np.float64(dt_s)

        # Inlet O3 (from generator model)
        C_in = np.float64(max(0.0, self._predict_o3_fn(float(pwr), self._flow_lpm)))

        # --- mg produced (generator output into vessel) ----------------------
        # Generator injects C_in at flow Q → mg/s = C_in × Q_Ls × K
        mg_produced = C_in * self._Q_Ls * K * dt
        self._mg_O3_produced += mg_produced

        # --- mg evacuated (exhaust out of vessel) ----------------------------
        # Vessel is well-mixed (CSTR): outlet = vessel concentration = C_out
        mg_evacuated = C_out * self._Q_Ls * K * dt
        self._mg_O3_evacuated += mg_evacuated

        # --- mg decayed (gas-phase first-order decay) ------------------------
        # dM_decay/dt = k_d_empty × C_out × V_eff × K
        mg_decayed = self._k_d_empty * C_out * self._V_eff * K * dt
        self._mg_O3_decayed += mg_decayed

        # --- mg absorbed by substrate ----------------------------------------
        # dM_abs/dt = k_abs × C_out × V_eff × K
        mg_absorbed = self._k_abs * C_out * self._V_eff * K * dt
        self._mg_O3_absorbed += mg_absorbed

        # --- Dose to substrate (mg/kg) ----------------------------------------
        dose_step = mg_absorbed / self._kg_substrate
        self._dose_running += np.float64(dose_step)

        self._n_samples += 1
        self._elapsed_s += dt

    # -------------------------------------------------------------------------
    # Properties
    # -------------------------------------------------------------------------

    @property
    def mg_O3_produced(self) -> float:
        """Total mg of O3 generated by the generator (injected into vessel)."""
        return float(self._mg_O3_produced)

    @property
    def mg_O3_evacuated(self) -> float:
        """Total mg of O3 exhausted from the vessel in outlet gas."""
        return float(self._mg_O3_evacuated)

    @property
    def mg_O3_decayed(self) -> float:
        """Total mg of O3 that decayed in the gas phase (first-order, k_d)."""
        return float(self._mg_O3_decayed)

    @property
    def mg_O3_absorbed(self) -> float:
        """Total mg of O3 absorbed by the substrate (k_abs pathway)."""
        return float(self._mg_O3_absorbed)

    @property
    def dose_running(self) -> float:
        """Running dose to substrate (mg/kg)."""
        return float(self._dose_running)

    @property
    def elapsed_s(self) -> float:
        """Total elapsed time accumulated (s)."""
        return float(self._elapsed_s)

    @property
    def n_samples(self) -> int:
        """Number of update() calls processed."""
        return self._n_samples

    # -------------------------------------------------------------------------
    # Serialization
    # -------------------------------------------------------------------------

    def summary_dict(self) -> dict:
        """Return a JSON-serializable summary of the accumulated dosimetry."""
        return {
            "mg_O3_produced": self.mg_O3_produced,
            "mg_O3_evacuated": self.mg_O3_evacuated,
            "mg_O3_decayed": self.mg_O3_decayed,
            "mg_O3_absorbed": self.mg_O3_absorbed,
            "dose_running_mg_per_kg": self.dose_running,
            "elapsed_s": self.elapsed_s,
            "n_samples": self.n_samples,
            "flow_lpm": self._flow_lpm,
            "k_d_empty": float(self._k_d_empty),
            "k_abs": float(self._k_abs),
            "V_eff_L": float(self._V_eff),
            "kg_substrate": float(self._kg_substrate),
        }

    def reset(self) -> None:
        """Reset all accumulators to zero (e.g., between batch phases)."""
        self._mg_O3_produced = np.float64(0.0)
        self._mg_O3_evacuated = np.float64(0.0)
        self._mg_O3_decayed = np.float64(0.0)
        self._mg_O3_absorbed = np.float64(0.0)
        self._dose_running = np.float64(0.0)
        self._n_samples = 0
        self._elapsed_s = np.float64(0.0)

    def check_divergence(self, predicted_dose_mg_per_kg: float,
                         threshold_pct: float = 15.0) -> tuple[bool, float]:
        """Check if running dose diverges from predicted dose.

        Args:
            predicted_dose_mg_per_kg: The solver-predicted dose (mg/kg).
            threshold_pct: Divergence threshold (%).

        Returns:
            Tuple of (diverged: bool, divergence_pct: float).
        """
        if predicted_dose_mg_per_kg <= 0.0:
            return False, 0.0
        divergence = abs(self.dose_running - predicted_dose_mg_per_kg) / predicted_dose_mg_per_kg * 100.0
        return divergence > threshold_pct, divergence


# =============================================================================
# Substrate config loader
# =============================================================================

_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "substrate_config.json"
)


def load_substrate_config() -> dict:
    """Load substrate_config.json; return empty dict on failure."""
    try:
        with open(_CONFIG_PATH) as f:
            return json.load(f)
    except Exception:
        return {}
