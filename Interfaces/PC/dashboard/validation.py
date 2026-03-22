"""
Validation analysis — recipe generation and statistical computation.

Recipe structure (Section 14):
    Step 0: power=0,     phase=baseline,    hold=15  (verify no residual O3)
    Step 1: random(20-50), phase=spot_low,  hold=10  (model check low range)
    Step 2: random(50-80), phase=spot_medium,hold=10 (model check mid range)
    Step 3: power=100,   phase=spot_max,    hold=10  (model check max power)
    Step 4: P_target,    phase=target,      hold=15  (validate specific target)
    Step 5: power=0,     phase=cooldown,    hold=5   (safe shutdown)

Constraints:
    - spot_low and spot_medium must NOT equal P_target.
    - If P_target == 100, spot_max and target collapse to one hold at 100%
      with 15 samples.
"""
from __future__ import annotations

import random as _rng
from typing import Any

import numpy as np

from dashboard.state import predict_o3_from_power

# Validation constants
VAL_TRANSIENT_SKIP = 7
VAL_MIN_STABLE = 3


def generate_val_recipe(
    p_target: int,
) -> list[tuple[int, str, int]]:
    """Return a validation recipe as [(power, phase_label, hold_samples), ...].

    Spot powers are drawn randomly and constrained so they never equal
    P_target.  If P_target == 100 the spot_max and target steps collapse
    into a single 15-sample hold at 100%.
    """
    recipe: list[tuple[int, str, int]] = []

    # Step 0 — baseline
    recipe.append((0, "baseline", 15))

    # Step 1 — spot_low  (random 20-50, != P_target)
    spot_low = _rng.randint(20, 50)
    while spot_low == p_target:
        spot_low = _rng.randint(20, 50)
    recipe.append((spot_low, "spot_low", 10))

    # Step 2 — spot_medium  (random 50-80, != P_target, != spot_low)
    spot_med = _rng.randint(50, 80)
    while spot_med == p_target or spot_med == spot_low:
        spot_med = _rng.randint(50, 80)
    recipe.append((spot_med, "spot_medium", 10))

    # Step 3 — spot_max  (100%)
    # Step 4 — target    (P_target)
    # If P_target == 100 they collapse into one step with 15 samples.
    if p_target == 100:
        recipe.append((100, "target", 15))
    else:
        recipe.append((100, "spot_max", 10))
        recipe.append((p_target, "target", 15))

    # Step 5 — cooldown
    recipe.append((0, "cooldown", 5))

    return recipe


def _analyze_validation(samples: list[dict], power_pct: float,
                        flow_lpm: float) -> dict:
    """Analyze validation samples and determine pass/fail.

    Each non-baseline/cooldown phase discards the first VAL_TRANSIENT_SKIP
    samples to account for motor pot settling + gas transit to the 106-H
    sensor.  Statistics are computed only on the remaining stable-state tail.

    Criteria (same for ALL spot phases and target):
    - Baseline: mean O3 < 0.02 %vol (no skip -- trivially stable at 0%)
    - Spot / target accuracy: within 0.15 %vol OR 15% relative of model
    - Spot / target stability: CV < 5%
    """
    skip = VAL_TRANSIENT_SKIP
    result: dict[str, Any] = {
        "power": power_pct,
        "flow": flow_lpm,
        "total_samples": len(samples),
        "transient_skip": skip,
    }

    # Group by phase
    by_phase: dict[str, list[dict]] = {}
    for s in samples:
        phase = s.get("phase", "unknown")
        by_phase.setdefault(phase, []).append(s)

    # Baseline check
    baseline = by_phase.get("baseline", [])
    if baseline:
        bl_mean = float(np.mean([s["o3_pct"] for s in baseline]))
        result["baseline_mean"] = bl_mean
        result["baseline_ok"] = bl_mean <= 0.02
    else:
        result["baseline_mean"] = 0.0
        result["baseline_ok"] = True

    # Spot / target correlation checks  (unified criteria for all phases)
    spot_phases = ("spot_low", "spot_medium", "spot_max", "target")
    spot_checks = []
    for phase_name in spot_phases:
        phase_data = by_phase.get(phase_name, [])
        if not phase_data:
            continue
        stable = phase_data[skip:]
        spot_power = phase_data[0].get("power_target", 0)
        expected = predict_o3_from_power(spot_power, flow_lpm)
        if len(stable) < VAL_MIN_STABLE:
            spot_checks.append({
                "phase": phase_name, "mean_o3": 0.0,
                "expected_o3": expected, "abs_err": 0.0,
                "rel_err": 0.0, "cv_pct": 0.0, "ok": None,
                "stable_samples": len(stable),
                "skip_reason": "insufficient_stable_data",
            })
            continue
        vals = [s["o3_pct"] for s in stable]
        spot_mean = float(np.mean(vals))
        spot_std = float(np.std(vals))
        abs_err = abs(spot_mean - expected)
        rel_err = (abs_err / expected * 100) if expected > 0 else 0
        cv = (spot_std / spot_mean * 100) if spot_mean > 0 else 0.0
        accuracy_ok = abs_err < 0.15 or rel_err < 15
        stability_ok = cv < 5.0
        spot_checks.append({
            "phase": phase_name, "mean_o3": spot_mean,
            "expected_o3": expected, "abs_err": abs_err,
            "rel_err": rel_err, "cv_pct": cv,
            "ok": accuracy_ok and stability_ok,
            "stable_samples": len(stable),
        })
    result["spot_checks"] = spot_checks
    result["spots_ok"] = all(
        sc["ok"] for sc in spot_checks if sc["ok"] is not None
    )

    # Target summary (convenience fields used by the UI result card)
    target = by_phase.get("target", [])
    target_stable = target[skip:]
    if len(target_stable) >= VAL_MIN_STABLE:
        t_o3 = [s["o3_pct"] for s in target_stable]
        mean_o3 = float(np.mean(t_o3))
        std_o3 = float(np.std(t_o3))
        mean_temp = float(np.mean([s["temp_c"] for s in target_stable]))
        expected_o3 = predict_o3_from_power(power_pct, flow_lpm)
        deviation_pct = (
            abs(mean_o3 - expected_o3) / expected_o3 * 100
            if expected_o3 > 0 else 0.0
        )
        cv = (std_o3 / mean_o3 * 100) if mean_o3 > 0 else 0.0

        result.update({
            "mean_o3": mean_o3,
            "std_o3": std_o3,
            "expected_o3": expected_o3,
            "deviation_pct": deviation_pct,
            "cv_pct": cv,
            "mean_temp": mean_temp,
            "target_samples": len(target),
            "stable_samples": len(target_stable),
            "target_ok": (abs(mean_o3 - expected_o3) < 0.15
                          or deviation_pct < 15),
            "stable": cv < 5.0,
        })
    else:
        result.update({
            "mean_o3": 0.0, "std_o3": 0.0, "expected_o3": 0.0,
            "deviation_pct": 0.0, "cv_pct": 0.0, "mean_temp": 0.0,
            "target_samples": len(target), "stable_samples": 0,
            "target_ok": False, "stable": False,
        })

    # Overall pass/fail (baseline is advisory only)
    result["passed"] = (
        result.get("spots_ok", True)
        and result.get("target_ok", False)
        and result.get("stable", False)
    )
    return result
