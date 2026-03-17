"""
Validation analysis — pure statistical computation on sample lists.
"""
from __future__ import annotations

from typing import Any

import numpy as np

from dashboard.state import predict_o3_from_power

# Validation constants
VAL_TRANSIENT_SKIP = 7
VAL_MIN_STABLE = 3


def _analyze_validation(samples: list[dict], power_pct: float,
                        flow_lpm: float) -> dict:
    """Analyze validation samples and determine pass/fail.

    Each non-baseline phase discards the first VAL_TRANSIENT_SKIP samples
    to account for motor pot settling + gas transit to the 106-H sensor.
    Statistics are computed only on the remaining stable-state tail.

    Criteria:
    - Baseline: mean O3 < 0.02 %vol (no skip — trivially stable at 0%)
    - Spot correlation: stable mean within 0.15 %vol or 15% relative of model
    - Target accuracy: stable mean within 10% relative of prediction
    - Target stability: CV of stable window < 5%
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

    # Spot correlation checks
    spot_checks = []
    for phase_name in ("spot_low", "spot_high"):
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
                "rel_err": 0.0, "ok": None,
                "stable_samples": len(stable),
                "skip_reason": "insufficient_stable_data",
            })
            continue
        spot_mean = float(np.mean([s["o3_pct"] for s in stable]))
        abs_err = abs(spot_mean - expected)
        rel_err = (abs_err / expected * 100) if expected > 0 else 0
        spot_ok = abs_err < 0.15 or rel_err < 15
        spot_checks.append({
            "phase": phase_name, "mean_o3": spot_mean,
            "expected_o3": expected, "abs_err": abs_err,
            "rel_err": rel_err, "ok": spot_ok,
            "stable_samples": len(stable),
        })
    result["spot_checks"] = spot_checks
    result["spots_ok"] = all(
        sc["ok"] for sc in spot_checks if sc["ok"] is not None
    )

    # Target analysis
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
            "target_ok": deviation_pct < 10,
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
