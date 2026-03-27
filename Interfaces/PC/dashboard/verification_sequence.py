"""
Verification sequence runner — sends recipe to ESP32, monitors completion,
runs stability analysis, saves CSV.

Uses the generic recipe protocol (sequence_start → seq_step → seq_run).
"""
from __future__ import annotations

import asyncio
import os
import time
from datetime import datetime

import pandas as pd

from dashboard.state import (
    S, log, _notify,
    DEFAULT_FLOW_LPM, VALIDATION_DIR,
)
from dashboard.verification import (
    generate_verify_recipe, analyze_verification, VerificationResult,
)

_POLL_INTERVAL = 2.0  # seconds between checks for sequence completion


async def _start_verification(**kwargs) -> bool:
    """Background task: run measurement verification via the recipe protocol."""
    if not S.connected:
        _notify("Not connected to ESP32", "negative")
        return False

    if S.sequence_active:
        _notify("Another sequence is already running", "negative")
        return False

    flow = kwargs.get("flow", DEFAULT_FLOW_LPM)
    power_hold = float(kwargs.get("power_hold", 50.0))

    # Build the recipe
    recipe = generate_verify_recipe(power_hold)
    total_samples = sum(h for _, _, h in recipe)

    # Initialize state
    S.verify_samples = []
    S.verify_result = None
    S.verify_file = ""
    S.sequence_active = True
    S.seq_type = "verify"
    S.seq_phase = "loading"
    S.seq_progress = 0.0
    S.seq_elapsed = 0.0
    S.seq_start_time = time.time()
    S.seq_step_idx = 0
    S.seq_step_total = len(recipe)
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""

    log(f"Verification: power_hold={power_hold:.0f}%, flow={flow} LPM, "
        f"{len(recipe)} steps / {total_samples} samples", "seq")

    try:
        from dashboard.tcp_server import tcp

        # sequence_start with relay prereqs (O2 + O3 ON, Air OFF)
        resp = await tcp.send_command(
            f"sequence_start,verify,flow={flow},"
            f"relay_o2=1,relay_o3=1,relay_air=0",
            timeout=5.0,
        )
        if not resp or "OK" not in resp:
            raise RuntimeError(f"sequence_start rejected: {resp}")

        # Load each step
        for i, (pwr, phase, hold) in enumerate(recipe):
            step_cmd = f"seq_step,{i},{pwr},{hold},{phase}"
            resp = await tcp.send_command(step_cmd, timeout=3.0)
            if not resp or "OK" not in resp:
                raise RuntimeError(f"seq_step {i} ({phase}) rejected: {resp}")

        # Start execution
        resp = await tcp.send_command("seq_run", timeout=5.0)
        if not resp or "OK" not in resp:
            raise RuntimeError(f"seq_run rejected: {resp}")

        S.seq_phase = "running"
        _notify(
            f"Verification started — {len(recipe)} steps, "
            f"~{total_samples * 2.5 / 60:.0f} min estimated",
            "positive",
        )

        # Wait for sequence to complete (ESP32 drives execution)
        while S.sequence_active and S.seq_type == "verify":
            await asyncio.sleep(_POLL_INTERVAL)
            if S.seq_step_total > 0:
                step_frac = S.seq_step_idx / S.seq_step_total
                sample_frac = len(S.verify_samples) / max(total_samples, 1)
                S.seq_progress = max(step_frac, sample_frac) * 100.0

        # Analyze results
        if S.verify_samples:
            result = analyze_verification(
                S.verify_samples, power_hold, flow
            )
            S.verify_result = result

            # Save CSV
            now = datetime.now()
            lpm_s = f"{flow:.2f}"
            tag = "PASS" if result.passed else "FAIL"
            fname = (
                f"{now:%Y-%m-%d_%H%M%S}_Verification_"
                f"{int(power_hold)}hold_{lpm_s}Lpm_{tag}.csv"
            )
            fpath = os.path.join(VALIDATION_DIR, fname)
            pd.DataFrame(S.verify_samples).to_csv(fpath, index=False)
            S.verify_file = fname
            log(f"Saved {len(S.verify_samples)} verify samples -> {fname}", "seq")

            if result.passed:
                _notify(
                    f"Verification PASSED — C_in@100%={result.c_in_100pct:.4f}%, "
                    f"C_in@hold={result.c_in_hold:.4f}%",
                    "positive",
                )
            else:
                reasons = []
                if not result.full_power_stable:
                    reasons.append("100% not stable")
                if not result.hold_power_stable:
                    reasons.append(f"{power_hold:.0f}% not stable")
                if not result.baseline_ok:
                    reasons.append("baseline elevated")
                _notify(
                    f"Verification incomplete — {', '.join(reasons)}",
                    "warning",
                )
        else:
            _notify("Verification complete (no samples)", "warning")

    except Exception as exc:
        log(f"Verification error: {exc}", "error")
        _notify(f"Verification failed: {exc}", "negative")
    finally:
        if S.seq_type == "verify":
            S.seq_phase = "complete"
            S.sequence_active = False

    return True
