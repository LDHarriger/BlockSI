"""Process batch sterilization sequence (WP-4).

Implements the hybrid resilience strategy from the WiFi disconnect audit:
the PC loads a complete recipe onto the ESP32 (so it continues autonomously
if TCP drops), then monitors execution via DATA telemetry for real-time
dosimetry.

Phases:
  1. Pre-flight       — validate params, load models
  2. Solve            — compute dosing schedule (DoseSchedule)
  3. Recipe load      — generate & send recipe to ESP32
  4. Monitor          — track progress, accumulate dosimetry per sample
  5. Post-process     — save CSV, dosimetry summary, batch log
  6. Inoculation      — operator prompts (post-treatment)
  7. Distribution     — operator prompts (post-treatment)
  8. Cleanup          — unconditional _safe_standby()
"""
from __future__ import annotations

import asyncio
import json
import os
import time
import traceback
from datetime import datetime
from typing import Optional

import numpy as np

from dashboard.state import (
    S, log, _notify,
    BATCH_DATA_DIR, MODEL_DIR, CSTR_MODEL_DIR, K_ABS_MODEL_DIR,
)
from dashboard.dosimetry import (
    DosimetryAccumulator,
    DoseSchedule,
    solve_dosing_schedule,
    load_substrate_config,
)
from analysis import (
    load_model_for_condition,
    load_cstr_model_from_dir,
    load_k_abs_model_from_dir,
    predict_o3,
    predict_power,
)
from dashboard.state import compute_effective_o2_pct
from dashboard.data_io import _find_valid_cert
from dashboard.verification import TRANSIENT_SKIP, VAL_MIN_STABLE

# =============================================================================
# Constants
# =============================================================================

_POLL_INTERVAL_S = 2.0          # Telemetry polling interval
_STALE_THRESHOLD_S = 15.0       # Warn if telemetry older than this
_PROMPT_TIMEOUT_S = 600.0       # 10 min timeout for operator prompts
_PROMPT_POLL_S = 1.0            # Prompt polling interval
_BASELINE_SAMPLES = 15          # ~37.5s at ~2.5s/sample
_SAMPLE_PERIOD_EST_S = 2.5      # Estimated, for sample count conversion
_RAMP_SAFETY_FACTOR = 1.5       # Overshoot factor for ramp sample count
_EVAC_FLUSH_EXTRA_SAMPLES = 120 # ~5 min flush after O3 below threshold
_DIVERGENCE_CHECK_INTERVAL = 60 # Samples between divergence checks
_PROGRESS_LOG_INTERVAL = 30     # Samples between progress log entries

# =============================================================================
# Debug logging
# =============================================================================

_debug_file = None


def _flog(msg: str) -> None:
    """Write to both the debug log file and the dashboard log."""
    global _debug_file
    line = f"[{datetime.now():%H:%M:%S.%f}] {msg}\n"
    if _debug_file:
        try:
            _debug_file.write(line)
            _debug_file.flush()
        except Exception:
            pass
    log(msg, "batch")


# =============================================================================
# Operator prompt helper
# =============================================================================


async def _await_operator_confirm(prompt_id: str, prompt_text: str) -> None:
    """Block until operator confirms, or raise on abort/timeout."""
    _flog(f"Operator prompt [{prompt_id}]: {prompt_text!r}")
    S.seq_confirmed = False
    S.pending_prompt_id = prompt_id
    S.pending_prompt_text = prompt_text

    deadline = time.time() + _PROMPT_TIMEOUT_S
    while not S.seq_confirmed:
        if not S.sequence_active:
            S.pending_prompt_id = ""
            raise RuntimeError(f"Batch aborted while waiting for '{prompt_id}'")
        if time.time() > deadline:
            S.pending_prompt_id = ""
            raise RuntimeError(
                f"Operator prompt '{prompt_id}' timed out "
                f"after {_PROMPT_TIMEOUT_S:.0f}s"
            )
        await asyncio.sleep(_PROMPT_POLL_S)

    S.pending_prompt_id = ""
    S.seq_confirmed = False


# =============================================================================
# Telemetry snapshot helper (factory — returns a closure)
# =============================================================================


def _make_snap():
    """Return a _snap(phase) closure that tracks dynamic dt from ESP32 ts."""
    prev_esp_ts: list[Optional[int]] = [None]

    def _snap(phase: str) -> dict:
        now = datetime.now()
        staleness = (
            (now - S.last_update).total_seconds() if S.last_update else 999.0
        )
        if staleness > _STALE_THRESHOLD_S:
            _flog(f"WARN stale telemetry in {phase}: {staleness:.1f}s ago")

        # Dynamic dt from ESP32 timestamps (NEVER hardcoded)
        dt_s = 0.0
        if (
            prev_esp_ts[0] is not None
            and S.last_esp_ts_ms > prev_esp_ts[0]
        ):
            dt_s = (S.last_esp_ts_ms - prev_esp_ts[0]) / 1000.0
        prev_esp_ts[0] = S.last_esp_ts_ms

        return {
            "timestamp": now,
            "esp_ts_ms": S.last_esp_ts_ms,
            "vessel_o3_pct": S.vessel_o3_pct,
            "cell_temp_c": S.cell_temp_c,
            "vessel_temp_c": S.vessel_temp_c,
            "power_pct": S.power_target_pct,
            "power_actual_pct": S.power_actual_pct,
            "phase": phase,
            "dt_s": dt_s,
        }

    return _snap


# =============================================================================
# CSV recording
# =============================================================================

_CSV_HEADER = (
    "timestamp,esp_ts_ms,elapsed_s,dt_s,"
    "vessel_o3_pct,cell_temp_c,vessel_temp_c,"
    "power_pct,power_actual_pct,phase,"
    "mg_produced,mg_evacuated,mg_decayed,mg_absorbed,"
    "dose_running_mg_per_kg\n"
)


def _write_csv(path: str, samples: list[dict]) -> None:
    """Write batch samples to CSV."""
    if not samples:
        return
    with open(path, "w") as f:
        f.write(_CSV_HEADER)
        t0 = samples[0].get("esp_ts_ms", 0)
        for s in samples:
            elapsed = (s.get("esp_ts_ms", 0) - t0) / 1000.0
            f.write(
                f"{s['timestamp']},{s['esp_ts_ms']},{elapsed:.2f},"
                f"{s.get('dt_s', 0.0):.3f},"
                f"{s['vessel_o3_pct']},{s['cell_temp_c']},"
                f"{s.get('vessel_temp_c', -999)},"
                f"{s.get('power_pct', 0)},{s.get('power_actual_pct', 0.0)},"
                f"{s.get('phase', '')},"
                f"{s.get('mg_produced', 0.0):.4f},"
                f"{s.get('mg_evacuated', 0.0):.4f},"
                f"{s.get('mg_decayed', 0.0):.4f},"
                f"{s.get('mg_absorbed', 0.0):.4f},"
                f"{s.get('dose_running', 0.0):.4f}\n"
            )
    log(f"Saved {len(samples)} batch samples -> {os.path.basename(path)}", "batch")


# =============================================================================
# Recipe generation
# =============================================================================


def _generate_recipe(
    schedule: DoseSchedule, config: dict
) -> list[tuple[int, int, int, str, int]]:
    """Generate ESP32 recipe steps from a DoseSchedule.

    Returns list of (idx, power_pct, hold_samples, phase_label, air_comp).
    """
    defaults = config.get("process_defaults", {})
    thresholds = config.get("thresholds", {})
    air_comp_evac = defaults.get("air_comp_evac", True)
    evac_flush_s = thresholds.get("evac_flush_duration_s", 300)

    # Convert durations to sample counts
    ramp_samples = max(
        15,
        int(schedule.t_switch_predicted * 60 / _SAMPLE_PERIOD_EST_S
            * _RAMP_SAFETY_FACTOR),
    )
    hold_samples = max(
        15, int(schedule.t_hold * 60 / _SAMPLE_PERIOD_EST_S)
    )
    evac_samples = max(
        60,
        int(schedule.t_evac_predicted * 60 / _SAMPLE_PERIOD_EST_S)
        + int(evac_flush_s / _SAMPLE_PERIOD_EST_S),
    )

    power_hold = int(round(schedule.power_hold))

    return [
        (0, 0, _BASELINE_SAMPLES, "baseline", 0),
        (1, 100, ramp_samples, "ramp", 0),
        (2, power_hold, hold_samples, "hold", 0),
        (3, 0, evac_samples, "evac", 1 if air_comp_evac else 0),
    ]


# =============================================================================
# Main sequence entry point
# =============================================================================


async def _start_process_batch(**kwargs) -> bool:
    """PC-driven process batch sterilization.

    Loads a complete recipe onto the ESP32 for autonomous execution, then
    monitors via DATA telemetry and feeds a DosimetryAccumulator in real
    time.  If TCP drops, the ESP32 continues the recipe independently.

    kwargs:
        flow            (float): O2 flow rate in LPM
        kg_substrate    (float): substrate mass (kg)
        target_dose     (float): target dose (mg/kg)
        process_time    (float): hold phase duration (min)
        experiment_type (str):   directory name (e.g. "Experiment1_Mold")
        batch_notes     (str):   optional notes
    """
    global _debug_file

    # ── Guard checks ────────────────────────────────────────────────
    if not S.connected:
        _notify("Not connected to ESP32", "negative")
        return False

    if S.sequence_active or S.fill_active:
        _notify("Another sequence is already running", "negative")
        return False

    # ── Extract parameters ──────────────────────────────────────────
    flow = float(kwargs.get("flow", 4.0))
    kg_substrate = float(kwargs.get("kg_substrate", 1.5))
    target_dose = float(kwargs.get("target_dose", 100.0))
    process_time = float(kwargs.get("process_time", 30.0))
    experiment_type = str(kwargs.get("experiment_type", "Experiment1_Mold"))
    batch_notes = str(kwargs.get("batch_notes", ""))

    # ── Load substrate config ───────────────────────────────────────
    config = load_substrate_config()
    thresholds = config.get("thresholds", {})
    ramp_switch_fraction = thresholds.get("ramp_switch_fraction", 0.85)
    evac_threshold_pct = thresholds.get("evac_threshold_pct", 0.01)
    divergence_pct = thresholds.get("dose_divergence_recal_pct", 15.0)
    baseline_max_o3 = thresholds.get("baseline_max_o3_pct", 0.02)

    # ── Load models ─────────────────────────────────────────────────
    try:
        # Power-O3 model for current flow condition
        o2_pct = compute_effective_o2_pct(flow, False)
        power_model = load_model_for_condition(MODEL_DIR, flow, o2_pct)
        if power_model is None or not power_model.is_valid:
            _notify(
                f"No valid Power-O3 model for {flow} LPM / {o2_pct}% O2",
                "negative",
            )
            return False

        # k_d model (empty vessel decay)
        k_d_model = load_cstr_model_from_dir(CSTR_MODEL_DIR)
        if k_d_model is None or not k_d_model.is_valid:
            _notify("No valid k_d model — run k_d calibration first", "negative")
            return False

        # k_abs model (substrate absorption)
        k_abs_model = load_k_abs_model_from_dir(K_ABS_MODEL_DIR)
        if k_abs_model is None or not k_abs_model.is_valid:
            _notify(
                "No valid k_abs model — run k_abs calibration first",
                "negative",
            )
            return False

        k_d_empty = k_d_model.decay_rate_per_s
        k_abs_val = k_abs_model.k_abs_per_s
        V_residual = k_abs_model.V_residual_L
        c_in_100pct = power_model.predict(100.0)

        def _predict_o3(pwr: float, flw: float) -> float:
            return predict_o3(pwr, flw, power_model)

        def _predict_power(c_target: float, flw: float) -> float:
            return predict_power(c_target, flw, power_model)

    except Exception as exc:
        _notify(f"Failed to load models: {exc}", "negative")
        log(f"Batch model load error: {exc}", "error")
        return False

    # ── Solve dosing schedule ───────────────────────────────────────
    try:
        lab_temp = S.cell_temp_c if S.cell_temp_c > 0 else 20.0
        schedule = solve_dosing_schedule(
            target_dose_mg_per_kg=target_dose,
            kg_substrate=kg_substrate,
            process_time_min=process_time,
            flow_lpm=flow,
            k_d_empty=k_d_empty,
            k_abs=k_abs_val,
            V_residual=V_residual,
            c_in_100pct=c_in_100pct,
            predict_o3_fn=_predict_o3,
            predict_power_fn=_predict_power,
            ramp_switch_fraction=ramp_switch_fraction,
            lab_temperature_c=lab_temp,
            evac_threshold_pct=evac_threshold_pct,
        )
        if not schedule.achievable:
            _notify(
                f"Target dose {target_dose} mg/kg is not achievable "
                f"at {flow} LPM — max power exceeds 100%",
                "negative",
            )
            return False
    except Exception as exc:
        _notify(f"Dosing schedule solve failed: {exc}", "negative")
        log(
            f"Batch schedule error: {exc}\n{traceback.format_exc()}", "error"
        )
        return False

    # ── Verification gate (after schedule solve, before ramp-up) ────
    import pandas as pd

    val_cert = _find_valid_cert(flow)
    if val_cert is None:
        _notify(
            "No valid verification certificate — run verification at "
            f"{flow} LPM before starting a batch",
            "negative",
        )
        return False

    # Compare measured C_in with model prediction
    try:
        vdf = pd.read_csv(val_cert)
        vdf.columns = [c.strip() for c in vdf.columns]
        vt = vdf[vdf["phase"].isin(["full_power", "target"])].iloc[TRANSIENT_SKIP:]
        if len(vt) >= VAL_MIN_STABLE:
            measured_c_in = float(vt["o3_pct"].mean())
            if c_in_100pct > 0.01:
                divergence = abs(measured_c_in - c_in_100pct) / c_in_100pct
                log(f"Batch C_in check: measured={measured_c_in:.4f}% "
                    f"model={c_in_100pct:.4f}% divergence={divergence:.1%} "
                    f"({os.path.basename(val_cert)})", "info")
                if divergence > 0.15:
                    _notify(
                        f"Verification C_in ({measured_c_in:.3f}%) diverges "
                        f"{divergence:.0%} from model ({c_in_100pct:.3f}%). "
                        f"Consider re-running Power-O3 calibration.",
                        "warning",
                    )
    except Exception as exc:
        log(f"Could not read verification cert for C_in check: {exc}", "warning")

    # ── Generate recipe ─────────────────────────────────────────────
    recipe = _generate_recipe(schedule, config)
    total_samples = sum(step[2] for step in recipe)

    # ── Prepare output directory ────────────────────────────────────
    batch_id = f"{datetime.now():%Y%m%d_%H%M%S}"
    batch_dir = os.path.join(BATCH_DATA_DIR, experiment_type, batch_id)
    os.makedirs(batch_dir, exist_ok=True)

    csv_path = os.path.join(batch_dir, f"{batch_id}_batch.csv")
    debug_path = os.path.join(batch_dir, f"{batch_id}_debug.log")
    schedule_path = os.path.join(batch_dir, f"{batch_id}_schedule.json")
    summary_path = os.path.join(batch_dir, f"{batch_id}_dosimetry.json")

    # ── Initialize SystemState ──────────────────────────────────────
    S.sequence_active = True
    S.seq_type = "process_batch"
    S.seq_phase = "loading"
    S.seq_progress = 0.0
    S.seq_start_time = time.time()
    S.seq_elapsed = 0.0
    S.seq_confirmed = False
    S.seq_cleanup_pending = False
    S.seq_step_idx = 0
    S.seq_step_total = len(recipe)
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    S.batch_samples = []
    S.batch_dose_running = 0.0
    S.batch_dose_target = target_dose
    S.batch_schedule = schedule

    # ── Initialize dosimetry accumulator ────────────────────────────
    accumulator = DosimetryAccumulator(
        flow_lpm=flow,
        k_d_empty=k_d_empty,
        V_residual=V_residual,
        kg_substrate=kg_substrate,
        predict_o3_fn=_predict_o3,
        k_abs=k_abs_val,
    )

    samples: list[dict] = []
    _snap = _make_snap()
    outcome = "unknown"

    try:
        # ── Open debug log ──────────────────────────────────────────
        _debug_file = open(debug_path, "w")
        _flog(f"Batch started: {experiment_type}/{batch_id}")
        _flog(
            f"Parameters: flow={flow} LPM, kg={kg_substrate}, "
            f"target_dose={target_dose} mg/kg, process_time={process_time} min"
        )
        _flog(
            f"Models: k_d={k_d_empty:.6e}/s, k_abs={k_abs_val:.6e}/s, "
            f"V_res={V_residual:.3f} L, c_in@100%={c_in_100pct:.4f}%"
        )
        _flog(f"Schedule: C_target={schedule.C_target:.4f}%, "
              f"power_hold={schedule.power_hold:.1f}%, "
              f"predicted_dose={schedule.dose_predicted:.2f} mg/kg")
        _flog(f"Recipe ({len(recipe)} steps, ~{total_samples} samples):")
        for idx, pwr, hold, phase, air in recipe:
            _flog(f"  Step {idx}: {phase} @ {pwr}% x {hold} samples, air={air}")

        # Save schedule for offline analysis
        with open(schedule_path, "w") as f:
            f.write(schedule.to_json())
        _flog(f"Schedule saved: {schedule_path}")

        # ── Load recipe onto ESP32 ──────────────────────────────────
        from dashboard.tcp_server import tcp

        # sequence_start with relay prereqs (O2 + O3 ON, Air OFF)
        resp = await tcp.send_command(
            f"sequence_start,process_batch,flow={flow},"
            f"relay_o2=1,relay_o3=1,relay_air=0",
            timeout=5.0,
        )
        if not resp or "OK" not in resp:
            raise RuntimeError(f"sequence_start rejected: {resp}")
        _flog(f"sequence_start -> {resp}")

        # Load each step
        for idx, pwr, hold, phase, air in recipe:
            step_cmd = f"seq_step,{idx},{pwr},{hold},{phase}"
            if air:
                step_cmd += f",{air}"
            resp = await tcp.send_command(step_cmd, timeout=3.0)
            if not resp or "OK" not in resp:
                raise RuntimeError(f"seq_step {idx} ({phase}) rejected: {resp}")
            _flog(f"seq_step {idx} -> OK")

        # Start execution
        resp = await tcp.send_command("seq_run", timeout=5.0)
        if not resp or "OK" not in resp:
            raise RuntimeError(f"seq_run rejected: {resp}")
        _flog("seq_run -> OK — recipe executing on ESP32")

        S.seq_phase = "running"
        _notify(
            f"Batch started — {len(recipe)} steps, "
            f"~{total_samples * _SAMPLE_PERIOD_EST_S / 60:.0f} min estimated",
            "positive",
        )

        # ── Monitoring loop ─────────────────────────────────────────
        # The ESP32 controls power per the loaded recipe.
        # We passively observe DATA telemetry and accumulate dosimetry.

        sample_idx = 0
        last_divergence_check = 0

        while S.sequence_active and S.connected:
            await asyncio.sleep(_POLL_INTERVAL_S)

            if not S.sequence_active:
                break

            # Snapshot from live telemetry (phase tracks ESP32 SEQ STEP)
            phase = S.seq_phase if S.seq_phase else "unknown"
            snap = _snap(phase)

            # Feed dosimetry accumulator
            if snap["dt_s"] > 0:
                accumulator.update(
                    C_out_pct=snap["vessel_o3_pct"],
                    power_actual_pct=snap["power_actual_pct"],
                    dt_s=snap["dt_s"],
                    temperature_c=snap["cell_temp_c"],
                    vessel_temp_c=snap.get("vessel_temp_c"),
                )

            # Enrich sample with cumulative dosimetry
            snap["mg_produced"] = accumulator.mg_O3_produced
            snap["mg_evacuated"] = accumulator.mg_O3_evacuated
            snap["mg_decayed"] = accumulator.mg_O3_decayed
            snap["mg_absorbed"] = accumulator.mg_O3_absorbed
            snap["dose_running"] = accumulator.dose_running
            samples.append(snap)

            # Mirror to SystemState for UI observer
            S.batch_dose_running = accumulator.dose_running
            S.seq_elapsed = time.time() - S.seq_start_time

            # Progress: use ESP32 step index (updated by tcp_server SEQ STEP)
            if S.seq_step_total > 0:
                step_frac = S.seq_step_idx / S.seq_step_total
            else:
                step_frac = sample_idx / max(1, total_samples)
            S.seq_progress = min(99.0, step_frac * 100.0)

            sample_idx += 1

            # Periodic divergence check during hold phase
            if (
                phase == "hold"
                and sample_idx - last_divergence_check
                >= _DIVERGENCE_CHECK_INTERVAL
            ):
                last_divergence_check = sample_idx
                diverged, div_pct = accumulator.check_divergence(
                    schedule.dose_predicted, divergence_pct
                )
                if diverged:
                    _flog(
                        f"WARN dose divergence: {div_pct:.1f}% "
                        f"> {divergence_pct}% threshold"
                    )
                    _notify(
                        f"Dose divergence: {div_pct:.1f}% "
                        "— consider recalibration",
                        "warning",
                    )

            # Periodic progress log
            if sample_idx % _PROGRESS_LOG_INTERVAL == 0:
                dose_pct = (
                    accumulator.dose_running / target_dose * 100
                    if target_dose > 0
                    else 0
                )
                _flog(
                    f"Sample {sample_idx}: phase={phase}, "
                    f"O3={snap['vessel_o3_pct']:.4f}%, "
                    f"pwr_act={snap['power_actual_pct']:.1f}%, "
                    f"dose={accumulator.dose_running:.2f} mg/kg "
                    f"({dose_pct:.1f}% of target)"
                )

        # ── Determine outcome ───────────────────────────────────────
        if S.seq_cleanup_pending or S.seq_phase == "saving":
            _flog("Recipe COMPLETE — ESP32 finished all steps")
            S.seq_progress = 100.0
            outcome = "complete"
        elif not S.connected:
            _flog(
                "WARNING: TCP disconnected. "
                "ESP32 continues recipe autonomously."
            )
            _notify(
                "Disconnected — ESP32 is still running the batch recipe!",
                "warning",
            )
            outcome = "disconnected"
        else:
            _flog("Batch ended — sequence no longer active (possibly aborted)")
            outcome = "aborted"

        # ── Dosimetry summary ───────────────────────────────────────
        dosimetry_summary = accumulator.summary_dict()
        dosimetry_summary.update({
            "outcome": outcome,
            "target_dose_mg_per_kg": target_dose,
            "schedule_predicted_dose": schedule.dose_predicted,
            "batch_id": batch_id,
            "experiment_type": experiment_type,
            "batch_notes": batch_notes,
            "process_time_min": process_time,
            "C_target_pct": schedule.C_target,
            "power_hold_pct": schedule.power_hold,
        })
        with open(summary_path, "w") as f:
            json.dump(dosimetry_summary, f, indent=2)
        _flog(f"Dosimetry summary saved: {summary_path}")
        _flog(
            f"Final dose: {accumulator.dose_running:.2f} mg/kg "
            f"(target: {target_dose}, "
            f"predicted: {schedule.dose_predicted:.2f})"
        )

        # ── Post-treatment prompts (only if complete & connected) ───
        if outcome == "complete" and S.connected:
            S.seq_phase = "inoculation"
            try:
                await _await_operator_confirm(
                    "batch_vessel_cool",
                    "Verify vessel has cooled to safe temperature for "
                    "inoculation (O3 < 0.01% vol).",
                )
                await _await_operator_confirm(
                    "batch_add_inoculant",
                    "Open vessel, add inoculant, close vessel. "
                    "Confirm when done.",
                )
                _flog("Inoculation prompts confirmed")
            except RuntimeError as exc:
                _flog(f"Inoculation prompt skipped: {exc}")

            S.seq_phase = "distribution"
            try:
                await _await_operator_confirm(
                    "batch_distribute",
                    "Mix substrate, distribute into bags/containers, "
                    f"label with Batch ID: {batch_id}. Confirm when done.",
                )
                _flog("Distribution prompt confirmed")
            except RuntimeError as exc:
                _flog(f"Distribution prompt skipped: {exc}")

        # Final notification
        if outcome == "complete":
            _notify(
                f"Batch {batch_id} complete — "
                f"dose: {accumulator.dose_running:.1f} mg/kg",
                "positive",
            )
        elif outcome == "disconnected":
            _notify(
                f"Batch {batch_id} — PC disconnected, ESP32 continues. "
                f"Last tracked dose: {accumulator.dose_running:.1f} mg/kg",
                "warning",
            )
        else:
            _notify(f"Batch {batch_id} aborted", "negative")

        return outcome == "complete"

    except RuntimeError as exc:
        _flog(f"ERROR (RuntimeError): {exc}\n{traceback.format_exc()}")
        _notify(f"Batch error: {exc}", "negative")
        outcome = "error"
        # Attempt to abort ESP32 recipe
        try:
            from dashboard.commands import cmd_sequence_abort
            await cmd_sequence_abort(f"pc_error")
        except Exception:
            pass
        return False

    except Exception as exc:
        _flog(f"UNEXPECTED ERROR: {exc}\n{traceback.format_exc()}")
        _notify(f"Batch failed: {exc}", "negative")
        outcome = "error"
        try:
            from dashboard.commands import cmd_sequence_abort
            await cmd_sequence_abort(f"pc_error")
        except Exception:
            pass
        return False

    finally:
        # ── Unconditional cleanup ───────────────────────────────────
        # Save CSV (even on error — partial data is still valuable)
        _write_csv(csv_path, samples)

        # Safe standby: power=0, all relays off
        from dashboard.commands import _safe_standby
        await _safe_standby()

        # Reset SystemState
        S.seq_phase = "complete"
        S.sequence_active = False
        S.seq_cleanup_pending = False
        S.pending_prompt_id = ""
        S.pending_prompt_text = ""
        S.batch_schedule = None

        # Close debug log
        if _debug_file:
            _flog(f"Batch sequence ended — cleanup complete (outcome={outcome})")
            _debug_file.close()
            _debug_file = None
