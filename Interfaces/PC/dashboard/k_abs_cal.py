"""
k_abs calibration sequence — PC-driven fill/hold/evacuation for loaded-vessel
k_abs (O3 absorption rate) and V_residual fitting.

Context:
  k_d (empty-vessel decay) is already known from a prior k_d calibration.
  This sequence fills a LOADED vessel (substrate present) at 100% power for
  a fixed 30-minute hold, records continuous 106-H data, then evacuates.
  The fitter extracts k_abs (substrate absorption) and V_residual (residual
  gas volume) from the loaded-vessel CSTR response, given k_d as a fixed
  known parameter.

Protocol:
  1. Operator loads substrate using experimental preset recipe. Mixing screw
     runs manually at 25–30 RPM.
  2. Prompt: "Confirm mixing screw is running at 25–30 RPM" — Confirm only.
  3. Prompt: "Confirm L-valve is set to VESSEL route" — Confirm only.
  4. Baseline: 15 samples at 0% power. Verify O3 < 0.02% vol.
  5. Fill + Hold: 100% power for 30 minutes (fixed). Record every 106-H
     sample as continuous CSV (phase="fill_hold").
  6. Evac: 0% power, O2 continues. When C_out < 0.01% vol, begin 5-minute
     flush, then end.
  7. Call model fitter (analysis.cstr_k_abs_model).

CSV columns:
  timestamp, esp_ts_ms, elapsed_s, dt_s, vessel_o3_pct, cell_temp_c,
  vessel_temp_c, power_actual_pct, phase

File naming:
  YYYYMMDD_HHMMSS_k_abs_cal_{LPM}Lpm.csv  in  Interfaces/Data/k_abs_cal/

NiceGUI pitfall §1 — operator prompts:
  ui.dialog() cannot be opened from a background task. Instead, the
  sequence sets S.pending_prompt_id and S.pending_prompt_text, then waits
  for the UI thread to set S.seq_confirmed=True via its normal confirm
  handler. The UI must also reset S.seq_confirmed=False before raising a
  new prompt.
"""
from __future__ import annotations

import asyncio
import os
import re as _re
import time
import traceback
from datetime import datetime
from typing import Optional

import numpy as np

from dashboard.state import (
    S, log, _notify,
    DEFAULT_FLOW_LPM, BASE_DIR, DATA_DIR,
    K_ABS_DATA_DIR, K_ABS_MODEL_DIR,
    predict_o3_from_power,
)

# =============================================================================
# Calibration constants
# =============================================================================
KABS_FILL_HOLD_DURATION_S = 1800.0     # Fixed 30-minute hold at 100% power
KABS_BASELINE_SAMPLES = 15
KABS_BASELINE_MAX_O3_PCT = 0.02        # Must be < 0.02% vol before fill
KABS_EVAC_THRESHOLD_PCT = 0.01         # C_out < 0.01% vol triggers flush
KABS_EVAC_FLUSH_DURATION_S = 300.0    # 5-minute flush after threshold
KABS_STALE_THRESHOLD = 8.0             # Telemetry staleness warning (s)
KABS_CMD_RETRIES = 3
KABS_CMD_RETRY_DELAY = 1.5
KABS_CHECKPOINT_INTERVAL = 10          # Save checkpoint every N samples
KABS_POWER_SET_TIMEOUT = 25.0
KABS_POWER_TOLERANCE = 5.0
KABS_FILL_POWER_MIN_PCT = 80.0         # Watchdog: power must stay above this
KABS_FILL_POWER_GRACE_SAMPLES = 5      # Samples before watchdog activates
KABS_FILL_POWER_RESEND_MAX = 3
KABS_PROMPT_POLL_INTERVAL_S = 0.5      # How often to check S.seq_confirmed
KABS_PROMPT_TIMEOUT_S = 300.0          # 5-minute operator timeout

_POLL_INTERVAL_S = 2.0


# =============================================================================
# CSV helpers
# =============================================================================
def _make_k_abs_csv_path(lpm: float) -> str:
    now = datetime.now()
    lpm_s = f"{lpm:.0f}" if lpm == int(lpm) else f"{lpm:.1f}"
    fname = f"{now:%Y%m%d_%H%M%S}_k_abs_cal_{lpm_s}Lpm.csv"
    return os.path.join(K_ABS_DATA_DIR, fname)


def _write_k_abs_csv(path: str, samples: list[dict]) -> None:
    """Write k_abs calibration samples to CSV. Idempotent; overwrites path."""
    if not samples:
        return
    header = (
        "timestamp,esp_ts_ms,elapsed_s,dt_s,"
        "vessel_o3_pct,cell_temp_c,vessel_temp_c,"
        "power_actual_pct,phase\n"
    )
    with open(path, "w") as f:
        f.write(header)
        t0 = samples[0].get("esp_ts_ms", 0)
        for s in samples:
            elapsed = (s.get("esp_ts_ms", 0) - t0) / 1000.0
            f.write(
                f"{s['timestamp']},{s['esp_ts_ms']},{elapsed:.2f},"
                f"{s.get('dt_s', 0.0):.3f},"
                f"{s['vessel_o3_pct']},{s['cell_temp_c']},"
                f"{s.get('vessel_temp_c', -999)},"
                f"{s.get('power_actual_pct', 0.0)},"
                f"{s.get('phase', '')}\n"
            )
    log(f"Saved {len(samples)} samples -> {os.path.basename(path)}", "seq")


# =============================================================================
# Debug file logger
# =============================================================================
_kabs_debug_file: Optional[str] = None


def _kabs_flog(msg: str) -> None:
    """Write msg to both the in-memory debug_log (category='seq') and the
    per-run debug file (if open)."""
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log(msg, "seq")
    if _kabs_debug_file:
        try:
            with open(_kabs_debug_file, "a") as _f:
                _f.write(f"[{ts}] {msg}\n")
        except OSError:
            pass


def _kabs_flog_state(label: str) -> None:
    _kabs_flog(
        f"STATE@{label}: connected={S.connected} "
        f"O3={S.vessel_o3_pct:.4f}% pwr_tgt={S.power_target_pct}% "
        f"pwr_act={S.power_actual_pct:.1f}% "
        f"last_update={S.last_update.strftime('%H:%M:%S') if S.last_update else 'None'} "
        f"esp_ts={S.last_esp_ts_ms} "
        f"relay(o3={S.relay_o3_gen} o2={S.relay_o2_conc} air={S.relay_air_comp}) "
        f"seq_cleanup_pending={S.seq_cleanup_pending}"
    )


# =============================================================================
# Command wrappers (with retries / telemetry fallback)
# =============================================================================
async def _kabs_set_power(pct: int) -> None:
    from dashboard.commands import cmd_set_power
    _kabs_flog(
        f"cmd_set_power({pct}%) — single attempt, timeout={KABS_POWER_SET_TIMEOUT}s"
    )
    ok = await cmd_set_power(pct, timeout=KABS_POWER_SET_TIMEOUT)
    if ok:
        _kabs_flog(
            f"cmd_set_power({pct}%) -> RSP OK (pwr_act={S.power_actual_pct:.1f}%)"
        )
        return

    _kabs_flog(
        f"cmd_set_power({pct}%) RSP timeout/failed "
        f"(connected={S.connected}, pwr_act={S.power_actual_pct:.1f}%)"
    )
    if abs(S.power_actual_pct - pct) <= KABS_POWER_TOLERANCE:
        _kabs_flog(
            f"WARN cmd_set_power({pct}%) RSP not received but "
            f"pwr_act={S.power_actual_pct:.1f}% is within {KABS_POWER_TOLERANCE}% "
            f"of target — treating as success (telemetry fallback)"
        )
        S.power_target_pct = int(pct)
        S.update_derived()
        return

    raise RuntimeError(
        f"Failed to set power to {pct}%: "
        f"RSP timed out and pwr_act={S.power_actual_pct:.1f}% is not near target"
    )


async def _kabs_set_relay(name: str, on: bool) -> None:
    from dashboard.commands import cmd_set_relay
    for attempt in range(1, KABS_CMD_RETRIES + 1):
        _kabs_flog(
            f"cmd_set_relay({name},{on}) attempt {attempt}/{KABS_CMD_RETRIES}"
        )
        ok = await cmd_set_relay(name, on)
        if ok:
            _kabs_flog(f"cmd_set_relay({name},{on}) -> OK")
            return
        _kabs_flog(f"cmd_set_relay({name},{on}) attempt {attempt} FAILED")
        if attempt < KABS_CMD_RETRIES:
            await asyncio.sleep(KABS_CMD_RETRY_DELAY)
    raise RuntimeError(
        f"Failed to set relay {name}={on} after {KABS_CMD_RETRIES} attempts"
    )


# =============================================================================
# Operator prompt helper (NiceGUI-safe: no ui.dialog from background task)
# =============================================================================
async def _kabs_await_operator_confirm(
    prompt_id: str, prompt_text: str
) -> None:
    """Set a pending operator prompt on S and block until S.seq_confirmed is set.

    The UI thread must detect S.pending_prompt_id != "" and show the dialog,
    then set S.seq_confirmed = True when the operator presses Confirm.
    The UI thread must also reset S.seq_confirmed = False before this call
    so we don't accidentally capture a stale True.

    Raises RuntimeError if:
      - fill_active is cleared (sequence aborted) while waiting, or
      - the operator does not confirm within KABS_PROMPT_TIMEOUT_S seconds.
    """
    _kabs_flog(f"Operator prompt [{prompt_id}]: {prompt_text!r}")
    S.seq_confirmed = False
    S.pending_prompt_id = prompt_id
    S.pending_prompt_text = prompt_text

    deadline = time.time() + KABS_PROMPT_TIMEOUT_S
    while not S.seq_confirmed:
        if not S.fill_active:
            S.pending_prompt_id = ""
            S.pending_prompt_text = ""
            raise RuntimeError(
                f"k_abs calibration aborted while waiting for operator "
                f"prompt '{prompt_id}'"
            )
        if time.time() > deadline:
            S.pending_prompt_id = ""
            S.pending_prompt_text = ""
            raise RuntimeError(
                f"Operator prompt '{prompt_id}' timed out after "
                f"{KABS_PROMPT_TIMEOUT_S:.0f}s — no operator response"
            )
        await asyncio.sleep(KABS_PROMPT_POLL_INTERVAL_S)

    # Clear prompt state after confirmation
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    S.seq_confirmed = False
    _kabs_flog(f"Operator confirmed prompt [{prompt_id}]")


# =============================================================================
# Main sequence coroutine
# =============================================================================
async def _start_k_abs_cal(**kwargs) -> bool:
    """PC-driven k_abs calibration — loaded-vessel fill/hold/evacuation.

    kwargs:
        flow  (float): O2 flow rate in LPM (default DEFAULT_FLOW_LPM)
    """
    global _kabs_debug_file

    if not S.connected:
        _notify("Not connected to ESP32", "negative")
        return False

    if S.sequence_active or S.fill_active:
        _notify("Another sequence is already running", "negative")
        return False

    flow = float(kwargs.get("flow", DEFAULT_FLOW_LPM))

    # Initialize sequence state
    S.fill_active = True
    S.fill_phase = "setup"
    S.cstr_samples = []
    S.cstr_csv_path = ""
    S.fill_lpm = flow
    S.fill_target_o3 = predict_o3_from_power(100, flow)
    S.sequence_active = True
    S.seq_type = "k_abs_cal"
    S.seq_phase = "setup"
    S.seq_progress = 0.0
    S.seq_start_time = time.time()
    S.seq_cleanup_pending = False
    S.seq_confirmed = False
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""

    now_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    _kabs_debug_file = os.path.join(
        K_ABS_DATA_DIR, f"{now_tag}_k_abs_cal_debug.log"
    )

    _kabs_flog(
        f"k_abs calibration starting: flow={flow} LPM, "
        f"fill_hold={KABS_FILL_HOLD_DURATION_S:.0f}s "
        f"({KABS_FILL_HOLD_DURATION_S/60:.0f}min fixed)"
    )
    _kabs_flog_state("start")

    # Collect samples into S.cstr_samples (shared with UI observers)
    # Track local k_abs samples separately to avoid confusion
    kabs_samples: list[dict] = []
    prev_esp_ts: int | None = None

    def _snap(phase: str) -> dict:
        nonlocal prev_esp_ts
        now = datetime.now()
        staleness = (
            (now - S.last_update).total_seconds() if S.last_update else 999.0
        )
        if staleness > KABS_STALE_THRESHOLD:
            _kabs_flog(
                f"WARN stale telemetry in {phase}: last_update "
                f"{staleness:.1f}s ago (>{KABS_STALE_THRESHOLD}s) — "
                f"O3={S.vessel_o3_pct:.4f}% may be stale"
            )
        # Dynamic dt from ESP32 timestamps — NEVER hardcoded
        dt_s = 0.0
        if prev_esp_ts is not None and S.last_esp_ts_ms > prev_esp_ts:
            dt_s = (S.last_esp_ts_ms - prev_esp_ts) / 1000.0
        prev_esp_ts = S.last_esp_ts_ms

        return {
            "timestamp": now,
            "esp_ts_ms": S.last_esp_ts_ms,
            "vessel_o3_pct": S.vessel_o3_pct,
            "cell_temp_c": S.cell_temp_c,
            "vessel_temp_c": S.vessel_temp_c,
            "power_actual_pct": S.power_actual_pct,
            "phase": phase,
            "dt_s": dt_s,
            "_staleness_s": round(staleness, 2),
        }

    def _checkpoint(label: str) -> None:
        if not kabs_samples:
            return
        try:
            ckpt_path = os.path.join(
                K_ABS_DATA_DIR,
                f"{now_tag}_k_abs_cal_checkpoint_{label}.csv",
            )
            _write_k_abs_csv(ckpt_path, kabs_samples)
            _kabs_flog(
                f"Checkpoint saved: {os.path.basename(ckpt_path)} "
                f"({len(kabs_samples)} samples)"
            )
        except OSError as ckpt_err:
            _kabs_flog(f"WARN checkpoint write failed: {ckpt_err}")

    try:
        # ------------------------------------------------------------------
        # Phase 1: Relay setup
        # ------------------------------------------------------------------
        S.fill_phase = "relay_setup"
        S.seq_phase = "relay_setup"
        _kabs_flog("Phase 1: relay setup")

        await _kabs_set_power(0)

        if not S.relay_o3_gen:
            await _kabs_set_relay("ozone_gen", True)
        else:
            _kabs_flog("ozone_gen already ON — skipping relay_set")
        if flow > 0 and not S.relay_o2_conc:
            await _kabs_set_relay("o2_conc", True)
        elif flow > 0:
            _kabs_flog("o2_conc already ON — skipping relay_set")
        if S.relay_air_comp:
            await _kabs_set_relay("air_comp", False)

        _kabs_flog("Relay setup complete — waiting 3s for stabilization")
        await asyncio.sleep(3.0)
        _kabs_flog_state("post_relay_setup")

        # ------------------------------------------------------------------
        # Phase 2: Operator prompt — mixing screw confirmation
        # ------------------------------------------------------------------
        S.fill_phase = "prompt_mixing_screw"
        S.seq_phase = "prompt_mixing_screw"
        S.seq_progress = 2.0
        _kabs_flog("Phase 2: operator prompt — confirm mixing screw running")
        _notify("Waiting for operator: mixing screw confirmation", "info")

        await _kabs_await_operator_confirm(
            prompt_id="k_abs_mixing_screw",
            prompt_text=(
                "Confirm mixing screw is running at 25\u201330\u00a0RPM.\n\n"
                "The substrate must be actively mixed throughout the "
                "fill+hold phase to ensure uniform O\u2083 exposure.\n\n"
                "Press Confirm when the mixing screw is verified."
            ),
        )
        _kabs_flog("Mixing screw confirmed by operator")

        # ------------------------------------------------------------------
        # Phase 3: Operator prompt — L-valve vessel route
        # ------------------------------------------------------------------
        S.fill_phase = "prompt_l_valve"
        S.seq_phase = "prompt_l_valve"
        S.seq_progress = 4.0
        _kabs_flog("Phase 3: operator prompt — confirm L-valve vessel route")
        _notify("Waiting for operator: L-valve confirmation", "info")

        await _kabs_await_operator_confirm(
            prompt_id="k_abs_l_valve",
            prompt_text=(
                "Confirm the L-valve is set to the VESSEL route.\n\n"
                "Gas flow must pass through the sterilization vessel and "
                "the 106-H sensor must be reading the vessel outlet.\n\n"
                "Press Confirm when the L-valve position is verified."
            ),
        )
        _kabs_flog("L-valve VESSEL route confirmed by operator")

        # ------------------------------------------------------------------
        # Phase 4: Baseline — 15 samples at 0% power
        # ------------------------------------------------------------------
        S.fill_phase = "baseline"
        S.seq_phase = "baseline"
        _kabs_flog(
            f"Phase 4: baseline — {KABS_BASELINE_SAMPLES} samples at 0% power, "
            f"verify O3 < {KABS_BASELINE_MAX_O3_PCT}% vol"
        )
        _notify("Baseline phase — measuring background O3 at 0% power", "info")

        baseline_collected = 0
        while baseline_collected < KABS_BASELINE_SAMPLES:
            if not S.fill_active or not S.connected:
                raise RuntimeError(
                    f"k_abs calibration aborted or disconnected during baseline "
                    f"(fill_active={S.fill_active} connected={S.connected})"
                )
            await asyncio.sleep(_POLL_INTERVAL_S)
            sample = _snap("baseline")
            kabs_samples.append(sample)
            S.cstr_samples = kabs_samples  # keep UI observer in sync
            baseline_collected += 1
            S.seq_progress = 4.0 + (baseline_collected / KABS_BASELINE_SAMPLES) * 6.0
            _kabs_flog(
                f"Baseline {baseline_collected}/{KABS_BASELINE_SAMPLES}: "
                f"O3={sample['vessel_o3_pct']:.4f}% "
                f"esp_ts={sample['esp_ts_ms']} dt={sample['dt_s']:.2f}s"
            )

        _kabs_flog_state("baseline_complete")
        _checkpoint("baseline")

        # Verify baseline O3 is below threshold
        bl_o3_vals = [s["vessel_o3_pct"] for s in kabs_samples if s["phase"] == "baseline"]
        bl_o3_mean = float(np.mean(bl_o3_vals)) if bl_o3_vals else 0.0
        bl_o3_max = float(np.max(bl_o3_vals)) if bl_o3_vals else 0.0
        _kabs_flog(
            f"Baseline complete: mean O3={bl_o3_mean:.4f}%, "
            f"max O3={bl_o3_max:.4f}%, "
            f"threshold={KABS_BASELINE_MAX_O3_PCT}%"
        )
        if bl_o3_max >= KABS_BASELINE_MAX_O3_PCT:
            _kabs_flog(
                f"WARN baseline O3 elevated: max={bl_o3_max:.4f}% >= "
                f"{KABS_BASELINE_MAX_O3_PCT}% — vessel may not be fully evacuated. "
                f"Proceeding with warning (data may be biased)."
            )
            _notify(
                f"WARNING: baseline O3={bl_o3_max:.4f}% >= "
                f"{KABS_BASELINE_MAX_O3_PCT}% — vessel may not be fully purged",
                "warning",
            )
        else:
            _kabs_flog(
                f"Baseline O3 OK: max={bl_o3_max:.4f}% < {KABS_BASELINE_MAX_O3_PCT}%"
            )

        # ------------------------------------------------------------------
        # Phase 5: Fill + Hold — 100% power for fixed 30 minutes
        # ------------------------------------------------------------------
        S.fill_phase = "fill_hold"
        S.seq_phase = "fill_hold"
        _kabs_flog(
            f"Phase 5: fill+hold — ramping to 100%, "
            f"fixed hold={KABS_FILL_HOLD_DURATION_S:.0f}s "
            f"({KABS_FILL_HOLD_DURATION_S/60:.0f}min)"
        )
        _notify(
            f"Fill+Hold phase — 100% power for "
            f"{KABS_FILL_HOLD_DURATION_S/60:.0f} minutes",
            "info",
        )

        await _kabs_set_power(100)
        _kabs_flog_state("fill_hold_power_set")

        fill_hold_start = time.time()
        fill_sample_idx = 0
        power_resend_count = 0

        while True:
            if not S.fill_active or not S.connected:
                raise RuntimeError(
                    f"k_abs calibration aborted or disconnected during fill+hold "
                    f"(fill_active={S.fill_active} connected={S.connected})"
                )
            await asyncio.sleep(_POLL_INTERVAL_S)
            sample = _snap("fill_hold")
            kabs_samples.append(sample)
            S.cstr_samples = kabs_samples
            fill_sample_idx += 1

            elapsed_hold = time.time() - fill_hold_start
            remaining_s = max(0.0, KABS_FILL_HOLD_DURATION_S - elapsed_hold)

            # Power watchdog — activated after grace period
            if fill_sample_idx > KABS_FILL_POWER_GRACE_SAMPLES:
                if S.power_actual_pct < KABS_FILL_POWER_MIN_PCT:
                    power_resend_count += 1
                    if power_resend_count > KABS_FILL_POWER_RESEND_MAX:
                        raise RuntimeError(
                            f"Power dropped below {KABS_FILL_POWER_MIN_PCT}% "
                            f"{power_resend_count} times during fill+hold — "
                            f"motor pot cannot hold position. "
                            f"pwr_act={S.power_actual_pct:.1f}%"
                        )
                    _kabs_flog(
                        f"WARN power drift: pwr_act={S.power_actual_pct:.1f}% "
                        f"< {KABS_FILL_POWER_MIN_PCT}% at fill+hold "
                        f"#{fill_sample_idx} — re-sending power command "
                        f"(resend {power_resend_count}/{KABS_FILL_POWER_RESEND_MAX})"
                    )
                    _notify(
                        f"Power drift detected ({S.power_actual_pct:.0f}%) "
                        f"— re-sending (#{power_resend_count})",
                        "warning",
                    )
                    await _kabs_set_power(100)
                    _kabs_flog_state(f"fill_hold_power_resend_{power_resend_count}")

            # Progress: 10%–70% maps to fill+hold phase
            hold_frac = min(1.0, elapsed_hold / KABS_FILL_HOLD_DURATION_S)
            S.seq_progress = 10.0 + hold_frac * 60.0

            if fill_sample_idx % 10 == 0:
                _kabs_flog(
                    f"Fill+Hold #{fill_sample_idx}: "
                    f"O3={sample['vessel_o3_pct']:.4f}% "
                    f"elapsed={elapsed_hold:.0f}s "
                    f"remaining={remaining_s:.0f}s "
                    f"({hold_frac*100:.1f}%) "
                    f"dt={sample['dt_s']:.2f}s "
                    f"pwr_act={S.power_actual_pct:.1f}%"
                )

            if fill_sample_idx % KABS_CHECKPOINT_INTERVAL == 0:
                _checkpoint(f"fill_hold_{fill_sample_idx}")

            # Fixed-time exit: hold is complete when elapsed >= 30 minutes
            if elapsed_hold >= KABS_FILL_HOLD_DURATION_S:
                _kabs_flog(
                    f"Fill+Hold complete: {elapsed_hold:.1f}s elapsed "
                    f">= {KABS_FILL_HOLD_DURATION_S:.0f}s target, "
                    f"{fill_sample_idx} samples, "
                    f"final O3={sample['vessel_o3_pct']:.4f}%"
                )
                break

        _kabs_flog_state("fill_hold_complete")
        _checkpoint(f"fill_hold_final_{fill_sample_idx}")

        _notify(
            f"Fill+Hold complete — {fill_sample_idx} samples, "
            f"O3={S.vessel_o3_pct:.3f}%",
            "positive",
        )

        # ------------------------------------------------------------------
        # Phase 6: Evacuation
        # ------------------------------------------------------------------
        S.fill_phase = "evac"
        S.seq_phase = "evac"
        S.seq_progress = 70.0
        _kabs_flog("Phase 6: evacuation — setting power to 0%, O2 continues")
        _notify("Evac phase — power off, purging vessel with O2", "info")

        await _kabs_set_power(0)
        _kabs_flog_state("evac_power_set")

        evac_start_o3 = S.vessel_o3_pct
        evac_sample_idx = 0
        flush_start_time: float | None = None
        _kabs_flog(
            f"Evac start O3={evac_start_o3:.4f}%, "
            f"threshold={KABS_EVAC_THRESHOLD_PCT}%, "
            f"flush={KABS_EVAC_FLUSH_DURATION_S/60:.0f}min after threshold"
        )

        while True:
            if not S.fill_active or not S.connected:
                raise RuntimeError(
                    f"k_abs calibration aborted or disconnected during evac "
                    f"(fill_active={S.fill_active} connected={S.connected})"
                )
            await asyncio.sleep(_POLL_INTERVAL_S)
            sample = _snap("evac")
            kabs_samples.append(sample)
            S.cstr_samples = kabs_samples
            evac_sample_idx += 1

            if evac_start_o3 > 0:
                evac_frac = 1.0 - min(1.0, sample["vessel_o3_pct"] / evac_start_o3)
            else:
                evac_frac = 1.0
            # Progress: 70%–90% maps to evac phase
            S.seq_progress = 70.0 + evac_frac * 20.0

            if evac_sample_idx % 10 == 0:
                flush_elapsed = (
                    f"{time.time() - flush_start_time:.0f}s"
                    if flush_start_time else "n/a"
                )
                _kabs_flog(
                    f"Evac #{evac_sample_idx}: "
                    f"O3={sample['vessel_o3_pct']:.4f}% "
                    f"dt={sample['dt_s']:.2f}s "
                    f"flush_elapsed={flush_elapsed}"
                )

            if evac_sample_idx % KABS_CHECKPOINT_INTERVAL == 0:
                _checkpoint(f"evac_{evac_sample_idx}")

            # Evac stopping: < 0.01% vol triggers 5-minute flush, then end
            if sample["vessel_o3_pct"] < KABS_EVAC_THRESHOLD_PCT:
                if flush_start_time is None:
                    flush_start_time = time.time()
                    _kabs_flog(
                        f"Evac below threshold ({KABS_EVAC_THRESHOLD_PCT}%), "
                        f"starting {KABS_EVAC_FLUSH_DURATION_S/60:.0f}-min flush"
                    )
                elif (time.time() - flush_start_time) >= KABS_EVAC_FLUSH_DURATION_S:
                    _kabs_flog(
                        f"Evacuation complete: O3={sample['vessel_o3_pct']:.4f}% < "
                        f"{KABS_EVAC_THRESHOLD_PCT}% for "
                        f"{KABS_EVAC_FLUSH_DURATION_S/60:.0f}min flush, "
                        f"{evac_sample_idx} evac samples"
                    )
                    break
            else:
                if flush_start_time is not None:
                    _kabs_flog(
                        f"WARN O3 rose above threshold during flush "
                        f"({sample['vessel_o3_pct']:.4f}%), resetting flush timer"
                    )
                    flush_start_time = None

        _kabs_flog_state("evac_complete")

        # ------------------------------------------------------------------
        # Phase 7: Save CSV and cleanup
        # ------------------------------------------------------------------
        S.fill_phase = "saving"
        S.seq_phase = "saving"
        S.seq_progress = 90.0

        csv_path = _make_k_abs_csv_path(flow)
        S.cstr_csv_path = csv_path
        _write_k_abs_csv(csv_path, kabs_samples)
        _kabs_flog(f"Final CSV saved: {os.path.basename(csv_path)}")

        from dashboard.commands import _safe_standby
        await _safe_standby()

        S.seq_progress = 100.0
        S.fill_phase = "complete"
        S.seq_phase = "complete"
        S.fill_active = False
        S.sequence_active = False

        n_baseline = sum(1 for s in kabs_samples if s.get("phase") == "baseline")
        n_fill = sum(1 for s in kabs_samples if s.get("phase") == "fill_hold")
        n_evac = sum(1 for s in kabs_samples if s.get("phase") == "evac")
        _kabs_flog(
            f"k_abs calibration COMPLETE: {len(kabs_samples)} total samples "
            f"({n_baseline} baseline + {n_fill} fill_hold + {n_evac} evac), "
            f"CSV: {os.path.basename(csv_path)}"
        )
        _notify(
            f"k_abs calibration complete — "
            f"{n_fill} fill+hold + {n_evac} evac samples",
            "positive",
        )
        return True

    except RuntimeError as exc:
        tb = traceback.format_exc()
        _kabs_flog(f"ERROR (RuntimeError): {exc}\n{tb}")
        _kabs_flog_state("error")
        _checkpoint("error")
        log(f"k_abs calibration error: {exc}", "error")
        _notify(f"k_abs calibration failed: {exc}", "negative")
        from dashboard.commands import _safe_standby
        await _safe_standby()
        S.fill_phase = "error"
        S.seq_phase = "error"
        S.fill_active = False
        S.sequence_active = False
        return False

    except Exception as exc:
        tb = traceback.format_exc()
        _kabs_flog(f"ERROR (unexpected): {exc}\n{tb}")
        _kabs_flog_state("error")
        _checkpoint("error")
        log(f"k_abs calibration unexpected error: {exc}", "error")
        _notify(f"k_abs calibration error: {exc}", "negative")
        from dashboard.commands import _safe_standby
        await _safe_standby()
        S.fill_phase = "error"
        S.seq_phase = "error"
        S.fill_active = False
        S.sequence_active = False
        return False


# =============================================================================
# Model fitting
# =============================================================================
async def _fit_and_save_k_abs_model() -> Optional[object]:
    """Fit k_abs and V_residual from all available k_abs calibration CSVs.

    Scans K_ABS_DATA_DIR for all *_k_abs_cal_*Lpm.csv files, passes them
    to the analysis.cstr_k_abs_model fitter together with k_d from the
    active CSTR model, and saves a timestamped JSON to K_ABS_MODEL_DIR.

    Returns the fitted model object (type depends on cstr_k_abs_model),
    or None on failure.

    Note: analysis.cstr_k_abs_model does not yet exist at time of writing.
    This function is structured to import it gracefully and surface a clear
    error if the module is absent, so the sequence can proceed without the
    fit step blocking production usage.
    """
    try:
        from analysis.cstr_k_abs_model import fit_k_abs_model, save_k_abs_model
    except ImportError:
        log(
            "analysis.cstr_k_abs_model not found — k_abs fitting skipped. "
            "Implement the module to enable automatic k_abs fitting.",
            "warning",
        )
        _notify(
            "k_abs model fitter not yet implemented "
            "(analysis.cstr_k_abs_model missing)",
            "warning",
        )
        return None

    # Require k_d from the active CSTR model
    if S.active_cstr_model is None or not S.active_cstr_model.is_valid:
        log(
            "k_abs fitting requires a valid k_d model — "
            "run k_d calibration first",
            "error",
        )
        _notify(
            "k_abs fitting skipped: no valid k_d model loaded", "negative"
        )
        return None

    k_d = S.active_cstr_model.decay_rate_per_s
    _kabs_flog(
        f"k_abs fitting: using k_d={k_d:.2e}/s from active CSTR model"
    )

    # Scan for all k_abs calibration CSVs
    csv_paths: list[str] = []
    csv_flows: list[float] = []
    csv_c_ins: list[float] = []

    if os.path.isdir(K_ABS_DATA_DIR):
        for fn in sorted(os.listdir(K_ABS_DATA_DIR)):
            if not fn.endswith(".csv"):
                continue
            m = _re.match(
                r"\d{8}_\d{6}_k_abs_cal_(\d+(?:\.\d+)?)Lpm\.csv$", fn
            )
            if m:
                flow = float(m.group(1))
                c_in = predict_o3_from_power(100, flow)
                if c_in > 0.01:
                    csv_paths.append(os.path.join(K_ABS_DATA_DIR, fn))
                    csv_flows.append(flow)
                    csv_c_ins.append(c_in)

    # Ensure the most-recently recorded CSV is included
    if S.cstr_csv_path and S.cstr_csv_path not in csv_paths:
        flow = S.fill_lpm
        c_in = predict_o3_from_power(100, flow)
        if c_in > 0.01:
            csv_paths.append(S.cstr_csv_path)
            csv_flows.append(flow)
            csv_c_ins.append(c_in)

    if not csv_paths:
        _notify("No k_abs calibration CSVs found to fit", "negative")
        return None

    _kabs_flog(
        f"k_abs fitting: {len(csv_paths)} CSV(s), "
        f"flows={[f'{f:.1f}' for f in csv_flows]} LPM, "
        f"k_d={k_d:.2e}/s"
    )

    try:
        model = fit_k_abs_model(
            csv_path=csv_paths,
            flow_lpm=csv_flows,
            c_in_pct=csv_c_ins,
            k_d=k_d,
        )
        path = save_k_abs_model(model, K_ABS_MODEL_DIR)
        log(
            f"k_abs model fitted from {len(csv_paths)} file(s): "
            f"k_abs={getattr(model, 'absorption_rate_per_s', 'n/a')}, "
            f"V_residual={getattr(model, 'v_residual_l', 'n/a')} L "
            f"-> {os.path.basename(path)}",
            "seq",
        )
        _notify(
            f"k_abs model saved: {os.path.basename(path)} "
            f"({len(csv_paths)} file(s))",
            "positive",
        )
        return model
    except Exception as exc:
        log(f"k_abs model fitting failed: {exc}", "error")
        _notify(f"k_abs model fitting failed: {exc}", "negative")
        return None
