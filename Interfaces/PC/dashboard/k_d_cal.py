"""
k_d calibration sequence — PC-driven fill/evacuation for empty-vessel k_d fitting.

Changes from legacy cstr_sequence.py (WP-1):
  - Runs at any user-selected (calibrated) flow rate
  - Computed fill duration from t_99 = −τ_eff × ln(0.01) with k_d=0
  - Evac stopping: < 0.01% vol triggers 5-minute flush, then end
  - Dynamic dt from timestamps (never hardcoded sample interval)
  - CSV naming: YYYYMMDD_HHMMSS_k_d_cal_{LPM}Lpm.csv
  - Multi-file fitting: fits k_d from ALL available calibration CSVs
"""
from __future__ import annotations

import asyncio
import math
import re as _re
import time
import traceback
from datetime import datetime
from typing import Optional

import numpy as np
import pandas as pd

from dashboard.state import (
    S, log, _notify,
    DEFAULT_FLOW_LPM, CSTR_DATA_DIR, CSTR_MODEL_DIR,
    predict_o3_from_power,
    CSTRModel, fit_cstr_model, save_cstr_model_json, load_cstr_model_from_dir,
)
from dashboard.data_io import _find_valid_cert
from dashboard.verification import TRANSIENT_SKIP, VAL_MIN_STABLE
from analysis.cstr_k_d_model import V_VESSEL_L, V_DEAD_L

# =============================================================================
# Calibration constants
# =============================================================================
FILL_STEADY_COUNT = 45
FILL_STEADY_RANGE = 0.08
FILL_STEADY_SLOPE = 0.0003
EVAC_THRESHOLD_PCT = 0.01
EVAC_FLUSH_DURATION_S = 300.0  # 5-minute flush after O3 drops below threshold
FILL_BASELINE_SAMPLES = 15
FILL_EXTEND_INTERVAL_S = 120.0  # 2-min extension increments
FILL_MAX_EXTENSIONS = 5
CSTR_STALE_THRESHOLD = 8.0
CSTR_CMD_RETRIES = 3
CSTR_CMD_RETRY_DELAY = 1.5
CSTR_CHECKPOINT_INTERVAL = 10
CSTR_POWER_SET_TIMEOUT = 25.0
CSTR_POWER_TOLERANCE = 5.0
FILL_POWER_MIN_PCT = 80.0
FILL_POWER_GRACE_SAMPLES = 5
FILL_POWER_RESEND_MAX = 3

_POLL_INTERVAL_S = 2.0


def _compute_fill_t99(flow_lpm: float) -> float:
    """Compute conservative fill duration t_99 assuming k_d=0 (upper bound).

    t_99 = −τ × ln(0.01) with k_d=0 → τ = V/Q
    """
    q = flow_lpm / 60.0
    if q <= 0:
        return 3600.0
    tau = V_VESSEL_L / q
    return -tau * math.log(0.01)


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------
def _make_k_d_csv_path(lpm: float) -> str:
    now = datetime.now()
    lpm_s = f"{lpm:.2f}"
    fname = f"{now:%Y%m%d_%H%M%S}_k_d_cal_{lpm_s}Lpm.csv"
    return os.path.join(CSTR_DATA_DIR, fname)


def _write_k_d_csv(path: str, samples: list[dict]) -> None:
    if not samples:
        return
    header = (
        "timestamp,esp_ts_ms,elapsed_s,dt_s,"
        "vessel_o3_pct,cell_temp_c,vessel_temp_c,"
        "power_pct,power_actual_pct,phase\n"
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
                f"{s.get('power_pct', 0)},{s.get('power_actual_pct', 0.0)},"
                f"{s.get('phase', '')}\n"
            )
    log(f"Saved {len(samples)} samples -> {os.path.basename(path)}", "seq")


# ---------------------------------------------------------------------------
# Debug logger
# ---------------------------------------------------------------------------
_cstr_debug_file: Optional[str] = None


def _cstr_flog(msg: str) -> None:
    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    log(msg, "seq")
    if _cstr_debug_file:
        try:
            with open(_cstr_debug_file, "a") as _f:
                _f.write(f"[{ts}] {msg}\n")
        except OSError:
            pass


def _cstr_flog_state(label: str) -> None:
    _cstr_flog(
        f"STATE@{label}: connected={S.connected} "
        f"O3={S.vessel_o3_pct:.4f}% pwr_tgt={S.power_target_pct}% "
        f"pwr_act={S.power_actual_pct:.1f}% "
        f"last_update={S.last_update.strftime('%H:%M:%S') if S.last_update else 'None'} "
        f"esp_ts={S.last_esp_ts_ms} "
        f"relay(o3={S.relay_o3_gen} o2={S.relay_o2_conc} air={S.relay_air_comp}) "
        f"seq_cleanup_pending={S.seq_cleanup_pending}"
    )


# ---------------------------------------------------------------------------
# Command wrappers (with retries/fallbacks)
# ---------------------------------------------------------------------------
async def _cstr_set_power(pct: int) -> None:
    from dashboard.commands import cmd_set_power
    _cstr_flog(
        f"cmd_set_power({pct}%) — single attempt, timeout={CSTR_POWER_SET_TIMEOUT}s"
    )
    ok = await cmd_set_power(pct, timeout=CSTR_POWER_SET_TIMEOUT)
    if ok:
        _cstr_flog(f"cmd_set_power({pct}%) -> RSP OK (pwr_act={S.power_actual_pct:.1f}%)")
        return

    _cstr_flog(
        f"cmd_set_power({pct}%) RSP timeout/failed "
        f"(connected={S.connected}, pwr_act={S.power_actual_pct:.1f}%)"
    )
    if abs(S.power_actual_pct - pct) <= CSTR_POWER_TOLERANCE:
        _cstr_flog(
            f"WARN cmd_set_power({pct}%) RSP not received but "
            f"pwr_act={S.power_actual_pct:.1f}% is within {CSTR_POWER_TOLERANCE}% "
            f"of target — treating as success (telemetry fallback)"
        )
        S.power_target_pct = int(pct)
        S.update_derived()
        return

    raise RuntimeError(
        f"Failed to set power to {pct}%: "
        f"RSP timed out and pwr_act={S.power_actual_pct:.1f}% is not near target"
    )


async def _cstr_set_relay(name: str, on: bool) -> None:
    from dashboard.commands import cmd_set_relay
    for attempt in range(1, CSTR_CMD_RETRIES + 1):
        _cstr_flog(f"cmd_set_relay({name},{on}) attempt {attempt}/{CSTR_CMD_RETRIES}")
        ok = await cmd_set_relay(name, on)
        if ok:
            _cstr_flog(f"cmd_set_relay({name},{on}) -> OK")
            return
        _cstr_flog(f"cmd_set_relay({name},{on}) attempt {attempt} FAILED")
        if attempt < CSTR_CMD_RETRIES:
            await asyncio.sleep(CSTR_CMD_RETRY_DELAY)
    raise RuntimeError(
        f"Failed to set relay {name}={on} after {CSTR_CMD_RETRIES} attempts"
    )


# ---------------------------------------------------------------------------
# Main sequence coroutine
# ---------------------------------------------------------------------------
async def _start_fill_evac(**kwargs) -> bool:
    """PC-driven fill/evacuation k_d calibration sequence."""
    global _cstr_debug_file

    if not S.connected:
        _notify("Not connected to ESP32", "negative")
        return False

    if S.sequence_active or S.fill_active:
        _notify("Another sequence is already running", "negative")
        return False

    flow = kwargs.get("flow", DEFAULT_FLOW_LPM)

    # Determine target O3 from validation cert or model
    target_o3 = 0.0
    val_cert = _find_valid_cert(flow, max_age_h=720)
    if val_cert:
        try:
            vdf = pd.read_csv(val_cert)
            vdf.columns = [c.strip() for c in vdf.columns]
            # Match new verification ("full_power") and legacy validation ("target")
            vt = vdf[vdf["phase"].isin(["full_power", "target"])].iloc[TRANSIENT_SKIP:]
            if len(vt) >= VAL_MIN_STABLE:
                target_o3 = float(vt["o3_pct"].mean())
                log(f"k_d cal fill target from verification: {target_o3:.4f}% "
                    f"({os.path.basename(val_cert)})", "info")
        except Exception as exc:
            log(f"Failed to read verification cert: {exc}", "warning")
    if target_o3 <= 0.01:
        target_o3 = predict_o3_from_power(100, flow)
        log(f"k_d cal fill target from sigmoid model: {target_o3:.4f}% "
            f"(no verification cert found)", "info")
    if target_o3 <= 0.01:
        _notify(
            "Cannot determine fill target — run a validation first",
            "negative",
        )
        return False

    # Compute fill duration from t_99
    fill_t99_s = _compute_fill_t99(flow)
    fill_max_samples = int(fill_t99_s / _POLL_INTERVAL_S) + 50

    # Initialize state
    S.fill_active = True
    S.fill_phase = "setup"
    S.cstr_samples = []
    S.cstr_csv_path = ""
    S.fill_target_o3 = target_o3
    S.fill_lpm = flow
    S.sequence_active = True
    S.seq_type = "cstr_cal"
    S.seq_phase = "setup"
    S.seq_progress = 0.0
    S.seq_start_time = time.time()
    S.seq_cleanup_pending = False

    now_tag = datetime.now().strftime("%Y%m%d_%H%M%S")
    _cstr_debug_file = os.path.join(
        CSTR_DATA_DIR, f"{now_tag}_k_d_cal_debug.log"
    )

    _cstr_flog(
        f"k_d calibration starting: flow={flow} LPM, "
        f"target C_in={target_o3:.3f} %vol, "
        f"computed t_99={fill_t99_s:.0f}s ({fill_t99_s/60:.1f}min)"
    )
    _cstr_flog_state("start")

    prev_esp_ts: int | None = None

    def _snap(phase: str) -> dict:
        nonlocal prev_esp_ts
        now = datetime.now()
        staleness = (now - S.last_update).total_seconds() if S.last_update else 999.0
        if staleness > CSTR_STALE_THRESHOLD:
            _cstr_flog(
                f"WARN stale telemetry in {phase}: last_update "
                f"{staleness:.1f}s ago (>{CSTR_STALE_THRESHOLD}s) — "
                f"O3={S.vessel_o3_pct:.4f}% may be stale"
            )
        # Dynamic dt from ESP32 timestamps (never hardcoded)
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
            "power_pct": S.power_target_pct,
            "power_actual_pct": S.power_actual_pct,
            "phase": phase,
            "dt_s": dt_s,
            "_staleness_s": round(staleness, 2),
        }

    def _checkpoint(label: str) -> None:
        if not S.cstr_samples:
            return
        try:
            ckpt_path = os.path.join(
                CSTR_DATA_DIR,
                f"{now_tag}_k_d_cal_checkpoint_{label}.csv",
            )
            _write_k_d_csv(ckpt_path, S.cstr_samples)
            _cstr_flog(
                f"Checkpoint saved: {os.path.basename(ckpt_path)} "
                f"({len(S.cstr_samples)} samples)"
            )
        except OSError as ckpt_err:
            _cstr_flog(f"WARN checkpoint write failed: {ckpt_err}")

    try:
        # Phase 1: Relay setup
        S.fill_phase = "relay_setup"
        S.seq_phase = "relay_setup"
        _cstr_flog("Phase 1: relay setup")

        await _cstr_set_power(0)

        if not S.relay_o3_gen:
            await _cstr_set_relay("ozone_gen", True)
        else:
            _cstr_flog("ozone_gen already ON — skipping relay_set")
        if flow > 0 and not S.relay_o2_conc:
            await _cstr_set_relay("o2_conc", True)
        elif flow > 0:
            _cstr_flog("o2_conc already ON — skipping relay_set")
        if S.relay_air_comp:
            await _cstr_set_relay("air_comp", False)

        _cstr_flog("Relay setup complete — waiting 3 s for stabilization")
        await asyncio.sleep(3.0)
        _cstr_flog_state("post_relay_setup")

        # Phase 2: Baseline
        S.fill_phase = "baseline"
        S.seq_phase = "baseline"
        _cstr_flog(
            f"Phase 2: baseline — {FILL_BASELINE_SAMPLES} samples at 0% power"
        )

        baseline_collected = 0
        while baseline_collected < FILL_BASELINE_SAMPLES:
            if not S.fill_active or not S.connected:
                raise RuntimeError(
                    f"k_d calibration aborted or disconnected "
                    f"(fill_active={S.fill_active} connected={S.connected})"
                )
            await asyncio.sleep(_POLL_INTERVAL_S)
            sample = _snap("baseline")
            S.cstr_samples.append(sample)
            baseline_collected += 1
            S.seq_progress = (baseline_collected / FILL_BASELINE_SAMPLES) * 10
            _cstr_flog(
                f"Baseline {baseline_collected}/{FILL_BASELINE_SAMPLES}: "
                f"O3={sample['vessel_o3_pct']:.4f}% "
                f"esp_ts={sample['esp_ts_ms']} dt={sample['dt_s']:.2f}s"
            )

        _cstr_flog_state("baseline_complete")
        _checkpoint("baseline")

        # Phase 3: Fill
        S.fill_phase = "fill"
        S.seq_phase = "fill"
        _cstr_flog(
            f"Phase 3: fill — ramping to 100%, target={target_o3:.3f}% vol, "
            f"max_samples={fill_max_samples} (t_99={fill_t99_s:.0f}s)"
        )
        _notify("Fill phase — ramping to 100% power", "info")

        await _cstr_set_power(100)
        _cstr_flog_state("fill_power_set")

        fill_sample_idx = 0
        power_resend_count = 0
        recent_o3: list[float] = []
        fill_extensions = 0

        while True:
            if not S.fill_active or not S.connected:
                raise RuntimeError(
                    f"k_d calibration aborted or disconnected during fill "
                    f"(fill_active={S.fill_active} connected={S.connected})"
                )
            await asyncio.sleep(_POLL_INTERVAL_S)
            sample = _snap("fill")
            S.cstr_samples.append(sample)
            fill_sample_idx += 1

            # Power watchdog
            if fill_sample_idx > FILL_POWER_GRACE_SAMPLES:
                if S.power_actual_pct < FILL_POWER_MIN_PCT:
                    power_resend_count += 1
                    if power_resend_count > FILL_POWER_RESEND_MAX:
                        raise RuntimeError(
                            f"Power dropped below {FILL_POWER_MIN_PCT}% "
                            f"{power_resend_count} times during fill — motor pot "
                            f"cannot hold position. pwr_act={S.power_actual_pct:.1f}%"
                        )
                    _cstr_flog(
                        f"WARN power drift: pwr_act={S.power_actual_pct:.1f}% "
                        f"< {FILL_POWER_MIN_PCT}% at fill #{fill_sample_idx} — "
                        f"re-sending power command "
                        f"(resend {power_resend_count}/{FILL_POWER_RESEND_MAX})"
                    )
                    _notify(
                        f"Power drift detected ({S.power_actual_pct:.0f}%) "
                        f"— re-sending (#{power_resend_count})",
                        "warning",
                    )
                    await _cstr_set_power(100)
                    _cstr_flog_state(f"fill_power_resend_{power_resend_count}")
                    recent_o3.clear()

            recent_o3.append(sample["vessel_o3_pct"])
            if len(recent_o3) > FILL_STEADY_COUNT:
                recent_o3.pop(0)

            fill_frac = (
                min(1.0, sample["vessel_o3_pct"] / target_o3) if target_o3 > 0 else 0
            )
            S.seq_progress = 10 + fill_frac * 40

            rng = (max(recent_o3) - min(recent_o3)) if len(recent_o3) >= 2 else 999.0
            slope = 999.0
            if len(recent_o3) >= FILL_STEADY_COUNT:
                x = np.arange(len(recent_o3), dtype=float)
                y = np.array(recent_o3, dtype=float)
                slope = float(np.polyfit(x, y, 1)[0])
            is_stable = (
                len(recent_o3) >= FILL_STEADY_COUNT
                and rng < FILL_STEADY_RANGE
                and abs(slope) < FILL_STEADY_SLOPE
            )

            if fill_sample_idx % 10 == 0 or is_stable:
                _cstr_flog(
                    f"Fill #{fill_sample_idx}: O3={sample['vessel_o3_pct']:.4f}% "
                    f"({fill_frac*100:.1f}% of target) range={rng:.4f}% "
                    f"slope={slope:.6f} "
                    f"window={len(recent_o3)}/{FILL_STEADY_COUNT} "
                    f"dt={sample['dt_s']:.2f}s "
                    f"pwr_act={S.power_actual_pct:.1f}%"
                )

            if fill_sample_idx % CSTR_CHECKPOINT_INTERVAL == 0:
                _checkpoint(f"fill_{fill_sample_idx}")

            if is_stable:
                _cstr_flog(
                    f"Fill steady-state reached: O3={sample['vessel_o3_pct']:.4f}%, "
                    f"range={rng:.4f}%, slope={slope:.6f} "
                    f"over {FILL_STEADY_COUNT} samples"
                )
                if S.power_actual_pct < FILL_POWER_MIN_PCT:
                    raise RuntimeError(
                        f"Fill steady-state declared but pwr_act={S.power_actual_pct:.1f}% "
                        f"is below minimum {FILL_POWER_MIN_PCT}% — data is invalid."
                    )
                break

            if fill_sample_idx >= fill_max_samples:
                if fill_extensions < FILL_MAX_EXTENSIONS:
                    fill_extensions += 1
                    extend_samples = int(FILL_EXTEND_INTERVAL_S / _POLL_INTERVAL_S)
                    fill_max_samples += extend_samples
                    _cstr_flog(
                        f"Fill t_99 reached but steady-state NOT confirmed. "
                        f"Extending by {FILL_EXTEND_INTERVAL_S:.0f}s "
                        f"(extension {fill_extensions}/{FILL_MAX_EXTENSIONS})"
                    )
                    _notify(
                        f"Fill extending (+{FILL_EXTEND_INTERVAL_S:.0f}s) — "
                        f"steady-state not yet confirmed",
                        "warning",
                    )
                else:
                    _cstr_flog(
                        f"WARN fill hard limit reached ({fill_max_samples} samples, "
                        f"{FILL_MAX_EXTENSIONS} extensions). "
                        f"Steady-state NOT detected (range={rng:.4f}%). "
                        f"Proceeding to evac with available data."
                    )
                    _notify(
                        f"Fill limit reached — steady-state not achieved, "
                        f"proceeding anyway",
                        "warning",
                    )
                    break

        _cstr_flog_state("fill_complete")
        _checkpoint(f"fill_final_{fill_sample_idx}")

        # Phase 4: Transition
        S.fill_phase = "transition"
        S.seq_phase = "transition"
        S.seq_progress = 50.0

        _notify(
            f"Fill complete — {fill_sample_idx} samples, "
            f"O3={S.vessel_o3_pct:.3f}%",
            "positive",
        )

        # Phase 5: Evacuation (new criteria: < 0.01% → 5-min flush)
        S.fill_phase = "evac"
        S.seq_phase = "evac"
        _cstr_flog("Phase 5: evacuation — setting power to 0%")
        _notify("Evac phase — power off, purging vessel", "info")

        await _cstr_set_power(0)
        _cstr_flog_state("evac_power_set")

        evac_start_o3 = S.vessel_o3_pct
        evac_sample_idx = 0
        flush_start_time: float | None = None
        _cstr_flog(
            f"Evac start O3={evac_start_o3:.4f}%, threshold={EVAC_THRESHOLD_PCT}%, "
            f"flush={EVAC_FLUSH_DURATION_S/60:.0f}min after threshold"
        )

        while True:
            if not S.fill_active or not S.connected:
                raise RuntimeError(
                    f"k_d calibration aborted or disconnected during evac "
                    f"(fill_active={S.fill_active} connected={S.connected})"
                )
            await asyncio.sleep(_POLL_INTERVAL_S)
            sample = _snap("evac")
            S.cstr_samples.append(sample)
            evac_sample_idx += 1

            if evac_start_o3 > 0:
                evac_frac = 1.0 - min(1.0, sample["vessel_o3_pct"] / evac_start_o3)
            else:
                evac_frac = 1.0
            S.seq_progress = 50 + evac_frac * 40

            if evac_sample_idx % 10 == 0:
                flush_elapsed = (
                    f"{time.time() - flush_start_time:.0f}s" if flush_start_time else "n/a"
                )
                _cstr_flog(
                    f"Evac #{evac_sample_idx}: O3={sample['vessel_o3_pct']:.4f}% "
                    f"dt={sample['dt_s']:.2f}s flush_elapsed={flush_elapsed}"
                )

            if evac_sample_idx % CSTR_CHECKPOINT_INTERVAL == 0:
                _checkpoint(f"evac_{evac_sample_idx}")

            # Evac criteria: < 0.01% vol triggers 5-min flush
            if sample["vessel_o3_pct"] < EVAC_THRESHOLD_PCT:
                if flush_start_time is None:
                    flush_start_time = time.time()
                    _cstr_flog(
                        f"Evac below threshold ({EVAC_THRESHOLD_PCT}%), "
                        f"starting {EVAC_FLUSH_DURATION_S/60:.0f}-min flush"
                    )
                elif (time.time() - flush_start_time) >= EVAC_FLUSH_DURATION_S:
                    _cstr_flog(
                        f"Evacuation complete: O3={sample['vessel_o3_pct']:.4f}% < "
                        f"{EVAC_THRESHOLD_PCT}% for "
                        f"{EVAC_FLUSH_DURATION_S/60:.0f} min flush"
                    )
                    break
            else:
                if flush_start_time is not None:
                    _cstr_flog(
                        f"WARN O3 rose above threshold during flush "
                        f"({sample['vessel_o3_pct']:.4f}%), resetting flush timer"
                    )
                    flush_start_time = None

        _cstr_flog_state("evac_complete")

        # Phase 6: Save and cleanup
        S.fill_phase = "saving"
        S.seq_phase = "saving"
        S.seq_progress = 90.0

        csv_path = _make_k_d_csv_path(flow)
        S.cstr_csv_path = csv_path
        _write_k_d_csv(csv_path, S.cstr_samples)
        _cstr_flog(f"Final CSV saved: {os.path.basename(csv_path)}")

        from dashboard.commands import _safe_standby
        await _safe_standby()
        S.seq_progress = 100.0
        S.fill_phase = "complete"
        S.seq_phase = "complete"
        S.fill_active = False
        S.sequence_active = False

        n_fill = sum(1 for s in S.cstr_samples if s.get("phase") == "fill")
        n_evac = sum(1 for s in S.cstr_samples if s.get("phase") == "evac")
        _cstr_flog(
            f"k_d calibration COMPLETE: {len(S.cstr_samples)} total samples "
            f"({FILL_BASELINE_SAMPLES} baseline + {n_fill} fill + {n_evac} evac)"
        )
        _notify(
            f"k_d calibration complete — {n_fill} fill + {n_evac} evac samples",
            "positive",
        )
        return True

    except RuntimeError as exc:
        tb = traceback.format_exc()
        _cstr_flog(f"ERROR (RuntimeError): {exc}\n{tb}")
        _cstr_flog_state("error")
        _checkpoint("error")
        log(f"k_d calibration error: {exc}", "error")
        _notify(f"k_d calibration failed: {exc}", "negative")
        from dashboard.commands import _safe_standby
        await _safe_standby()
        S.fill_phase = "error"
        S.seq_phase = "error"
        S.fill_active = False
        S.sequence_active = False
        return False

    except Exception as exc:
        tb = traceback.format_exc()
        _cstr_flog(f"ERROR (unexpected): {exc}\n{tb}")
        _cstr_flog_state("error")
        _checkpoint("error")
        log(f"k_d calibration unexpected error: {exc}", "error")
        _notify(f"k_d calibration error: {exc}", "negative")
        from dashboard.commands import _safe_standby
        await _safe_standby()
        S.fill_phase = "error"
        S.seq_phase = "error"
        S.fill_active = False
        S.sequence_active = False
        return False


async def _fit_and_save_cstr_model() -> Optional[CSTRModel]:
    """Fit k_d from ALL available calibration CSVs and save timestamped model.

    Multi-file fitting: scans Data/k_d_cal/ for all calibration CSVs,
    extracts flow rates from filenames, and fits a single shared k_d
    weighted by 1/τ (lower flow → higher weight → more k_d sensitivity).
    """
    # Find all calibration CSVs (both new and legacy naming)
    csv_paths: list[str] = []
    csv_flows: list[float] = []
    csv_c_ins: list[float] = []

    if os.path.isdir(CSTR_DATA_DIR):
        for fn in sorted(os.listdir(CSTR_DATA_DIR)):
            if not fn.endswith(".csv"):
                continue
            # Match new naming: YYYYMMDD_HHMMSS_k_d_cal_{LPM}Lpm.csv
            m = _re.match(r"\d{8}_\d{6}_k_d_cal_(\d+(?:\.\d+)?)Lpm\.csv$", fn)
            if not m:
                # Match legacy naming: YYYY-MM-DD_HHMMSS_CSTR_{LPM}Lpm.csv
                m = _re.match(r"\d{4}-\d{2}-\d{2}_\d{6}_CSTR_(\d+(?:\.\d+)?)Lpm\.csv$", fn)
            if m:
                flow = float(m.group(1))
                c_in = predict_o3_from_power(100, flow)
                if c_in > 0.01:
                    csv_paths.append(os.path.join(CSTR_DATA_DIR, fn))
                    csv_flows.append(flow)
                    csv_c_ins.append(c_in)

    # If specific CSV was just recorded, ensure it's included
    if S.cstr_csv_path and S.cstr_csv_path not in csv_paths:
        flow = S.fill_lpm
        c_in = predict_o3_from_power(100, flow)
        if c_in > 0.01:
            csv_paths.append(S.cstr_csv_path)
            csv_flows.append(flow)
            csv_c_ins.append(c_in)

    if not csv_paths:
        _notify("No k_d calibration CSVs found to fit", "negative")
        return None

    try:
        model = fit_cstr_model(
            csv_path=csv_paths,
            flow_lpm=csv_flows,
            c_in_pct=csv_c_ins,
        )
        path = save_cstr_model_json(model, CSTR_MODEL_DIR)
        log(
            f"k_d model fitted from {len(csv_paths)} file(s): "
            f"k_d={model.decay_rate_per_s:.2e}/s "
            f"(evac cross-check: {model.decay_rate_evac_per_s:.2e}/s), "
            f"R²={model.r_squared_fill:.4f}/{model.r_squared_evac:.4f} "
            f"-> {os.path.basename(path)}",
            "seq",
        )
        _notify(
            f"k_d model: k_d={model.decay_rate_per_s:.2e}/s, "
            f"R²={model.r_squared_fill:.3f}/{model.r_squared_evac:.3f} "
            f"({len(csv_paths)} file(s))",
            "positive",
        )
        S.load_cstr_model()
        return model
    except Exception as exc:
        log(f"k_d model fitting failed: {exc}", "error")
        _notify(f"k_d model fitting failed: {exc}", "negative")
        return None
