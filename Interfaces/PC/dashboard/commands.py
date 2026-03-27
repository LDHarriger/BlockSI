"""
Command helpers — all async; COMMA-separated (never colons).
"""
from __future__ import annotations

import asyncio
import time
from typing import Optional

from dashboard.state import (
    S, log, _notify,
    DEFAULT_FLOW_LPM, compute_effective_o2_pct,
)
from dashboard.tcp_server import tcp


# =============================================================================
# Basic commands
# =============================================================================
async def cmd_set_power(pct: int, timeout: float = 2.0) -> bool:
    old = S.power_target_pct
    S.power_target_pct = int(pct)
    S.update_derived()
    resp = await tcp.send_command(f"power_set,{int(pct)}", timeout=timeout)
    ok = resp is not None and "OK" in resp
    if not ok:
        S.power_target_pct = old
        S.update_derived()
    return ok


async def cmd_set_relay(name: str, on: bool) -> bool:
    resp = await tcp.send_command(f"relay_set,{name},{1 if on else 0}")
    if resp and "OK" in resp:
        if name == "ozone_gen":
            S.relay_o3_gen = on
        elif name == "o2_conc":
            S.relay_o2_conc = on
        elif name == "air_comp":
            S.relay_air_comp = on
            S.load_model_for_current_condition()
        return True
    return False


async def cmd_sync_relays() -> None:
    if not S.connected:
        return
    await tcp.send_command("relay_get", timeout=1.0)


async def cmd_emergency_stop() -> None:
    if S.sequence_active:
        await cmd_sequence_abort()
    await cmd_set_power(0)
    await cmd_set_relay("air_comp", False)
    await cmd_set_relay("ozone_gen", False)
    await cmd_set_relay("o2_conc", False)


# =============================================================================
# Sequence commands
# =============================================================================
async def cmd_sequence_start(seq_type: str, **kwargs) -> bool:
    if not S.connected:
        _notify("Not connected to ESP32", "negative")
        return False

    if seq_type == "calibrate":
        return await _start_calibration(**kwargs)
    elif seq_type == "verify":
        from dashboard.verification_sequence import _start_verification
        asyncio.create_task(_start_verification(**kwargs))
        return True
    elif seq_type == "cstr_cal":
        from dashboard.k_d_cal import _start_fill_evac
        asyncio.create_task(_start_fill_evac(**kwargs))
        return True
    elif seq_type == "k_abs_cal":
        from dashboard.k_abs_cal import _start_k_abs_cal
        asyncio.create_task(_start_k_abs_cal(**kwargs))
        return True
    elif seq_type == "process_batch":
        from dashboard.batch_sequence import _start_process_batch
        asyncio.create_task(_start_process_batch(**kwargs))
        return True
    else:
        log(f"Unknown sequence type: {seq_type}", "error")
        return False


async def _start_calibration(**kwargs) -> bool:
    import random as _rng

    flow = kwargs.get("flow", DEFAULT_FLOW_LPM)
    num_random = int(kwargs.get("num_random", 0))
    air_comp = bool(kwargs.get("air_comp", False))

    log("Pre-flight: setting power to 0%", "seq")
    if not await cmd_set_power(0):
        _notify("Calibration aborted — failed to set power to 0%", "negative")
        return False

    random_powers: list[int] = []
    if num_random > 0:
        window = 100.0 / num_random
        levels = sorted(set(
            max(0, min(100, int(_rng.uniform(i * window, (i + 1) * window))))
            for i in range(num_random)
        ))
        random_powers = levels + levels[::-1]

    total_steps = 203 + len(random_powers)

    S.seq_type = "calibrate"
    S.seq_phase = "starting"
    S.seq_progress = 0.0
    S.seq_elapsed = 0.0
    S.seq_start_time = 0.0
    S.seq_confirmed = False
    S.seq_step_idx = 0
    S.seq_step_total = total_steps
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    S.cal_samples = []
    S.cal_file = ""
    S.cal_lpm = flow

    cmd = f"calibrate,flow={flow}"
    if air_comp:
        cmd += ",air_comp=1"
    else:
        cmd += ",air_comp=0"
    if random_powers:
        pwr_str = ",".join(str(p) for p in random_powers)
        cmd += f",random={pwr_str}"

    resp = await tcp.send_command(cmd, timeout=5.0)
    if resp and "OK" in resp:
        S.sequence_active = True
        S.seq_confirmed = True
        S.seq_start_time = time.time()
        log(f"Calibration started: flow={flow} LPM, air={air_comp}, "
            f"{num_random} rnd levels, {total_steps} steps", "seq")
        _notify("Calibration started", "positive")
        return True

    log(f"Calibration failed: {resp}", "error")
    _notify(f"Calibration failed to start: {resp}", "negative")
    return False


async def cmd_sequence_abort(reason: str = "") -> bool:
    cmd = "sequence_abort"
    if reason:
        cmd += f",{reason}"
    resp = await tcp.send_command(cmd)
    return resp is not None and "OK" in resp


async def cmd_sequence_stop() -> bool:
    result = await cmd_sequence_abort()
    await _sequence_cleanup("stop")
    S.seq_phase = "aborted"
    S.sequence_active = False
    return result


async def _safe_standby() -> None:
    """Unconditionally set power=0% and all relays off."""
    log("Safe standby: power=0, all relays off", "seq")
    await cmd_set_power(0)
    await cmd_set_relay("ozone_gen", False)
    await cmd_set_relay("o2_conc", False)
    await cmd_set_relay("air_comp", False)


async def _sequence_cleanup(source: str = "unknown") -> None:
    log(f"Sequence cleanup ({source})", "seq")
    S.seq_confirmed = False
    await _safe_standby()


async def cmd_sequence_confirm() -> bool:
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    resp = await tcp.send_command("sequence_confirm")
    return resp is not None and "OK" in resp
