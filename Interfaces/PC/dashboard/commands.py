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
    elif seq_type == "validate":
        return await _start_validation(**kwargs)
    elif seq_type == "cstr_cal":
        from dashboard.cstr_sequence import _start_fill_evac
        asyncio.create_task(_start_fill_evac(**kwargs))
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


async def _start_validation(**kwargs) -> bool:
    from nicegui import ui

    power = int(kwargs.get("power", 75))
    flow = kwargs.get("flow", DEFAULT_FLOW_LPM)
    S.val_power = power
    S.val_lpm = flow

    log("Pre-flight: setting power to 0%", "seq")
    if not await cmd_set_power(0):
        _notify("Validation aborted — failed to set power to 0%", "negative")
        return False

    if not S.relay_o2_conc:
        log("Pre-enabling O2 concentrator relay", "seq")
        await cmd_set_relay("o2_conc", True)
    if not S.relay_o3_gen:
        log("Pre-enabling ozone generator relay", "seq")
        await cmd_set_relay("ozone_gen", True)
    if S.relay_air_comp:
        log("Pre-disabling air compressor relay", "seq")
        await cmd_set_relay("air_comp", False)

    # Pre-flight: warn if baseline O3 reading is elevated
    if S.vessel_o3_pct > 0.01:
        proceed = asyncio.Event()
        cancelled = [False]

        with ui.dialog().props("persistent") as bl_dlg:
            with ui.card().classes("q-pa-lg").style("min-width: 420px"):
                ui.icon("warning").classes("text-h3 text-amber q-mb-sm")
                ui.label("Baseline may not be stable").classes(
                    "text-h5 q-mb-sm"
                )
                ui.label(
                    f"Current O\u2083 reading: {S.vessel_o3_pct:.3f} %vol. "
                    "The sensor may not have fully zeroed. "
                    "Proceeding may affect data accuracy."
                ).classes("text-body1 q-mb-lg")
                with ui.row().classes("justify-end w-full q-gutter-sm"):
                    def _cancel():
                        cancelled[0] = True
                        bl_dlg.close()
                        proceed.set()

                    def _proceed():
                        bl_dlg.close()
                        proceed.set()

                    ui.button("Cancel", color="red",
                              on_click=_cancel).props("flat")
                    ui.button("Proceed anyway", color="green",
                              on_click=_proceed).props("unelevated")
        bl_dlg.open()
        await proceed.wait()
        if cancelled[0]:
            log("Validation cancelled — elevated baseline", "seq")
            _notify("Validation cancelled by user", "warning")
            return False

    S.seq_type = "validate"
    S.seq_phase = "starting"
    S.seq_progress = 0.0
    S.seq_elapsed = 0.0
    S.seq_start_time = 0.0
    S.seq_confirmed = False
    S.seq_step_idx = 0
    S.seq_step_total = 5
    S.pending_prompt_id = ""
    S.pending_prompt_text = ""
    S.val_samples = []
    S.val_result = {}
    S.val_file = ""

    resp = await tcp.send_command(
        f"validate,power={power},flow={flow}", timeout=5.0
    )
    if resp and "OK" in resp:
        S.sequence_active = True
        S.seq_confirmed = True
        S.seq_start_time = time.time()
        log(f"Validation running: power={power}%, flow={flow} LPM", "seq")
        _notify("Validation started", "positive")
        return True

    log(f"validate failed: {resp}", "error")
    _notify(f"Validation failed to start: {resp}", "negative")
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
