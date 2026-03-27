"""
UI builder — the @ui.page index function and periodic tick.

This is the ONLY module with ``ui.*`` calls (aside from the ``_notify``
fallback and the baseline warning dialog in ``commands.py``).
"""
from __future__ import annotations

import asyncio
import json
import os
import time
from datetime import datetime
from typing import Any

import numpy as np
import pandas as pd
import plotly.graph_objects as go
from nicegui import ui, app

from dashboard.state import (
    S, log, _notify, _notify_queue, data_buf, debug_log,
    DEFAULT_FLOW_LPM, TELEMETRY_DIR, MODEL_DIR,
    PROMPT_CONTENT,
    predict_o3_from_power, predict_power_from_o3,
    o3_pct_to_mg_per_s, mg_per_s_to_o3_pct,
    mg_per_s_to_g_at_time, g_at_time_to_mg_per_s,
    generate_power_curve, compute_effective_o2_pct,
    # Analysis re-exports needed by UI
    fit_sigmoid_model, save_model_json, list_saved_models,
)
from dashboard.data_io import (
    list_calibration_files, _find_valid_cert,
    _find_valid_calibration, _find_valid_calibration_model,
    list_calibrated_flow_rates,
)
from dashboard.commands import (
    cmd_set_power, cmd_set_relay, cmd_sync_relays,
    cmd_emergency_stop, cmd_sequence_start, cmd_sequence_stop,
    cmd_sequence_confirm, _sequence_cleanup,
)
from dashboard.k_d_cal import (
    _fit_and_save_cstr_model,
    FILL_STEADY_RANGE, FILL_STEADY_COUNT,
)
from dashboard.tcp_server import tcp
from dashboard.ui_tab_verification import (
    build_verification_tab, update_verification_tab,
)


# =============================================================================
# UI  (built once per browser session)
# =============================================================================
@ui.page("/")
async def index():
    _updating = False

    # -- derived settings sync -------------------------------------------
    def _sync_derived_to_ui() -> None:
        S.update_derived()
        if inp_o3 is not None:
            inp_o3._props['model-value'] = round(S.target_o3_pct, 2)
            inp_o3.update()
        if inp_mg is not None:
            inp_mg._props['model-value'] = round(S.target_mg_per_s, 2)
            inp_mg.update()
        if inp_g30 is not None:
            inp_g30._props['model-value'] = round(S.target_g_30min, 2)
            inp_g30.update()

    # -- power callbacks --------------------------------------------------
    async def _on_power_slide(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating or S.sequence_active:
            return
        _updating = True
        pct = int(e.value)
        if pct == S.power_target_pct:
            _updating = False
            return
        await cmd_set_power(pct)
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pct
            inp_pwr.update()
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    async def _on_power_input(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating or S.sequence_active:
            return
        if e.value is None:
            return
        _updating = True
        pct = int(e.value)
        if pct == S.power_target_pct:
            _updating = False
            return
        await cmd_set_power(pct)
        if slider is not None:
            slider._props['model-value'] = pct
            slider.update()
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    async def _on_o3_input(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating:
            return
        _updating = True
        pwr = int(predict_power_from_o3(e.value, S.flow_lpm))
        await cmd_set_power(pwr)
        if slider is not None:
            slider._props['model-value'] = pwr
            slider.update()
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pwr
            inp_pwr.update()
        S.update_derived()
        if inp_mg is not None:
            inp_mg._props['model-value'] = round(S.target_mg_per_s, 2)
            inp_mg.update()
        if inp_g30 is not None:
            inp_g30._props['model-value'] = round(S.target_g_30min, 2)
            inp_g30.update()
        _updating = False

    async def _on_mg_input(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating:
            return
        _updating = True
        o3 = mg_per_s_to_o3_pct(e.value, S.flow_lpm)
        pwr = int(predict_power_from_o3(o3, S.flow_lpm))
        await cmd_set_power(pwr)
        if slider is not None:
            slider._props['model-value'] = pwr
            slider.update()
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pwr
            inp_pwr.update()
        S.update_derived()
        if inp_o3 is not None:
            inp_o3._props['model-value'] = round(S.target_o3_pct, 2)
            inp_o3.update()
        if inp_g30 is not None:
            inp_g30._props['model-value'] = round(S.target_g_30min, 2)
            inp_g30.update()
        _updating = False

    async def _on_g30_input(e) -> None:
        nonlocal _updating
        if e.value is None:
            return
        if _updating:
            return
        _updating = True
        mg = g_at_time_to_mg_per_s(e.value, 30.0)
        o3 = mg_per_s_to_o3_pct(mg, S.flow_lpm)
        pwr = int(predict_power_from_o3(o3, S.flow_lpm))
        await cmd_set_power(pwr)
        if slider is not None:
            slider._props['model-value'] = pwr
            slider.update()
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pwr
            inp_pwr.update()
        S.update_derived()
        if inp_o3 is not None:
            inp_o3._props['model-value'] = round(S.target_o3_pct, 2)
            inp_o3.update()
        if inp_mg is not None:
            inp_mg._props['model-value'] = round(S.target_mg_per_s, 2)
            inp_mg.update()
        _updating = False

    async def _on_lpm_change(e) -> None:
        nonlocal _updating
        if _updating:
            return
        _updating = True
        S.flow_lpm = float(e.value)
        S.load_model_for_current_condition()
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    async def _preset_click(pct: int) -> None:
        nonlocal _updating
        if _updating or S.sequence_active:
            return
        _updating = True
        await cmd_set_power(pct)
        if slider is not None:
            slider._props['model-value'] = pct
            slider.update()
        if inp_pwr is not None:
            inp_pwr._props['model-value'] = pct
            inp_pwr.update()
        _sync_derived_to_ui()
        _update_power_curve()
        _updating = False

    # -- relay callbacks --------------------------------------------------
    async def _toggle_air() -> None:
        await cmd_set_relay("air_comp", not S.relay_air_comp)

    async def _toggle_o2() -> None:
        await cmd_set_relay("o2_conc", not S.relay_o2_conc)

    async def _toggle_o3() -> None:
        await cmd_set_relay("ozone_gen", not S.relay_o3_gen)

    async def _estop() -> None:
        await cmd_emergency_stop()
        if slider is not None:
            slider.value = 0
        if inp_pwr is not None:
            inp_pwr.value = 0
        _sync_derived_to_ui()

    async def _send_debug_cmd() -> None:
        cmd = debug_input.value
        if not cmd:
            return
        resp = await tcp.send_command(cmd)
        debug_resp_label.text = resp or "No response"

    # -- chart update helpers ---------------------------------------------
    def _update_power_curve() -> None:
        pwr, o3 = generate_power_curve(S.flow_lpm)
        target_o3 = predict_o3_from_power(S.power_target_pct, S.flow_lpm)
        fig = _make_power_fig(
            pwr, o3,
            S.power_target_pct, target_o3,
            S.power_actual_pct, S.vessel_o3_pct,
        )
        power_plot.figure = fig
        power_plot.update()

    def _make_power_fig(pwr, o3, tgt_pct, tgt_o3, act_pct, act_o3):
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=pwr, y=o3, mode="lines",
            line=dict(color="royalblue", width=2),
            showlegend=False, name="Model",
        ))
        mdl = S.active_model
        if mdl and mdl.spread_sigma and mdl.spread_power and any(s > 0 for s in mdl.spread_sigma):
            sp_pwr = np.array(mdl.spread_power)
            sp_o3 = np.array([mdl.predict(p) for p in sp_pwr])
            sp_s = np.array(mdl.spread_sigma)
            fig.add_trace(go.Scatter(
                x=list(sp_pwr) + list(sp_pwr[::-1]),
                y=list(sp_o3 + sp_s) + list((sp_o3 - sp_s)[::-1]),
                fill="toself",
                fillcolor="rgba(65,105,225,0.20)",
                line=dict(color="rgba(65,105,225,0.45)", width=1),
                showlegend=False, name="±1σ spread",
                hoverinfo="skip",
            ))
        else:
            fig.add_trace(go.Scatter(
                x=[], y=[], mode="lines",
                showlegend=False, visible=False,
            ))
        fig.add_trace(go.Scatter(
            x=[tgt_pct], y=[tgt_o3], mode="markers",
            marker=dict(color="rgba(0,0,0,0)", size=18,
                        line=dict(color="black", width=3)),
            showlegend=False, name="Target",
        ))
        fig.add_trace(go.Scatter(
            x=[act_pct], y=[act_o3], mode="markers",
            marker=dict(color="limegreen", size=14),
            showlegend=False, name="Actual",
        ))
        y_max = max(o3) * 1.1 if max(o3) > 0 else 1
        fig.update_layout(
            xaxis_title="Power %", yaxis_title="O3 %vol",
            height=300, margin=dict(l=50, r=20, t=10, b=50),
            xaxis=dict(range=[0, 105], dtick=10),
            yaxis=dict(range=[0, y_max]),
        )
        return fig

    def _restyle_markers() -> None:
        target_o3 = predict_o3_from_power(S.power_target_pct, S.flow_lpm)
        js = (
            f"const el = getElement({power_plot.id});"
            f"if(el && el.$el){{"
            f"Plotly.restyle(el.$el,"
            f"{{x:[[{S.power_target_pct}],[{S.power_actual_pct}]],"
            f"y:[[{target_o3:.6f}],[{S.vessel_o3_pct:.6f}]]}},[2,3]);"
            f"}}"
        )
        try:
            ui.run_javascript(js)
        except RuntimeError:
            pass

    # =====================================================================
    # BUILD THE PAGE
    # =====================================================================
    ui.page_title("BlockSI Control")
    dark = ui.dark_mode(True)

    ui.add_head_html("""
    <style>
    @keyframes blink-disconnect {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.2; }
    }
    .conn-blink { animation: blink-disconnect 0.6s ease-in-out infinite; }
    .conn-steady { animation: none; }
    .sensor-card {
      min-width: 170px; padding: 8px 12px;
      border-radius: 6px; border-left: 3px solid;
    }
    .sensor-card-o3   { border-left-color: #2196F3; }
    .sensor-card-room  { border-left-color: #4CAF50; }
    .sensor-card-temp  { border-left-color: #FF9800; }
    .sensor-card-flow  { border-left-color: #9C27B0; }
    .log-send  { color: #42A5F5; }
    .log-recv  { color: #66BB6A; }
    .log-error { color: #EF5350; }
    .log-warn  { color: #FFA726; }
    .log-info  { color: #BDBDBD; }
    .log-seq   { color: #CE93D8; }
    .log-cal   { color: #4DD0E1; }
    .log-val   { color: #AED581; }
    .log-state { color: #FFEE58; }
    .power-slider .q-slider__track-container { height: 8px !important; }
    .power-slider .q-slider__thumb { width: 24px !important; height: 24px !important; }
    </style>
    """)

    # -- LEFT DRAWER (sidebar) -------------------------------------------
    with ui.left_drawer(value=True).classes(
        "bg-dark q-pa-md"
    ).style("width:240px") as drawer:
        ui.label("BlockSI v2").classes("text-h6 text-weight-bold q-mb-sm")

        conn_badge = ui.badge("Disconnected", color="red").classes(
            "q-mb-sm conn-blink"
        )
        ui.separator().classes("q-my-sm")

        ui.label("Relays").classes("text-subtitle2 q-mt-sm")
        with ui.row().classes("q-gutter-xs"):
            btn_air = ui.button("Air", on_click=_toggle_air).props(
                "dense color=grey size=sm"
            )
            btn_o2 = ui.button("O2", on_click=_toggle_o2).props(
                "dense color=grey size=sm"
            )
            btn_o3 = ui.button("O3", on_click=_toggle_o3).props(
                "dense color=grey size=sm"
            )

        ui.separator().classes("q-my-sm")

        ui.label("O2 LPM").classes("text-caption")
        inp_lpm_sb = ui.number(
            value=S.flow_lpm, min=1.0, max=15.0, step=0.25,
            format="%.2f", on_change=_on_lpm_change,
        ).classes("q-mb-sm")

        ui.separator().classes("q-my-sm")

        ui.label("Readings").classes("text-subtitle2")
        with ui.card().classes("w-full q-pa-sm").props("flat bordered"):
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("O2 Flow").classes("text-caption text-grey")
                card_flow_val = ui.label(f"{S.flow_lpm:.2f}").classes("text-subtitle1 text-weight-medium text-purple")
                ui.label("LPM").classes("text-caption text-grey")
            ui.separator().classes("q-my-xs")
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Vessel O3").classes("text-caption text-grey")
                card_o3_val = ui.label(f"{S.vessel_o3_pct:.3f}").classes("text-subtitle1 text-weight-medium text-blue")
                ui.label("%vol").classes("text-caption text-grey")
            ui.separator().classes("q-my-xs")
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Room O3").classes("text-caption text-grey")
                card_room_val = ui.label(f"{S.room_o3_ppm:.3f}").classes("text-subtitle1 text-weight-medium text-green")
                ui.label("ppm").classes("text-caption text-grey")
            ui.separator().classes("q-my-xs")
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Vessel Temp").classes("text-caption text-grey")
                card_vtemp_val = ui.label("N/A").classes("text-subtitle1 text-weight-medium text-orange")
                ui.label("\u00b0C").classes("text-caption text-grey")
            ui.separator().classes("q-my-xs")
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("Cell Temp").classes("text-caption text-grey")
                card_ctemp_val = ui.label(f"{S.cell_temp_c:.1f}").classes("text-subtitle1 text-weight-medium text-orange")
                ui.label("\u00b0C").classes("text-caption text-grey")

    # -- HEADER -----------------------------------------------------------
    with ui.header().classes("bg-primary items-center q-px-md"):
        ui.button(icon="menu", on_click=lambda: drawer.toggle()).props(
            "flat dense round color=white"
        )
        ui.label("BlockSI Control").classes("text-h6 q-ml-md")
        ui.space()
        ui.button(icon="dark_mode", on_click=lambda: dark.toggle()).props(
            "flat dense round color=white"
        ).tooltip("Toggle dark/light")

    # -- SEQUENCE BANNER --------------------------------------------------
    with ui.column().classes("w-full q-pa-md"):
        with ui.row().classes(
            "w-full items-center q-pa-sm q-px-md q-gutter-md"
        ).style(
            "background: #F57F17; border-radius: 4px; min-height: 44px"
        ) as seq_banner:
            ui.icon("science", size="sm").classes("text-white")
            seq_name_lbl = ui.label("").classes("text-white text-weight-bold")
            seq_phase_lbl = ui.label("").classes("text-white-50 text-caption")
            seq_progress_bar = ui.linear_progress(
                value=0, show_value=False,
            ).props("color=white track-color=amber-4").classes(
                "col-grow"
            ).style("max-width: 220px")
            seq_elapsed_lbl = ui.label("").classes("text-white text-caption")
            ui.button(
                "ABORT", icon="stop",
                on_click=lambda: asyncio.create_task(cmd_sequence_stop()),
                color="red-10",
            ).props("dense size=sm")
        seq_banner.visible = False

        # -- PROMPT DIALOG ------------------------------------------------
        prompt_dialog = ui.dialog().props("persistent")
        prompt_icon_ref: list = []
        prompt_title_ref: list = []
        prompt_body_ref: list = []

        with prompt_dialog, ui.card().classes("q-pa-lg").style("min-width: 420px"):
            p_icon = ui.icon("help").classes("text-h3 text-amber q-mb-sm")
            prompt_icon_ref.append(p_icon)
            p_title = ui.label("Action Required").classes("text-h5 q-mb-sm")
            prompt_title_ref.append(p_title)
            p_body = ui.html("").classes("text-body1 q-mb-lg")
            prompt_body_ref.append(p_body)

            with ui.row().classes("justify-end w-full q-gutter-sm"):
                ui.button("Abort Sequence", color="red", on_click=lambda: (
                    prompt_dialog.close(),
                    asyncio.create_task(cmd_sequence_stop()),
                )).props("flat")
                ui.button("Confirm", color="green", icon="check", on_click=lambda: (
                    prompt_dialog.close(),
                    asyncio.create_task(cmd_sequence_confirm()),
                )).props("unelevated")

        # -- TABS ---------------------------------------------------------
        with ui.tabs().classes("w-full") as tabs:
            tab_control = ui.tab("Control", icon="gamepad")
            tab_calibration = ui.tab("Calibration", icon="tune")
            tab_processing = ui.tab("Processing", icon="science")
            tab_verification = ui.tab("Verification", icon="verified")
            tab_telem = ui.tab("Telemetry", icon="show_chart")
            tab_debug = ui.tab("Debug", icon="bug_report")
            tab_settings = ui.tab("Settings", icon="settings")

        with ui.tab_panels(tabs, value=tab_control).classes("w-full"):

            # =============================================================
            # CONTROL TAB
            # =============================================================
            with ui.tab_panel(tab_control):

                with ui.row().classes("w-full items-start q-gutter-md"):
                    with ui.column().classes("col-8"):
                        ui.label("Power (%)").classes("text-subtitle2")
                        slider = ui.slider(
                            min=0, max=100, step=1,
                            value=S.power_target_pct,
                            on_change=_on_power_slide,
                        ).classes("w-full power-slider")

                        preset_btns = []
                        with ui.row().classes("q-gutter-xs q-mt-xs"):
                            for p in range(0, 110, 10):
                                _p = p
                                _btn = ui.button(
                                    str(_p),
                                    on_click=lambda _p=_p: _preset_click(_p),
                                ).props("dense flat size=sm")
                                preset_btns.append(_btn)

                        init_pwr, init_o3 = generate_power_curve(S.flow_lpm)
                        tgt_o3_init = predict_o3_from_power(
                            S.power_target_pct, S.flow_lpm
                        )
                        power_plot = ui.plotly(
                            _make_power_fig(
                                init_pwr, init_o3,
                                S.power_target_pct, tgt_o3_init,
                                S.power_actual_pct, S.vessel_o3_pct,
                            )
                        ).classes("w-full")

                    with ui.column().classes("col-3"):
                        ui.label("Settings").classes("text-subtitle2 q-mb-sm")
                        ui.label("LPM").classes("text-caption")
                        inp_lpm_settings = ui.number(
                            value=S.flow_lpm, min=1.0, max=15.0,
                            step=0.25, format="%.2f",
                            on_change=_on_lpm_change,
                        )
                        ui.label("% Power").classes("text-caption q-mt-sm")
                        inp_pwr = ui.number(
                            value=S.power_target_pct,
                            min=0, max=100, step=1,
                            on_change=_on_power_input,
                        )
                        ui.label("% vol O3").classes("text-caption q-mt-sm")
                        inp_o3 = ui.number(
                            value=round(S.target_o3_pct, 2),
                            min=0.0, max=5.0, step=0.01, format="%.2f",
                            on_change=_on_o3_input,
                        )
                        ui.label("mg O3/s").classes("text-caption q-mt-sm")
                        inp_mg = ui.number(
                            value=round(S.target_mg_per_s, 2),
                            min=0.0, max=10.0, step=0.01, format="%.2f",
                            on_change=_on_mg_input,
                        )
                        ui.label("g O3 @ 30 min").classes("text-caption q-mt-sm")
                        inp_g30 = ui.number(
                            value=round(S.target_g_30min, 2),
                            min=0.0, max=20.0, step=0.1, format="%.2f",
                            on_change=_on_g30_input,
                        )

                ui.separator().classes("q-my-sm")
                ui.button(
                    "EMERGENCY STOP", icon="dangerous",
                    on_click=_estop, color="red",
                ).props("size=lg")

            # =============================================================
            # CALIBRATION TAB
            # =============================================================
            with ui.tab_panel(tab_calibration):

                # ---- Power-O3 Calibration section -----------------------
                with ui.expansion(
                    "Power-O3 Calibration", icon="tune",
                ).classes("w-full").props("default-opened"):
                    ui.markdown(
                        "Calibration: Baseline → Sweep Up (0→100%) → "
                        "Sweep Down (100→0%) → Random hold (~20 samples each). "
                        "ESP32 runs sweep autonomously."
                    ).classes("text-caption text-grey q-mb-sm")

                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        cal_rnd_input = ui.number(
                            label="# Rnd Lvls", value=15,
                            min=0, max=50, step=1, format="%.0f",
                        ).classes("w-24")

                        # Hidden value — set by rotameter prompt
                        cal_lpm_input = ui.number(
                            value=DEFAULT_FLOW_LPM,
                        )
                        cal_lpm_input.visible = False
                        # Air compressor removed from calibration (relocated
                        # downstream, used only for evacuation)
                        cal_air_toggle = ui.switch("Air ON")
                        cal_air_toggle.visible = False

                        async def _start_cal():
                            num_rnd = int(cal_rnd_input.value or 0)
                            # Turn on O2 concentrator first
                            if not S.relay_o2_conc:
                                await cmd_set_relay("o2_conc", True)

                            # Prompt operator for rotameter reading
                            _flow_event = asyncio.Event()
                            _flow_val = [0.0]

                            with ui.dialog().props("persistent") as _flow_dlg, \
                                 ui.card().classes("q-pa-lg").style("min-width: 400px"):
                                ui.icon("speed").classes("text-h3 text-blue q-mb-sm")
                                ui.label("Read Rotameter").classes("text-h5 q-mb-sm")
                                ui.label(
                                    "The O2 concentrator is now ON. Read the "
                                    "rotameter and enter the actual flow rate "
                                    "to the nearest 0.25 LPM."
                                ).classes("text-body1 q-mb-md")
                                _lpm_in = ui.number(
                                    label="Actual Flow Rate (LPM)",
                                    value=DEFAULT_FLOW_LPM,
                                    min=0.25, max=10.0, step=0.25,
                                    format="%.2f",
                                ).classes("w-full q-mb-md")
                                with ui.row().classes("justify-end w-full q-gutter-sm"):
                                    def _cancel_flow():
                                        _flow_dlg.close()
                                        _flow_event.set()

                                    def _confirm_flow():
                                        _flow_val[0] = float(_lpm_in.value or 0)
                                        _flow_dlg.close()
                                        _flow_event.set()

                                    ui.button(
                                        "Cancel", on_click=_cancel_flow,
                                    ).props("flat color=red")
                                    ui.button(
                                        "Start Calibration", on_click=_confirm_flow,
                                        color="primary",
                                    ).props("unelevated")
                            _flow_dlg.open()
                            await _flow_event.wait()

                            lpm = _flow_val[0]
                            if lpm < 0.25:
                                _notify("Calibration cancelled", "warning")
                                return
                            cal_lpm_input.value = lpm
                            await cmd_sequence_start(
                                "calibrate", flow=lpm,
                                num_random=num_rnd, air_comp=False,
                            )

                        cal_start_btn = ui.button(
                            "Start", icon="play_arrow",
                            on_click=_start_cal, color="primary",
                        )
                        cal_stop_btn = ui.button(
                            "Stop", icon="stop",
                            on_click=lambda: asyncio.create_task(
                                cmd_sequence_stop()
                            ),
                            color="grey",
                        )

                    CAL_PHASES_DEF = [
                        ("baseline", "Baseline", "~37s @ 0%"),
                        ("sweep_up", "Sweep Up", "0 → 100%"),
                        ("sweep_down", "Sweep Down", "100 → 0%"),
                        ("random", "Random", "Hold ~20 samp"),
                    ]
                    cal_step_cards: dict[str, Any] = {}
                    with ui.row().classes("w-full q-gutter-xs q-mb-sm"):
                        for phase_key, name, desc in CAL_PHASES_DEF:
                            with ui.card().classes("col q-pa-xs text-center").style(
                                "opacity: 0.4; border: 1px solid #555"
                            ) as sc:
                                ui.label(name).classes("text-caption text-weight-bold")
                                ui.label(desc).classes("text-caption text-grey")
                            cal_step_cards[phase_key] = sc

                    cal_phase_lbl = ui.label("Phase: --").classes("text-body2")
                    cal_progress = ui.linear_progress(
                        value=0, show_value=False
                    ).classes("q-mb-xs")
                    cal_info_lbl = ui.label("").classes("text-caption")

                    cal_chart = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "item"},
                        "legend": {"data": ["Sweep Up", "Sweep Down", "Random"]},
                        "xAxis": {"type": "value", "name": "Power %",
                                  "min": 0, "max": 100},
                        "yAxis": {"type": "value", "name": "O3 %vol"},
                        "series": [
                            {"name": "Sweep Up", "type": "scatter", "data": [],
                             "itemStyle": {"color": "#42A5F5"}},
                            {"name": "Sweep Down", "type": "scatter", "data": [],
                             "itemStyle": {"color": "#FFA726"}},
                            {"name": "Random", "type": "scatter", "data": [],
                             "itemStyle": {"color": "#66BB6A"}},
                        ],
                    }).classes("w-full").style("height: 280px")

                    ui.separator().classes("q-my-sm")
                    ui.label("Calibration Files").classes("text-subtitle2")
                    cal_files_container = ui.column().classes("w-full")

                    def _render_cal_files() -> None:
                        cal_files_container.clear()
                        files = list_calibration_files()
                        with cal_files_container:
                            if not files:
                                ui.label("No calibration files found").classes(
                                    "text-caption"
                                )
                            else:
                                for (lpm, o2) in sorted(files):
                                    flist = files[(lpm, o2)]
                                    with ui.expansion(
                                        f"{lpm} LPM / {o2}% O2 ({len(flist)} files)"
                                    ):
                                        for fp in sorted(flist):
                                            ui.label(os.path.basename(fp)).classes(
                                                "text-caption"
                                            )

                    _render_cal_files()

                    ui.separator().classes("q-my-sm")

                    ui.label("Model Fitting").classes("text-subtitle2")
                    ui.markdown(
                        "Fit a 4-parameter sigmoid model to calibration data. "
                        "All files for the selected condition are auto-aggregated."
                    ).classes("text-caption text-grey q-mb-sm")

                    model_status_card = ui.card().classes(
                        "w-full q-pa-sm q-mb-sm"
                    ).props("flat bordered")
                    with model_status_card:
                        with ui.row().classes("items-center q-gutter-sm"):
                            model_icon = ui.icon("model_training", size="sm")
                            model_status_lbl = ui.label(S.model_status).classes(
                                "text-body2"
                            )

                    model_fit_container = ui.column().classes("w-full")

                    def _render_model_fitting() -> None:
                        model_fit_container.clear()
                        files = list_calibration_files()
                        existing_models = list_saved_models(MODEL_DIR)
                        model_map = {
                            (m.flow_lpm, m.o2_pct): m for m in existing_models
                        }
                        with model_fit_container:
                            if not files:
                                ui.label(
                                    "Run a calibration first"
                                ).classes("text-caption text-grey")
                                return
                            for (lpm, o2) in sorted(files):
                                flist = files[(lpm, o2)]
                                existing = model_map.get((lpm, o2))
                                with ui.card().classes(
                                    "w-full q-pa-sm q-mb-xs"
                                ).props("flat bordered"):
                                    with ui.row().classes(
                                        "items-center q-gutter-sm"
                                    ):
                                        ui.label(
                                            f"{lpm} LPM / {o2}% O2"
                                        ).classes("text-weight-bold")
                                        ui.label(
                                            f"({len(flist)} file{'s' if len(flist) != 1 else ''})"
                                        ).classes("text-caption text-grey")
                                        if existing and existing.is_valid:
                                            ui.badge(
                                                f"R²={existing.r_squared:.3f}",
                                                color="green",
                                            )
                                        else:
                                            ui.badge(
                                                "No model",
                                                color="grey",
                                            )
                                        ui.space()

                                        async def _do_fit(
                                            _lpm=lpm, _o2=o2, _files=flist,
                                        ):
                                            await _fit_model_for_condition(
                                                _files, _lpm, _o2,
                                            )

                                        ui.button(
                                            "Fit Model",
                                            icon="analytics",
                                            on_click=_do_fit,
                                            color="primary",
                                        ).props("dense size=sm")

                                    if existing and existing.is_valid:
                                        ui.label(
                                            f"L={existing.L:.3f}  k={existing.k:.3f}  "
                                            f"P0={existing.P0:.1f}  b={existing.b:.3f}  "
                                            f"RMSE={existing.rmse:.4f}  n={existing.n_points}"
                                        ).classes(
                                            "text-caption text-grey q-ml-md"
                                        )

                    _render_model_fitting()

                    fit_result_container = ui.column().classes("w-full")

                    async def _fit_model_for_condition(
                        filepaths: list[str],
                        lpm: float,
                        o2: int,
                    ) -> None:
                        try:
                            model = fit_sigmoid_model(
                                filepaths, flow_lpm=lpm, o2_pct=o2,
                            )
                            path = save_model_json(model, MODEL_DIR)
                            log(
                                f"Model fitted: {model.summary()} -> "
                                f"{os.path.basename(path)}",
                                "cal",
                            )
                            _notify(
                                f"Model fitted: R²={model.r_squared:.4f}, "
                                f"RMSE={model.rmse:.4f}",
                                "positive",
                            )
                            S.load_model_for_current_condition()
                            _update_model_status()
                            _render_model_fitting()
                            _update_power_curve()
                            _sync_derived_to_ui()
                        except Exception as exc:
                            log(f"Model fit failed: {exc}", "error")
                            _notify(f"Model fit failed: {exc}", "negative")

                    def _update_model_status() -> None:
                        if S.active_model and S.active_model.is_valid:
                            model_icon.name = "check_circle"
                            model_icon.classes("text-green")
                            model_status_lbl.text = (
                                f"Active: {S.active_model.flow_lpm} LPM / "
                                f"{S.active_model.o2_pct}% O2 — "
                                f"{S.model_status}"
                            )
                        else:
                            model_icon.name = "info"
                            model_icon.classes("text-orange")
                            model_status_lbl.text = S.model_status

                    _update_model_status()

                # ---- k_d Calibration section ----------------------------
                with ui.expansion(
                    "k_d Calibration", icon="science",
                ).classes("w-full"):
                    ui.markdown(
                        "Fill the vessel at 100% power until steady-state, "
                        "then evacuate at 0% power until O3 clears. "
                        "Fits a decay-aware CSTR model to extract system volume, "
                        "O3 decay rate, and dead volume. Air compressor is OFF "
                        "during calibration for best decay sensitivity. "
                        "Parameters generalise to any flow rate and air config."
                    ).classes("text-caption text-grey q-mb-sm")
                    _kd_flow_rates = list_calibrated_flow_rates()
                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        fill_lpm_input = ui.select(
                            label="Flow Rate (LPM)",
                            options={r: f"{r:.2f} LPM" for r in _kd_flow_rates}
                                if _kd_flow_rates
                                else {0: "No calibrated rates"},
                            value=_kd_flow_rates[0] if _kd_flow_rates else 0,
                        ).classes("w-24")

                    fill_target_lbl = ui.label("").classes(
                        "text-caption text-grey q-mb-xs"
                    )

                    def _update_fill_target_label() -> None:
                        lpm_val = fill_lpm_input.value
                        lpm = float(lpm_val) if lpm_val else DEFAULT_FLOW_LPM
                        tgt = predict_o3_from_power(100, lpm)
                        if tgt > 0.01:
                            fill_target_lbl.text = (
                                f"C_in at 100% power: {tgt:.3f} %vol "
                                f"(steady-state: range < {FILL_STEADY_RANGE}% "
                                f"over {FILL_STEADY_COUNT} samples)"
                            )
                        else:
                            fill_target_lbl.text = (
                                "No power model — fit a model first"
                            )

                    _update_fill_target_label()
                    fill_lpm_input.on("update:model-value", lambda _: _update_fill_target_label())

                    with ui.row().classes("q-gutter-sm items-center q-mb-sm"):
                        async def _start_fill_seq():
                            lpm_val = fill_lpm_input.value
                            if not lpm_val or float(lpm_val) <= 0:
                                _notify(
                                    "No calibrated flow rate selected — run "
                                    "Power-O3 calibration first",
                                    "negative",
                                )
                                return
                            lpm = float(lpm_val)
                            cert = _find_valid_cert(lpm)
                            if cert is None:
                                with ui.dialog() as _cert_dlg, ui.card().classes("q-pa-md"):
                                    ui.label(
                                        "No valid verification certificate found"
                                    ).classes("text-subtitle1 text-bold q-mb-sm")
                                    ui.label(
                                        "A verification at the selected flow rate "
                                        "must pass within the last 24 hours before running "
                                        "CSTR calibration. This ensures the C_in used for "
                                        "model fitting is accurate."
                                    ).classes("text-body2 q-mb-md")
                                    with ui.row().classes("q-gutter-sm justify-end"):
                                        ui.button(
                                            "Cancel", on_click=_cert_dlg.close,
                                        ).props("flat")

                                        async def _run_ver_from_dlg():
                                            _cert_dlg.close()
                                            _ver.hold_input.value = 50
                                            _ver.lpm_input.value = lpm
                                            tabs.value = tab_verification
                                            await cmd_sequence_start(
                                                "verify", power_hold=50, flow=lpm,
                                            )

                                        ui.button(
                                            "Run Verification", icon="verified",
                                            on_click=_run_ver_from_dlg,
                                            color="blue",
                                        )
                                _cert_dlg.open()
                                return
                            await cmd_sequence_start(
                                "cstr_cal",
                                flow=lpm,
                            )

                        fill_start_btn = ui.button(
                            "Start k_d Calibration", icon="play_arrow",
                            on_click=_start_fill_seq, color="teal",
                        )

                        async def _stop_fill_seq():
                            S.fill_active = False
                            await cmd_sequence_stop()

                        fill_stop_btn = ui.button(
                            "Stop", icon="stop",
                            on_click=_stop_fill_seq, color="red",
                        ).props("flat")

                    fill_progress = ui.linear_progress(
                        value=0, show_value=False,
                    ).classes("w-full q-mb-xs")
                    fill_phase_lbl = ui.label("").classes(
                        "text-caption text-grey"
                    )

                    with ui.row().classes("items-center q-gutter-xs q-mt-sm"):
                        cstr_model_icon = ui.icon("info").classes("text-orange")
                        cstr_model_lbl = ui.label(
                            S.cstr_model_status
                        ).classes("text-caption")

                    def _update_cstr_model_status() -> None:
                        if S.active_cstr_model and S.active_cstr_model.is_valid:
                            cstr_model_icon.name = "check_circle"
                            cstr_model_icon.classes("text-green", remove="text-orange")
                            cstr_model_lbl.text = (
                                f"Active: {S.cstr_model_status}"
                            )
                        else:
                            cstr_model_icon.name = "info"
                            cstr_model_icon.classes("text-orange", remove="text-green")
                            cstr_model_lbl.text = S.cstr_model_status

                    _update_cstr_model_status()

                    async def _fit_cstr_model_click():
                        model = await _fit_and_save_cstr_model()
                        if model:
                            _update_cstr_model_status()

                    fill_fit_btn = ui.button(
                        "Fit CSTR Model", icon="show_chart",
                        on_click=_fit_cstr_model_click, color="purple",
                    ).props("flat")

                    fill_chart = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "axis"},
                        "xAxis": {"type": "category", "name": "Sample",
                                  "data": []},
                        "yAxis": {"type": "value", "name": "O3 (%vol)"},
                        "series": [
                            {"name": "O3", "type": "line", "smooth": True,
                             "data": [], "showSymbol": False,
                             "lineStyle": {"color": "#26A69A"},
                             "areaStyle": {"opacity": 0.1, "color": "#26A69A"}},
                        ],
                    }).classes("w-full").style("height: 220px")

                # ---- k_abs Calibration section ---------------------------
                with ui.expansion(
                    "k_abs Calibration (Loaded Vessel)", icon="biotech",
                ).classes("w-full"):
                    ui.markdown(
                        "Fill a substrate-loaded vessel at 100% power for 30 min, "
                        "then evacuate. Fits k_abs (substrate absorption rate) and "
                        "V_residual (free gas volume) from the transient response. "
                        "Requires a valid k_d model and power-O3 calibration."
                    ).classes("text-caption text-grey q-mb-sm")


                    # Substrate preset display
                    _cfg_path = os.path.join(
                        os.path.dirname(os.path.abspath(__file__)),
                        "substrate_config.json",
                    )
                    _substrate_cfg: dict = {}
                    try:
                        with open(_cfg_path) as _f:
                            _substrate_cfg = json.load(_f)
                    except Exception:
                        pass
                    _presets = {
                        k: v for k, v in _substrate_cfg.get("presets", {}).items()
                        if isinstance(v, (int, float))
                    }
                    if _presets:
                        with ui.card().classes("q-pa-xs q-mb-sm").props("flat bordered"):
                            with ui.row().classes("q-gutter-sm items-center"):
                                ui.icon("science", size="xs").classes("text-grey")
                                ui.label("Substrate Presets:").classes("text-caption text-weight-bold")
                                for _k, _v in _presets.items():
                                    ui.label(f"{_k}: {_v} kg").classes("text-caption text-grey")
                                _kg_total = sum(_presets.values())
                                ui.label(f"| Total: {_kg_total:.3f} kg").classes(
                                    "text-caption text-weight-bold"
                                )

                    # Flow rate selector (calibrated rates only)
                    _kabs_flow_rates = list_calibrated_flow_rates()
                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        kabs_flow_select = ui.select(
                            label="Flow Rate (LPM)",
                            options={r: f"{r:.2f} LPM" for r in _kabs_flow_rates}
                                if _kabs_flow_rates
                                else {0: "No calibrated rates"},
                            value=_kabs_flow_rates[0] if _kabs_flow_rates else 0,
                        ).classes("w-40")

                        async def _start_kabs_seq():
                            flow_val = kabs_flow_select.value
                            if not flow_val or flow_val <= 0:
                                _notify(
                                    "No calibrated flow rate selected — run "
                                    "Power-O3 calibration first",
                                    "negative",
                                )
                                return
                            flow = float(flow_val)
                            # Check power-O3 calibration freshness
                            cal_model = _find_valid_calibration_model(flow)
                            if cal_model is None:
                                _notify(
                                    f"No valid power-O3 model for {flow:.2f} LPM "
                                    f"(max 2 weeks old). Run calibration first.",
                                    "negative",
                                )
                                return
                            # Check k_d model exists
                            if not S.active_cstr_model or not S.active_cstr_model.is_valid:
                                _notify(
                                    "No valid k_d model. Run k_d calibration first.",
                                    "negative",
                                )
                                return
                            # Check verification cert
                            cert = _find_valid_cert(flow)
                            if cert is None:
                                with ui.dialog() as _cert_dlg, ui.card().classes("q-pa-md"):
                                    ui.label(
                                        "No valid verification certificate"
                                    ).classes("text-subtitle1 text-bold q-mb-sm")
                                    ui.label(
                                        "A verification at the selected flow "
                                        "rate must pass within the last 24 hours."
                                    ).classes("text-body2 q-mb-md")
                                    with ui.row().classes("q-gutter-sm justify-end"):
                                        ui.button(
                                            "Cancel", on_click=_cert_dlg.close,
                                        ).props("flat")

                                        async def _run_ver_from_kabs_dlg():
                                            _cert_dlg.close()
                                            _ver.hold_input.value = 50
                                            _ver.lpm_input.value = flow
                                            tabs.value = tab_verification
                                            await cmd_sequence_start(
                                                "verify", power_hold=50, flow=flow,
                                            )

                                        ui.button(
                                            "Run Verification", icon="verified",
                                            on_click=_run_ver_from_kabs_dlg,
                                            color="blue",
                                        )
                                _cert_dlg.open()
                                return
                            await cmd_sequence_start("k_abs_cal", flow=flow)

                        kabs_start_btn = ui.button(
                            "Start k_abs Calibration", icon="play_arrow",
                            on_click=_start_kabs_seq, color="deep-purple",
                        )
                        if not _kabs_flow_rates:
                            kabs_start_btn.props("disable")

                        async def _stop_kabs_seq():
                            S.fill_active = False
                            await cmd_sequence_stop()

                        kabs_stop_btn = ui.button(
                            "Stop", icon="stop",
                            on_click=_stop_kabs_seq, color="red",
                        ).props("flat")

                    kabs_progress = ui.linear_progress(
                        value=0, show_value=False,
                    ).classes("w-full q-mb-xs")
                    kabs_phase_lbl = ui.label("").classes("text-caption text-grey")

            # =============================================================
            # PROCESSING TAB
            # =============================================================
            with ui.tab_panel(tab_processing):
                ui.label("Process Batch").classes("text-h6 q-mb-sm")
                ui.markdown(
                    "Configure and run an ozone sterilization batch. "
                    "The dosing schedule is solved from calibration models, "
                    "then loaded as a recipe onto the ESP32 for autonomous "
                    "execution."
                ).classes("text-caption text-grey q-mb-sm")

                _proc_flow_rates = list_calibrated_flow_rates()
                _proc_config = {}
                try:
                    from dashboard.dosimetry import load_substrate_config
                    _proc_config = load_substrate_config()
                except Exception:
                    pass
                _proc_presets = _proc_config.get("presets", {})
                _proc_defaults = _proc_config.get("process_defaults", {})

                # ── Initialization form ──────────────────────────────
                with ui.card().classes("q-pa-md q-mb-sm w-full"):
                    ui.label("Batch Parameters").classes(
                        "text-subtitle2 q-mb-xs"
                    )
                    with ui.row().classes(
                        "q-gutter-sm items-end q-mb-xs w-full"
                    ):
                        proc_flow_select = ui.select(
                            options=_proc_flow_rates if _proc_flow_rates
                                    else [4.0],
                            value=(_proc_flow_rates[0]
                                   if _proc_flow_rates else 4.0),
                            label="Flow Rate (LPM)",
                        ).classes("w-28").props(
                            "" if _proc_flow_rates else "disable"
                        )
                        proc_kg_input = ui.number(
                            label="Substrate Mass (kg)",
                            value=_proc_presets.get("kg_oak", 1.5),
                            min=0.1, max=10.0, step=0.1,
                            format="%.2f",
                        ).classes("w-32")
                        proc_dose_input = ui.number(
                            label="Target Dose (mg/kg)",
                            value=100.0,
                            min=1.0, max=10000.0, step=10.0,
                            format="%.0f",
                        ).classes("w-36")
                        proc_time_input = ui.number(
                            label="Process Time (min)",
                            value=_proc_defaults.get(
                                "process_time_min", 30
                            ),
                            min=5, max=120, step=5,
                            format="%.0f",
                        ).classes("w-28")

                    with ui.row().classes(
                        "q-gutter-sm items-end q-mb-xs w-full"
                    ):
                        proc_expt_input = ui.input(
                            label="Experiment Type",
                            value="Experiment1_Mold",
                        ).classes("w-48")
                        # Substrate preset quick-fill buttons
                        for _pname, _pval in _proc_presets.items():
                            if _pname.startswith("_"):
                                continue
                            def _set_kg(v=_pval):
                                proc_kg_input.value = v
                            ui.button(
                                _pname.replace("kg_", ""),
                                on_click=_set_kg,
                            ).props("flat dense size=sm").classes(
                                "text-caption"
                            )

                    proc_notes_input = ui.input(
                        label="Batch Notes (optional)",
                    ).classes("w-full q-mb-xs")

                    if not _proc_flow_rates:
                        ui.label(
                            "No calibrated flow rates — run Power-O3 "
                            "calibration first"
                        ).classes("text-caption text-negative q-mb-xs")

                # ── Schedule preview ─────────────────────────────────
                with ui.card().classes("q-pa-md q-mb-sm w-full"):
                    ui.label("Dosing Schedule Preview").classes(
                        "text-subtitle2 q-mb-xs"
                    )
                    proc_schedule_lbl = ui.label("").classes(
                        "text-caption text-grey"
                    )
                    proc_schedule_detail = ui.markdown("").classes(
                        "text-caption"
                    )

                    async def _solve_schedule_click():
                        try:
                            from dashboard.dosimetry import (
                                solve_dosing_schedule,
                                load_substrate_config,
                            )
                            from analysis import (
                                load_model_for_condition,
                                load_cstr_model_from_dir,
                                load_k_abs_model_from_dir,
                                predict_o3, predict_power,
                            )
                            from dashboard.state import (
                                MODEL_DIR, CSTR_MODEL_DIR,
                                K_ABS_MODEL_DIR,
                                compute_effective_o2_pct,
                            )
                            _flow = float(proc_flow_select.value)
                            _o2 = compute_effective_o2_pct(_flow, False)
                            _pm = load_model_for_condition(
                                MODEL_DIR, _flow, _o2,
                            )
                            if not _pm or not _pm.is_valid:
                                proc_schedule_lbl.text = (
                                    "No Power-O3 model for this flow rate"
                                )
                                return
                            _km = load_cstr_model_from_dir(CSTR_MODEL_DIR)
                            if not _km or not _km.is_valid:
                                proc_schedule_lbl.text = "No k_d model"
                                return
                            _am = load_k_abs_model_from_dir(
                                K_ABS_MODEL_DIR,
                            )
                            if not _am or not _am.is_valid:
                                proc_schedule_lbl.text = "No k_abs model"
                                return
                            _cfg = load_substrate_config()
                            _th = _cfg.get("thresholds", {})
                            _lt = (
                                S.cell_temp_c
                                if S.cell_temp_c > 0 else 20.0
                            )
                            sched = solve_dosing_schedule(
                                target_dose_mg_per_kg=float(
                                    proc_dose_input.value
                                ),
                                kg_substrate=float(proc_kg_input.value),
                                process_time_min=float(
                                    proc_time_input.value
                                ),
                                flow_lpm=_flow,
                                k_d_empty=_km.decay_rate_per_s,
                                k_abs=_am.k_abs_per_s,
                                V_residual=_am.V_residual_L,
                                c_in_100pct=_pm.predict(100.0),
                                predict_o3_fn=lambda p, f: predict_o3(
                                    p, f, _pm,
                                ),
                                predict_power_fn=lambda c, f: predict_power(
                                    c, f, _pm,
                                ),
                                ramp_switch_fraction=_th.get(
                                    "ramp_switch_fraction", 0.85
                                ),
                                lab_temperature_c=_lt,
                                evac_threshold_pct=_th.get(
                                    "evac_threshold_pct", 0.01
                                ),
                            )
                            if sched.achievable:
                                proc_schedule_lbl.text = (
                                    f"Achievable — "
                                    f"C_target={sched.C_target:.4f}%, "
                                    f"P_hold={sched.power_hold:.1f}%"
                                )
                                proc_schedule_detail.content = (
                                    f"**Ramp**: 100% for "
                                    f"~{sched.t_switch_predicted:.1f} min | "
                                    f"**Hold**: "
                                    f"{sched.power_hold:.0f}% for "
                                    f"{sched.t_hold:.1f} min | "
                                    f"**Evac**: "
                                    f"~{sched.t_evac_predicted:.1f} min\n\n"
                                    f"Predicted dose: "
                                    f"**{sched.dose_predicted:.1f}** mg/kg "
                                    f"(total O3: "
                                    f"{sched.mg_O3_total:.0f} mg)"
                                )
                            else:
                                proc_schedule_lbl.text = (
                                    "NOT achievable — target dose "
                                    "requires > 100% power"
                                )
                                proc_schedule_detail.content = ""
                        except Exception as exc:
                            proc_schedule_lbl.text = f"Solve error: {exc}"
                            proc_schedule_detail.content = ""

                    with ui.row().classes("q-gutter-sm"):
                        ui.button(
                            "Solve Schedule", icon="calculate",
                            on_click=_solve_schedule_click, color="blue",
                        ).props("flat")

                # ── Control buttons ──────────────────────────────────
                with ui.row().classes("q-gutter-sm items-center q-mb-sm"):
                    async def _start_batch():
                        _flow = float(proc_flow_select.value)
                        await cmd_sequence_start(
                            "process_batch",
                            flow=_flow,
                            kg_substrate=float(proc_kg_input.value),
                            target_dose=float(proc_dose_input.value),
                            process_time=float(proc_time_input.value),
                            experiment_type=str(
                                proc_expt_input.value
                            ).strip() or "Experiment1_Mold",
                            batch_notes=str(
                                proc_notes_input.value
                            ).strip(),
                        )

                    proc_start_btn = ui.button(
                        "Start Batch", icon="play_arrow",
                        on_click=_start_batch, color="deep-orange",
                    )

                    async def _stop_batch():
                        await cmd_sequence_stop()

                    proc_stop_btn = ui.button(
                        "E-STOP", icon="stop",
                        on_click=_stop_batch, color="red",
                    ).props("flat")

                # ── Live monitoring panel ────────────────────────────
                with ui.card().classes("q-pa-md q-mb-sm w-full"):
                    ui.label("Live Monitoring").classes(
                        "text-subtitle2 q-mb-xs"
                    )
                    with ui.row().classes(
                        "q-gutter-md items-center q-mb-xs w-full"
                    ):
                        proc_phase_chip = ui.chip(
                            "Idle", icon="circle", color="grey",
                        ).classes("text-bold")
                        proc_elapsed_lbl = ui.label("--:--").classes(
                            "text-caption"
                        )
                        proc_dose_lbl = ui.label(
                            "Dose: -- / -- mg/kg"
                        ).classes("text-caption")

                    ui.label("Dose Progress").classes(
                        "text-caption text-grey"
                    )
                    proc_dose_bar = ui.linear_progress(
                        value=0, show_value=False,
                    ).classes("w-full q-mb-xs")

                    proc_progress = ui.linear_progress(
                        value=0, show_value=False,
                    ).classes("w-full q-mb-xs")
                    proc_phase_lbl = ui.label("").classes(
                        "text-caption text-grey"
                    )

                # ── O3 concentration chart ───────────────────────────
                proc_conc_chart = ui.echart({
                    "darkMode": True,
                    "tooltip": {"trigger": "axis"},
                    "legend": {"data": ["O3 (measured)", "C_target"],
                               "textStyle": {"color": "#aaa"}},
                    "xAxis": {"type": "category", "name": "Sample",
                              "data": []},
                    "yAxis": {"type": "value", "name": "O3 (%vol)"},
                    "series": [
                        {"name": "O3 (measured)", "type": "line",
                         "smooth": True, "data": [],
                         "showSymbol": False,
                         "lineStyle": {"color": "#FF7043"},
                         "areaStyle": {"opacity": 0.08,
                                       "color": "#FF7043"}},
                        {"name": "C_target", "type": "line",
                         "data": [], "showSymbol": False,
                         "lineStyle": {"color": "#66BB6A",
                                       "type": "dashed"}},
                    ],
                }).classes("w-full").style("height: 220px")

                # ── O3 mass balance stacked chart ────────────────────
                proc_mass_chart = ui.echart({
                    "darkMode": True,
                    "tooltip": {"trigger": "axis"},
                    "legend": {
                        "data": [
                            "Produced", "Absorbed", "Decayed", "Evacuated",
                        ],
                        "textStyle": {"color": "#aaa"},
                    },
                    "xAxis": {"type": "category", "name": "Sample",
                              "data": []},
                    "yAxis": {"type": "value", "name": "mg O3"},
                    "series": [
                        {"name": "Produced", "type": "line",
                         "stack": "total", "areaStyle": {"opacity": 0.3},
                         "data": [], "showSymbol": False,
                         "lineStyle": {"color": "#42A5F5"},
                         "itemStyle": {"color": "#42A5F5"}},
                        {"name": "Absorbed", "type": "line",
                         "stack": "total", "areaStyle": {"opacity": 0.3},
                         "data": [], "showSymbol": False,
                         "lineStyle": {"color": "#66BB6A"},
                         "itemStyle": {"color": "#66BB6A"}},
                        {"name": "Decayed", "type": "line",
                         "stack": "total", "areaStyle": {"opacity": 0.3},
                         "data": [], "showSymbol": False,
                         "lineStyle": {"color": "#FFA726"},
                         "itemStyle": {"color": "#FFA726"}},
                        {"name": "Evacuated", "type": "line",
                         "stack": "total", "areaStyle": {"opacity": 0.3},
                         "data": [], "showSymbol": False,
                         "lineStyle": {"color": "#EF5350"},
                         "itemStyle": {"color": "#EF5350"}},
                    ],
                }).classes("w-full").style("height: 220px")

            # =============================================================
            # VERIFICATION TAB  (extracted to ui_tab_verification.py)
            # =============================================================
            _ver = build_verification_tab(tab_verification, cmd_sequence_start)
            ver_start_btn = _ver.start_btn

            # =============================================================
            # TELEMETRY TAB
            # =============================================================
            with ui.tab_panel(tab_telem):
                with ui.row().classes("q-gutter-md q-mb-md"):
                    met_o3 = ui.label(
                        f"Vessel O3: {S.vessel_o3_pct:.4f} %vol"
                    ).classes("text-body1")
                    met_room = ui.label(
                        f"Room O3: {S.room_o3_ppm:.3f} ppm"
                    ).classes("text-body1")
                    met_vt = ui.label("Vessel T: N/A").classes("text-body1")
                    met_ct = ui.label(
                        f"Cell T: {S.cell_temp_c:.1f} \u00b0C"
                    ).classes("text-body1")

                telem_skeleton = ui.column().classes("w-full")
                with telem_skeleton:
                    for _ in range(2):
                        ui.skeleton(type="rect").classes(
                            "w-full q-mb-sm"
                        ).style("height: 200px")
                telem_skeleton.visible = True

                # -- O3 Concentration chart (expandable) ------------------
                with ui.expansion(
                    "O3 Concentration", icon="bubble_chart",
                ).classes("w-full").props("default-opened"):
                    echart_o3 = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "axis"},
                        "legend": {"data": ["Vessel O3 (%vol)", "Room O3 (ppm)"]},
                        "xAxis": {"type": "time", "name": "Time"},
                        "yAxis": [
                            {"type": "value", "name": "O3 %vol", "position": "left"},
                            {"type": "value", "name": "Room ppm", "position": "right"},
                        ],
                        "series": [
                            {"name": "Vessel O3 (%vol)", "type": "line", "smooth": True,
                             "yAxisIndex": 0, "data": [], "showSymbol": False,
                             "lineStyle": {"width": 2, "color": "#42A5F5"},
                             "areaStyle": {"opacity": 0.08}},
                            {"name": "Room O3 (ppm)", "type": "line", "smooth": True,
                             "yAxisIndex": 1, "data": [], "showSymbol": False,
                             "lineStyle": {"width": 2, "color": "#66BB6A"}},
                        ],
                        "dataZoom": [{"type": "inside"}, {"type": "slider"}],
                    }).classes("w-full").style("height: 260px")
                    echart_o3.visible = False

                # -- Power chart (expandable) -----------------------------
                with ui.expansion(
                    "Power", icon="bolt",
                ).classes("w-full").props("default-opened"):
                    echart_pwr = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "axis"},
                        "legend": {"data": ["Power Target (%)", "Power Actual (%)"]},
                        "xAxis": {"type": "time", "name": "Time"},
                        "yAxis": [
                            {"type": "value", "name": "Power %", "position": "left",
                             "max": 105},
                        ],
                        "series": [
                            {"name": "Power Target (%)", "type": "line", "smooth": True,
                             "data": [], "showSymbol": False,
                             "lineStyle": {"width": 2, "color": "#FFA726",
                                           "type": "dashed"}},
                            {"name": "Power Actual (%)", "type": "line", "smooth": True,
                             "data": [], "showSymbol": False,
                             "lineStyle": {"width": 2, "color": "#FF7043"}},
                        ],
                        "dataZoom": [{"type": "inside"}, {"type": "slider"}],
                    }).classes("w-full").style("height: 260px")
                    echart_pwr.visible = False

                # -- Temperature chart (expandable) -----------------------
                with ui.expansion(
                    "Temperature", icon="thermostat",
                ).classes("w-full"):
                    echart_temp = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "axis"},
                        "legend": {"data": [
                            "Cell Temp (\u00b0C)", "Vessel Temp (\u00b0C)",
                        ]},
                        "xAxis": {"type": "time", "name": "Time"},
                        "yAxis": [
                            {"type": "value", "name": "\u00b0C", "position": "left"},
                        ],
                        "series": [
                            {"name": "Cell Temp (\u00b0C)", "type": "line",
                             "smooth": True, "data": [], "showSymbol": False,
                             "lineStyle": {"width": 2, "color": "#EF5350"}},
                            {"name": "Vessel Temp (\u00b0C)", "type": "line",
                             "smooth": True, "data": [], "showSymbol": False,
                             "lineStyle": {"width": 2, "color": "#FF8A65"}},
                        ],
                        "dataZoom": [{"type": "inside"}, {"type": "slider"}],
                    }).classes("w-full").style("height: 260px")
                    echart_temp.visible = False

                # -- Pressure / Sensor chart (expandable) -----------------
                with ui.expansion(
                    "Pressure / Sensor", icon="speed",
                ).classes("w-full"):
                    echart_sensor = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "axis"},
                        "legend": {"data": [
                            "Pressure (mbar)", "Sample V", "Ref V",
                        ]},
                        "xAxis": {"type": "time", "name": "Time"},
                        "yAxis": [
                            {"type": "value", "name": "mbar", "position": "left"},
                            {"type": "value", "name": "V", "position": "right"},
                        ],
                        "series": [
                            {"name": "Pressure (mbar)", "type": "line",
                             "smooth": True, "yAxisIndex": 0,
                             "data": [], "showSymbol": False,
                             "lineStyle": {"width": 2, "color": "#AB47BC"}},
                            {"name": "Sample V", "type": "line",
                             "smooth": True, "yAxisIndex": 1,
                             "data": [], "showSymbol": False,
                             "lineStyle": {"width": 2, "color": "#26C6DA"}},
                            {"name": "Ref V", "type": "line",
                             "smooth": True, "yAxisIndex": 1,
                             "data": [], "showSymbol": False,
                             "lineStyle": {"width": 2, "color": "#9CCC65"}},
                        ],
                        "dataZoom": [{"type": "inside"}, {"type": "slider"}],
                    }).classes("w-full").style("height: 260px")
                    echart_sensor.visible = False

                with ui.expansion("Raw data", icon="table_chart").classes(
                    "w-full q-mt-sm"
                ):
                    async def _export_csv():
                        if not data_buf:
                            _notify("No data to export", "warning")
                            return
                        df = pd.DataFrame(list(data_buf))
                        fname = f"{datetime.now():%Y-%m-%d_%H%M%S}_Export.csv"
                        fpath = os.path.join(TELEMETRY_DIR, fname)
                        df.to_csv(fpath, index=False)
                        _notify(f"Exported {len(df)} rows to {fname}")
                        log(f"CSV export -> {fname}")

                    ui.button(
                        "Export CSV", icon="download", on_click=_export_csv,
                    ).props("dense flat size=sm").classes("q-mb-sm")

                    raw_table = ui.table(
                        columns=[
                            {"name": "timestamp", "label": "Time",
                             "field": "timestamp", "align": "left", "sortable": True},
                            {"name": "vessel_o3_pct", "label": "O3 %vol",
                             "field": "vessel_o3_pct", "sortable": True},
                            {"name": "cell_temp_c", "label": "Cell \u00b0C",
                             "field": "cell_temp_c", "sortable": True},
                            {"name": "pressure_mbar", "label": "Press mbar",
                             "field": "pressure_mbar", "sortable": True},
                            {"name": "sample_v", "label": "Sample V",
                             "field": "sample_v", "sortable": True},
                            {"name": "ref_v", "label": "Ref V",
                             "field": "ref_v", "sortable": True},
                            {"name": "room_o3_ppm", "label": "Room ppm",
                             "field": "room_o3_ppm", "sortable": True},
                            {"name": "vessel_temp_c", "label": "Vessel \u00b0C",
                             "field": "vessel_temp_c", "sortable": True},
                            {"name": "power_target_pct", "label": "Pwr Tgt %",
                             "field": "power_target_pct", "sortable": True},
                            {"name": "power_actual_pct", "label": "Pwr Act %",
                             "field": "power_actual_pct", "sortable": True},
                            {"name": "wiper_voltage", "label": "Wiper V",
                             "field": "wiper_voltage", "sortable": True},
                            {"name": "esp_ts_ms", "label": "ESP ts",
                             "field": "esp_ts_ms", "sortable": True},
                        ],
                        rows=[],
                        pagination={"rowsPerPage": 15},
                    ).classes("w-full")

            # =============================================================
            # DEBUG TAB
            # =============================================================
            with ui.tab_panel(tab_debug):
                ui.label("Debug Console").classes("text-h6 q-mb-sm")
                with ui.row().classes("w-full q-gutter-sm items-end"):
                    debug_input = ui.input(
                        "Command", placeholder="status"
                    ).classes("col-9")
                    ui.button("Send", on_click=_send_debug_cmd).classes("col-2")
                debug_resp_label = ui.label("").classes("text-body2 q-mt-sm")

                ui.separator().classes("q-my-md")
                ui.label("System State").classes("text-subtitle2")
                dbg_state_lbl = ui.code("").classes("w-full")

                ui.label("Log").classes("text-subtitle2 q-mt-md")
                dbg_log_html = ui.html("").classes("w-full").style(
                    "max-height: 320px; overflow-y: auto; "
                    "font-family: monospace; font-size: 0.82rem; "
                    "background: var(--q-dark-page, #1d1d1d); "
                    "border-radius: 8px; padding: 8px"
                )

            # =============================================================
            # SETTINGS TAB
            # =============================================================
            with ui.tab_panel(tab_settings):
                ui.label("Settings").classes("text-h6 q-mb-md")
                with ui.card().classes("q-pa-md"):
                    ui.label("Toast Notifications").classes("text-subtitle2 q-mb-sm")
                    ui.label(
                        "Control which notifications appear as pop-ups"
                    ).classes("text-caption text-grey q-mb-sm")
                    notify_select = ui.select(
                        options=["all", "errors", "none"],
                        value=S.notify_level,
                        label="Notification level",
                    ).classes("w-48")

                    def _on_notify_change(e):
                        S.notify_level = e.value
                        log(f"Notification level -> {e.value}")

                    notify_select.on("update:model-value", _on_notify_change)

    # =====================================================================
    # PERIODIC UI REFRESH  (1s timer)
    # =====================================================================
    _relay_ts: float = 0.0
    _echart_has_data: bool = False
    _last_prompt_id: str = ""
    _last_cal_active: bool = False
    _last_seq_active: bool = False
    _tick_running: bool = False

    def _refresh_flow_rate_selects() -> None:
        """Re-scan Models/O3Power/ and update all flow-rate select widgets."""
        fresh = list_calibrated_flow_rates()
        if not fresh:
            return
        # k_d flow select
        fill_lpm_input.options = {r: f"{r:.2f} LPM" for r in fresh}
        fill_lpm_input.update()
        # k_abs flow select
        kabs_flow_select.options = {r: f"{r:.2f} LPM" for r in fresh}
        kabs_flow_select.update()
        # processing flow select
        proc_flow_select.options = fresh
        proc_flow_select.update()

    async def _tick() -> None:
        nonlocal _updating, _relay_ts, _echart_has_data, _last_prompt_id, _tick_running
        nonlocal _last_seq_active
        if _tick_running:
            return
        _tick_running = True
        try:
            await _tick_inner()
        finally:
            _tick_running = False

    async def _tick_inner() -> None:
        nonlocal _updating, _relay_ts, _echart_has_data, _last_prompt_id
        nonlocal _last_cal_active, _last_seq_active

        while _notify_queue:
            _msg, _lvl = _notify_queue.popleft()
            _notify(_msg, _lvl)

        if S.seq_cleanup_pending and S.connected and not S.fill_active:
            S.seq_cleanup_pending = False
            log("Running deferred sequence cleanup from _tick", "seq")
            await _sequence_cleanup("deferred")

        if S.backfill_active and time.time() - S.backfill_start_time > 30:
            log("Backfill timeout (30s) — forcing backfill_active=False", "warn")
            S.backfill_active = False
            S.backfill_expected = 0
            S.backfill_received = 0

        if S.connected and not S.sequence_active and time.time() - _relay_ts > 10:
            await cmd_sync_relays()
            _relay_ts = time.time()

        # -- sidebar status -----------------------------------------------
        if S.connected:
            conn_badge.text = "Connected"
            conn_badge.props('color="green"')
            conn_badge.classes(remove="conn-blink")
            conn_badge.classes(add="conn-steady")
        else:
            conn_badge.text = "Disconnected"
            conn_badge.props('color="red"')
            conn_badge.classes(remove="conn-steady")
            conn_badge.classes(add="conn-blink")
        conn_badge.update()

        btn_air.props(f'color={"green" if S.relay_air_comp else "grey"}')
        btn_o2.props(f'color={"green" if S.relay_o2_conc else "grey"}')
        btn_o3.props(f'color={"green" if S.relay_o3_gen else "grey"}')

        card_flow_val.text = f"{S.flow_lpm:.2f}"
        card_o3_val.text = f"{S.vessel_o3_pct:.3f}"
        card_room_val.text = f"{S.room_o3_ppm:.3f}"
        card_vtemp_val.text = (
            f"{S.vessel_temp_c:.1f}" if S.vessel_temp_c > -900 else "N/A"
        )
        card_ctemp_val.text = f"{S.cell_temp_c:.1f}"

        if slider._props.get('model-value') != S.power_target_pct:
            slider._props['model-value'] = S.power_target_pct
            slider.update()
        if inp_pwr._props.get('model-value') != S.power_target_pct:
            inp_pwr._props['model-value'] = S.power_target_pct
            inp_pwr.update()
        _sync_derived_to_ui()

        # -- sequence banner + control lock --------------------------------
        seq_banner.visible = S.sequence_active
        if S.sequence_active:
            type_names = {
                "calibrate": "CALIBRATE",
                "verify": "VERIFY",
                "cstr_cal": "CSTR CAL",
                "k_abs_cal": "k_abs CAL",
                "process_batch": "BATCH",
            }
            display_name = type_names.get(S.seq_type, S.seq_type.upper())

            phase = S.seq_phase
            _PHASE_DESCS = {
                "loading": "Preparing...",
                "starting": f"Starting @ {S.cal_lpm:.2f} LPM",
                "started": "Initializing...",
                "relay_setup": "Enabling relays",
                "stabilizing": "Equipment warm-up (~3s)",
                "baseline": f"Initial baseline (0% power)",
                "sweep_up": f"Sweep up (0→100%)",
                "sweep_down": f"Sweep down (100→0%)",
                "random": f"Random hold @ {S.seq_power:.0f}%",
                "spot_low": f"Spot check low ({S.seq_power:.0f}%)",
                "spot_medium": f"Spot check mid ({S.seq_power:.0f}%)",
                "spot_max": f"Spot check max (100%)",
                "target": f"Target hold ({S.seq_power:.0f}%)",
                "cooldown": "Cooldown (0%)",
                "fill": f"Filling @ 100% — O3={S.vessel_o3_pct:.3f}%",
                "transition": "Transitioning to evac...",
                "evac": f"Evacuating @ 0% — O3={S.vessel_o3_pct:.4f}%",
                "setup": "Initializing CSTR calibration...",
                "saving": "Saving file...",
                "complete": "Complete!",
                "error": "Error — stopped",
            }
            phase_desc = _PHASE_DESCS.get(phase, phase.replace("_", " ").title())

            if phase in ("loading", "starting", "started"):
                seq_name_lbl.text = f"{display_name} — {phase_desc}"
                seq_phase_lbl.text = f"{S.seq_step_total} steps"
            elif phase == "relay_setup":
                seq_name_lbl.text = f"{display_name} — {phase_desc}"
                seq_phase_lbl.text = ""
            elif phase == "stabilizing":
                seq_name_lbl.text = f"{display_name} — {phase_desc}"
                seq_phase_lbl.text = "~3s warm-up"
            elif phase in ("saving", "complete"):
                seq_name_lbl.text = f"{display_name} — {phase_desc}"
                if S.seq_type == "cstr_cal":
                    seq_phase_lbl.text = f"{len(S.cstr_samples)} samples"
                else:
                    seq_phase_lbl.text = f"{len(S.cal_samples)} samples"
            else:
                if S.seq_type == "cstr_cal":
                    seq_name_lbl.text = f"{display_name} — {phase_desc}"
                    seq_phase_lbl.text = f"{len(S.cstr_samples)} samples"
                else:
                    seq_name_lbl.text = (
                        f"{display_name} — Step {S.seq_step_idx}/{S.seq_step_total}"
                    )
                    seq_phase_lbl.text = phase_desc
            seq_progress_bar.value = S.seq_progress / 100
            if S.seq_start_time > 0:
                S.seq_elapsed = time.time() - S.seq_start_time
            mins = int(S.seq_elapsed) // 60
            secs = int(S.seq_elapsed) % 60
            seq_elapsed_lbl.text = f"{mins}:{secs:02d}"

        lockable = [slider, inp_pwr, inp_o3, inp_mg, inp_g30,
                     btn_air, btn_o2, btn_o3, inp_lpm_sb, inp_lpm_settings,
                     cal_start_btn, cal_lpm_input, cal_rnd_input,
                     cal_air_toggle, ver_start_btn, fill_start_btn,
                     kabs_start_btn, kabs_flow_select,
                     proc_start_btn, proc_flow_select, proc_kg_input,
                     proc_dose_input, proc_time_input]
        lockable.extend(preset_btns)
        for _el in lockable:
            if S.sequence_active:
                _el.props("disable")
            else:
                _el.props(remove="disable")

        # -- prompt dialog ------------------------------------------------
        if S.pending_prompt_id and S.pending_prompt_id != _last_prompt_id:
            _last_prompt_id = S.pending_prompt_id
            content = PROMPT_CONTENT.get(S.pending_prompt_id, {
                "title": "Action Required",
                "icon": "help",
                "body": S.pending_prompt_text or "Please confirm to continue.",
            })
            prompt_icon_ref[0].name = content.get("icon", "help")
            prompt_icon_ref[0].update()
            prompt_title_ref[0].text = content["title"]
            prompt_body_ref[0].content = content["body"]
            prompt_dialog.open()
        elif not S.pending_prompt_id:
            _last_prompt_id = ""

        try:
            _restyle_markers()
        except RuntimeError:
            pass

        # -- telemetry ECharts ---------------------------------------------
        if data_buf:
            met_o3.text = f"Vessel O3: {S.vessel_o3_pct:.4f} %vol"
            met_room.text = f"Room O3: {S.room_o3_ppm:.3f} ppm"
            met_vt.text = (
                f"Vessel T: {S.vessel_temp_c:.1f} \u00b0C"
                if S.vessel_temp_c > -900 else "Vessel T: N/A"
            )
            met_ct.text = f"Cell T: {S.cell_temp_c:.1f} \u00b0C"

            if not _echart_has_data:
                telem_skeleton.visible = False
                echart_o3.visible = True
                echart_pwr.visible = True
                echart_temp.visible = True
                echart_sensor.visible = True
                _echart_has_data = True

            o3_data, room_data = [], []
            pwr_tgt_data, pwr_act_data = [], []
            cell_temp_data, vessel_temp_data = [], []
            pressure_data, sample_v_data, ref_v_data = [], [], []
            for s in data_buf:
                ts_ms = int(s["timestamp"].timestamp() * 1000)
                o3_data.append([ts_ms, s["vessel_o3_pct"]])
                room_data.append([ts_ms, s["room_o3_ppm"]])
                pwr_tgt_data.append([ts_ms, s.get("power_target_pct", 0)])
                pwr_act_data.append([ts_ms, s.get("power_actual_pct", 0)])
                cell_temp_data.append([ts_ms, s.get("cell_temp_c", 0)])
                vt = s.get("vessel_temp_c", -999)
                if vt > -900:
                    vessel_temp_data.append([ts_ms, vt])
                pressure_data.append([ts_ms, s.get("pressure_mbar", 0)])
                sample_v_data.append([ts_ms, s.get("sample_v", 0)])
                ref_v_data.append([ts_ms, s.get("ref_v", 0)])

            echart_o3.options["series"][0]["data"] = o3_data
            echart_o3.options["series"][1]["data"] = room_data
            echart_o3.update()
            echart_pwr.options["series"][0]["data"] = pwr_tgt_data
            echart_pwr.options["series"][1]["data"] = pwr_act_data
            echart_pwr.update()
            echart_temp.options["series"][0]["data"] = cell_temp_data
            echart_temp.options["series"][1]["data"] = vessel_temp_data
            echart_temp.update()
            echart_sensor.options["series"][0]["data"] = pressure_data
            echart_sensor.options["series"][1]["data"] = sample_v_data
            echart_sensor.options["series"][2]["data"] = ref_v_data
            echart_sensor.update()

            rows = []
            for s in list(data_buf)[-20:]:
                vt = s.get("vessel_temp_c", -999)
                rows.append({
                    "timestamp": str(s["timestamp"].strftime("%H:%M:%S")),
                    "vessel_o3_pct": f"{s['vessel_o3_pct']:.4f}",
                    "cell_temp_c": f"{s.get('cell_temp_c', 0):.1f}",
                    "pressure_mbar": f"{s.get('pressure_mbar', 0):.1f}",
                    "sample_v": f"{s.get('sample_v', 0):.4f}",
                    "ref_v": f"{s.get('ref_v', 0):.4f}",
                    "room_o3_ppm": f"{s['room_o3_ppm']:.3f}",
                    "vessel_temp_c": f"{vt:.1f}" if vt > -900 else "N/A",
                    "power_target_pct": f"{s.get('power_target_pct', 0)}",
                    "power_actual_pct": f"{s.get('power_actual_pct', 0):.1f}",
                    "wiper_voltage": f"{s.get('wiper_voltage', 0):.3f}",
                    "esp_ts_ms": str(s.get("esp_ts_ms", "")),
                })
            raw_table.rows = rows
            raw_table.update()

        # -- calibration observer UI --------------------------------------
        _cal_is_active = S.sequence_active and S.seq_type == "calibrate"
        if _cal_is_active:
            _CAL_TAB_LABELS = {
                "loading": "Preparing calibration",
                "starting": f"Starting calibration @ {S.cal_lpm:.2f} LPM",
                "started": "Initializing",
                "relay_setup": "Enabling relays",
                "stabilizing": "Equipment warm-up",
                "baseline": "Initial baseline (0% power)",
                "sweep_up": f"Sweep up (0→100%) @ {S.seq_power:.0f}%",
                "sweep_down": f"Sweep down (100→0%) @ {S.seq_power:.0f}%",
                "random": f"Random hold @ {S.seq_power:.0f}%",
                "saving": "Saving calibration file",
                "complete": "Calibration complete",
            }
            phase_text = _CAL_TAB_LABELS.get(
                S.seq_phase,
                S.seq_phase.replace("_", " ").title()
            )
            cal_phase_lbl.text = (
                f"Phase: {phase_text}  "
                f"Step {S.seq_step_idx}/{S.seq_step_total}"
            )
            cal_progress.value = S.seq_progress / 100
            cal_info_lbl.text = (
                f"Power={S.seq_power:.0f}%  "
                f"O3={S.vessel_o3_pct:.2f}%  "
                f"Samples={len(S.cal_samples)}"
            )
            phase_order = ["baseline", "sweep_up", "sweep_down", "random"]
            if S.seq_phase in ("saving", "complete"):
                active_idx = len(phase_order)
            else:
                active_idx = (
                    phase_order.index(S.seq_phase)
                    if S.seq_phase in phase_order else -1
                )
            for i, pk in enumerate(phase_order):
                card = cal_step_cards.get(pk)
                if card is None:
                    continue
                if i < active_idx:
                    card.style("opacity: 0.7; border: 1px solid #66BB6A")
                elif i == active_idx:
                    card.style(
                        "opacity: 1.0; border: 2px solid #42A5F5; "
                        "box-shadow: 0 0 8px rgba(66,165,245,0.4)"
                    )
                else:
                    card.style("opacity: 0.4; border: 1px solid #555")
        elif not S.sequence_active:
            # Detect calibration just completed → prompt for verification
            if _last_cal_active:
                _notify(
                    "Calibration complete — fit a model, then run "
                    "Verification before processing.",
                    "info",
                )
            cal_phase_lbl.text = "Phase: --"
            cal_progress.value = 0
            cal_info_lbl.text = ""
            for card in cal_step_cards.values():
                card.style("opacity: 0.4; border: 1px solid #555")
        _last_cal_active = _cal_is_active

        # Detect any sequence just completed → refresh flow rate selects
        if _last_seq_active and not S.sequence_active:
            _refresh_flow_rate_selects()
        _last_seq_active = S.sequence_active

        if S.cal_samples:
            sweep_up = [
                [s["power_actual"], s["o3_pct"]]
                for s in S.cal_samples if s.get("phase") == "sweep_up"
            ]
            sweep_down = [
                [s["power_actual"], s["o3_pct"]]
                for s in S.cal_samples if s.get("phase") == "sweep_down"
            ]
            random_pts = [
                [s["power_actual"], s["o3_pct"]]
                for s in S.cal_samples if s.get("phase") == "random"
            ]
            cal_chart.options["series"][0]["data"] = sweep_up
            cal_chart.options["series"][1]["data"] = sweep_down
            cal_chart.options["series"][2]["data"] = random_pts
            cal_chart.update()

        # -- verification observer UI (extracted) -------------------------
        update_verification_tab(_ver)

        # -- fill/evac observer UI ----------------------------------------
        if S.fill_active or S.fill_phase in ("complete", "error"):
            fill_progress.value = S.seq_progress / 100
            n_fill = sum(1 for s in S.cstr_samples if s.get("phase") == "fill")
            n_evac = sum(1 for s in S.cstr_samples if s.get("phase") == "evac")
            _FILL_PHASE_DESCS = {
                "setup": "Initializing...",
                "relay_setup": "Enabling relays",
                "baseline": f"Baseline (0% power) — {len(S.cstr_samples)} samples",
                "fill": (
                    f"Filling @ 100% — O3={S.vessel_o3_pct:.3f}% "
                    f"/ C_in={S.fill_target_o3:.3f}%"
                ),
                "transition": "Transitioning to evac...",
                "evac": (
                    f"Evacuating @ 0% — O3={S.vessel_o3_pct:.4f}%"
                ),
                "saving": "Saving data...",
                "complete": (
                    f"Complete — {n_fill} fill + {n_evac} evac samples"
                ),
                "error": "Error — sequence stopped",
            }
            fill_phase_lbl.text = _FILL_PHASE_DESCS.get(
                S.fill_phase, S.fill_phase.replace("_", " ").title()
            )

            if S.cstr_samples:
                x_labels = list(range(len(S.cstr_samples)))
                y_data = [s.get("vessel_o3_pct", 0) for s in S.cstr_samples]
                fill_chart.options["xAxis"]["data"] = x_labels
                fill_chart.options["series"][0]["data"] = y_data
                fill_chart.update()

        if not S.fill_active and S.fill_phase not in ("complete", "error"):
            fill_progress.value = 0
            fill_phase_lbl.text = ""

        # -- k_abs observer UI --------------------------------------------
        if S.sequence_active and S.seq_type == "k_abs_cal":
            kabs_progress.value = S.seq_progress / 100
            _KABS_DESCS = {
                "setup": "Initializing k_abs calibration...",
                "relay_setup": "Enabling relays",
                "prompt_mixing": "Awaiting mixing screw confirmation",
                "prompt_valve": "Awaiting L-valve confirmation",
                "baseline": f"Baseline — {len(S.cstr_samples)} samples",
                "fill_hold": (
                    f"Fill/Hold @ 100% — O3={S.vessel_o3_pct:.3f}%"
                ),
                "evac": f"Evacuating — O3={S.vessel_o3_pct:.4f}%",
                "saving": "Fitting model & saving...",
                "complete": "k_abs calibration complete!",
                "error": "Error — sequence stopped",
            }
            kabs_phase_lbl.text = _KABS_DESCS.get(
                S.fill_phase, S.fill_phase.replace("_", " ").title()
            )
        elif not S.sequence_active or S.seq_type != "k_abs_cal":
            kabs_progress.value = 0
            if not (S.sequence_active and S.seq_type == "k_abs_cal"):
                kabs_phase_lbl.text = ""

        # -- batch (process_batch) observer UI ----------------------------
        if S.sequence_active and S.seq_type == "process_batch":
            proc_progress.value = S.seq_progress / 100
            _phase = S.seq_phase or "idle"
            _BATCH_PHASE_COLORS = {
                "loading": "blue-grey",
                "recipe_load": "blue-grey",
                "running": "deep-orange",
                "baseline": "blue",
                "ramp": "orange",
                "hold": "green",
                "evac": "purple",
                "inoculation": "teal",
                "distribution": "cyan",
                "complete": "positive",
            }
            proc_phase_chip.text = _phase.replace("_", " ").upper()
            proc_phase_chip._props["color"] = _BATCH_PHASE_COLORS.get(
                _phase, "grey"
            )
            proc_phase_chip.update()

            _batch_mins = int(S.seq_elapsed) // 60
            _batch_secs = int(S.seq_elapsed) % 60
            proc_elapsed_lbl.text = f"Elapsed: {_batch_mins}:{_batch_secs:02d}"

            proc_dose_lbl.text = (
                f"Dose: {S.batch_dose_running:.1f} / "
                f"{S.batch_dose_target:.0f} mg/kg"
            )
            if S.batch_dose_target > 0:
                proc_dose_bar.value = min(
                    1.0, S.batch_dose_running / S.batch_dose_target
                )

            proc_phase_lbl.text = (
                f"Step {S.seq_step_idx}/{S.seq_step_total} — "
                f"O3={S.vessel_o3_pct:.4f}% — "
                f"Pwr={S.power_actual_pct:.0f}%"
            )

            # Update concentration chart with batch samples
            _bsamp = S.batch_samples[-500:]
            if _bsamp:
                _bx = list(range(len(_bsamp)))
                _by = [s.get("o3_pct", 0.0) for s in _bsamp]
                proc_conc_chart.options["xAxis"]["data"] = _bx
                proc_conc_chart.options["series"][0]["data"] = _by
                # C_target line (from schedule if available)
                _ct = (
                    S.batch_schedule.C_target
                    if S.batch_schedule
                    and hasattr(S.batch_schedule, "C_target")
                    else 0
                )
                proc_conc_chart.options["series"][1]["data"] = (
                    [round(_ct, 4)] * len(_bsamp) if _ct > 0 else []
                )
                proc_conc_chart.update()
                # ── Mass balance chart ──
                proc_mass_chart.options["xAxis"]["data"] = _bx
                proc_mass_chart.options["series"][0]["data"] = [
                    s.get("mg_produced", 0.0) for s in _bsamp]
                proc_mass_chart.options["series"][1]["data"] = [
                    s.get("mg_absorbed", 0.0) for s in _bsamp]
                proc_mass_chart.options["series"][2]["data"] = [
                    s.get("mg_decayed", 0.0) for s in _bsamp]
                proc_mass_chart.options["series"][3]["data"] = [
                    s.get("mg_evacuated", 0.0) for s in _bsamp]
                proc_mass_chart.update()

        elif S.seq_type != "process_batch":
            proc_phase_chip.text = "Idle"
            proc_phase_chip._props["color"] = "grey"
            proc_phase_chip.update()
            proc_elapsed_lbl.text = "--:--"
            proc_dose_lbl.text = "Dose: -- / -- mg/kg"
            proc_dose_bar.value = 0
            proc_progress.value = 0
            proc_phase_lbl.text = ""

        # -- debug state dump ---------------------------------------------
        dbg_state_lbl.content = json.dumps({
            "connected": S.connected,
            "time_synced": S.time_synced,
            "power_target": S.power_target_pct,
            "power_actual": round(S.power_actual_pct, 1),
            "power_error": S.power_error,
            "relays": {
                "o3_gen": S.relay_o3_gen,
                "o2_conc": S.relay_o2_conc,
                "air_comp": S.relay_air_comp,
            },
            "flow_lpm": S.flow_lpm,
            "vessel_o3": round(S.vessel_o3_pct, 4),
            "room_o3": round(S.room_o3_ppm, 3),
            "sequence_active": S.sequence_active,
            "seq_type": S.seq_type,
            "seq_phase": S.seq_phase,
            "seq_step": f"{S.seq_step_idx}/{S.seq_step_total}",
            "seq_progress": round(S.seq_progress, 1),
            "pending_prompt": S.pending_prompt_id,
            "cal_samples": len(S.cal_samples),
            "verify_samples": len(S.verify_samples),
            "verify_passed": getattr(S.verify_result, "passed", None),
            "fill_active": S.fill_active,
            "fill_phase": S.fill_phase,
            "cstr_samples": len(S.cstr_samples),
            "fill_target_o3": round(S.fill_target_o3, 4),
            "data_buf": len(data_buf),
            "last_update": str(S.last_update) if S.last_update else None,
        }, indent=2)

        lines_html = []
        for ts, cat, msg in list(debug_log):
            css_cls = f"log-{cat}" if cat else "log-info"
            lines_html.append(
                f'<div class="{css_cls}">'
                f'<span style="opacity:0.5">{ts}</span> {msg}</div>'
            )
        dbg_log_html.content = "\n".join(lines_html[-80:])

    ui.timer(1.0, _tick)
