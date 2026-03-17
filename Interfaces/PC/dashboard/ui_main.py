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
)
from dashboard.commands import (
    cmd_set_power, cmd_set_relay, cmd_sync_relays,
    cmd_emergency_stop, cmd_sequence_start, cmd_sequence_stop,
    cmd_sequence_confirm, _sequence_cleanup,
)
from dashboard.cstr_sequence import (
    _fit_and_save_cstr_model,
    FILL_STEADY_RANGE, FILL_STEADY_COUNT,
)
from dashboard.tcp_server import tcp


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
            value=S.flow_lpm, min=1.0, max=15.0, step=0.5,
            format="%.1f", on_change=_on_lpm_change,
        ).classes("q-mb-sm")

        ui.separator().classes("q-my-sm")

        ui.label("Readings").classes("text-subtitle2")
        with ui.card().classes("w-full q-pa-sm").props("flat bordered"):
            with ui.row().classes("w-full items-center justify-between no-wrap"):
                ui.label("O2 Flow").classes("text-caption text-grey")
                card_flow_val = ui.label(f"{S.flow_lpm:.1f}").classes("text-subtitle1 text-weight-medium text-purple")
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
            tab_power = ui.tab("Power", icon="bolt")
            tab_telem = ui.tab("Telemetry", icon="show_chart")
            tab_debug = ui.tab("Debug", icon="bug_report")
            tab_settings = ui.tab("Settings", icon="settings")

        with ui.tab_panels(tabs, value=tab_power).classes("w-full"):

            # =============================================================
            # POWER TAB
            # =============================================================
            with ui.tab_panel(tab_power):

                with ui.expansion("Power Control", icon="speed",
                                  value=True).classes("w-full q-mb-sm"):
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
                                step=0.5, format="%.1f",
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

                # ---- Calibration section --------------------------------
                with ui.expansion("Calibration", icon="tune").classes(
                    "w-full q-mb-sm"
                ):
                    ui.markdown(
                        "Calibration: Baseline → Sweep Up (0→100%) → "
                        "Sweep Down (100→0%) → Random hold (~20 samples each). "
                        "ESP32 runs sweep autonomously."
                    ).classes("text-caption text-grey q-mb-sm")

                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        cal_lpm_input = ui.number(
                            label="O2 LPM", value=DEFAULT_FLOW_LPM,
                            min=0.0, max=15, step=0.5, format="%.1f",
                        ).classes("w-24")
                        cal_rnd_input = ui.number(
                            label="# Rnd Lvls", value=15,
                            min=0, max=50, step=1, format="%.0f",
                        ).classes("w-24")
                        cal_air_toggle = ui.switch("Air ON").classes(
                            "q-ml-sm"
                        )

                        async def _start_cal():
                            lpm = float(cal_lpm_input.value or 0.0)
                            num_rnd = int(cal_rnd_input.value or 0)
                            air_on = bool(cal_air_toggle.value)
                            if lpm <= 0.0 and not air_on:
                                _notify(
                                    "Error: O2 LPM must be > 0 or Air "
                                    "Compressor must be enabled",
                                    "negative",
                                )
                                return
                            await cmd_sequence_start(
                                "calibrate", flow=lpm,
                                num_random=num_rnd, air_comp=air_on,
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

                # ---- Validation section ---------------------------------
                with ui.expansion("Validation", icon="verified").classes(
                    "w-full q-mb-sm"
                ):
                    ui.markdown(
                        "Pre-flight check: PC generates a recipe with baseline, "
                        "spot checks, and target power hold. Compares measured "
                        "O3 to model prediction. Air compressor must be OFF."
                    ).classes("text-caption text-grey q-mb-sm")

                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        val_pwr_input = ui.number(
                            label="Power %", value=75.0,
                            min=10, max=100, step=5, format="%.0f",
                        ).classes("w-24")
                        val_lpm_input = ui.number(
                            label="O2 LPM", value=DEFAULT_FLOW_LPM,
                            min=0.5, max=15, step=0.5, format="%.1f",
                        ).classes("w-24")

                        async def _start_val():
                            pwr = float(val_pwr_input.value or 75)
                            lpm = float(val_lpm_input.value or DEFAULT_FLOW_LPM)
                            await cmd_sequence_start(
                                "validate", power=pwr, flow=lpm,
                            )

                        val_start_btn = ui.button(
                            "Validate", icon="play_arrow",
                            on_click=_start_val, color="blue",
                        )

                    val_result_card = ui.card().classes(
                        "w-full q-pa-md q-mt-sm"
                    )
                    val_result_card.visible = False
                    with val_result_card:
                        with ui.row().classes(
                            "w-full items-center justify-between"
                        ):
                            with ui.row().classes("items-center q-gutter-sm"):
                                val_result_icon = ui.icon(
                                    "check_circle"
                                ).classes("text-h3")
                                val_result_title = ui.label("").classes(
                                    "text-h6"
                                )

                            def _dismiss_val_result():
                                S.val_result = {}
                                val_result_card.visible = False

                            ui.button(
                                icon="close",
                                on_click=_dismiss_val_result,
                                color="grey",
                            ).props("flat dense round size=sm")
                        with ui.row().classes("q-gutter-md q-mt-sm"):
                            val_mean_lbl = ui.label("")
                            val_expected_lbl = ui.label("")
                            val_dev_lbl = ui.label("")
                            val_std_lbl = ui.label("")
                        val_file_lbl = ui.label("").classes(
                            "text-caption text-grey q-mt-xs"
                        )

                    val_chart = ui.echart({
                        "darkMode": True,
                        "tooltip": {"trigger": "axis"},
                        "xAxis": {"type": "category", "name": "Sample",
                                  "data": []},
                        "yAxis": {"type": "value", "name": "O3 (%vol)"},
                        "series": [
                            {"name": "O3", "type": "line", "smooth": True,
                             "data": [], "showSymbol": False,
                             "areaStyle": {"opacity": 0.1}},
                        ],
                    }).classes("w-full").style("height: 220px")

                # ---- Fill / Evacuation Calibration section ---------------
                with ui.expansion(
                    "CSTR Calibration", icon="air"
                ).classes("w-full q-mb-sm"):
                    ui.markdown(
                        "Fill the vessel at 100% power until steady-state, "
                        "then evacuate at 0% power until O3 clears. "
                        "Fits a decay-aware CSTR model to extract system volume, "
                        "O3 decay rate, and dead volume. Air compressor is OFF "
                        "during calibration for best decay sensitivity. "
                        "Parameters generalise to any flow rate and air config."
                    ).classes("text-caption text-grey q-mb-sm")

                    with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
                        fill_lpm_input = ui.number(
                            label="O2 LPM", value=DEFAULT_FLOW_LPM,
                            min=0.5, max=5, step=0.5, format="%.1f",
                        ).classes("w-24")

                    fill_target_lbl = ui.label("").classes(
                        "text-caption text-grey q-mb-xs"
                    )

                    def _update_fill_target_label() -> None:
                        lpm = float(fill_lpm_input.value or DEFAULT_FLOW_LPM)
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
                            lpm = float(fill_lpm_input.value or DEFAULT_FLOW_LPM)
                            cert = _find_valid_cert(100, lpm)
                            if cert is None:
                                with ui.dialog() as _cert_dlg, ui.card().classes("q-pa-md"):
                                    ui.label(
                                        "No valid validation certificate found"
                                    ).classes("text-subtitle1 text-bold q-mb-sm")
                                    ui.label(
                                        "A 100% power validation at the selected flow rate "
                                        "must pass within the last 24 hours before running "
                                        "CSTR calibration. This ensures the C_in used for "
                                        "model fitting is accurate."
                                    ).classes("text-body2 q-mb-md")
                                    with ui.row().classes("q-gutter-sm justify-end"):
                                        ui.button(
                                            "Cancel", on_click=_cert_dlg.close,
                                        ).props("flat")

                                        async def _run_val_from_dlg():
                                            _cert_dlg.close()
                                            val_pwr_input.value = 100
                                            val_lpm_input.value = lpm
                                            await cmd_sequence_start(
                                                "validate", power=100, flow=lpm,
                                            )

                                        ui.button(
                                            "Run Validation", icon="verified",
                                            on_click=_run_val_from_dlg,
                                            color="blue",
                                        )
                                _cert_dlg.open()
                                return
                            await cmd_sequence_start(
                                "cstr_cal",
                                flow=lpm,
                            )

                        fill_start_btn = ui.button(
                            "Start CSTR Calibration", icon="play_arrow",
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

                echart_pwr = ui.echart({
                    "darkMode": True,
                    "tooltip": {"trigger": "axis"},
                    "legend": {"data": ["Power (%)", "Cell Temp (\u00b0C)"]},
                    "xAxis": {"type": "time", "name": "Time"},
                    "yAxis": [
                        {"type": "value", "name": "Power %", "position": "left", "max": 105},
                        {"type": "value", "name": "\u00b0C", "position": "right"},
                    ],
                    "series": [
                        {"name": "Power (%)", "type": "line", "smooth": True,
                         "yAxisIndex": 0, "data": [], "showSymbol": False,
                         "lineStyle": {"width": 2, "color": "#FFA726"}},
                        {"name": "Cell Temp (\u00b0C)", "type": "line", "smooth": True,
                         "yAxisIndex": 1, "data": [], "showSymbol": False,
                         "lineStyle": {"width": 2, "color": "#EF5350"}},
                    ],
                    "dataZoom": [{"type": "inside"}, {"type": "slider"}],
                }).classes("w-full").style("height: 260px")
                echart_pwr.visible = False

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
                            {"name": "vessel_o3_pct", "label": "O3 %",
                             "field": "vessel_o3_pct", "sortable": True},
                            {"name": "room_o3_ppm", "label": "Room ppm",
                             "field": "room_o3_ppm", "sortable": True},
                            {"name": "power_actual_pct", "label": "Power %",
                             "field": "power_actual_pct", "sortable": True},
                            {"name": "cell_temp_c", "label": "Cell \u00b0C",
                             "field": "cell_temp_c", "sortable": True},
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
    _tick_running: bool = False

    async def _tick() -> None:
        nonlocal _updating, _relay_ts, _echart_has_data, _last_prompt_id, _tick_running

        if _tick_running:
            return
        _tick_running = True
        try:
            await _tick_inner()
        finally:
            _tick_running = False

    async def _tick_inner() -> None:
        nonlocal _updating, _relay_ts, _echart_has_data, _last_prompt_id

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

        card_flow_val.text = f"{S.flow_lpm:.1f}"
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
                "validate": "VALIDATE",
                "cstr_cal": "CSTR CAL",
            }
            display_name = type_names.get(S.seq_type, S.seq_type.upper())

            phase = S.seq_phase
            _PHASE_DESCS = {
                "loading": "Preparing...",
                "starting": f"Starting @ {S.cal_lpm:.1f} LPM",
                "started": "Initializing...",
                "relay_setup": "Enabling relays",
                "stabilizing": "Equipment warm-up (~3s)",
                "baseline": f"Initial baseline (0% power)",
                "sweep_up": f"Sweep up (0→100%)",
                "sweep_down": f"Sweep down (100→0%)",
                "random": f"Random hold @ {S.seq_power:.0f}%",
                "spot_low": f"Spot check low ({S.seq_power:.0f}%)",
                "spot_high": f"Spot check high ({S.seq_power:.0f}%)",
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
                     btn_air, btn_o2, btn_o3, inp_lpm_sb,
                     cal_start_btn, cal_lpm_input, cal_rnd_input,
                     cal_air_toggle, val_start_btn, fill_start_btn]
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
                _echart_has_data = True

            o3_data, room_data, pwr_data, temp_data = [], [], [], []
            for s in data_buf:
                ts_ms = int(s["timestamp"].timestamp() * 1000)
                o3_data.append([ts_ms, s["vessel_o3_pct"]])
                room_data.append([ts_ms, s["room_o3_ppm"]])
                pwr_data.append([ts_ms, s.get("power_actual_pct", 0)])
                temp_data.append([ts_ms, s.get("cell_temp_c", 0)])

            echart_o3.options["series"][0]["data"] = o3_data
            echart_o3.options["series"][1]["data"] = room_data
            echart_o3.update()
            echart_pwr.options["series"][0]["data"] = pwr_data
            echart_pwr.options["series"][1]["data"] = temp_data
            echart_pwr.update()

            rows = []
            for s in list(data_buf)[-20:]:
                rows.append({
                    "timestamp": str(s["timestamp"].strftime("%H:%M:%S")),
                    "vessel_o3_pct": f"{s['vessel_o3_pct']:.4f}",
                    "room_o3_ppm": f"{s['room_o3_ppm']:.3f}",
                    "power_actual_pct": f"{s.get('power_actual_pct', 0):.1f}",
                    "cell_temp_c": f"{s.get('cell_temp_c', 0):.1f}",
                })
            raw_table.rows = rows
            raw_table.update()

        # -- calibration observer UI --------------------------------------
        if S.sequence_active and S.seq_type == "calibrate":
            _CAL_TAB_LABELS = {
                "loading": "Preparing calibration",
                "starting": f"Starting calibration @ {S.cal_lpm:.1f} LPM",
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
            cal_phase_lbl.text = "Phase: --"
            cal_progress.value = 0
            cal_info_lbl.text = ""
            for card in cal_step_cards.values():
                card.style("opacity: 0.4; border: 1px solid #555")

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

        # -- validation observer UI ---------------------------------------
        if S.val_samples:
            val_chart.options["xAxis"]["data"] = list(range(len(S.val_samples)))
            val_chart.options["series"][0]["data"] = [
                s["o3_pct"] for s in S.val_samples
            ]
            val_chart.update()

        if S.val_result:
            val_result_card.visible = True
            r = S.val_result
            passed = r.get("passed", False)
            dev = r.get("deviation_pct", 0.0)
            cv = r.get("cv_pct", 0.0)
            if passed:
                color, icon_name = "green", "check_circle"
            elif isinstance(dev, (int, float)) and dev < 20:
                color, icon_name = "amber", "warning"
            else:
                color, icon_name = "red", "error"
            val_result_icon.name = icon_name
            val_result_icon._classes = [f"text-h3 text-{color}"]
            val_result_icon.update()
            status = "PASSED" if passed else "FAILED"
            val_result_title.text = f"{status} — {dev:.1f}% deviation"
            val_mean_lbl.text = f"Mean O3: {r.get('mean_o3', 0):.3f}%"
            val_expected_lbl.text = f"Expected: {r.get('expected_o3', 0):.3f}%"
            val_dev_lbl.text = f"CV: {cv:.1f}%"
            val_std_lbl.text = f"Std: {r.get('std_o3', 0):.4f}%"
            val_file_lbl.text = (
                f"Saved: {S.val_file}" if S.val_file else ""
            )

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
            "val_samples": len(S.val_samples),
            "val_passed": S.val_result.get("passed", None),
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
