"""
Verification tab — build + tick-update logic.

Extracted from ui_main.py to reduce monolith size.
"""
from __future__ import annotations

from types import SimpleNamespace
from typing import Any

from nicegui import ui

from dashboard.state import S, _notify
from dashboard.data_io import list_calibrated_flow_rates


# ── builder ─────────────────────────────────────────────────────────────
def build_verification_tab(tab_ref, cmd_sequence_start) -> SimpleNamespace:
    """Build the Verification tab panel and return widget references."""
    w = SimpleNamespace()

    with ui.tab_panel(tab_ref):
        ui.label("Verification").classes("text-h6 q-mb-sm")
        ui.markdown(
            "Pre-batch verification: measures C_in at 100% power and "
            "a hold power level. Checks stability in each phase. "
            "Air compressor must be OFF."
        ).classes("text-caption text-grey q-mb-sm")

        _ver_flow_rates = list_calibrated_flow_rates()
        with ui.row().classes("q-gutter-sm items-end q-mb-sm"):
            ver_hold_input = ui.number(
                label="Hold Power %", value=50.0,
                min=10, max=100, step=5, format="%.0f",
            ).classes("w-28")
            w.hold_input = ver_hold_input
            ver_lpm_input = ui.select(
                label="Flow Rate (LPM)",
                options={r: f"{r:.2f} LPM" for r in _ver_flow_rates}
                    if _ver_flow_rates
                    else {0: "No calibrated rates"},
                value=_ver_flow_rates[0] if _ver_flow_rates else 0,
            ).classes("w-40")
            w.lpm_input = ver_lpm_input
            async def _start_verify():
                hold_pwr = float(ver_hold_input.value or 50)
                lpm_val = ver_lpm_input.value
                if not lpm_val or float(lpm_val) <= 0:
                    _notify(
                        "No calibrated flow rate selected",
                        "negative",
                    )
                    return
                lpm = float(lpm_val)
                await cmd_sequence_start(
                    "verify", power_hold=hold_pwr, flow=lpm,
                )

            w.start_btn = ui.button(
                "Verify", icon="play_arrow",
                on_click=_start_verify, color="blue",
            )

        # Phase progress cards
        VER_PHASES_DEF = [
            ("baseline", "Baseline", "0% — residual check"),
            ("full_power", "Full Power", "100% — measure C_in"),
            ("hold_power", "Hold Power", "Hold % — measure C_in"),
            ("cooldown", "Cooldown", "0% — shutdown"),
        ]
        w.step_cards = {}
        with ui.row().classes("w-full q-gutter-xs q-mb-sm"):
            for phase_key, name, desc in VER_PHASES_DEF:
                with ui.card().classes(
                    "col q-pa-xs text-center"
                ).style(
                    "opacity: 0.4; border: 1px solid #555"
                ) as vsc:
                    ui.label(name).classes(
                        "text-caption text-weight-bold"
                    )
                    ui.label(desc).classes("text-caption text-grey")
                w.step_cards[phase_key] = vsc

        w.phase_lbl = ui.label("Phase: --").classes("text-body2")
        w.progress = ui.linear_progress(
            value=0, show_value=False,
        ).classes("q-mb-xs")
        w.info_lbl = ui.label("").classes("text-caption")

        # Result card
        w.result_card = ui.card().classes("w-full q-pa-md q-mt-sm")
        w.result_card.visible = False
        with w.result_card:
            with ui.row().classes(
                "w-full items-center justify-between"
            ):
                with ui.row().classes("items-center q-gutter-sm"):
                    w.result_icon = ui.icon(
                        "check_circle"
                    ).classes("text-h3")
                    w.result_title = ui.label("").classes("text-h6")

                def _dismiss_ver_result():
                    S.verify_result = None
                    w.result_card.visible = False

                ui.button(
                    icon="close",
                    on_click=_dismiss_ver_result,
                    color="grey",
                ).props("flat dense round size=sm")
            with ui.row().classes("q-gutter-md q-mt-sm"):
                w.cin100_lbl = ui.label("")
                w.cinhold_lbl = ui.label("")
                w.baseline_lbl = ui.label("")
            with ui.row().classes("q-gutter-md q-mt-xs"):
                w.fp_stable_lbl = ui.label("")
                w.hp_stable_lbl = ui.label("")
            w.file_lbl = ui.label("").classes(
                "text-caption text-grey q-mt-xs"
            )

        w.chart = ui.echart({
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

    return w


# ── tick updater ────────────────────────────────────────────────────────
def update_verification_tab(w: SimpleNamespace) -> None:
    """Called every tick from _tick_inner — update verification widgets."""
    if S.sequence_active and S.seq_type == "verify":
        _VER_PHASE_LABELS = {
            "loading": "Preparing verification",
            "running": "Running verification",
            "baseline": "Baseline (0% power)",
            "full_power": f"Full power (100%) @ {S.seq_power:.0f}%",
            "hold_power": f"Hold power @ {S.seq_power:.0f}%",
            "cooldown": "Cooldown (0% power)",
            "complete": "Verification complete",
        }
        phase_text = _VER_PHASE_LABELS.get(
            S.seq_phase,
            S.seq_phase.replace("_", " ").title(),
        )
        w.phase_lbl.text = (
            f"Phase: {phase_text}  "
            f"Step {S.seq_step_idx}/{S.seq_step_total}"
        )
        w.progress.value = S.seq_progress / 100
        w.info_lbl.text = (
            f"Power={S.seq_power:.0f}%  "
            f"O3={S.vessel_o3_pct:.2f}%  "
            f"Samples={len(S.verify_samples)}"
        )
        phase_order = ["baseline", "full_power", "hold_power", "cooldown"]
        if S.seq_phase in ("complete",):
            active_idx = len(phase_order)
        else:
            active_idx = (
                phase_order.index(S.seq_phase)
                if S.seq_phase in phase_order else -1
            )
        for i, pk in enumerate(phase_order):
            card = w.step_cards.get(pk)
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
    elif not (S.sequence_active and S.seq_type == "verify"):
        w.phase_lbl.text = "Phase: --"
        w.progress.value = 0
        w.info_lbl.text = ""
        for card in w.step_cards.values():
            card.style("opacity: 0.4; border: 1px solid #555")

    if S.verify_samples:
        w.chart.options["xAxis"]["data"] = list(
            range(len(S.verify_samples))
        )
        w.chart.options["series"][0]["data"] = [
            s["o3_pct"] for s in S.verify_samples
        ]
        w.chart.update()

    if S.verify_result is not None:
        w.result_card.visible = True
        r = S.verify_result
        passed = getattr(r, "passed", False)
        if passed:
            color, icon_name = "green", "check_circle"
        else:
            color, icon_name = "red", "error"
        w.result_icon.name = icon_name
        w.result_icon._classes = [f"text-h3 text-{color}"]
        w.result_icon.update()
        status = "PASSED" if passed else "FAILED"
        w.result_title.text = f"Verification {status}"
        w.cin100_lbl.text = (
            f"C_in @ 100%: {getattr(r, 'c_in_100pct', 0):.4f} %vol"
        )
        w.cinhold_lbl.text = (
            f"C_in @ {getattr(r, 'power_hold_pct', 0):.0f}%: "
            f"{getattr(r, 'c_in_hold', 0):.4f} %vol"
        )
        w.baseline_lbl.text = (
            f"Baseline: {getattr(r, 'baseline_mean', 0):.4f} %vol"
        )
        fp_ok = getattr(r, "full_power_stable", False)
        hp_ok = getattr(r, "hold_power_stable", False)
        w.fp_stable_lbl.text = (
            f"100% stable: {'Yes' if fp_ok else 'No'} "
            f"(slope={getattr(r, 'full_power_slope', 0):.5f})"
        )
        w.hp_stable_lbl.text = (
            f"Hold stable: {'Yes' if hp_ok else 'No'} "
            f"(slope={getattr(r, 'hold_power_slope', 0):.5f})"
        )
        w.file_lbl.text = (
            f"Saved: {S.verify_file}" if S.verify_file else ""
        )
