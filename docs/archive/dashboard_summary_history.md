# Dashboard Agent Summary — Session History Archive

> Historical session changes archived for reference.
> Active summary: `docs/dashboard_agent_summary.md`
> Archived: 2026-03-15

---

## What Changed in Session 15
- **Evac-derived fill stopping**: Replaced range-based steady-state check with slope+range criterion grounded in measured evac asymptotic behaviour. The 2026-03-10 CSTR run's evac data shows |slope| ≈ 0.0003 at convergence (45-sample window, O3 0.02→0.01%). Fill's premature-stop slope was 0.00184 — 6× above threshold. `FILL_STEADY_SLOPE = 0.0003`, `FILL_STEADY_COUNT = 45`, `FILL_STEADY_RANGE = 0.08` (sanity bound). Removed `FILL_MIN_SAMPLES` (redundant — 45-sample window is natural guard).
- **Validation-measured target O3**: `target_o3` now uses `mean(o3_pct)` from the most recent 100% validation PASS CSV (direct-to-sensor, held at steady-state). Falls back to sigmoid prediction only if no validation cert exists. Avoids circular model reasoning.
- **Updated constants**: `FILL_STEADY_COUNT` 30→45, `FILL_STEADY_RANGE` 0.05→0.08, `FILL_STEADY_SLOPE = 0.0003` (evac-derived), removed `FILL_MIN_SAMPLES`

## What Changed in Session 14
- **CSTR Power Watchdog**: Fill loop now monitors `power_actual_pct` every sample after a 5-sample grace period. If motor pot drifts below `FILL_POWER_MIN_PCT` (80%), auto-resends `cmd_set_power(100)` up to `FILL_POWER_RESEND_MAX` (3) times. Aborts with RuntimeError if resends exhausted. O3 ring buffer cleared after resend (drift readings invalidate steady-state window).
- **`power_actual_pct` in CSTR CSV**: `_snap()` now records both `power_pct` (target) and `power_actual_pct` (ESP32 ADC-read position) in every sample.
- **Root cause analysis**: Runs 1-3 caused by deferred cleanup race (fixed in commit `a27c224`). Run 4 caused by motor pot physical drift.
- **New constants**: `FILL_POWER_GRACE_SAMPLES = 5`, `FILL_POWER_RESEND_MAX = 3`

## What Changed in Session 13b
- `_notify` Background-Task Safety: `_notify_queue` pattern for safe UI notifications from background tasks
- Validation Certificate: PASS/FAIL suffix on CSV filename; `_find_valid_cert()` scans for valid certs; CSTR calibration requires a 100% PASS cert within 24h

## What Changed in Session 13
- Backfill-Active Stuck Fix (ROOT CAUSE for frozen sidebar + stuck green dot)
- Power-O3 Chart: Static Curve + Dynamic Markers (`_restyle_markers()`)
- CI Band Opacity Increase (0.15 → 0.25)
- Compact Sidebar Cards (5 → 1 dense card)
- Decay-Aware CSTR Model rewrite (`CSTRModel` dataclass)
- Directory Restructure: `Models/Fill/` → `Models/CSTR/`, `Data/Fill/` + `Data/Evac/` → `Data/CSTR/`
- Fill Termination: 30 consecutive samples with range < 0.05 %vol

## What Changed in Session 12
- Power Curve Live Update Fix (`power_plot.figure = fig; power_plot.update()`)
- Validation Baseline: Advisory Warning, Not Hard Gate
- ±1σ Confidence Band on Power-O3 Curve (Jacobian-propagated)
- Validation Result Card Simplified (removed confusing sample count)

## What Changed in Session 11
- K constant reconciliation: `O3_MASS_FLOW_K = 0.3327`
- Fill/Evac analysis module (`fill_model.py`)
- Fill/Evac sequence coroutine + UI expansion
- ESP32 dosimetry defaults
- Telemetry improvements: per-connection stream naming, backfill protocol

## What Changed in Session 10
- Random Phase + Air Toggle for calibration
- Safe Standby (`_safe_standby()` unconditional)
- DFRobot I2C Fix (100kHz, separate transactions)
- Single-Agent Model decision

## What Changed in Session 9
- Sigmoid model fitting system (`power_o3_model.py`)
- Dashboard integration: `predict_o3_from_power/predict_power_from_o3`
- Model Fitting UI
- Bug fixes: reconnection race, dispatch safety, deadlock fixes
