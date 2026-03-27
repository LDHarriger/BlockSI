# MEMORY_COPILOT.md — Copilot Chat Agent Working Memory

> Agent: GitHub Copilot Chat (local VS Code plugin)
> Last updated: 2026-03-26

---

## Active Locks

*(none)*

---

## Active Tasks

- Power mismatch audit: ✅ RESOLVED — documented in `docs/cases/power_mismatch_audit.md`
- Flow rate precision: ✅ Complete — all formatting upgraded to 2 decimals
- Verification slope threshold: ✅ Complete — relative slope criterion added
- Dashboard refresh: ✅ Complete — flow rate selects refresh after sequence completion
- Task 4 UI improvements: ✅ Complete — tab redirect, 4 expandable telemetry plots, 12-column raw table
- Uncommitted changes: All session work + prior refactoring session changes

---

## Session Context

- Current branch: `main`
- Dashboard: `Interfaces/PC/blocksi_dashboard.py` (entry point; modules in `dashboard/`)
- Python venv: `.venv\Scripts\python.exe`
- NiceGUI 3.8.0, Python 3.14

### Current Session (2026-03-25 — Task Package continuation)

| Task | Description | Status |
|------|-------------|--------|
| Task 2.4 | Firmware diagnostic commands (diag_power_drift, diag_power_noise) | ✅ |
| Task 2.5 | Enhanced DIAG,power_mismatch with adc, settle fields | ✅ |
| Task 2.6 | Dashboard DIAG logging to Data/Diagnostics/ | ✅ |
| Task 3 | Rename Data/Calibration → Data/Power-O3_cal (code refs updated) | ✅ (dir rename blocked) |
| Task 1 | Measurement Verification (verification.py + verification_sequence.py) | ✅ |
| Task 4 | GUI Improvements (UI tab, collapsible sections, post-cal prompt) | ✅ |

### Files changed this session

| File | Change |
|------|--------|
| `Interfaces/ControlSystem/main/main.c` | Added diag_power_drift + diag_power_noise commands |
| `Interfaces/ControlSystem/main/blocksi_state.c` | Enhanced DIAG,power_mismatch with adc + settle fields |
| `Interfaces/PC/dashboard/state.py` | Added DIAGNOSTICS_DIR, verify_* state fields, CALIBRATION_DIR → Power-O3_cal |
| `Interfaces/PC/dashboard/tcp_server.py` | DIAG log persistence, verify SAMPLE routing |
| `Interfaces/PC/dashboard/verification.py` | **NEW** — stability analysis + VerificationResult |
| `Interfaces/PC/dashboard/verification_sequence.py` | **NEW** — recipe protocol background task |
| `Interfaces/PC/dashboard/commands.py` | Added "verify" dispatch |
| `Interfaces/PC/dashboard/data_io.py` | Updated comment for Power-O3_cal path |
| `Interfaces/PC/dashboard/ui_main.py` | Replaced Validation tab → Verification tab, collapsible calibration sections, post-cal prompt, verification observer in _tick_inner() |
| `docs/decisions_log.md` | Added 2026-03-25 verification architecture decision |

### Pending Feature Work

| Item | Status |
|------|--------|
| Historical data viewer (load & plot old CSVs) | `[PROPOSED]` |
| Migrate power curve from Plotly to ECharts | `[PROPOSED]` |
| Firmware changes from WP-0 audit | See `docs/user/TODO_checklist.md` §5 |

---

## Notes for Next Session

- **Model filenames changed**: `power_o3_3.8lpm_95o2.json` → `power_o3_3.80lpm_95o2.json` (2-decimal format). Existing models won't be found by the new `_model_filename()`. User should rename existing model files or re-run calibration + model fit.
- **Verification data filenames changed**: New files use 2-decimal LPM (e.g., `3.75Lpm`). `_find_valid_cert()` regex updated to match any decimal format so old PASS files still work.
- Empty `Interfaces/Data/Calibration/` directory remains — user needs to delete it
- **validation.py archived** → `Old/PC/validation.py` — no code imports it anymore
- `_save_val_csv()` definition remains in data_io.py (dead code, zero callers) — clean up later
- **ui_main.py split started**: Verification tab extracted to `ui_tab_verification.py` (pattern established)
  - Remaining tabs to extract: Calibration (~566 lines), Processing (~330 lines), Telemetry (~150 lines)
- All changes uncommitted

### Session (2026-03-26) — Multi-Part Implementation

| File | Change |
|------|--------|
| `docs/cases/power_mismatch_audit.md` | Updated status to RESOLVED, added Resolution section with full summary |
| `dashboard/ui_main.py` | Flow rate inputs: step 0.5→0.25, format `%.1f`→`%.2f`; 4 expandable telemetry charts (O3, Power, Temp, Pressure/Sensor); raw data table expanded from 5 to 12 columns; tab redirect on verification from cert dialogs; flow rate select refresh on sequence completion |
| `dashboard/verification.py` | Added `STABILITY_REL_SLOPE_THRESHOLD=0.0005`; `_check_stability()` now passes if absolute OR relative slope criterion met |
| `dashboard/verification_sequence.py` | lpm_s format: smart→`:.2f` |
| `dashboard/data_io.py` | `_save_cal_csv()`, `_save_val_csv()`, `_find_valid_cert()`: lpm_s format→`:.2f`; cert regex updated to match any decimal LPM in filename |
| `dashboard/k_d_cal.py` | lpm_s format→`:.2f` |
| `dashboard/k_abs_cal.py` | lpm_s format→`:.2f`; flow display→`:.2f`; added "Run Verification" button to cert dialog |
| `dashboard/dosimetry.py` | flow display→`:.2f` |
| `analysis/power_o3_model.py` | `_model_filename()` lpm_s format→`:.2f` |
| `analysis/cstr_k_d_model.py` | flow display→`:.2f` |

---

## Notes for Next Session
