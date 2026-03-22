# MEMORY_COPILOT.md — Copilot Chat Agent Working Memory

> Agent: GitHub Copilot Chat (local VS Code plugin)
> Last updated: 2026-03-22

---

## Active Locks

*(none)*

---

## Active Tasks

*(none)*

---

## Session Context

- Current branch: `main`
- Dashboard: `Interfaces/PC/blocksi_dashboard.py` (entry point; modules in `dashboard/`)
- Python venv: `.venv\Scripts\python.exe`
- NiceGUI 3.8.0, Python 3.14

### Recently Completed (2026-03-22 — process_batch implementation)

| WP | Description | Status |
|----|-------------|--------|
| WP-0 | WiFi Disconnect Audit | ✅ Complete (audit + stale ref fix) |
| WP-1 | k_d Calibration Rewrite | ✅ Complete (prior session) |
| WP-2 | k_abs Model + Sequence | ✅ Complete (model + sequence + wiring) |
| WP-3 | Dosimetry Solver | ✅ Complete (solver + accumulator + config loader) |
| WP-4 | Batch Sequence | ✅ `batch_sequence.py` created — recipe protocol + dosimetry |
| WP-5 | Processing Tab UI | ✅ Full form, solver preview, live monitoring, charts |
| WP-6 | Calibration Enforcement | ✅ `data_io.py` functions + UI enforcement in click handlers |
| WP-7 | Flow Rate UI Overhaul | ✅ Rotameter prompt + calibrated rate dropdowns |
| WP-8 | Tab Restructuring | ✅ 7 tabs: Control/Calibration/Processing/Validation/Telemetry/Debug/Settings |
| WP-9 | Documentation | ✅ operating_procedures.md + TODO_checklist.md |

### Pending Feature Work

| Item | Status |
|------|--------|
| Historical data viewer (load & plot old CSVs) | `[PROPOSED]` |
| Migrate power curve from Plotly to ECharts | `[PROPOSED]` |
| Firmware changes from WP-0 audit | See `docs/user/TODO_checklist.md` §5 |

---

## Notes for Next Session

- Batch sequence uses ESP32 recipe protocol (sequence_start → seq_step → seq_run) for disconnect resilience
- `S.batch_samples`, `S.batch_dose_running`, `S.batch_dose_target`, `S.batch_schedule` added to SystemState
- tcp_server.py: `process_batch` type routes SEQ SAMPLE messages to `S.batch_samples`
- Prompt content for batch phases (vessel_cool, add_inoculant, distribute) added to `PROMPT_CONTENT`
- `BATCH_DATA_DIR = Data/Batch/` added to state.py with auto-mkdir
- `_notify_queue` pattern is in place — never call `ui.*` directly from background tasks
- Pre-flight checks that need UI dialogs MUST be in click handlers (slot context), not background tasks

---

<history_archive>

*(Previously: CSTR fill stopping criteria, validation target O3 from PASS cert, power watchdog in fill loop)*

</history_archive>
