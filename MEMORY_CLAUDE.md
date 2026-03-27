# MEMORY_CLAUDE.md — Claude_VSCode Agent Working Memory

> Agent: Claude_VSCode (Claude Code VS Code extension)
> Last updated: 2026-03-24

---

## Active Locks

*(releasing all locks — handing off to Copilot)*

---

## Active Tasks

### 2026-03-24: Testing Feedback & Validation Redesign Task Package

**Handoff doc**: `docs/cases/task_package_handoff.md`

**Task status** (priority order):

- [~] **Task 2**: Power Mismatch Audit
  - [x] Firmware code review (all relevant files read)
  - [x] Data characterization (3 calibration CSVs analyzed)
  - [x] Audit report written: `docs/cases/power_mismatch_audit.md`
  - [ ] Firmware diagnostic commands (NOT IMPLEMENTED — see handoff for spec)
  - [ ] Dashboard DIAG logging for diagnostics
- [ ] **Task 1**: Measurement Verification — NOT STARTED
- [ ] **Task 3**: Rename Data/Calibration/ → Data/Power-O3_cal/ — NOT STARTED
- [ ] **Task 4**: GUI improvements — NOT STARTED (blocked by Task 1)
- [x] **Task 5**: Resource documentation audit — DONE (`docs/documentation_gaps.md`)

### Bug Fix: TCP Singleton (DONE, UNCOMMITTED)

- `tcp_server.py` line 486: `None` → `TCPServer()` (eager instantiation)
- `blocksi_dashboard.py`: startup uses existing singleton instead of creating new

---

## Session Context

- Current branch: `main`
- Dashboard modularized: entry point is `blocksi_dashboard.py` (~40 lines), modules in `dashboard/`
- Python venv: `.venv\Scripts\python.exe`
- Copilot completed process_batch WP-0 through WP-9 (commits `7e52d12`, `164d594`)

---

## Pending (Awaiting User Input)

- **Commit**: TCP singleton fix + power mismatch audit report
- **Copilot handoff**: `docs/cases/task_package_handoff.md` has full details for remaining work

---

<history_archive>

### 2026-03-24: Testing Feedback & Validation Redesign (partial)

**Work done (Claude_VSCode)**:
- Fixed TCP singleton bug preventing ESP32↔Dashboard connection (modularization regression from `0d5f7d9`)
- Task 2 partial: Read all firmware (motor_pot, o3_power_control, blocksi_state, main.c command handler). Analyzed 3 calibration CSVs. Found systematic under-delivery at mid-high power, bimodal ADC noise in recent sessions, progressive degradation. Wrote audit report with 3 hypotheses (coast drift, ADC noise, hardware degradation). Specified diagnostic commands but did not implement in firmware.

### 2026-03-22: process_batch Plan Execution (partial)

**Work done (Claude_VSCode)**:
- WP-1: Renamed fill_model.py → cstr_k_d_model.py, cstr_sequence.py → k_d_cal.py, Data/CSTR → Data/k_d_cal, Models/CSTR → Models/cstr_k_d. Updated all imports. Rewrote model fitting and sequence.
- WP-6: Added calibration enforcement functions to data_io.py.
- Created `dashboard/substrate_config.json` with experimental presets.
- Launched background agents for WP-0, WP-2, WP-3, WP-8/§14.

### 2026-03-15: Context Engineering Audit

- Rewrote RULES.md with XML behavioral blocks, tiered startup, agent identities
- Created docs/reference/{hardware,models,sequences}.md
- Created docs/user/{best_practices,managing_agents}.md

### 2026-03-11: Infrastructure Setup Session

- Created RULES.md, MEMORY_CLAUDE.md, MEMORY_COPILOT.md, CLAUDE.md

</history_archive>
