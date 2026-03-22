# MEMORY_CLAUDE.md — Claude_VSCode Agent Working Memory

> Agent: Claude_VSCode (Claude Code VS Code extension)
> Last updated: 2026-03-22

---

## Active Locks

*(releasing all locks — handing off to Copilot)*

---

## Active Tasks

### 2026-03-22: process_batch Implementation Plan Execution (IN PROGRESS — HANDOFF)

**Plan source**: `~/Downloads/DRAFT_process_batch_plan.md` (FINAL, approved)
**Handoff doc**: `docs/cases/process_batch_handoff.md`

**Work packages status**:
- [x] **WP-1**: k_d Calibration Rewrite — DONE (renames, model rewrite, sequence rewrite)
- [x] **WP-6**: Calibration Enforcement — DONE (backend functions in data_io.py)
- [~] **WP-0**: WiFi Disconnect Audit — agent created `docs/cases/wifi_disconnect_audit.md`, NEEDS VERIFICATION
- [~] **WP-2**: k_abs model file exists (`cstr_k_abs_model.py`), sequence file (`k_abs_cal.py`) NOT created
- [~] **WP-3**: Dosimetry solver file exists (`dosimetry.py`), NEEDS REVIEW
- [~] **WP-7**: Backend done (`list_calibrated_flow_rates()`), UI changes NOT done
- [~] **WP-8**: Tab restructuring agent launched, CHECK if ui_main.py was modified
- [~] **§14**: Validation update agent launched, CHECK if validation.py was modified
- [ ] **WP-4**: process_batch Sequence — NOT STARTED (blocked by WP-0)
- [ ] **WP-5**: Processing Tab UI — NOT STARTED (blocked by WP-4)
- [ ] **WP-9**: User Documentation — NOT STARTED

**UNCOMMITTED CHANGES**: All work is staged/unstaged, nothing committed yet.

---

## Session Context

- Current branch: `main`
- Dashboard: `Interfaces/PC/blocksi_dashboard.py` (~3350 lines, NiceGUI)
- Python venv: `.venv\Scripts\python.exe`

---

## Pending (Awaiting User Input)

- **Review + Commit** — large changeset from process_batch implementation needs review before commit
- **Copilot handoff** — see `docs/cases/process_batch_handoff.md` for detailed status

---

<history_archive>

### 2026-03-22: process_batch Plan Execution (partial)

**Work done (Claude_VSCode)**:
- WP-1: Renamed fill_model.py → cstr_k_d_model.py, cstr_sequence.py → k_d_cal.py, Data/CSTR → Data/k_d_cal, Models/CSTR → Models/cstr_k_d. Updated all imports. Rewrote model fitting (fixed V=9.27, V_dead=0.020, fit only k_d, multi-file support, timestamped JSON output). Rewrote sequence (computed fill duration, 5-min flush evac, dynamic dt).
- WP-6: Added `_find_valid_calibration()`, `_find_valid_calibration_model()`, `list_calibrated_flow_rates()` to data_io.py.
- Created `dashboard/substrate_config.json` with experimental presets and thresholds.
- Launched background agents for WP-0, WP-2, WP-3, WP-8/§14 — results in codebase but unreviewed.

### 2026-03-15: Context Engineering Audit

**Work done:**
- Rewrote RULES.md with XML behavioral blocks, tiered startup, agent identities
- Created docs/reference/{hardware,models,sequences}.md
- Created docs/user/{best_practices,managing_agents}.md
- Trimmed dashboard_agent_summary.md, archived session history

### 2026-03-11: Infrastructure Setup Session

**Work done:**
- Created RULES.md, MEMORY_CLAUDE.md, MEMORY_COPILOT.md, CLAUDE.md

</history_archive>
