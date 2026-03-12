# BlockSI Codebase Cleanup Plan

> Status: **AWAITING USER APPROVAL** — no files will be modified until this plan is approved.
> Created: 2026-03-11 by Claude Code (infrastructure audit session)

This plan identifies redundant files, legacy stubs, and unused components discovered during
a full codebase audit. Items are grouped by confidence level.

---

## Group A — High Confidence: Archive or Delete

These files are explicitly documented as removed from the build, superseded, or belong to
deprecated workflows.

### A1. ESP32 Firmware — Files Removed from Build

Documented in `esp32_agent_summary.md` as "removed from build (kept on disk for reference)":

| File | Reason | Proposed Action |
|------|--------|----------------|
| `Interfaces/ControlSystem/main/seq_power_cal.c` | Superseded by `seq_executor.c` | Archive to `Old/ESP32/` |
| `Interfaces/ControlSystem/main/seq_power_cal.h` | Header for above | Archive to `Old/ESP32/` |
| `Interfaces/ControlSystem/main/seq_airflow_val.c` | Superseded by `seq_executor.c` | Archive to `Old/ESP32/` |
| `Interfaces/ControlSystem/main/seq_airflow_val.h` | Header for above | Archive to `Old/ESP32/` |

### A2. ESP32 Firmware — Legacy Calibration (Unreferenced)

Documented in `esp32_agent_summary.md`: "`power_calibration_v2.c` is no longer referenced by any command handler (all `calibrate_*` commands route to seq_executor). Can be removed from `CMakeLists.txt` in a future cleanup."

| File | Reason | Proposed Action |
|------|--------|----------------|
| `Interfaces/ControlSystem/main/power_calibration_v2.c` | All calibrate commands now route to seq_executor | Archive to `Old/ESP32/` |

> **Note on `power_calibration.c`** (original, v1): This is a distinct file from v2. Its status is less clear — the CMakeLists.txt should be checked to confirm it is also unreferenced before archiving. Flagged as Group B below.

### A3. Legacy PC Dashboard Versions

| File | Reason | Proposed Action |
|------|--------|----------------|
| `Interfaces/PC/Old/blocksi_dashboard_v8.py` | Pre-v9 Streamlit version; NiceGUI migration complete | Delete |
| `Interfaces/PC/Old/blocksi_dashboard_pre_observer.py` | Pre-observer-pattern refactor (69KB); superseded by current dashboard | Delete |

### A4. Docker Artifacts (Root `Old/`)

Docker was removed from the project workflow (last modified Oct 2024).

| File | Reason | Proposed Action |
|------|--------|----------------|
| `Old/Dockerfile` | Docker workflow removed | Delete `Old/` directory entirely |
| `Old/docker-compose.yml` | Docker workflow removed | Delete with directory |

### A5. Data: Files Marked for Removal

| Path | Reason | Proposed Action |
|------|--------|----------------|
| `Interfaces/Data/Calibration/Remove/` (3 files) | User-marked for deletion (directory named `Remove/`) | Delete directory and contents |

### A6. SDK Config Backup

| File | Reason | Proposed Action |
|------|--------|----------------|
| `Interfaces/ControlSystem/sdkconfig.old` | Auto-generated backup from previous `menuconfig` run; no functional value | Delete |

---

## Group B — Review Needed: Verify Before Archiving

These files were not explicitly documented as removed/legacy but appear potentially unused
based on the audit. **Verify before acting.**

### B1. `power_calibration.c` (Original v1)

`esp32_agent_summary.md` only mentions removing `power_calibration_v2.c` from CMakeLists.
The original `power_calibration.c` (22KB, Jan 5) was not explicitly mentioned.

**Action needed**: Check `Interfaces/ControlSystem/main/CMakeLists.txt` — if it is not listed in `SRCS`, archive to `Old/ESP32/`. If listed, leave in place until confirmed removable.

### B2. `dac_power_test.c`

Not mentioned in any documentation or the `esp32_agent_summary.md` module list. Appears to be a one-off hardware test utility.

**Action needed**: Confirm it is not referenced in `CMakeLists.txt`. If unused, archive to `Old/ESP32/`.

### B3. `Interfaces/PC/blocksi_receiver.py`

A standalone TCP receiver script not mentioned in any documentation. Could be a development utility predating the integrated dashboard, or an active standalone tool.

**Action needed**: Open and review — if it duplicates functionality now in `blocksi_dashboard.py` and is not actively used, archive to `Interfaces/PC/Old/`.

### B4. `Interfaces/ControlSystem/main/data_cache.{c,h}`

`data_cache` is not listed in the `esp32_agent_summary.md` module table, but `backup_storage` is. These may be the same concept under a different name, or data_cache may be a newer addition not yet documented.

**Action needed**: Check whether `data_cache` is referenced in `CMakeLists.txt` and `main.c`. If active and undocumented, update `esp32_agent_summary.md`. If unused, archive.

### B5. `.github/agents/Test.agent.md`

A GitHub Copilot custom agent configuration file. Not referenced in any documentation.

**Action needed**: If this is an intentional Copilot agent config, document it in RULES.md. If it is a test artifact, delete it.

---

## Group C — Structural Issues: .gitignore / Build Artifacts

### C1. Build Directory Tracked in Git

`Interfaces/ControlSystem/build/` contains compiled ESP-IDF build artifacts and is currently tracked in git. This bloats the repository with binary files that should be regenerated from source.

**Proposed Action**:
1. Add `Interfaces/ControlSystem/build/` to `.gitignore`
2. Run `git rm -r --cached Interfaces/ControlSystem/build/` to stop tracking it

> ⚠ Confirm with user that the `build/` directory is NOT intentionally tracked (e.g., for pre-built binaries distribution). If it is, this item should be removed from the plan.

### C2. Orphaned `Data/Fill/` and `Data/Evac/` Directories

Per `dashboard_agent_summary.md` Session 13: "`Data/Fill/` + `Data/Evac/` → `Data/CSTR/`" (directory restructure completed). The old directories remain as empty `.gitkeep` placeholders.

| Path | Reason | Proposed Action |
|------|--------|----------------|
| `Interfaces/Data/Fill/` (`.gitkeep` only) | Replaced by `Data/CSTR/` | Remove directory + `.gitkeep` |
| `Interfaces/Data/Evac/` (`.gitkeep` only) | Replaced by `Data/CSTR/` | Remove directory + `.gitkeep` |

### C3. Legacy Data Archive

`Interfaces/Data/Old/` contains historical test data from Dec 2025 and Feb 2026 (50+ CSV/XLSX files). These are not needed for active development but may have scientific/reference value.

**Proposed Action**: Move to a dedicated external archive or confirm they can be deleted. Do NOT delete without explicit user approval — this is a data preservation question.

---

## Group D — Retain (Documented Reference Material)

These files appear outdated but serve a documented purpose and should be kept.

| File | Reason to Retain |
|------|-----------------|
| `Interfaces/ControlSystem/sdkconfig.defaults` | Default SDK configuration for reproducible builds |
| `Interfaces/ControlSystem/sdkconfig.defaults.template` | Template for onboarding — retain unless confirmed unused |
| `Interfaces/ControlSystem/main/main_backup_handlers.c` | Possibly split from `main.c`; needs verification before touching |
| `Resources/*.pdf` | Equipment manuals (106-H, MP-8000) — always retain |
| `docs/cases/cstr_power_stability.md` | Case study for active debugging reference |

---

## Proposed Archive Structure

If the user prefers archiving over deleting:

```
Old/
  ESP32/
    seq_power_cal.c/.h          (from Group A1)
    seq_airflow_val.c/.h        (from Group A1)
    power_calibration_v2.c     (from Group A2)
    power_calibration.c        (from Group B1, if confirmed unused)
    dac_power_test.c           (from Group B2, if confirmed unused)
  Dashboard/
    blocksi_dashboard_v8.py    (already in Interfaces/PC/Old/)
    blocksi_dashboard_pre_observer.py  (already in Interfaces/PC/Old/)
```

---

## Items NOT in this Plan (Out of Scope)

- Any file under `Interfaces/PC/analysis/` — active modules
- Any `.json` under `Interfaces/Models/` — active model data
- All files in `Interfaces/Data/Telemetry/`, `Calibration/`, `Validation/`, `CSTR/` — operational data
- `Resources/` PDF manuals
- `docs/` documentation

---

## Summary Table

| Group | Items | Disposition |
|-------|-------|-------------|
| A — High confidence | 11 files + 1 directory | Archive or delete (no code impact) |
| B — Review needed | 5 items | Verify CMakeLists / usage first |
| C — Structural | 3 issues | `.gitignore` fix + empty dir removal |
| D — Retain | 6 items | Keep as-is |

**No files will be modified until this plan is reviewed and approved by the user.**
