# MEMORY_COPILOT.md — Copilot Chat Agent Working Memory

> Agent: GitHub Copilot Chat (local VS Code plugin)
> Last updated: 2026-03-11

---

## Active Locks

*(none)*

---

## Active Tasks

*(none — see `CLEANUP_PLAN.md` awaiting user approval for codebase cleanup)*

---

## Session Context

- Current branch: `main`
- Dashboard: `Interfaces/PC/blocksi_dashboard.py` (~3350 lines, NiceGUI)
- Python venv: `.venv\Scripts\python.exe`

### Pending Feature Work (from `dashboard_agent_summary.md`)

| Item | Status |
|------|--------|
| Historical data viewer (load & plot old CSVs) | `[PROPOSED]` |
| Sterilization batch sequence (Fill→Hold→Evac) | `[PROPOSED]` |
| Migrate power curve from Plotly to ECharts | `[PROPOSED]` |

---

## Notes for Next Session

- CSTR fill stopping criteria updated in Session 15: slope < 0.0003 %vol/sample AND range < 0.08 over 45 samples
- Validation target O3 now uses direct measurement from 100% PASS cert instead of sigmoid prediction
- Power watchdog active in CSTR fill loop: if `power_actual_pct` < 80%, auto-resends `power_set(100)` up to 3×
- `_notify_queue` pattern is in place — never call `ui.*` directly from background tasks

---

<history_archive>

*(no archived sessions yet — file initialized 2026-03-11 by Claude Code during infrastructure setup)*

</history_archive>
