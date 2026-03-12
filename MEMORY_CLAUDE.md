# MEMORY_CLAUDE.md — Claude Code Agent Working Memory

> Agent: Claude Code (local VS Code CLI)
> Last updated: 2026-03-11

---

## Active Locks

*(none)*

---

## Active Tasks

*(none)*

---

## Session Context

- Current branch: `main`
- Dashboard: `Interfaces/PC/blocksi_dashboard.py` (~3350 lines, NiceGUI)
- Python venv: `.venv\Scripts\python.exe`

---

## Pending (Awaiting User Input)

- **Commit** — infrastructure + cleanup changes ready to stage and commit; user should review and commit when ready

---

<history_archive>

### 2026-03-11: Infrastructure Setup Session

**Work done:**
- Read all `docs/` docs and `copilot-instructions.md`
- Performed full codebase audit (file inventory, legacy file identification)
- Created `RULES.md` — unified project rules, revised Collaboration section for local-first workflow
- Created `MEMORY_CLAUDE.md` (this file) and `MEMORY_COPILOT.md`
- Created `CLAUDE.md` as single pointer to `RULES.md`
- Updated `.github/copilot-instructions.md` as single pointer to `RULES.md`
- Created `CLEANUP_PLAN.md` with proposed cleanup actions (awaiting user approval)

**Key decisions made:**
- Both local agents (Claude Code + Copilot Chat) push directly to `main`
- Claude.ai cloud agent documented as legacy/fallback only (push to `claude/*` branches)
- Locking protocol: `<FILE_LOCK path="..."/>` in memory file, checked before editing shared files
- Preservation policy: completed tasks go to `<history_archive>`, never deleted

</history_archive>
