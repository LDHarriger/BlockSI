# MEMORY_CLAUDE.md — Claude_VSCode Agent Working Memory

> Agent: Claude_VSCode (Claude Code VS Code extension)
> Last updated: 2026-03-15

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

- **Commit** — context engineering audit changes ready to stage and commit; user should review

---

<history_archive>

### 2026-03-15: Context Engineering Audit

**Work done:**
- Reviewed Anthropic docs (prompting best practices, context engineering, memory tool, memory cookbook)
- Compared current implementation against best practices, identified 8 gaps
- Created `docs/user/best_practices.md` — condensed resource summary
- Created `docs/user/managing_agents.md` — human operator guide
- Created `docs/user/audit_plan.md` → implemented → archived to `docs/archive/`
- Archived stale docs: `collaboration_protocol.md`, `claude_code_agent.md` → `docs/archive/`
- Extracted reference material from RULES.md → `docs/reference/hardware.md`, `models.md`, `sequences.md`
- Rewrote RULES.md: tiered startup, XML behavioral blocks, agent identities, motivation for rules, memory hygiene
- Trimmed `dashboard_agent_summary.md` (362→~150 lines), archived session history
- Rewrote `CLAUDE.md` and `.github/copilot-instructions.md` as agent-specific entry points

**Key decisions:**
- Keep project-level MEMORY files (not Anthropic API memory tool — different purpose)
- Markdown format with XML-tagged instruction blocks (not pure XML)
- Agent identities: Claude_VSCode, Copilot_VSCode, Claude_Cloud [FALLBACK], Claude_CLI (defer)
- XML blocks added: do_not_act_before_instructions, verify_before_finishing, reflect_on_results, subagent_guidance, memory_protocol, memory_security, context_awareness
- Copilot_VSCode gets additional snippets (parallel_tool_calls, reversibility) not needed in Claude Code

### 2026-03-11: Infrastructure Setup Session

**Work done:**
- Created `RULES.md`, `MEMORY_CLAUDE.md`, `MEMORY_COPILOT.md`, `CLAUDE.md`
- Updated `.github/copilot-instructions.md` as pointer to `RULES.md`

**Key decisions:**
- Both local agents push directly to `main`
- Claude.ai cloud agent = legacy/fallback only
- Locking protocol: `<FILE_LOCK path="..."/>` in memory file

</history_archive>
