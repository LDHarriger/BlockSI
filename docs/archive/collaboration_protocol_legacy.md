# BlockSI Multi-Agent Collaboration Protocol (LEGACY)

> **Superseded by `RULES.md` as of 2026-03-11.**
> Retained for historical reference only. Do not follow these instructions — they describe a deprecated workflow.
> The current agent roster, branch model, and collaboration rules are defined in `RULES.md`.

---

*Original content preserved below for reference.*

---

> Last updated: 2026-03-10

## Overview

BlockSI development uses **two AI coding agents** working in parallel:

1. **VS Code Copilot Agent** — runs locally, pushes directly to `main` branch
2. **Claude Code Agent** — runs on cloud, pushes to `claude/*` feature branches

**Workflow**: Claude Code pushes to `claude/*` feature branches. VS Code Copilot
fetches, merges, and pushes those branches into `main` on your behalf. See
`claude_code_agent.md` for the cloud agent's specific coordination rules.

**The collaboration docs remain active**: They serve as persistent memory
across context windows (summaries), architectural reference (interface contract),
and decision history (decisions log) — all valuable regardless of how many
agents are active.

## Active Agents

| Agent | Platform | Branch | Merge Authority |
|-------|----------|--------|------------------|
| **VS Code Copilot** | Local VS Code | `main` | Self — direct push + merges `claude/*` |
| **Claude Code** | Cloud | `claude/*` | VS Code Copilot (merges on user request) |

> **Previous model** (before 2026-03-04): Two domain-separated agents —
> "Dashboard Agent" (Interfaces/PC/) and "ESP32 Agent" (Interfaces/ControlSystem/).
> See decisions_log.md entry 2026-03-04 for rationale.

## Collaboration Documents (`docs/`)

| File | Purpose | Written by |
|------|---------|------------|
| `collaboration_protocol.md` | This file -- rules and conventions | Human |
| `claude_code_agent.md` | Coordination rules for the cloud agent | VS Code Agent / Human |
| `interface_contract.md` | LAN protocol, DATA format, shared constants | Either agent (must coordinate) |
| `dashboard_agent_summary.md` | Dashboard agent's current state and pending work | Either agent |
| `esp32_agent_summary.md` | ESP32 agent's current state and pending work | Either agent |
| `decisions_log.md` | Architectural decisions with date + rationale | Either agent |
| `pitfalls.md` | Recurring bugs and NiceGUI/asyncio gotchas | Either agent |

## Status Taxonomy

- **`[IMPLEMENTED]`** -- Done, in code, tested or verified working
- **`[DECIDED]`** -- Agreed upon but not yet built
- **`[PROPOSED]`** -- Open idea, not committed -- requires discussion before building

## Summary Update Convention

1. Every agent appends `> Last updated: YYYY-MM-DD HH:MM` at the top of its summary after substantive implementation work.
2. When the user tells an agent "update your summary" (or the agent recognises the context window is getting large), the agent **rewrites** its summary with current-state information and discards session-specific history.
3. Summaries should be concise enough that another agent can read them in under 2 minutes of context.

## Interface Change Rule (CRITICAL)

If either agent changes anything that crosses the ESP32 <-> PC boundary:

1. **Update `interface_contract.md` FIRST** with the new/changed definition.
2. Tag the change as `[IMPLEMENTED]` or `[DECIDED]` as appropriate.
3. Note the change in the agent's own summary under a "Recent Changes" section.
4. The other agent will pick up the change when it next reads `interface_contract.md`.

## Context Window Management

- The AI agents **cannot** see their own context window utilisation.
- The human should prompt "update your summary" when the context window approaches ~85%.
- When prompted, the agent rewrites its summary with all current-state info.

## Decision Logging

1. The agent appends it to `decisions_log.md` immediately.
2. Format: `### YYYY-MM-DD: <title>` followed by Context, Decision, Rationale.
3. Decisions are **immutable** once logged.

## New Chat Startup Procedure

1. Read `copilot-instructions.md`
2. Read `collaboration_protocol.md` (this file)
3. Read `interface_contract.md`
4. Read `dashboard_agent_summary.md`
5. Read `esp32_agent_summary.md`
6. Read `decisions_log.md` (recent entries)
7. Skim `pitfalls.md`

## End-of-Session Procedure

1. Update your agent summary
2. Commit and push all changes
3. Confirm to the user
