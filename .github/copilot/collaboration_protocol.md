# BlockSI Multi-Agent Collaboration Protocol

> Last updated: 2026-03-04

## Overview

BlockSI development previously used **domain-separated AI coding agents**
(one for ESP32, one for the PC dashboard).  As of 2026-03-04, this has been
**changed to a single-agent model** — one GitHub Copilot agent handles both
the ESP32 firmware and the PC dashboard within the same chat session.

**Rationale**: When a feature spans the ESP32↔PC boundary (e.g. adding a new
calibration phase, changing a command argument format), the dual-agent model
required writing the interface contract, waiting for the other agent to read
it, and coordinating through documentation handoffs.  This introduced lag,
context drift, and subtle regressions.  A single agent with full-stack context
builds both sides of a feature simultaneously, keeping the interface contract
always in sync.

**The collaboration docs remain active**: They serve as persistent memory
across context windows (summaries), architectural reference (interface contract),
and decision history (decisions log) — all valuable regardless of how many
agents are active.

## Active Agent

| Agent | Domain | Owns |
|-------|--------|------|
| **BlockSI Agent** | Full-stack | All firmware `.c/.h` files, `Interfaces/PC/blocksi_dashboard.py`, collaboration docs |

> **Previous model** (before 2026-03-04): Two domain-separated agents —
> "Dashboard Agent" (Interfaces/PC/) and "ESP32 Agent" (Interfaces/ControlSystem/).
> See decisions_log.md entry 2026-03-04 for rationale.

Either session may **read and write** any file in the repo.  Both summaries
(`dashboard_agent_summary.md` and `esp32_agent_summary.md`) are still maintained
for continuity — they cover their respective domains.

## Collaboration Documents (`.github/copilot/`)

| File | Purpose | Written by |
|------|---------|------------|
| `collaboration_protocol.md` | This file -- rules and conventions | Human |
| `interface_contract.md` | LAN protocol, DATA format, shared constants | Either agent (must coordinate) |
| `dashboard_agent_summary.md` | Dashboard agent's current state and pending work | Dashboard Agent |
| `esp32_agent_summary.md` | ESP32 agent's current state and pending work | ESP32 Agent |
| `decisions_log.md` | Architectural decisions with date + rationale | Either agent |

## Status Taxonomy

Every item in summaries and the decisions log MUST use one of these tags:

- **`[IMPLEMENTED]`** -- Done, in code, tested or verified working
- **`[DECIDED]`** -- Agreed upon but not yet built
- **`[PROPOSED]`** -- Open idea, not committed -- requires discussion before building

## Summary Update Convention

1. Every agent appends `> Last updated: YYYY-MM-DD HH:MM` at the top of
   its summary after substantive implementation work.
2. When the user tells an agent "update your summary" (or the agent
   recognises the context window is getting large), the agent **rewrites**
   its summary with current-state information and discards session-specific
   history.
3. Summaries should be concise enough that another agent can read them in
   under 2 minutes of context.

## Interface Change Rule (CRITICAL)

If either agent changes anything that crosses the ESP32 <-> PC boundary:

1. **Update `interface_contract.md` FIRST** with the new/changed definition.
2. Tag the change as `[IMPLEMENTED]` or `[DECIDED]` as appropriate.
3. Note the change in the agent's own summary under a "Recent Changes" section.
4. The other agent will pick up the change when it next reads `interface_contract.md`.

Changes that require this:
- Adding, removing, or modifying a LAN command
- Changing the DATA telemetry format
- Changing the TCP connection model (port, who connects to whom)
- Adding a new message type (e.g., `SEQ,...` status lines)
- Changing shared constants (flow rates, model coefficients)

## Context Window Management

- The AI agents **cannot** see their own context window utilisation.
- The human should prompt "update your summary" when the context window
  approaches ~85% (visible in the VS Code Copilot UI).
- When prompted, the agent rewrites its summary with all current-state info,
  ensuring a new chat starting from that summary has full continuity.

## Decision Logging

When an architectural decision is made during a chat:
1. The agent appends it to `decisions_log.md` immediately.
2. Format: `### YYYY-MM-DD: <title>` followed by Context, Decision, Rationale.
3. Decisions are **immutable** once logged -- they can be superseded by a
   new decision entry but never silently edited.

## New Chat Startup Procedure

When starting a new agent chat session:
1. Ask the agent to read `copilot-instructions.md` (project-level context).
2. Ask it to read `collaboration_protocol.md` (this file).
3. Ask it to read `interface_contract.md` (shared protocol definitions).
4. Ask it to read its own summary (e.g., `dashboard_agent_summary.md`).
5. Optionally, ask it to read the other agent's summary if cross-domain
   awareness is needed.

## End-of-Session Procedure (MANDATORY)

Every agent MUST perform these steps at the end of a development session,
or when prompted by the user:

### 1. Update your agent summary
Rewrite your summary file with current state, including any work done in
this session.  Add/update the "Recent Changes" section.

### 2. Commit and push all changes
Run the following git commands:

```powershell
cd c:\Users\ldhar\Documents\Shroom-E_Co\BlockSI
git status --short                    # Review changes
git add -A                            # Stage all changes
git commit -m "<Agent>: <summary>"    # Commit with agent prefix
git push                              # Push to remote
```

**Commit message conventions:**
- Prefix with agent domain: `ESP32:`, `Dashboard:`, or `docs:`
- If changes span multiple concerns, split into multiple commits:
  1. `chore:` — gitignore, build config, tooling
  2. `ESP32:` or `Dashboard:` — domain-specific code changes
  3. `docs:` — collaboration docs, interface contract, decisions log
- If committing another agent's uncommitted work (e.g., found in working
  tree), note it: `"Dashboard: ... (committed by ESP32 agent on behalf of Dashboard session)"`

**Before staging, verify:**
- No build artifacts (`build/`, `__pycache__/`, `*.pyc`) are staged
- No credentials (`sdkconfig` with WiFi/PSK) are staged
- `.gitignore` covers all generated/sensitive files

**If push fails** (e.g., remote has diverged):
```powershell
git pull --rebase       # Rebase local commits on top of remote
git push                # Try again
```
If there are merge conflicts, resolve them and inform the user.

### 3. Confirm to the user
Report: number of commits, what was pushed, and any issues encountered.
