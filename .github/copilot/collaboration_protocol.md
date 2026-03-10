# BlockSI Multi-Agent Collaboration Protocol

> Last updated: 2026-03-10

## Overview

BlockSI is developed by **two concurrent AI agents** plus the human developer:

| Agent | Environment | Branch | Push target |
|-------|-------------|--------|-------------|
| **Copilot** (VS Code) | Local Windows machine | `main` | pushes directly to `main` |
| **Claude Code** (web) | Remote Linux container | `claude/<id>` | pushes to feature branch only |

The human developer is the **integration point**: after Claude Code pushes a feature branch, the developer merges it into `main` locally before Copilot can see the changes.

**The collaboration docs are the coordination mechanism.** Both agents must read `.github/copilot/` at session start to stay in sync regardless of branch differences.

## Branch Workflow

### Copilot (VS Code)
- Works directly on `main`
- Commits and pushes to `main` per the end-of-session procedure below
- Always `git pull` before starting a session to pick up Claude Code's merged work

### Claude Code (web)
- Constrained by task runner to push to a `claude/<session-id>` branch (e.g. `claude/review-project-docs-DvDeR`)
- **Never** pushes to `main` directly
- After Claude Code pushes, the human merges the branch into `main`:

```powershell
git fetch origin
git merge origin/claude/<branch-name>   # or via GitHub PR
git push origin main
```

### After Merging Claude Code's Work
1. Human merges the `claude/` branch into `main` and pushes
2. Human tells Copilot: *"Pull main — Claude Code merged new changes."*
3. Copilot runs `git pull` and reads the updated collaboration docs

## Active Agents

Both agents are **full-stack** — either may modify any file in the repo.
The summaries and interface contract are the shared source of truth.

## Collaboration Documents (`.github/copilot/`)

| File | Purpose | Written by |
|------|---------|------------|
| `collaboration_protocol.md` | This file — rules and conventions | Human + agents |
| `interface_contract.md` | LAN protocol, DATA format, shared constants | Either agent (must coordinate) |
| `dashboard_agent_summary.md` | Dashboard/PC state and pending work | Either agent |
| `esp32_agent_summary.md` | ESP32 firmware state and pending work | Either agent |
| `decisions_log.md` | Architectural decisions with date + rationale | Either agent |
| `pitfalls.md` | Recurring bugs and NiceGUI/asyncio gotchas | Either agent |

## Status Taxonomy

Every item in summaries and the decisions log MUST use one of these tags:

- **`[IMPLEMENTED]`** — Done, in code, tested or verified working
- **`[DECIDED]`** — Agreed upon but not yet built
- **`[PROPOSED]`** — Open idea, not committed — requires discussion before building

## Summary Update Convention

1. Every agent appends `> Last updated: YYYY-MM-DD` at the top of its summary after substantive implementation work.
2. When the user says "update your summary", the agent **rewrites** it with current-state information, discarding session-specific history.
3. Summaries should be concise enough that another agent can read them in under 2 minutes of context.

## Interface Change Rule (CRITICAL)

If either agent changes anything that crosses the ESP32 ↔ PC boundary:

1. **Update `interface_contract.md` FIRST** with the new/changed definition.
2. Tag the change as `[IMPLEMENTED]` or `[DECIDED]`.
3. Note the change in the relevant agent summary under a "Recent Changes" section.

Changes that require this:
- Adding, removing, or modifying a LAN command
- Changing the DATA telemetry format
- Changing the TCP connection model (port, who connects to whom)
- Adding a new message type (e.g., `SEQ,...` status lines)
- Changing shared constants (flow rates, model coefficients)

## Context Window Management

- The human should prompt "update your summary" when the VS Code Copilot context window approaches ~85%.
- Claude Code sessions start fresh each time — the summaries are its only persistent memory.

## Decision Logging

When an architectural decision is made:
1. The agent appends it to `decisions_log.md` immediately.
2. Format: `### YYYY-MM-DD: <title>` followed by Context, Decision, Rationale, Status tag.
3. Decisions are **immutable** once logged — supersede with a new entry, never silently edit.

## New Chat Startup Procedure

When starting a new agent session:
1. Read `collaboration_protocol.md` (this file) — understand the branch model
2. Read `interface_contract.md` — shared protocol definitions
3. Read `dashboard_agent_summary.md` — current PC/dashboard state
4. Read `esp32_agent_summary.md` — current ESP32 state
5. Read `decisions_log.md` (recent entries) — architectural context
6. Skim `pitfalls.md` — known recurring bugs and framework gotchas to avoid

## End-of-Session Procedure (MANDATORY)

### Copilot (VS Code) — pushes to `main`

```powershell
cd c:\Users\ldhar\Documents\Shroom-E_Co\BlockSI
git status --short
git add -A
git commit -m "Dashboard: <summary>"   # or ESP32:, docs:
git push
```

### Claude Code (web) — pushes to `claude/` branch

Claude Code commits and pushes to its assigned feature branch. The human then merges into `main` (see Branch Workflow above).

**Commit message conventions (both agents):**
- `ESP32:` — firmware changes
- `Dashboard:` — PC dashboard changes
- `docs:` — collaboration docs only
- Split commits if changes span multiple concerns

**Before staging, verify:**
- No build artifacts (`build/`, `__pycache__/`, `*.pyc`)
- No credentials (`sdkconfig` with WiFi/PSK)
- `.gitignore` covers all generated/sensitive files
