# BlockSI Multi-Agent Collaboration Protocol

> Last updated: 2026-02-25

## Overview

BlockSI development uses **domain-separated AI coding agents**, each working
in its own chat session with focused context.  This document defines how
those agents coordinate.

## Active Agents

| Agent | Domain | Owns |
|-------|--------|------|
| **Dashboard Agent** | `Interfaces/PC/` | `blocksi_dashboard.py`, PC-side calibration logic, NiceGUI UI |
| **ESP32 Agent** | `Interfaces/ControlSystem/` | All firmware `.c/.h` files, `CMakeLists.txt`, `sdkconfig` |

Either agent may **read** any file in the repo, but must only **write** to
files in its own domain, plus the shared collaboration docs listed below.

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
