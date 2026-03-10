# Claude Code Agent — Coordination Instructions

> For: Claude Code (cloud agent) working on feature branches
> Updated: 2026-03-10

## You Are One of Multiple Agents

This project has **two active AI agents** working simultaneously:

1. **VS Code Copilot Agent** — runs locally in VS Code, works on `main` branch,
   handles interactive development (UI debugging, live testing with ESP32 hardware)
2. **Claude Code Agent (you)** — runs on cloud, works on **feature branches**,
   handles analytical tasks (model refactoring, algorithm design, documentation)

**Both agents push to the same GitHub repo.** Merge conflicts are inevitable
if you aren't careful.

## CRITICAL: Do Not Revert Existing Changes

**Before making any edits, read the current state of files on `main`.**
Your feature branch is based on a snapshot of `main` at branch creation time.
If `main` has received commits since then, your branch may be stale.

**Before pushing, always:**
```bash
git fetch origin
git rebase origin/main    # Rebase onto latest main
# Resolve any conflicts, preserving BOTH your changes AND main's changes
git push --force-with-lease origin <your-branch>
```

**The #1 rule: Never silently delete or overwrite code you didn't write.**
If `main` has changes you don't recognise, they were made by the other agent
and must be preserved. When in doubt, keep both.

### What Went Wrong (2026-03-10)

Your CSTR model commit (`4f00699`) deleted these Session 13 fixes from `main`:
- Backfill-active stuck fix (reset on connect/disconnect + 30s timeout)
- Plotly chart restyle optimization (fixed 4-trace layout + `_restyle_markers()`)
- Compact sidebar cards (5 individual cards → single dense card)
- CI band opacity increase (0.15 → 0.25)

These were **critical bug fixes** for live sensor display. They had to be
manually re-merged by the VS Code agent.

## Branching & Merge Protocol

1. **Always work on a feature branch** (never push directly to `main`)
2. **Before starting work**, rebase your branch onto latest `origin/main`
3. **Before pushing**, rebase again onto latest `origin/main`
4. **Merge strategy**: The VS Code agent (or human) will merge your branch
   into `main` after review. Do not merge yourself.
5. **Conflict resolution**: When rebasing, preserve ALL existing code from
   `main`. Add your changes on top, don't replace.

## Documentation Updates

This project uses structured documentation in `.github/copilot/`:

| File | Purpose | Rules |
|------|---------|-------|
| `decisions_log.md` | Architectural decisions | **Append-only.** Never delete or modify existing entries. Add new entries at the top. |
| `dashboard_agent_summary.md` | Dashboard state | **Additive.** Add your session's changes to the "What Changed" section. Don't rewrite sections written by other sessions. |
| `esp32_agent_summary.md` | ESP32 firmware state | Same as above. |
| `interface_contract.md` | LAN protocol spec | Update if you change any cross-boundary protocol. |
| `collaboration_protocol.md` | Collaboration rules | Read-only for agents. |

### decisions_log.md Rules
- Entries are **immutable** — never edit or delete an existing entry
- To supersede a decision, add a NEW entry that references/supersedes the old one
- Always add new entries at the TOP (after the header), before existing entries
- Format: `### YYYY-MM-DD: <title>`, then Context, Decision, Rationale, Status

### dashboard_agent_summary.md Rules
- Update the `> Last updated:` line with your session info
- Add your changes to the current session's "What Changed" section
- Do NOT delete or rewrite other agents' change descriptions
- If starting a new session number, move old session content down

## What to Read Before Starting Work

1. `copilot-instructions.md` — project overview, architecture, conventions
2. `collaboration_protocol.md` — collaboration rules
3. `interface_contract.md` — LAN protocol definitions
4. `dashboard_agent_summary.md` — current dashboard state
5. `esp32_agent_summary.md` — current firmware state
6. `decisions_log.md` — architectural decisions history
7. **This file** — your specific coordination rules

## Communication Channel

There is no real-time communication between agents. Coordination happens through:
- **Git history** — always read recent commits on `main` before starting
- **Documentation files** — the shared state lives in `.github/copilot/`
- **The human** — the user relays information between agents when needed

## Code Ownership

Both agents can read and write any file, but be aware:
- The VS Code agent handles **live debugging** with hardware — their fixes to
  UI rendering, TCP handling, and sensor display are battle-tested against real
  ESP32 connections
- Your changes to **models and algorithms** are your strength — focus there
- When your changes touch UI/TCP code, be extra careful not to break existing
  fixes

## Commit Message Convention

```
<scope>: <summary>

<body>
```

Scopes: `Dashboard:`, `ESP32:`, `docs:`, `analysis:`, `chore:`
