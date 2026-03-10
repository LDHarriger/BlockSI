# Claude Code Agent — Coordination Instructions

> For: Claude Code (cloud agent)
> Updated: 2026-03-10

## You Are One of Multiple Agents

This project has **two active AI agents** working simultaneously:

1. **VS Code Copilot Agent** — runs locally in VS Code, pushes directly to `main`
2. **Claude Code Agent (you)** — runs on cloud, pushes to `claude/*` feature branches

**Workflow**: You push to feature branches. The project manager reviews and merges
your branches into `main`. The VS Code agent pulls from `main` to sync with your work.

Merge conflicts can occur if `main` has advanced since your branch was created.

## CRITICAL: Do Not Revert Existing Changes

**The #1 rule: Never silently delete or overwrite code you didn't write.**

When rebasing your feature branch onto `main`, you may encounter conflicts.
If `main` has changes you don't recognise, they were made by the VS Code agent
and **must be preserved**. During conflict resolution, keep both sets of changes.
When in doubt, keep both.

### What Went Wrong (2026-03-10)

Your CSTR model commit (`4f00699`) deleted these Session 13 fixes from `main`:
- Backfill-active stuck fix (reset on connect/disconnect + 30s timeout)
- Plotly chart restyle optimization (fixed 4-trace layout + `_restyle_markers()`)
- Compact sidebar cards (5 individual cards → single dense card)
- CI band opacity increase (0.15 → 0.25)

These were **critical bug fixes** for live sensor display. They had to be
manually re-merged by the VS Code agent.

## Branching & Commit Protocol

**You are constrained to push to feature branches** (environment restriction).
The VS Code Copilot agent will fetch, review, and merge your branches into `main`
when prompted by the user.

**Branch naming**: Use `claude/<descriptive-name>` (e.g., `claude/review-project-docs-DvDeR`).

**Before starting any session:**
```bash
git fetch origin
git checkout -b claude/<feature-name> origin/main  # Create branch from latest main
```

**Before pushing:**
```bash
git fetch origin
git rebase origin/main    # Incorporate any commits made to main since you started
# Resolve any conflicts, preserving BOTH your changes AND main's changes
git push origin claude/<feature-name>
```

**After push**: Notify the user that your branch is ready. The VS Code Copilot
agent will handle fetching, reviewing, and merging into `main`.

**Conflict resolution:** When rebasing onto `main`, preserve ALL existing code from
the VS Code agent. Add your changes on top, don't replace. When in doubt, keep both.

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

## Commit Message Convention

```
<scope>: <summary>

<body>
```

Scopes: `Dashboard:`, `ESP32:`, `docs:`, `analysis:`, `chore:`
