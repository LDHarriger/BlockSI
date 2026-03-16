# Managing AI Agents — Human Operator Guide

> For: The human coordinator managing Claude Code and Copilot Chat on this project.
> Last updated: 2026-03-15

---

## Your Role in the Multi-Agent Workflow

You are the **coordination layer** between agents. Agents communicate asynchronously through shared files and git history — they cannot message each other in real time. Your job is to:

1. Sequence work so agents don't collide on the same files
2. Review agent output before it reaches `main`
3. Relay information between agents when needed (copy/paste relevant context)
4. Trigger end-of-session procedures and manage the memory files

---

## Starting a Session

### Tell the agent what domain it will work in
Claude Code performs better with an upfront scope declaration:
> "Today we're working on the PC dashboard only — ESP32 stays untouched"

This lets the agent skip loading irrelevant context (ESP32 firmware state) and focus its attention budget on the relevant domain.

### Trigger the startup protocol explicitly
Claude Code will read CLAUDE.md → RULES.md on its own. Prompt it:
> "Read your startup documents and confirm what's in MEMORY_CLAUDE.md before we begin."

This verifies the agent has checked for active locks and pending tasks from the prior session.

### Check for stale locks
Before work begins, scan both memory files for `<FILE_LOCK>` entries. A lock is stale if there are no recent commits from that agent touching the locked file. You can clear a stale lock by editing the memory file.

---

## During a Session

### Monitor context window usage (Copilot Chat)
Copilot Chat shows context usage in its UI. When it approaches ~85% full:
- Prompt: **"Update your summary before we continue."**
- The agent will rewrite `dashboard_agent_summary.md` or `esp32_agent_summary.md` with current state, compressing history into the archive
- This prevents context rot and preserves continuity for the next session

Claude Code (VS Code CLI) compresses automatically but you can also manually trigger: **"Summarize where we are and update your memory file."**

### Prevent agent collision on large files
`blocksi_dashboard.py` (~3350 lines) is the highest collision-risk file. Before assigning work to an agent:
1. Check `MEMORY_CLAUDE.md` and `MEMORY_COPILOT.md` for `<FILE_LOCK path="...blocksi_dashboard.py"/>` entries
2. If unlocked, have the agent write the lock before starting work
3. Only one agent should edit this file at a time

### Relaying information between agents
Agents don't share context windows. If Agent A made a significant change:
- Copy the relevant section of its updated summary or the diff
- Paste it into Agent B's chat: *"Agent A just made these changes: [paste]. Take note before continuing."*

---

## Ending a Session

Prompt at end of every development session:
> "Run end-of-session procedure: update your memory file, remove any locks, update the relevant agent summary, then commit and push."

The agent should do all of the following:
1. Move completed tasks → `<history_archive>` in its memory file
2. Remove `<FILE_LOCK>` entries
3. Append to `docs/dashboard_agent_summary.md` or `esp32_agent_summary.md`
4. `git status`, stage specific files (not `git add -A`), commit with scope prefix, push

Review the commit before it hits `main` — especially verify no `sdkconfig`, `build/`, or `__pycache__` was staged.

---

## Handling Agent Mistakes

### The #1 risk: one agent overwrites another's work
This happened in Session 13 (Claude Code's CSTR commit deleted Copilot's live-sensor fixes). How to prevent it:
- Enforce that both agents rebase on `origin/main` before pushing
- Never let an agent `git add -A` blindly — always `git add <specific files>`
- For Claude.ai (legacy cloud agent): always push to `claude/<branch>` and review the diff yourself before merging

### If an agent seems confused or hallucinates about code
Prompt: *"Before answering, read the relevant file — don't speculate about code you haven't opened."*
This directly addresses the Anthropic-documented failure mode of overconfident speculation.

### If an agent is adding unnecessary complexity
Prompt: *"Stop. Only make the changes I explicitly asked for. Don't add abstractions, helpers, or docstrings to code you didn't need to touch."*

---

## The Documentation Files — What Each Does

| File | Purpose | Who updates |
|------|---------|-------------|
| `RULES.md` | Master rules for all agents — behavioral, git, interface | Human (you) or agents by coordination |
| `MEMORY_CLAUDE.md` | Claude Code's working memory — locks, active tasks, session state | Claude Code only |
| `MEMORY_COPILOT.md` | Copilot's working memory — locks, active tasks, session state | Copilot Chat only |
| `docs/dashboard_agent_summary.md` | Current state of the PC dashboard | Either agent, additive |
| `docs/esp32_agent_summary.md` | Current state of ESP32 firmware | Either agent, additive |
| `docs/interface_contract.md` | ESP32 ↔ PC protocol (must be updated FIRST on interface changes) | Either agent |
| `docs/decisions_log.md` | Immutable architectural decisions log | Either agent, append-only |
| `docs/pitfalls.md` | NiceGUI/asyncio recurring bugs and gotchas | Either agent |

**Files you should never need to edit in normal operation:**
- `interface_contract.md` (agents update this; you review)
- `decisions_log.md` (append-only, immutable entries)
- Either memory file (agent-owned)

---

## Key Concepts to Understand

### Context window = working memory
Each agent session has a finite attention budget. Putting everything into a session is counterproductive — it dilutes attention and degrades quality. Good agents and good prompts load only what's relevant.

### Memory files ≠ summaries
The agent memory files (`MEMORY_CLAUDE.md`, `MEMORY_COPILOT.md`) are for **locks and active tasks**. The `*_agent_summary.md` files are for **persistent cross-session state**. Don't confuse the two.

### Decisions log is immutable
`decisions_log.md` entries are never edited, even to fix typos in reasoning. To supersede a decision, add a new entry at the top that references the old one. This preserves the historical reasoning trail.

### Status tags are mandatory
Every feature/item in summaries and the decisions log must have one of:
- `[IMPLEMENTED]` — in code, working
- `[DECIDED]` — agreed but not built
- `[PROPOSED]` — open idea, needs discussion before building

If an agent drops these tags, prompt it to add them back.

---

## Red Flags to Watch For

| Signal | What it might mean |
|--------|-------------------|
| Agent edits code without reading the file first | Hallucinating about implementation; prompt it to read first |
| `git add -A` in end-of-session commit | Risk of staging sdkconfig or build artifacts; redirect to specific files |
| Agent removes `[IMPLEMENTED]` items from summaries | Violates additive policy; remind it not to overwrite other sessions' content |
| Lock entries still present after session ends | Agent forgot cleanup; remove manually and note it in the session |
| Memory file growing beyond ~100 lines of active content | Archive completed tasks to `<history_archive>`; prompt agent to compress |
| Agent proposes changes to `RULES.md` unilaterally | Should flag to you first — RULES.md changes affect all agents |
