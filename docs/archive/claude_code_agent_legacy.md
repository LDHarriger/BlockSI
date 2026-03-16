# Claude Code Agent — Coordination Instructions (LEGACY)

> **Superseded by `RULES.md` as of 2026-03-11.**
> Retained for historical reference only. The Claude_Cloud `[FALLBACK]` workflow is now documented in `RULES.md`.

---

> For: Claude Code (cloud agent)
> Updated: 2026-03-10

## You Are One of Multiple Agents

This project has **two active AI agents** working simultaneously:

1. **VS Code Copilot Agent** — runs locally in VS Code, pushes directly to `main`
2. **Claude Code Agent (you)** — runs on cloud, pushes to `claude/*` feature branches

## CRITICAL: Do Not Revert Existing Changes

**The #1 rule: Never silently delete or overwrite code you didn't write.**

### What Went Wrong (2026-03-10)

Your CSTR model commit (`4f00699`) deleted these Session 13 fixes from `main`:
- Backfill-active stuck fix (reset on connect/disconnect + 30s timeout)
- Plotly chart restyle optimization (fixed 4-trace layout + `_restyle_markers()`)
- Compact sidebar cards (5 individual cards → single dense card)
- CI band opacity increase (0.15 → 0.25)

These were **critical bug fixes** for live sensor display. They had to be
manually re-merged by the VS Code agent.

## Branching & Commit Protocol

**Branch naming**: Use `claude/<descriptive-name>`

**Before starting any session:**
```bash
git fetch origin
git checkout -b claude/<feature-name> origin/main
```

**Before pushing:**
```bash
git fetch origin
git rebase origin/main
git push origin claude/<feature-name>
```

**Conflict resolution:** Preserve ALL existing code from the VS Code agent. Add your changes on top.

## Communication Channel

- **Git history** — always read recent commits on `main` before starting
- **Documentation files** — the shared state lives in `docs/`
- **The human** — the user relays information between agents when needed
