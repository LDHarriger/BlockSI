# BlockSI Project Rules

> **This document is the single source of truth for all agent behavior in this project.**
> Last updated: 2026-03-11
> See also: `MEMORY_CLAUDE.md` (Claude Code working memory), `MEMORY_COPILOT.md` (Copilot Chat working memory)

---

## Project Overview

BlockSI is an ozone sterilization monitoring and control system for Shroom-E Co., comprising:

- **ESP32 Firmware** (`Interfaces/ControlSystem/`): ESP-IDF v5.4+ project controlling ozone generation, monitoring sensors, and streaming data
- **PC Dashboard** (`Interfaces/PC/`): NiceGUI dashboard (`blocksi_dashboard.py`, ~3350 lines)
- **Data** (`Interfaces/Data/`): CSV telemetry and sequence logs, organised by type:
  - `Telemetry/` — Per-connection stream CSVs
  - `Calibration/` — Power-O3 characterisation CSVs
  - `Validation/` — Validation run CSVs (suffix `_PASS` or `_FAIL`)
  - `CSTR/` — Combined fill+evac calibration CSVs (single CSV per run, `phase` column)
  - `Sterilization/` — Future sequence type
- **Models** (`Interfaces/Models/`): Fitted prediction models (git-tracked):
  - `O3Power/` — Power→O3 sigmoid models (JSON, per LPM/O2% condition)
  - `CSTR/` — Decay-aware CSTR model (`cstr_model.json`, flow-rate independent)

---

## Architecture

```
ESP32 = Arms (hardware control, timing)
PC    = Brains (analysis, recipes, UI, pass/fail)
```

```
106-H Ozone Monitor (RS232, vessel OUTLET)
    → ESP32 (UART2)
    → Golioth Cloud + LAN TCP:5000
    → PC Dashboard
```

**Gas path**: O2 concentrator (+ optional air compressor) → MP-8000 generator → Vessel → 106-H sensor (outlet). L-valve switchable for direct-to-sensor bypass (validation). Vessel: ~11.3L tank, ~60% fill → ~4L residual gas volume.

---

## Agent Roster

| Agent | Platform | Branch Authority |
|-------|----------|-----------------|
| **Claude Code** (this agent) | Local VS Code CLI | Direct push to `main` |
| **Copilot Chat** | Local VS Code plugin | Direct push to `main` |
| **Claude.ai** *(legacy/fallback)* | Cloud | Push to `claude/*` branches only — user must merge |

Both local agents operate on `main` directly. The cloud agent is deprecated as primary but documented in the [Legacy Claude.ai section](#legacy-claudeai-cloud-agent-abbreviated) below.

---

## Locking Protocol

To prevent simultaneous edits to large shared files:

1. **Before starting work on a file**, write a lock entry at the top of your memory file:
   ```xml
   <FILE_LOCK path="Interfaces/PC/blocksi_dashboard.py"/>
   ```
2. **Upon completing work or handing off**, remove the lock entry from your memory file.
3. **Before editing a locked file**: read both `MEMORY_CLAUDE.md` and `MEMORY_COPILOT.md`. If the file is locked by the other agent, wait or coordinate through the user.
4. **Lock expiry**: A lock is stale if the locking agent has no recent commits touching that file. Stale locks may be cleared.

**Files that MUST be locked before editing:**

| File | Risk |
|------|------|
| `Interfaces/PC/blocksi_dashboard.py` | ~3350 lines; frequent edits by both agents |
| `docs/interface_contract.md` | Shared protocol spec — edits affect both ESP32 and PC |
| `docs/decisions_log.md` | Append-only; concurrent appends can interleave |
| `RULES.md` (this file) | Master rules — edits need coordination |

---

## Memory & Handoff Architecture

| File | Owner | Purpose |
|------|-------|---------|
| `MEMORY_CLAUDE.md` | Claude Code | Working memory: active tasks, file locks, session state |
| `MEMORY_COPILOT.md` | Copilot Chat | Working memory: active tasks, file locks, session state |
| `docs/dashboard_agent_summary.md` | Either | PC dashboard state — persistent cross-session reference |
| `docs/esp32_agent_summary.md` | Either | ESP32 firmware state — persistent cross-session reference |
| `docs/decisions_log.md` | Either | Immutable architectural decisions log |
| `docs/interface_contract.md` | Either | Single source of truth for ESP32 ↔ PC protocol |

### Preservation Policy

- **Never overwrite objectives or history.** When tasks complete or pivot, move them to the `<history_archive>` section in your memory file — do not delete them.
- **Never delete or edit `decisions_log.md` entries.** To supersede a decision, add a NEW entry at the top that references the old one.
- **Agent summaries are additive** — append your session's "What Changed" block; do not rewrite other sessions' content.

---

## New Session Startup

Read in this order:

1. `RULES.md` (this file) — rules, architecture, conventions
2. Your memory file (`MEMORY_CLAUDE.md` or `MEMORY_COPILOT.md`) — check active locks and tasks
3. `docs/interface_contract.md` — LAN protocol definitions
4. `docs/dashboard_agent_summary.md` — current dashboard state
5. `docs/esp32_agent_summary.md` — current ESP32 firmware state
6. `docs/decisions_log.md` (recent entries) — architectural context
7. `docs/pitfalls.md` — NiceGUI/asyncio gotchas to avoid

---

## End-of-Session Procedure

1. **Update your memory file**: Move completed tasks to `<history_archive>`; update `<active_tasks>` with pending work.
2. **Remove file locks** from your memory file.
3. **Update agent summary** (`docs/dashboard_agent_summary.md` or `esp32_agent_summary.md`) if significant changes were made. Append your session; do not overwrite prior sessions.
4. **Commit and push**:
   ```bash
   git status --short                    # Review — ensure no credentials or build artifacts
   git add <specific files>              # Stage specific files; avoid git add -A blindly
   git commit -m "<scope>: <summary>"
   git push
   ```
5. **Confirm to user**: what was done, what is pending for the next session.

### Commit Scopes

`Dashboard:` | `ESP32:` | `docs:` | `analysis:` | `chore:`

Split multi-domain changes into separate commits. Example: firmware change + doc update = two commits.

### Critical Git Rules

- **Never commit `sdkconfig`** — contains WiFi credentials and Golioth PSK
- **Never commit `build/` artifacts** or `__pycache__/`, `*.pyc`
- **Never skip hooks** (`--no-verify`)
- **Never force-push to `main`**
- **Never amend published commits** on `main`

---

## Interface Change Rule (CRITICAL)

When changing **anything** that crosses the ESP32 ↔ PC boundary:

1. Update `interface_contract.md` **FIRST** with the new/changed definition
2. Tag the change `[IMPLEMENTED]` or `[DECIDED]`
3. Note it in your agent summary under "Recent Changes"

**Changes requiring this:**
- Adding, removing, or modifying a LAN command
- Changing the DATA telemetry format
- Changing TCP connection model (port, who connects)
- Adding a new message type (`SEQ,...`, `STATE,...`, etc.)
- Changing shared constants (flow rates, model coefficients)

---

## Status Tags

All items in summaries and decisions log MUST use one of:

- **`[IMPLEMENTED]`** — Done, in code, tested/verified working
- **`[DECIDED]`** — Agreed but not yet built
- **`[PROPOSED]`** — Open idea, not committed — requires discussion before building

---

## LAN Command Protocol

ESP32 ↔ PC uses **comma-separated** commands over TCP port 5000:

```
PC → ESP32:   CMD,command_name[,arg1[,arg2]]\n
ESP32 → PC:   RSP,OK|ERR,command_name,<response_data>\n
ESP32 → PC:   DATA,<17 fields>\n          (periodic telemetry)
ESP32 → PC:   SEQ,<type>,<ACTION>,...\n   (sequence events)
ESP32 → PC:   STATE,ozone_gen=..,o2_conc=..,air_comp=..,power=..,flow=..\n
```

**⚠ WARNING**: Commands use **COMMAS** as separators — NEVER colons.
- Correct: `CMD,power_set,50`
- WRONG: `CMD,power_set:50` (ESP32 parses `power_set:50` as the command name — silently fails)

Full specification: `docs/interface_contract.md`

---

## ESP32 Firmware Conventions

### Build & Flash
**DO NOT build or flash from an agent terminal.** Describe changes; the user builds/flashes manually.

Reference commands (human use only):
```powershell
idf.py build
idf.py -p COM3 flash monitor    # exit: Ctrl+]
idf.py menuconfig
```

### Code Conventions
- Returns: `esp_err_t`, logging: `ESP_LOG*` macros, tasks: FreeRTOS
- Module pattern: `module_init()` → `module_deinit()` → `module_is_initialized()`
- Static state: `static struct { ... } s_module = {0};`
- Tags: `static const char *TAG = "MODULE_NAME";`
- **Pin assignments**: Always check `Interfaces/ControlSystem/main/blocksi_pins.h` before adding hardware

### Key GPIO Assignments (from `blocksi_pins.h`)
| Bus/Function | GPIOs | Connected to |
|-------------|-------|--------------|
| I2C | 21, 22 | DFRobot O3 sensor (100kHz) |
| SPI | 18, 19, 5 | MAX31855 thermocouple |
| UART2 | 16, 17 | 106-H RS232 (via level shifter) |
| Relays | 12, 13, 27 | SSR control (3 relays) |
| Motor Pot | 25, 26, 34 | DRV8833 H-bridge + ADC feedback |

---

## PC Dashboard Conventions

- **Run**: `.venv\Scripts\python.exe Interfaces\PC\blocksi_dashboard.py [--port 5000]`
- **UI port**: http://localhost:8080 (NiceGUI)
- **Dependencies**: nicegui, numpy, pandas, plotly, scipy (in `.venv`)
- **Venv required**: Must use `.venv\Scripts\python.exe`, not system Python

### NiceGUI Patterns (Critical)
- **Quasar disable**: `.props("disable")` / `.props(remove="disable")`
- **Background tasks**: Never call `ui.*` from `asyncio.create_task()` — use `_notify_queue` or `SystemState` field mutations (read by `_tick_inner()` which has the correct slot context)
- **Plotly live update**: Full redraws via `power_plot.figure = fig; power_plot.update()`; per-tick marker updates via `_restyle_markers()` using `ui.run_javascript()` → `Plotly.restyle(el.$el, ...)`
  - `run_method('react', ...)` FAILS — do not use
  - `run_method('restyle', ...)` unverified — do not use
  - Use `el.$el` not `el.$refs.qRef` for `ui.plotly` DOM access

### Power Authority
**PC is sole authority for `power_target_pct`** — never accept this value from ESP32 telemetry. (See `decisions_log.md` entry 2026-02-24.)

### Sequence Lockout
When `sequence_active=True`: power controls (slider, presets), relay toggles, and LPM input must be disabled via `.props("disable")`. E-STOP remains always active.

### Key Constants
```python
O3_MASS_FLOW_K   = 0.3327   # mg/s per (%vol × LPM), V_m=24.04 L/mol at 20°C
AIR_COMP_LPM     = 10.0     # LPM added when air compressor on
DEFAULT_FLOW_LPM = 4.0      # Default O2 flow rate
O2_CONC_PCT      = 95       # O2 concentrator purity
AIR_COMP_O2_PCT  = 21       # Atmospheric O2
TCP_PORT         = 5000
```

---

## Key File Paths

| Purpose | Path |
|---------|------|
| PC Dashboard | `Interfaces/PC/blocksi_dashboard.py` |
| Power model | `Interfaces/PC/analysis/power_o3_model.py` |
| Fill/CSTR model | `Interfaces/PC/analysis/fill_model.py` |
| O3Power models | `Interfaces/Models/O3Power/*.json` |
| CSTR model | `Interfaces/Models/CSTR/cstr_model.json` |
| Pin assignments | `Interfaces/ControlSystem/main/blocksi_pins.h` |
| LAN command handler | `Interfaces/ControlSystem/main/main.c` (~line 350) |

---

## Known Pitfalls

See `docs/pitfalls.md` for detailed NiceGUI/asyncio gotchas. Summary:

1. **`ui.notify()` from background tasks** → use `_notify_queue` pattern
2. **`on_change` fires twice on linked elements** → use `_updating` boolean guard
3. **`_sequence_cleanup()` must be unconditional** — never gate on `seq_confirmed`
4. **`asyncio.Event` dialogs must run in button-click handlers** — not inside `asyncio.create_task()` coroutines
5. **`ui.run_javascript()` crashes on dead client** → wrap in `try/except RuntimeError`
6. **`ui.plotly` DOM access**: use `el.$el` — `el.$refs.qRef` is `undefined` on `ui.plotly`

---

## Power-O3 Model Reference

- **Type**: 4-parameter sigmoid: `O3 = L / (1 + exp(-k*(P - P0))) + b`
- **Dataclass**: `PowerO3Model(L, k, P0, b, flow_lpm, o2_pct, r_squared, rmse, n_points, ci_power, ci_sigma)`
- **CI bands**: 101-point Jacobian-propagated ±1σ (from `pcov`), stored in model JSON
- **Load safety**: `load_model()` filters unknown JSON keys via `__dataclass_fields__`

## CSTR Model Reference

- **ODE**: `dC/dt = (C_in - C)/τ - k_d·C`
- **Parameters**: `V` (gas volume L), `k_d` (decay rate s⁻¹), `V_dead` (dead volume L)
- **Stored**: `Interfaces/Models/CSTR/cstr_model.json` (universal — all conditions)
- **Fitted via**: `scipy.optimize.curve_fit` in `fill_model.py`

---

## Relay Names & Hardware Interlock

| Name | Controls | Notes |
|------|----------|-------|
| `ozone_gen` | MP-8000 generator SSR | Main O3 production |
| `o2_conc` | O2 concentrator SSR | Feed gas source |
| `air_comp` | Air compressor SSR | Adds ~10 LPM @ ~21% O2; **internal to MP-8000** — requires `ozone_gen` ON |

**Hardware Interlock `[IMPLEMENTED]`**: `air_comp ON` with `ozone_gen OFF` → rejected. `ozone_gen OFF` → auto-sets `air_comp OFF`. Enforced in firmware `relay_set_with_source()`.

SSRs are **active-high** (Kerwinn KG1-1DA25).

---

## Calibration & Validation Summary

### Calibration (single-command)
```
PC → CMD,calibrate,flow=<lpm>,air_comp=<0|1>[,random=<p1,p2,...>]
ESP32 → SEQ,calibrate,STARTED / SAMPLE / COMPLETE
```
- ESP32 owns the 203-step sweep (0→100→0% in 1% increments)
- Random phase: PC generates N stratified levels, sent as `random=p1,...`

### Validation (recipe protocol)
```
PC → CMD,sequence_start,validate,...
PC → CMD,seq_step,<idx>,<pwr>,<hold>,<phase>  (×N)
PC → CMD,seq_run
ESP32 → SEQ,validate,SAMPLE / COMPLETE
```
- 5 phases: baseline, spot_low (~33%), spot_high (~66%), target, cooldown
- Pass/fail criteria: spot correlation ±15%, target accuracy <10% deviation, CV <5%
- Baseline is advisory only (warning dialog if vessel O3 > 0.01 %vol at start)

### CSTR Calibration (PC-driven)
- PC directly sends `power_set` / `relay_set` commands, monitoring DATA telemetry
- Fill termination: slope < 0.0003 %vol/sample AND range < 0.08 %vol over 45 samples
- Evac termination: 5 consecutive samples < 0.01 %vol
- Requires a valid 100% power PASS validation certificate (< 24h old) before starting

---

## Legacy Claude.ai Cloud Agent (Abbreviated)

> **Status**: Deprecated as primary agent. Use only if local Claude Code CLI is unavailable.

**Branch protocol**: Push to `claude/<feature-name>` branches only.

**Before starting:**
```bash
git fetch origin
git checkout -b claude/<feature-name> origin/main
```

**Before pushing:**
```bash
git fetch origin
git rebase origin/main    # Preserve ALL changes from main — keep both sets on conflict
git push origin claude/<feature-name>
# Notify user: branch ready for review
```

**Conflict resolution**: Always preserve `main` changes. Never silently delete or overwrite code you didn't write.

**Documentation rules**:
- `decisions_log.md`: append-only, new entries at TOP, never edit existing entries
- Agent summaries: additive only — append your "What Changed" block

---

## Sequence Architecture Reference

**Responsibility Matrix**:

| Responsibility | Owner |
|---|---|
| Calibration sweep pattern (203 steps) | ESP32 |
| Validation / custom recipes | PC |
| Hardware control during sequence | ESP32 |
| Sample-counted hold timing | ESP32 |
| Per-sample data streaming | ESP32 |
| Statistical analysis (mean, std, CV) | PC |
| Model fitting & prediction | PC |
| Pass/fail determination | PC |
| E-STOP | PC (always active — sends `sequence_abort` + `power_set,0`) |
