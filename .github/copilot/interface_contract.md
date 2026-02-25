# BlockSI Interface Contract

> Last updated: 2026-02-25
>
> This is the **single source of truth** for everything shared between
> the ESP32 firmware and the PC dashboard.  Both agents MUST read this
> file and MUST update it before making cross-boundary changes.

## TCP Connection

| Property | Value |
|----------|-------|
| Transport | TCP over LAN |
| Port | 5000 (configurable via `--port` on PC) |
| Direction | ESP32 **connects to** PC (PC runs `asyncio.start_server`) |
| Encoding | UTF-8, newline-delimited (`\n`) |

## Message Types

### PC -> ESP32:  Commands
```
CMD,<command_name>[,arg1[,arg2]]\n
```

### ESP32 -> PC:  Responses
```
RSP,OK|ERR,<command_name>,<response_data>\n
```

### ESP32 -> PC:  Telemetry (unsolicited, periodic)
```
DATA,<17 fields>\n
```

### ESP32 -> PC:  Calibration data (during sweep)  `[IMPLEMENTED]`
```
CAL_DATA,power_pct,actual_pct,o3_pct,temp_c,direction,elapsed_s\n
```

### ESP32 -> PC:  State push (on connect/reconnect)  `[IMPLEMENTED]`
```
STATE,ozone_gen=<0|1>,o2_conc=<0|1>,air_comp=<0|1>,power=<pct>,flow=<lpm>\n
```
Sent automatically by ESP32 immediately after TCP connection is established.
PC dashboard should parse this to synchronize its relay/power display with
the ESP32's actual state.  This ensures the PC is never out of sync after
a disconnect/reconnect cycle.

### Future:  Sequence status  `[IMPLEMENTED]`
```
SEQ,<phase_name>,<progress_pct>,<power_pct>,<air_comp>,<elapsed_s>\n
```
Phase names vary by sequence type.  For power calibration (`cal`):
`baseline`, `sweep_up`, `sweep_down`, `random_pair`, `complete`.
For airflow validation (`validate`):
`prompt_vessel`, `prompt_direct`, `stabilize`, `measure`, `complete`.

Sent approximately every 1 second during sequence execution.
PC dashboard enters observer mode (`sequence_active=True`) when a sequence
is running, showing progress and disabling manual power controls.

### ESP32 -> PC:  Interactive prompt  `[IMPLEMENTED]`
```
SEQ,prompt,<prompt_id>,<message_text>\n
```
Sent when a sequence needs operator confirmation (e.g., valve routing).
PC dashboard should display the message and provide a confirm button.
Operator responds via `CMD,sequence_confirm,<prompt_id>[,<value>]`.
The sequence blocks until confirmation is received or timeout.

Prompt IDs for airflow validation (`validate`):
- `prompt_vessel` — operator routes L-valve to vessel
- `prompt_direct` — operator routes L-valve direct to 106-H

### ESP32 -> PC:  Sequence completion notification
```
SEQ_DONE,<type>,<result>,<elapsed_s>,pts=<count>\n
```
Result is one of: `ok`, `aborted`, `error`.

### ESP32 -> PC:  Calibration data (during cal sequence)  `[IMPLEMENTED]`
```
CAL_DATA,<timestamp_ms>,<power_pct>,<actual_pct>,<o3_pct>,<o2_lpm>,<air_comp_on>,<total_lpm>,<cell_temp_c>,<phase>\n
```
Phase is one of: `baseline`, `sweep_up`, `sweep_down`, `random_pair`.

### ESP32 -> PC:  Calibration metadata (at sequence start)
```
CAL_START,o2_lpm=<val>,random_count=<n>,step_pct=<n>\n
```

### ESP32 -> PC:  Calibration summary (at sequence end)
```
CAL_COMPLETE,total=<n>,baseline=<n>,sweep_up=<n>,sweep_down=<n>,random=<n>,elapsed=<s>\n
```

### ESP32 -> PC:  Validation data (during validate sequence)  `[IMPLEMENTED]`
```
VAL_DATA,<timestamp_ms>,<power_pct>,<actual_pct>,<o3_pct>,<o2_lpm>,<cell_temp_c>\n
```

### ESP32 -> PC:  Validation metadata (at sequence start)
```
VAL_START,power=<pct>,o2_lpm=<lpm>\n
```

### ESP32 -> PC:  Validation result (at sequence end)
```
VAL_RESULT,power=<pct>,o2_lpm=<lpm>,mean_o3=<pct>,std_o3=<pct>,expected_o3=<pct>,mean_temp=<c>,samples=<n>,elapsed=<s>\n
```

---

## CRITICAL:  Separator Rule

Commands use **COMMAS** as separators.  **NEVER** use colons.

- Correct: `CMD,power_set,50\n`
- WRONG:   `CMD,power_set:50\n`  (ESP32 parses `power_set:50` as the command name -- silently fails)

---

## DATA Telemetry Format

18 comma-separated tokens (prefix + 17 fields):

| Index | Field | Type | Unit |
|-------|-------|------|------|
| 0 | `DATA` | prefix | -- |
| 1 | `esp_timestamp_ms` | int | ms since boot |
| 2 | `vessel_o3_pct` | float | %vol |
| 3 | `temp_c` (cell) | float | C |
| 4 | `pressure_mbar` | float | mbar |
| 5 | `sample_v` | float | V |
| 6 | `ref_v` | float | V |
| 7 | `day` | int | -- |
| 8 | `month` | int | -- |
| 9 | `year` | int | -- |
| 10 | `hour` | int | -- |
| 11 | `minute` | int | -- |
| 12 | `second` | int | -- |
| 13 | `room_o3_ppm` | float | ppm |
| 14 | `vessel_temp_c` | float | C (-999 = N/A) |
| 15 | `power_target_pct` | int | % (0-100) |
| 16 | `power_actual_pct` | float | % |
| 17 | `wiper_voltage` | float | V |

**Authority note**: PC dashboard ignores `power_target_pct` (index 15)
from telemetry.  PC is sole authority for commanded power.
See `decisions_log.md` entry 2026-02-24.

---

## LAN Commands  (32 total, as of 2026-02-25)

### Relay Control
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `relay_set` | `<name>,<0\|1>` | `<name>=on\|off` | Names: `ozone_gen`, `o2_conc`, `air_comp` |
| `relay_get` | none | `ozone_gen=<0\|1>,o2_conc=<0\|1>,air_comp=<0\|1>` | |

### Power Control
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `power_set` | `<pct>` (0-100) | `power=<pct>,predicted_o3=<val>` | Integer percentage |
| `power_get` | none | `power=<pct>,actual=<val>` | |
| `flow_set` | `<lpm>` (0.1-10.0) | `flow=<lpm>` | O2 flow for prediction model |
| `flow_get` | none | `flow=<lpm>` | |
| `predict_o3` | `<flow>,<power>` | `predicted_o3=<val>` | Stateless prediction |

### Motor Pot
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `motor_pot_status` | none | Status string | ADC, wiper position |
| `motor_pot_calibrate` | none | `adc_min=<val>,adc_max=<val>,range=<val>` | Full sweep calibration |

### Calibration Sweep  (legacy, still functional)
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `calibrate_start` | none | `calibration=started` | Requires O3 gen relay ON, motor pot init |
| `calibrate_stop` | none | `calibration=stopping` | |
| `calibrate_status` | none | `active=<0\|1>,pct=<val>,dir=<val>,pts=<val>` | |

### Autonomous Sequences  `[IMPLEMENTED]`
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `sequence_start` | `<type>[,<params>]` | `type=<type>,status=started` | Types: `cal`, `validate`. Params vary by type |
| `sequence_stop` | none | `stopping` | Graceful abort of active sequence |
| `sequence_status` | none | `state=<s>,type=<t>,phase=<p>,progress=<f>,power=<n>,air=<0\|1>,elapsed=<f>` | States: idle, running, stopping, complete, error |
| `sequence_confirm` | `<prompt_id>[,<value>]` | `confirmed=<prompt_id>` | Responds to interactive prompts during sequences |

**Sequence types:**
| Type | Params | Description |
|------|--------|-------------|
| `cal` | `<o2_lpm>` (default 4.0) | Power-O3 calibration: baseline, sweep up/down, random pairs |
| `validate` | `<power_pct>,<o2_lpm>` (default 75, 4.0) | Airflow/concentration validation: interactive prompts, stabilize, measure |

### Sensors
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `sensors_get` | none | `pot=<ok\|err>,lab_o3=<ok\|err>,thermo=<ok\|err>,room_o3=<val>,vessel_temp=<val>` | |
| `room_o3_alarm` | none | Alarm status | DFRobot sensor |

### Recording / Backup (SPIFFS)
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `recording_start` | `[seq_name]` | Recording started | Optional sequence name |
| `recording_stop` | none | Recording stopped | |
| `recording_status` | none | Status string | |
| `backup_list` | none | File list | |
| `backup_delete` | `<filename>` | Deleted | |

### 106-H Monitor
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `106h_status` | none | Connection/config status | |
| `106h_avg_set` | `<option>` (1-5) | Averaging mode set | |
| `106h_avg_get` | none | Current averaging mode | |
| `106h_log_start` | none | Raw logging started | |
| `106h_log_stop` | none | Raw logging stopped | |

### System
| Command | Args | Response (OK) | Notes |
|---------|------|---------------|-------|
| `version` | none | Firmware version | |
| `status` | none | Full system status | |
| `i2c_scan` | none | Detected I2C addresses | |
| `time_sync` | `<pc_timestamp_ms>` | `synced,esp=<ms>,pc=<ms>` | Sent on connect |

---

## Relay Names

| Name | Controls | Notes |
|------|----------|-------|
| `ozone_gen` | MP-8000 generator SSR | Main O3 production |
| `o2_conc` | O2 concentrator SSR | Feed gas source |
| `air_comp` | Air compressor SSR | Adds ~10 LPM @ ~21% O2 |

SSRs are **active-high** (Kerwinn KG1-1DA25).

---

## Shared Constants

These constants must match between ESP32 firmware and PC dashboard:

| Constant | Value | Used by |
|----------|-------|---------|
| `O3_MASS_FLOW_K` | 0.357 | mg/s = %vol * LPM * K |
| `AIR_COMP_LPM` | 10.0 | LPM added when air compressor on |
| `POWER_MODEL_A` | 1.78 | O3_max = A/F + B (to be replaced by fitted model) |
| `POWER_MODEL_B` | 1.40 | O3_max = A/F + B |
| `DEFAULT_FLOW_LPM` | 4.0 | Default O2 flow rate |
| TCP Port | 5000 | Default LAN connection port |

---

## Sequence Integration Guide  `[DECIDED]`

> This section defines how the ESP32 sequence framework and the PC dashboard
> interact.  Both agents MUST follow these conventions.

### Architecture: ESP32 Executes, Dashboard Observes

```
Dashboard                          ESP32
─────────                          ─────
CMD,sequence_start,<type>,<params> →   prepare() + spawn task
                                    ← RSP,OK,sequence_start,...
                                    ← SEQ,<phase>,<progress>,...  (every ~1s)
                                    ← seq-specific data lines
                                    ← SEQ,prompt,<id>,<message>   (if interactive)
CMD,sequence_confirm,<id>[,<val>]  →   unblock, continue
                                    ← SEQ,...  (resumes)
CMD,sequence_stop                  →   request_stop() + cleanup()
                                    ← RSP,OK,sequence_stop,...
                                    ← SEQ_DONE,<type>,aborted,...
    (or on normal completion:)
                                    ← SEQ_DONE,<type>,ok,...
```

**Key principle**: The ESP32 owns all timing-critical execution.  The
Dashboard is a thin observer that shows progress, renders prompts, and
can abort.  The Dashboard NEVER sends `power_set` or `relay_set` during
an active sequence — the ESP32 drives hardware directly.

### Responsibility Matrix

| Responsibility | Owner | Notes |
|---|---|---|
| Sequence timing & phase transitions | **ESP32** | Deterministic, no LAN latency |
| Hardware control during sequence | **ESP32** | `power_set`, relay toggling |
| Sensor reading & data streaming | **ESP32** | `CAL_DATA`, `VAL_DATA`, etc. |
| Statistics computation | **ESP32** | Mean, std, deviation in `VAL_RESULT` |
| Prompt mechanism (send/block/unblock) | **ESP32** | `seq_prompt_user()` / `sequence_runner_provide_confirmation()` |
| Sequence initiation (user clicks "Start") | **Dashboard** | Sends `CMD,sequence_start,...` |
| Abort button | **Dashboard** | Sends `CMD,sequence_stop` |
| Progress display (banner, progress bar) | **Dashboard** | Parses `SEQ,...` lines |
| Interactive prompt UI (dialogs, instructions) | **Dashboard** | Parses `SEQ,prompt,...`, sends `CMD,sequence_confirm,...` |
| Rich operator guidance (diagrams, step text) | **Dashboard** | Maps prompt IDs to UI — NOT limited to ESP32's fallback `<message>` |
| Data visualization during sequence | **Dashboard** | Live charts from `CAL_DATA`, `VAL_DATA` |
| Result display & storage | **Dashboard** | Parses `CAL_COMPLETE`, `VAL_RESULT`, saves CSV |
| Control lockout during sequence | **Dashboard** | Disable power slider, relay toggles, preset buttons |
| E-STOP (always active, even mid-sequence) | **Dashboard** | Calls `cmd_emergency_stop()` which sends `sequence_stop` + `power_set,0` |

### Dashboard: Required Message Handling

The Dashboard TCP server must parse these message types from the ESP32:

| Message prefix | When | Dashboard action |
|---|---|---|
| `SEQ,<phase>,<pct>,...` | Every ~1s during any sequence | Update sequence banner: phase label, progress bar, power, elapsed |
| `SEQ,prompt,<id>,<msg>` | When sequence needs operator input | Show interactive dialog (see Prompt UI below) |
| `SEQ_DONE,<type>,<result>,...` | Sequence finished | Clear sequence banner, re-enable controls, show result toast |
| `CAL_START,...` | Cal sequence begins | Initialize calibration chart/data store |
| `CAL_DATA,...` | Each cal measurement point | Append to live scatter plot |
| `CAL_COMPLETE,...` | Cal sequence ends normally | Save CSV, show summary |
| `VAL_START,...` | Validation begins | Initialize validation view |
| `VAL_DATA,...` | Each validation sample | Append to live O3 time-series |
| `VAL_RESULT,...` | Validation ends normally | Show pass/fail report with deviation % |
| `STATE,...` | On TCP connect/reconnect | Sync relay states + power display |

### Dashboard: Prompt UI Design

When the Dashboard receives `SEQ,prompt,<prompt_id>,<message_text>`:

1. **Show a modal dialog** (blocks other interaction except E-STOP and abort)
2. **Map prompt ID to rich content** — the `<message_text>` from the ESP32 is
   a plain-text fallback.  The Dashboard should maintain a mapping of prompt
   IDs to richer instructions:

| Prompt ID | Sequence | Rich UI guidance |
|---|---|---|
| `prompt_vessel` | `validate` | "**Step 1 — Route to Vessel**<br>Turn the L-valve so airflow goes through the sterilization vessel.<br>Verify the rotameter reads the target LPM.<br>Optionally enter the actual rotameter reading below." |
| `prompt_direct` | `validate` | "**Step 2 — Route Direct to Sensor**<br>Turn the L-valve so airflow goes directly to the 106-H sensor, bypassing the vessel.<br>Adjust the needle valve until the rotameter matches the vessel route flow.<br>Optionally enter the confirmed LPM." |

3. **Provide a Confirm button** + optional numeric input (LPM reading)
4. **Send** `CMD,sequence_confirm,<prompt_id>[,<lpm_value>]` when operator confirms
5. **Show a "Waiting for ESP32..." spinner** after confirming (ESP32 resumes and
   sends next `SEQ,...` status within ~1s)
6. **Handle timeout**: If no prompt response in 5 minutes, ESP32 will abort
   the sequence and send `SEQ_DONE,...,error`.  Dashboard should detect this
   and clean up.

### Dashboard: Sequence Observer Mode

When `sequence_active=True`:

```
┌──────────────────────────────────────────────────────────┐
│  ⚠ VALIDATE — Phase: stabilize — 45%  ██████░░░░  ABORT │
└──────────────────────────────────────────────────────────┘
```

- **Amber banner** at top of page (similar to current calibration banner)
- **Power controls disabled**: slider, preset buttons, linked inputs all `.props("disable")`
- **Relay controls disabled**: toggle buttons `.props("disable")` (ESP32 manages relays)
- **E-STOP remains active**: Always functional, sends `sequence_stop` first then `power_set,0`
- **Phase label and progress** updated from `SEQ,...` lines
- **Abort** sends `CMD,sequence_stop` and dashboard waits for `SEQ_DONE,...,aborted`

### Dashboard: Refactoring `CalibrationRunner`

The existing `CalibrationRunner` class (~175 lines) runs calibration **PC-side**
by sending individual `CMD,power_set,N` commands.  It must be refactored to
**observer mode**:

| Current (PC-driven) | Target (ESP32-driven) |
|---|---|
| `CalibrationRunner.start()` sends `CMD,power_set,0` | Send `CMD,sequence_start,cal,<lpm>` |
| `CalibrationRunner.step()` increments power each tick | Parse `SEQ,...` for phase/progress |
| `CalibrationRunner._pair_step()` toggles air compressor | Parse `CAL_DATA,...` for live plot |
| Manages `S.cal_phase`, `S.cal_current_power` | Update from `SEQ,...` fields |
| Saves CSV from `state.cal_data[]` | Save CSV from streamed `CAL_DATA` lines |
| `CalibrationRunner.stop()` sends `CMD,power_set,0` | Send `CMD,sequence_stop` |

### Dashboard: Validation Sequence UI

The Calibration Tab should be extended (or a new Validation sub-section added):

1. **Start controls**: Power % input (default 75), O2 LPM input (default 4.0), "Validate" button
2. **Progress area**: Shows current phase, progress bar, elapsed time
3. **Live chart**: O3 % over time from `VAL_DATA` points (shows stabilization + measurement)
4. **Result card**: At completion, shows:
   - Mean O3 %, Std Dev
   - Expected O3 % (from model)
   - Deviation %
   - Pass/Fail indicator (green <10%, amber 10-20%, red >20%)
   - Sample count and total elapsed time

### Future Sequence Types (Planned)

These sequences are not yet implemented on the ESP32 but follow the same
observer pattern.  Documenting here so the Dashboard agent can plan UI for them:

| Type | Name | ESP32 does | Dashboard does |
|---|---|---|---|
| `fill` | Fill model calibration | Fills vessel with known O3, measures transit time | Shows fill progress, stores model params |
| `decay` | O3 decay testing | Fills, seals, measures decay curve | Live decay chart, curve fitting display |
| `sterilize` | Sterilize batch | Full cycle: validate → fill → hold → vent | Multi-phase progress, dose tracker, completion report |

**Note on `sterilize`**: This will likely chain `validate` as a sub-sequence,
then proceed to fill/hold/vent.  The Dashboard must handle the same prompt
IDs from the validation phase within the larger sterilization flow.
