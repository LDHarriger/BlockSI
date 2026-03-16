# Sequence Protocols Reference

> Load this file when working on calibration, validation, or CSTR sequence code.

---

## Calibration (single-command protocol)

```
PC → CMD,calibrate,flow=<lpm>,air_comp=<0|1>[,random=<p1,p2,...>]
ESP32 → SEQ,calibrate,STARTED / SAMPLE / COMPLETE
```

- ESP32 owns the 203-step sweep (0→100→0% in 1% increments)
- Random phase: PC generates N stratified levels, sent as `random=p1,...`
- Step total: `203 + 2*N` (N unique levels visited twice — ascending then descending)

---

## Validation (recipe protocol)

```
PC → CMD,sequence_start,validate,...
PC → CMD,seq_step,<idx>,<pwr>,<hold>,<phase>  (×N steps)
PC → CMD,seq_prompt,<before>,<id>,<text>       (×M prompts)
PC → CMD,seq_run
ESP32 → SEQ,validate,SAMPLE / COMPLETE
```

- 5 phases: baseline, spot_low (~33%), spot_high (~66%), target, cooldown
- 2 prompts: `check_flow` (before step 0), `check_route` (before step 1)

### Pass/Fail Criteria (PC-computed)

| Check | Criterion |
|-------|-----------|
| Baseline | Advisory only — warning dialog if > 0.01 %vol at start; not a hard gate |
| Spot correlation | Within 0.15 %vol or 15% relative of model |
| Target accuracy | Mean within 10% relative of prediction |
| Target stability | CV < 5% |

---

## CSTR Calibration (PC-driven)

- PC directly sends `power_set` / `relay_set` commands, monitoring DATA telemetry
- **Fill termination**: slope < 0.0003 %vol/sample AND range < 0.08 %vol over 45 samples
- **Evac termination**: 5 consecutive samples < 0.01 %vol
- **Pre-flight**: Requires a valid 100% power PASS validation certificate (< 24h old) before starting
- **Power watchdog**: If `power_actual_pct` < 80% during fill, auto-resends `power_set(100)` up to 3×

---

## Sequence Architecture — Responsibility Matrix

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
