# BlockSI Operating Procedures

> **System**: BlockSI Ozone Sterilization Chamber
> **Last updated**: 2026-03-22

---

## 1. System Startup

### 1.1 Hardware Checklist
1. Verify all gas connections are leak-free
2. Confirm rotameter is visible and readable
3. Ensure 106-H ozone sensor is powered and communicating
4. Check DFRobot room O3 sensor is mounted near the exhaust
5. Verify ESP32 is powered and the status LED is solid

### 1.2 Software Startup
1. Navigate to `Interfaces/PC/`
2. Activate the virtual environment: `.venv\Scripts\activate`
3. Launch the dashboard: `python blocksi_dashboard.py`
4. Open a browser to `http://localhost:8080` (auto-opens by default)
5. Wait for the TCP connection indicator to turn green (ESP32 connects to PC)
6. Verify time sync completes (displayed in Debug tab)

### 1.3 Post-Startup Verification
- **Telemetry**: Confirm vessel O3, cell temperature, and pressure are updating
- **Power**: Set power to 0% and verify DAC reads ~0%
- **Relays**: All three relays should show OFF (grey)

---

## 2. Calibration SOPs

### 2.1 Power-O3 Calibration (Calibration Tab)

**Purpose**: Map generator power (0-100%) to O3 output concentration for the current flow rate.

**Pre-requisites**:
- O2 concentrator warmed up (15+ min)
- Gas route connected through 106-H sensor (direct, NOT through vessel)
- Room O3 sensor active

**Procedure**:
1. In the **Calibration** tab, click **Start Calibration**
2. The system will turn on the O2 relay, then prompt: "Read the rotameter and enter the actual LPM to the nearest 0.25"
3. Read the rotameter and enter the value
4. The ESP32 runs a 0→100→0% sweep (203 steps, ~17 min)
5. Optionally enter custom random power levels for additional validation points
6. Wait for completion — model is auto-fitted and saved
7. Verify model status shows green check mark

**Frequency**: Every 2 weeks, or after any hardware change.

### 2.2 k_d Calibration (Calibration Tab → CSTR k_d Section)

**Purpose**: Measure the first-order O3 decay rate in the empty vessel.

**Pre-requisites**:
- Valid Power-O3 calibration at the selected flow rate (within 14 days)
- Valid 100% power validation certificate (within 24 hours)
- Vessel EMPTY and sealed
- Gas route through vessel

**Procedure**:
1. Select the calibrated flow rate from the dropdown
2. Click **Start k_d Calibration**
3. System runs: baseline → fill at 100% → detect steady-state → evacuation
4. Fill extends automatically if steady-state is not reached (up to 5 extensions × 2 min)
5. Evacuation runs until O3 < 0.01% vol, then 5-min flush
6. Model is auto-fitted from ALL k_d CSV files and saved
7. Verify model status shows green check mark

**Frequency**: After any vessel modification or monthly.

### 2.3 k_abs Calibration (Calibration Tab → k_abs Section)

**Purpose**: Measure the O3 absorption rate into loaded substrate.

**Pre-requisites**:
- Valid Power-O3 model and k_d model
- Valid 100% power validation certificate (within 24 hours)
- Vessel LOADED with substrate (record mass accurately)
- Mixing screw running at 25-30 RPM
- Gas route through vessel via L-valve

**Procedure**:
1. Select the calibrated flow rate
2. Click **Start k_abs Calibration**
3. Confirm mixing screw is running (operator prompt)
4. Confirm L-valve routes gas through vessel (operator prompt)
5. System runs: baseline → 30-min fill/hold at 100% → evacuation
6. Model is auto-fitted (1-param and optional biphasic)
7. Review: V_residual, k_abs, loaded material density

**Frequency**: At least once per substrate type; rerun if substrate formulation changes.

### 2.4 Validation (Validation Tab)

**Purpose**: Verify the Power-O3 model at specific operating points.

**Pre-requisites**:
- Valid Power-O3 calibration at the selected flow rate
- Gas route through 106-H sensor (direct)

**Procedure**:
1. Set target power and select flow rate
2. Click **Start Validation**
3. ESP32 runs a 6-step recipe: baseline, spot_low, spot_medium, spot_max, target hold, cooldown
4. PC compares measured O3 to model prediction at each point
5. Pass/fail result displayed; certificate saved if pass

**Frequency**: Before each k_d or k_abs calibration; recommended daily during active use.

---

## 3. Batch Sterilization SOP

### 3.1 Pre-Batch Checklist
- [ ] All three calibrations complete and current (Power-O3, k_d, k_abs)
- [ ] Validation certificate at 100% power within 24 hours
- [ ] Substrate loaded in vessel, mass recorded
- [ ] Mixing screw verified operational
- [ ] Gas route: O2 → generator → vessel → 106-H sensor → exhaust
- [ ] Room O3 sensor active
- [ ] Inoculant prepared and ready

### 3.2 Running a Batch (Processing Tab)

1. Go to the **Processing** tab
2. Set parameters:
   - **Flow Rate**: Select from calibrated rates
   - **Substrate Mass**: Enter accurately (use preset buttons for common values)
   - **Target Dose**: Enter desired mg O3 per kg substrate
   - **Process Time**: Hold phase duration (typically 30 min)
   - **Experiment Type**: Directory name for data storage
3. Click **Solve Schedule** to preview the dosing plan
4. Verify the schedule is achievable and parameters look correct
5. Click **Start Batch**
6. Monitor the live panel:
   - Phase indicator shows current stage (baseline → ramp → hold → evac)
   - Dose progress bar tracks accumulated vs. target
   - Charts show O3 concentration and mass balance in real time
7. After sterilization completes:
   - **Vessel Cool-Down**: Confirm vessel has cooled and O3 < 0.01% vol
   - **Add Inoculant**: Open vessel, add inoculant, close vessel
   - **Distribute & Label**: Mix, bag, and label with batch ID

### 3.3 Data Output
Each batch creates a directory: `Data/Batch/{ExperimentType}/{BatchID}/`

Contents:
- `{BatchID}_batch.csv` — per-sample telemetry + dosimetry
- `{BatchID}_schedule.json` — solver output (traceability)
- `{BatchID}_dosimetry.json` — final dose summary
- `{BatchID}_debug.log` — detailed execution log

### 3.4 Disconnect Resilience
If TCP disconnects during a batch:
- The ESP32 continues executing the recipe autonomously
- The dashboard displays a warning but the batch is NOT lost
- On reconnect, telemetry backfill recovers up to ~20 min of DATA
- The batch CSV will have a gap in PC-side dosimetry data
- ESP32 dosimetry accumulator (when activated) preserves total dose

---

## 4. Emergency Procedures

### 4.1 E-STOP
- Click the red **E-STOP** button in the sequence banner (any tab)
- This sends `sequence_abort` to the ESP32 and runs safe standby
- Safe standby: power=0%, all relays OFF

### 4.2 Room O3 Alarm
- If room O3 exceeds 0.1 ppm, evacuate the room immediately
- The DFRobot sensor reading is shown in the sidebar
- Check for leaks after the alarm clears

### 4.3 Power Failure
- ESP32 loses state on reboot — no recipe survives power loss
- Generator shuts off (SSR de-energizes), but residual O3 remains in vessel
- Ventilate the vessel manually before opening

---

## 5. Maintenance

### 5.1 106-H Sensor
- Calibrate per manufacturer schedule (annually or after re-gassing)
- The 106-H averaging mode affects sample rate (~2-3s per sample)

### 5.2 O2 Concentrator
- Record actual O2 purity periodically (affects model accuracy)
- Replace molecular sieve beds per manufacturer schedule

### 5.3 Generator (MP-8000)
- The DAC-to-power relationship may drift — recalibrate Power-O3 if
  the model residuals increase significantly

### 5.4 Data Backup
- All data is stored in `Interfaces/Data/` and `Interfaces/Models/`
- Back up these directories regularly
- Model JSON files are timestamped — keep historical versions
