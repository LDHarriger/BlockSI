# BlockSI TODO Checklist

> Prioritized experiments, calibrations, and firmware changes.
> Last updated: 2026-03-22

---

## Priority 1: First-Time Setup

- [ ] Run Power-O3 calibration at 4.0 LPM (empty vessel, direct route)
- [ ] Run 100% power validation at 4.0 LPM
- [ ] Run k_d calibration at 4.0 LPM (empty vessel, sealed)
- [ ] Load substrate (oak or soyhull), record mass to ±0.01 kg
- [ ] Run k_abs calibration at 4.0 LPM (loaded vessel)
- [ ] Solve dosing schedule and verify achievability
- [ ] Run first test batch at conservative dose (e.g., 50 mg/kg, 15 min)

## Priority 2: Multi-Flow-Rate Calibration

- [ ] Run Power-O3 calibration at 2.0 LPM
- [ ] Run Power-O3 calibration at 6.0 LPM
- [ ] Run k_d calibration at each new flow rate
- [ ] Compare k_d values across flow rates (should be flow-independent)

## Priority 3: Substrate Characterization

- [ ] k_abs calibration: oak pellets (1.5 kg)
- [ ] k_abs calibration: soy hull (1.5 kg)
- [ ] k_abs calibration: water loading (3.75 kg)
- [ ] k_abs calibration: liquid culture (0.75 kg — preliminary, revisit)
- [ ] Compare k_abs across substrate types
- [ ] Verify loaded_material_density is physically reasonable

## Priority 4: Process Optimization

- [ ] Run batch at target dose, compare predicted vs. actual
- [ ] Run batch at 2× target dose, check equipment limits
- [ ] Characterize dose divergence: if > 15%, recalibrate k_abs
- [ ] Determine optimal ramp_switch_fraction (default: 0.85)
- [ ] Measure actual evac time vs. predicted

## Priority 5: Firmware Changes (Requires ESP-IDF Build)

These were identified in the WiFi Disconnect Audit (WP-0) and cannot be
implemented by software agents — they require compiling and flashing firmware.

- [ ] **Guard power_set during active sequence** (~10 lines in `main.c`)
  - Reject `CMD,power_set` if `seq_executor_get_state() == RUNNING`
  - Exception: allow `power_set,0` (E-STOP)
  - Similarly guard `relay_set`
- [ ] **Increase DATA_CACHE_SLOTS from 500 to 1000** (1-line change in `data_cache.h`)
  - Extends backfill coverage from ~20 min to ~41 min
  - RAM cost: ~160 KB
- [ ] **Integrate dosimetry into seq_executor** (~20 lines)
  - Call `dosimetry_cycle_start()` at recipe start
  - Call `dosimetry_process_sample()` per reading in the sample loop
  - Call `dosimetry_cycle_stop()` at recipe end
- [ ] **Add CMD,dosimetry_get command** (~15 lines)
  - Returns accumulated dose data for reconnect recovery
- [ ] **Add prompt timeout for batch recipes** (~5 lines)
  - Prevents infinite stall if PC is offline during a prompt
- [ ] **Cache SEQ SAMPLE messages in ring buffer** (significant effort)
  - Currently only DATA messages are cached; SEQ messages are lost on disconnect

## Priority 6: Documentation & Validation

- [ ] Collect 3+ batch runs for statistical validation of dosimetry accuracy
- [ ] Document standard operating conditions for each substrate type
- [ ] Create calibration schedule (which calibrations, how often)
- [ ] Peer review of dosimetry formulas against published CSTR theory
