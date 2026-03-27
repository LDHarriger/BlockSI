# Documentation Gaps Audit

**Date**: 2026-03-24
**Source**: Automated audit of `Resources/` vs `docs/` cross-references
**Status**: DRAFT — needs user review

---

## Resources/ Inventory

| File | Type | In Use? |
|------|------|---------|
| `Manual_model_106-H_revF-8.pdf` | Manual | Yes — 106-H ozone monitor |
| `MP-8000-110V-Manual-1.2.2.pdf` | Manual | Yes — ozone generator |
| `PRM16_MotorPot.pdf` | Datasheet | Yes — PRM162 motorized pot |
| `drv8833_MotorDriver.pdf` | Datasheet | Yes — DRV8833 H-bridge |
| `DRI0044_TB6612FNG_MotorDriver.pdf` | Datasheet | **No** — evaluated alternative |
| `ESP32-WROOM-32-Datasheet.pdf` | Datasheet | Yes — main MCU |
| `ESP32-DOIT-DEVKIT-V1-Board-Pinout-30-GPIOs-Copy.webp` | Reference | Yes — pinout diagram |
| `PortaSense_D16.pdf` | Datasheet | **No** — evaluated alternative |
| `DoubleHelixScrewImpellerDrawing_mm.pdf` | Drawing | Yes — mixing screw |
| `FullMixerAssemblyCAD.JPG` | CAD image | Yes — vessel assembly |
| `FullMixerAssemblyReal.jpg` | Photo | Yes — as-built photo |

None of the Resource files are cited by filename in any documentation.

---

## Critical Gaps (Safety / Correctness)

### 1. Air Compressor GPIO Mismatch

`docs/reference/hardware.md` line 29 says air_comp uses **GPIO 27**.
`blocksi_pins.h` line 96 defines `RELAY_AIR_COMP_GPIO = 14`.
GPIO 27 is listed as "Available (was DRV8833 SLP, now hardwired)."
**`hardware.md` is wrong — must be corrected to GPIO 14.**

### 2. DFRobot O3 Alarm Thresholds Not in Safety Docs

`blocksi_pins.h` defines three alarm levels never mentioned in docs:
- `LAB_O3_ALARM_WARNING = 0.07 ppm` (OSHA action level)
- `LAB_O3_ALARM_DANGER = 0.10 ppm` (OSHA PEL)
- `LAB_O3_ALARM_CRITICAL = 0.30 ppm` (immediate danger — triggered the Session 14 bug)

`operating_procedures.md` §4.2 mentions 0.1 ppm only.

### 3. PRM162 Floating-Ground Wiring (Section 1)

Section 1 of PRM162 floats with MP-8000 (~10V above ground). Section 2 gives
ADC feedback. Connecting Section 1 to ESP32 ADC would exceed voltage limits.
This is only in code comments (`motor_pot.h`, `esp32_agent_summary.md`), not
in `hardware.md`.

### 4. Vessel Volume Inconsistency

- `hardware.md`: "~11.3L tank"
- CSTR model fit: V = 9.27 L (`cstr_k_d_model.py`)
- No doc explains which value to use for which purpose

---

## Important Gaps (Reproducibility)

### 5. 106-H Non-Standard D9 Pinout

The 106-H has a non-standard female D9 pinout (documented in the PDF manual
only). Not extracted into any text doc — requires PDF lookup to rewire.

### 6. RS232 Level Shifter Unspecified

The level shifter between ESP32 UART2 (3.3V) and 106-H RS232 (±12V) is
mentioned but the part/model is never documented anywhere.

### 7. Mixing Screw Motor Undocumented

`operating_procedures.md` says "25-30 RPM" but doesn't document: motor type,
power supply, speed control method. `DoubleHelixScrewImpellerDrawing_mm.pdf`
is unlinked.

### 8. O2 Concentrator Make/Model Unspecified

`hardware.md` notes 5 LPM max but no manufacturer/model. The air compressor
is identified as "Wailea" only in a case file, not in hardware docs.

### 9. Gas Path Plumbing Details Missing

T-fitting and check valve configuration (O2 + air → MP-8000 inlet) described
only in `docs/cases/air_compressor_flow_issue.md`. Missing from reference docs.

### 10. Exhaust/Charcoal Filter Undocumented

`interface_contract.md` shows "Exhaust / charcoal filter" as terminal but no
doc describes filter type, capacity, placement, or replacement schedule.

---

## Informational Gaps (Hygiene)

### 11. Orphaned Resources

`PortaSense_D16.pdf` and `DRI0044_TB6612FNG_MotorDriver.pdf` have zero
references. Should be noted as "evaluated and rejected" with rationale.

### 12. Pinout Image Unlinked

`ESP32-DOIT-DEVKIT-V1-Board-Pinout-30-GPIOs-Copy.webp` not referenced from
any doc. Should be linked in `hardware.md`.

### 13. Motor Pot Timing/ADC Constants Missing from Docs

Operationally critical values only in code: 15s timeout, ~8s full travel,
ADC range 100–3996, deadband 20 counts, PWM 1kHz/8-bit.

### 14. Dosimetry Dead Volume Undocumented

`dosimetry.c` defines `DEFAULT_SENSOR_PATH_L = 0.1` (tubing from vessel to
sensor). No measurement basis noted; not cross-referenced to CSTR V_dead.

### 15. Stale POWER_MODEL Constants in Interface Contract

`interface_contract.md` lists `POWER_MODEL_A = 1.78`, `POWER_MODEL_B = 1.40`
with note "(to be replaced by fitted model)." Sigmoid JSON models now in use.
These should be marked `[DEPRECATED]`.

### 16. Stale Handoff Reference

`process_batch_handoff.md` states `operating_procedures.md` does not exist —
it now does.

### 17. Sample Interval Inconsistency

Docs say "~2.5s" throughout but actual interval is "~2–3s depending on 106-H
averaging mode" per `wifi_disconnect_audit.md`.
