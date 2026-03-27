# Hardware Reference

> Load this file when working on GPIO, relay, or gas path related code.
> Source of truth for pin assignments: `Interfaces/ControlSystem/main/blocksi_pins.h`

---

## Gas Path Topology

```
O2 concentrator (+ optional air compressor)
  → MP-8000 ozone generator
    → Vessel (~11.3L tank, ~60% fill → ~4L residual gas volume)
      → 106-H sensor (outlet, RS232)
```

L-valve switchable for direct-to-sensor bypass (used during validation).

---

## Key GPIO Assignments (from `blocksi_pins.h`)

| Bus/Function | GPIOs | Connected to |
|-------------|-------|--------------|
| I2C | 21, 22 | DFRobot O3 sensor (100kHz) |
| SPI | 18, 19, 5 | MAX31855 thermocouple |
| UART2 | 16, 17 | 106-H RS232 (via level shifter) |
| Relays | 12, 13, 14 | SSR control (3 relays) |
| Motor Pot | 25, 26, 34 | DRV8833 H-bridge + ADC feedback |

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

## Sensor Details

- **106-H Ozone Monitor**: RS232 via UART2 (GPIOs 16/17), mounted at vessel outlet. ~2.5s sample interval.
- **DFRobot O3 Sensor**: I2C (GPIOs 21/22, 100kHz), room ambient ozone. I2C speed must be 100kHz (not 400kHz).
- **MAX31855 Thermocouple**: SPI (GPIOs 18/19/5), vessel temperature.
- **Motor Potentiometer**: DRV8833 H-bridge (GPIOs 25/26) + ADC feedback (GPIO 34). Controls MP-8000 power level.
