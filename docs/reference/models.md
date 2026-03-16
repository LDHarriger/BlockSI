# Model Reference

> Load this file when working on power-O3 model or CSTR model code.

---

## Power-O3 Sigmoid Model

- **Equation**: `O3 = L / (1 + exp(-k*(P - P0))) + b`
- **Type**: 4-parameter sigmoid fitted per (flow_lpm, o2_pct) condition
- **Dataclass**: `PowerO3Model(L, k, P0, b, flow_lpm, o2_pct, r_squared, rmse, n_points, ci_power, ci_sigma)`
- **CI bands**: 101-point Jacobian-propagated ±1σ (from `pcov`), stored in model JSON
- **Load safety**: `load_model()` filters unknown JSON keys via `__dataclass_fields__`
- **Storage**: `Interfaces/Models/O3Power/*.json` (one file per LPM/O2% condition)
- **Analysis module**: `Interfaces/PC/analysis/power_o3_model.py`

---

## CSTR (Vessel Fill/Evacuation) Model

- **ODE**: `dC/dt = (C_in - C)/τ - k_d·C`
- **Parameters**: `V` (gas volume L), `k_d` (decay rate s⁻¹), `V_dead` (dead volume L)
- **Universality**: Single model for all conditions (flow-rate independent)
- **Storage**: `Interfaces/Models/CSTR/cstr_model.json`
- **Fitted via**: `scipy.optimize.curve_fit` in `Interfaces/PC/analysis/fill_model.py`
- **Calibration data**: `Interfaces/Data/CSTR/` (combined CSV with `phase` column: baseline, fill, evac)

---

## Key Constants

```python
O3_MASS_FLOW_K   = 0.3327   # mg/s per (%vol × LPM), V_m=24.04 L/mol at 20°C
AIR_COMP_LPM     = 10.0     # LPM added when air compressor on
DEFAULT_FLOW_LPM = 4.0      # Default O2 flow rate
O2_CONC_PCT      = 95       # O2 concentrator purity
AIR_COMP_O2_PCT  = 21       # Atmospheric O2
TCP_PORT         = 5000
```
