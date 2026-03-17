# Case: Air Compressor Flow Degradation & Parallel Flow Failure

> Opened: 2026-03-12
> Status: **OPEN — troubleshooting in progress**
> Hardware affected: Wailea air compressor (internal to MP-8000 ozone generator)

---

## Observed Symptoms

1. **Flow dropped**: Air compressor previously measured ~10 LPM; now delivers <4 LPM when
   run alone (rotameter reading).
2. **No additive flow in parallel**: When O2 concentrator (4 LPM) and air compressor run
   simultaneously through T-fitting with check valves, total flow stays at ~4 LPM. Rotameter
   ball does not bounce when air compressor relay is toggled ON while O2 is running.
3. **Normal behavior when isolated**: Turning O2 OFF and running air compressor alone shows
   3–4 LPM on rotameter.

---

## Root Cause Analysis

### Issue 1: Flow Degradation (10 → <4 LPM)

**Primary hypothesis: Diaphragm degradation from ozone exposure.**

The compressor lives inside the MP-8000 housing, which contains elevated ozone during
operation. Ozone is a strong oxidizer that attacks standard rubber compounds (neoprene,
NBR). Symptoms of ozone-degraded diaphragm:
- Reduced flow at a given back pressure
- Reduced stall pressure (maximum pressure at zero flow)
- Progressive worsening over time
- Visual: tan/brown discoloration, surface cracking, loss of elasticity, micro-pinholes

Secondary hypothesis: Internal reed/flapper valve fatigue or warping (same net effect —
reduced compression efficiency).

### Issue 2: Zero Additive Flow in Parallel Operation

**Cause: Air compressor stall pressure now below system junction pressure.**

Parallel pump physics: two pumps share a junction pressure P_j set by downstream
resistance (MP-8000 generator inlet). Each pump contributes flow only if its stall
pressure exceeds P_j. With O2 running at 4 LPM, P_j is determined by the O2
concentrator's characteristic curve at that flow rate.

If the degraded air compressor's stall pressure < P_j (O2 operating), the compressor's
check valve cannot open — the O2 pressure holds it closed. Measured result: zero
contribution, exactly consistent with observation (no ball movement on rotameter).

When O2 is OFF, P_j drops to just the generator inlet restriction, which the weakened
compressor can overcome → 3–4 LPM observed.

---

## Diagnostic Protocol

Run in order; stop when root cause is confirmed.

1. **Free-delivery test**: Disconnect compressor outlet from T-fitting. Measure flow into
   open air (zero back pressure). If <4 LPM → pump is internally degraded, not a system
   interaction issue.

2. **Stall pressure test**: Block compressor outlet, attach pressure gauge. Record maximum
   developed pressure. Compare to system operating pressure with O2 running at 4 LPM.
   If stall pressure < system P_j → confirms it cannot contribute flow in parallel.

3. **Visual inspection**: Open pump housing. Inspect diaphragm for:
   - Discoloration (tan/brown = ozone degradation)
   - Surface cracking or micro-pinholes
   - Loss of elasticity (stiff/brittle feel)
   - Check reed valves for warping or cracks

4. If diaphragm damaged: attempt diaphragm replacement (if Wailea parts available).
   Use ozone-resistant material: **PTFE, EPDM, or Viton**. Do NOT use neoprene/NBR —
   it will degrade again on the same timeline.

5. If pump is unserviceable or parts unavailable: source replacement compressor.
   Requirements:
   - Stall pressure: ≥ system back pressure + 20–30 cmH₂O margin
   - Flow: ≥ 8–10 LPM at system operating pressure
   - Diaphragm/wetted materials: ozone-resistant
   - Form factor: fits MP-8000 housing (or mounts externally)

---

## Impact on Current Code & Constants

`AIR_COMP_LPM = 10.0` is currently hardcoded in `blocksi_dashboard.py` and used in:
- O2% weighted-average calculation: `compute_effective_o2_pct(flow_lpm, air_comp_on)`
- Mass flow and dosimetry estimates

**This constant is unreliable until hardware is resolved.** Do not run air-blend
calibration or validation until the compressor issue is fixed and flow is re-verified.
The constant should be updated to match actual measured flow once the hardware is
repaired or replaced.

No code changes pending hardware resolution.

---

## Decisions & Next Steps

- [ ] Run free-delivery and stall pressure tests (diagnostic step 1 & 2)
- [ ] Inspect diaphragm (step 3)
- [ ] Determine: repair (diaphragm replacement) vs. replace pump
- [ ] If replacing: evaluate external mounting vs. internal (housing fit)
- [ ] After fix: re-measure free-delivery flow and update `AIR_COMP_LPM` constant
- [ ] Re-run air-blend calibration at new confirmed flow rate

---

## Notes

- The relay interlock (`air_comp` requires `ozone_gen` ON) remains correct — do not
  remove. The electrical dependency is unchanged; this is a fluidic/mechanical issue.
- If a replacement compressor is mounted externally (outside MP-8000 housing), ozone
  exposure risk to the diaphragm is reduced, extending service life.
- Consider sourcing a pump with a spec sheet showing stall pressure curve, not just
  free-delivery LPM, to ensure it can operate in parallel with the O2 concentrator.
