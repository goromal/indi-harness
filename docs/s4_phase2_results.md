# S4 Phase 2 — Measured-RPM actuator feedback platform + the C2-structural-instability finding

**Status:** complete (2026-08-28). Phase 2 delivers the measured-RPM platform extension + C2 firmware + a load-bearing controller finding; the full C3 rotor-speed² allocation is deferred to Phase 3 (see "Conclusion").

## Summary

Phase 2 set out to make the INDI inner loop lag-robust by feeding the *measured* rotor speed into the increment (Layer-C C2), and thereby close the Phase-1 actuator-lag buzz. It built the whole measured-RPM path — backend rotor-inertia + synthetic bidi-eRPM, a sim-only `SIM_JSON`→`AP_ESC_Telem` channel, the C2 firmware, and the G1/G2 sysid — all unit-verified. In flight it established a load-bearing finding that **inverts the Phase-1 result's implied fix**: reconstructing the measured actuator state in the stock mixer's *torque space* and feeding it as the INDI operating point does not merely fail to help — it **destabilizes the loop into divergence**. The instability is structural (a phase/gain-margin property of the increment), not a reconstruction bug, and it is the concrete, quantitative motivation for the full C3 allocation.

## The platform extension

- **Rotor-inertia yaw reaction (`Ir`)** enabled in the pysignals JSON backend (`QuadJsonModel._wrench`), matching the S0 oracle `QuadSim` (Phase 1 pinned `Ir=0`). Oracle parity *improved* — attitude RMSE 6.4e-4 → 3.3e-6 rad vs the oracle — and a load-bearing test proves the term is real (cranked-Ir parity 3.2e-3 with the term vs 0.64 without).
- **Synthetic bidi-eRPM** emitted per motor (from the true lagged `self._omega`) as top-level `rpm_1..rpm_4` scalars in the JSON reply.
- **Sim-only RPM channel:** the `goromal/ardupilot` fork's `SIM_JSON` parses those four scalars and publishes them to `AP_ESC_Telem` (`DataKey` bits 1ULL<<36..39), lighting up the real `AP_INDI_RpmSource_ESC` path (unpopulated under JSON SITL until now). Zero flight-code impact.
- **Sysid** (`indi_harness.sysid`) recovers the effectiveness `G1 = 1/J = [200, 200, 111]` rad/s²/(N·m) and rotor-inertia `G2 = Ir = 5e-7` from doublet excitation through the oracle, matching the analytic seed (non-circular: `identify_g1` never references `J`, `identify_g2` never references `Ir`).

## The C2 firmware

`AC_CustomControl_INDI` gains `CC3_USE_RPM` (index 17): when set, `u0` is reconstructed from measured rotor speed instead of the previous mixer command — `u_meas = Bᵀ·(Ω²_meas/Ω²_max)` projected onto the stock mixer's own per-motor factors and normalized by `Σf²` — plus a `CC3_G2_YAW` rotor-inertia yaw term, plus an `INDC` `.BIN` message logging per-motor Ω/Ω̇, the reconstruction, the stock previous-command (diagnostic `Cx/Cy/Cz`), and the fallback flag. `CC3_USE_RPM=0` is byte-for-byte the shipped path; gtests 19/19.

## The finding — C2-in-torque-space diverges (not buzzes)

Flying C2 (`CC_TYPE=3`, `CC3_USE_RPM=1`, `CC3_G2_YAW=0`, `CC3_OMG_FILT=80`) on the backend, drag off, over the battery. Engagement proven (STATUSTEXT "ON" + active `du`), RPM channel live (`INDC` fallback_frac = 0.000, mean|Ω| ≈ 670 rad/s).

| config | circle_slow track_rms | peak alt | saturation | early-window roll recon | divergence onset |
|---|---|---|---|---|---|
| shipped INDI (Phase-1 ref) | 3–10.5 m (**buzz**) | ~10 m | 3–13% | — | — (bounded) |
| **C2, scale-fixed, normal MOT** | **138 m** | **865 m** | 16–21% | slope 0.73, corr 0.92 | ~900 ms |
| **C2, + thrust linearization** | **2444 m** | **11000 m** | 96–100% | slope **0.99**, corr 0.96 | **~30 ms** |

**Reading.** Two principled fixes were applied and neither stopped the divergence:
1. A **genuine reconstruction scale bug** — hardcoded ±0.5 factors without the `/Σf²` projection normalization, doubling the roll/pitch operating point — was found and fixed (project onto the mixer's own `get_roll_factor/get_pitch_factor` and divide by `Σf²`). Still diverged.
2. **Thrust-map linearization** (`MOT_THST_EXPO 0` + `MOT_SPIN_MIN/MAX` so `o2n = (pwm−1000)/1000` equals the mixer's `thrust_rpyt_out` exactly). This made the roll reconstruction *faithful* — early-window slope 0.73 → **0.99**, confirming the residual scale distortion was the `≈0.8` spin gain + EXPO curve — yet the loop diverged **faster** (900 ms → 30 ms).

Windowing the `.BIN` to *before* divergence (stock command `INDC.Cx/Cy/Cz` vs reconstruction `Ux/Uy/Uz`) is decisive: **the reconstruction recovers the command** (roll corr 0.92, slope→0.99), so reconstruction fidelity is *not* the cause. The divergence is a **loop-structure instability**: feeding the *measured, lagged* actuator state into `u_cmd = LPF(u_act) + G1⁻¹(ν − ω̇_meas)` puts the actuator lag inside `u_filt`. The previous-command path avoids that lag — which is precisely why the shipped path merely *buzzes* (bounded limit cycle) instead of diverging. Making `u0` "more correct" (the measured physical state) removes the very term that was providing the loop its phase margin.

## Conclusion — why the full C3 is Phase 3

Linear-in-torque-space measured feedback is the wrong place to inject the measured actuator state, because the C1 rate loop was structured and tuned around an *unlagged* previous-command `u0`. The fix is not more reconstruction tuning; it is the **C3 architecture**: measured Ω² feeding a native rotor-speed² **allocation** `ΔΩ² = G1⁻¹(Δτ − G2·Ω̇)` with `Ω²_cmd = Ω²_meas + ΔΩ²` and a per-motor PI RPM loop, so the rate loop is never asked to invert through a lagged measured feedback path. This is the spec's pre-designed C2-insufficient → C3 contingency (Layer-C spec §2/§3), and per the 2026-08-28 user decision it becomes **Phase 3** (own spec→plan→build).

This is not a dead end; like the Phase-1 reframe it is the thesis made concrete:

> The measured actuator state is the right signal, but torque-space reconstruction fed into the increment is the wrong mechanism — it destabilizes the loop the previous-command estimate kept (barely) stable. Only a proper rotor-speed² allocation, where Ω² is the native control variable, can use the measured state without asking the command-tuned rate loop to invert through lag.

## Artifacts

- Backend ext: `indi_harness/sitl/jsonsim/model.py` (Ir), `.../protocol.py` + `params.py` (eRPM); tests `test_jsonsim_oracle.py`, `test_jsonsim_protocol.py`.
- RPM channel (ardupilot `dev/controller`): `libraries/SITL/SIM_JSON.{h,cpp}`.
- C2 firmware: `libraries/AC_CustomControl/AC_CustomControl_INDI.{h,cpp}` (`CC3_USE_RPM`, `measured_actuator_torque`, `g2_yaw_correction`, `INDC` log); gtests `tests/test_indi_math.cpp`.
- Sysid + scorer: `indi_harness/sysid.py` (`identify_g1/g2`), `indi_harness/buzz_score.py`, `indi_harness/sitl/binlog.py` (`read_indc_health`).
- Diagnostic env (fail-by-design): `anixpkgs/pkgs/nixos/sitl-envs/indi-s4-phase2-c2.nix` + `-score.py` (with the U-vs-C reconstruction diagnostic).
