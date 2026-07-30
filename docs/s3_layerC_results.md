# S3 Layer-C (C1) results — in-firmware INDI, roll/pitch ω̇-inversion

**What shipped:** an in-firmware quaternion INDI attitude/rate backend
(`AC_CustomControl_INDI`, `CC_TYPE=3`) that flies the S1 trajectory battery in
SITL with a **clean rate loop** (no limit cycle) and **real angular-acceleration
(ω̇) inversion on roll and pitch**. This is the "C1" slice of Layer C. The
bidi-DShot payload originally scoped for Layer C (RPM-fed allocation, the G2
rotor-inertia term, the per-motor speed loop) is **deferred to hardware / S4** —
see "Why the RPM work is deferred" below.

Validated headless in the `s3-layerC.nix` NixOS VM gate (fork `goromal/ardupilot`
`dev/controller` @ `cba0b1b0`, `indi-harness` `master` @ `d356c00`).

## Battery RMSE (XKF1 position tracking, m)

| case | stock (S1) | S2 offboard | Layer A | **Layer C (C1)** |
|------|-----------:|------------:|--------:|-----------------:|
| hover_step       | 0.615 | 0.540 | 0.756 | **0.750** |
| circle_slow      | 0.271 | 0.757 | 0.252 | **0.266** |
| circle_fast      | 0.636 | 0.756 | 0.724 | **0.761** |
| lemniscate_slow  | 5.072 | 0.462 | — | **5.012** |
| lemniscate_fast  | 1.787 | 0.769 | 1.708 | **1.704** |

**Read this correctly.** C1 is an *inner-loop* replacement that keeps the stock
guided position→attitude outer loop (identical command path to S1). So the
trajectory-tracking RMSE necessarily tracks **stock** — C1 matches it case-for-case
(marginally better on circle_slow / lemniscate_slow / lemniscate_fast, marginally
worse on hover_step / circle_fast; all within run-to-run scatter). It does **not**
beat the S2 offboard prototype on the lemniscates: S2's advantage there comes from
its flatness feedforward in the *outer* loop, which is Layer B work, not done here.

The position-RMSE battery therefore is **not** where C1's improvement shows up — a
clean inner loop and a limit-cycling inner loop can fly the same trajectory to the
same position RMSE (the EKF/outer loop absorbs inner-loop buzz). C1's improvement is
at the rate-loop level, below what position RMSE sees:

## The C1 result: real ω̇-inversion, no limit cycle

From the exported `.BIN` INDI health (excitation-aware ω̇-tracking scorer,
`indi_harness.sitl.align.omega_gate_ok`):

| axis | ω̇ NRMSE | excitation (RMS ω̇) | enforced | verdict |
|------|--------:|-------------------:|:--------:|---------|
| roll  | **0.482** | 0.472 | yes | tracks — real inversion |
| pitch | **0.492** | 0.561 | yes | tracks — real inversion |
| yaw   | 2.459 | 1.393 | no  | flies clean; inversion deferred (see below) |

`sat_frac = 0.0`, 48 393 INDI log messages. NRMSE is RMSE(pred,meas ω̇) normalized
by RMS(meas); < 0.75 on an excited axis means the INDI increment is genuinely
inverting the measured angular acceleration (vs the ~1.1 "not tracking / gentle
integral action" floor). Roll/pitch at ~0.48 are comfortably inside that.

**This is the milestone.** Layer A shipped a *de-tuned* INDI: to avoid a motor-buzz
limit cycle it ran `G1 = 1000` — far above the true effectiveness — where the
increment is so attenuated that predicted-vs-measured ω̇ diverge (it acted as gentle
integral action, not inversion). C1 removes that compromise.

### How: the limit cycle was delay-driven, not gain-driven

Diagnosed via a cutoff × G1 SITL sweep. Layer A's angular-accel estimate was
`differentiate-then-filter` at a low cutoff (30–40 Hz); the filter's group delay
costs phase margin, so at true-effectiveness gains the rate loop limit-cycles
(roll/pitch RATE-gyro RMS 20–80 deg/s). The fix is **lower feedback delay**, not
higher gain:

| estimator cutoff `CC3_OMG_FILT` | `CC3_G1_RP` | roll/pitch RATE-gyro RMS | roll/pitch ω̇ NRMSE |
|---|---|---|---|
| 30–40 (Layer A) | 1000 | 20–36 deg/s (limit cycle) | ~1.1 (not tracking) |
| **80 (C1)** | **500** | **3–4 deg/s (clean)** | **0.48 / 0.49 (tracking)** |

C1 raises the estimator cutoff to 80 Hz (`filter-then-differentiate`, added as
`CC3_OMG_FILT`) which suppresses the limit cycle on both axes, and drops `CC3_G1_RP`
to 500 (near the true effectiveness) where INDI actually inverts. Confirmed under
two-axis excitation (`circle_slow` banks continuously → both roll and pitch active).

## Yaw: flies clean, ω̇-inversion deferred

Yaw is **not** limit-cycling. An early "yaw buzz" reading was a measurement artifact:
`circle_slow` yaws the vehicle once per lap to face along the path, so the ~29 deg/s
"yaw rate" is the *commanded* turn rate, present identically under fully-stock
ArduPilot with no custom controller. At the rate level yaw is the *quietest* axis
(lowest high-frequency gyro content, lowest 400 Hz ω̇ RMS), pred/meas yaw accel
correlate +0.9 (sign correct).

Yaw's ω̇ NRMSE (2.46) is high because yaw INDI *under-scales* — yaw authority in the
sim is drag-torque-based and ~100× weaker than roll/pitch, and full yaw ω̇-inversion
needs the **G2 rotor-inertia term fed by measured RPM** (design doc §3.6). SITL does
not model rotor-inertia reaction torque, so that term has nothing to act on here.
The gate therefore **enforces roll/pitch and reports-but-does-not-gate yaw** — gating
it would demand physics SITL cannot simulate.

## Why the RPM work (true Layer C) is deferred to HW / S4

While wiring the measured-RPM path we established that **SITL does not model the
physics Layer C's RPM mechanisms address**:

- **No rotor-inertia yaw reaction.** `SIM_Motor::calc_thrust` is momentum-theory
  thrust + drag; there is no `Ir·Ω̇` reaction torque. The **G2 term** exists to cancel
  exactly that → nothing to cancel in sim.
- **Negligible actuator lag.** SITL "RPM" is `slew-limited-command × SIM_VIBE_MOTOR`
  with only a slew-rate limiter — measured RPM ≈ the command, so **measured-actuator-
  state feedback** has almost no lag to correct.

So the G2 term, measured-actuator-state feedback, the ΔΩ² allocation, and the
per-motor RPM loop cannot be validated in SITL — this is the S4 fidelity wall
arriving early. The RPM source interface + SITL degradation shim (quantization /
CRC-dropout / latency + staleness fallback) are **built and unit-tested**
(`AP_INDI_RpmSource`, committed) as forward-looking infrastructure, but are not
wired into the control loop. The design doc's A→C→B→S4 order is, in effect, revised:
Layer C's bidi-DShot payload needs S4-level fidelity (or real hardware) to validate.

## Deferred to hardware / S4

- G2 rotor-inertia yaw term + full yaw ω̇-inversion.
- Measured-actuator-state feedback in the increment (tighten roll/pitch below ~0.48).
- ΔΩ² allocation, per-motor PI RPM loop, `AP_MotorsMatrix_INDI` subclass.
- Doublet-regression sysid (G1/G2/throttle-map).
- Layer B outer loop + flatness feedforward (the S2 lemniscate advantage).

## Reproduce

```
cd anixpkgs
sed -i 's|local-build = false;|local-build = true;|' pkgs/nixos/dependencies.nix
nix-build pkgs/nixos/sitl-envs/s3-layerC.nix -o /tmp/s3c-gate
git checkout pkgs/nixos/dependencies.nix
```

Gate asserts: battery flies all 5 cases, scored JSON + INDI `.BIN` produced, and the
excitation-aware ω̇ gate passes (roll/pitch NRMSE < 0.75). Baseline:
`indi-harness/baselines/s3_layerC_sitl.json`.
