# S4 Phase 1 — Aero-fidelity platform + INDI-under-actuator-lag finding

**Status:** complete (2026-08-24). Phase 1 delivers a custom aero-fidelity SITL platform and a controller finding; the drag-rejection A/B demonstration is deferred to Phase 2 (see "Conclusion").

## Summary

S4 set out to demonstrate INDI's drag-rejection advantage over stock PID on a physics model realistic enough to *have* drag — something benign SITL lacks. Phase 1 built that platform (a custom ArduPilot-SITL JSON physics backend on the `pysignals` rigid-body library) and validated it end-to-end. In the process it established a load-bearing finding: **the shipped in-firmware Layer-B INDI controller limit-cycles on the realistic backend because of first-order actuator lag the benign SITL never modeled.** Inner-loop retuning only partially helps. This is exactly the failure mode that measured-rotor-speed actuator feedback (Layer-C C2/C3) is designed to remove, and it quantitatively motivates Phase 2.

## The platform

A Python process (`indi_harness.sitl.jsonsim`) speaks ArduPilot's SITL JSON UDP protocol on port 9002 and integrates a quad rigid body with `pysignals.RigidBody6DOFModel` (NED world, FRD body). Per frame it maps the four motor PWMs to a body wrench — rotor mixer + first-order motor lag (`tau_m=0.03 s`) + Faessler linear rotor drag (`F_drag=−R·diag(kx,ky,kz)·Rᵀ·v`, toggled `--drag/--no-drag`) — and replies the JSON state (position, velocity, quaternion, gyro, `accel_body`).

- **Oracle-validated.** Driven by identical rotor-speed sequences, the backend reproduces the S0 numpy `QuadSim` (`simmodel.py`) trajectory to < 2e-2 m position RMSE / < 1e-2 rad attitude, drag on and off.
- **Deterministic + CI-gated.** Runs in SITL lockstep inside a NixOS-VM test, like the rest of the project's gates.
- **Bring-up bugs found and fixed** (real integration issues benign SITL never exposes): JSON replies must be newline-terminated (`SIM_JSON` splits on `\n`); a disarmed (motors-off) rigid body free-falls into an EKF NaN, so an opt-in ground-contact constraint holds it on the pad until thrust exceeds weight; the backend's true hover throttle (~0.30) differs from ArduPilot's default `MOT_THST_HOVER=0.39`, so the gate sets `MOT_THST_HOVER 0.30`, `MOT_HOVER_LEARN 0`.

## Result 1 — frame/sign validation (stock PID, drag off): PASS

Stock ArduCopter flies on the backend with correct frames (green NixOS-VM gate `indi-drag-rejection.nix`):

- takeoff to 10 m → `LOCAL_POSITION_NED z = −10.0` (up = −D, correct)
- NORTH command → dN +33.2 m, dE ≈ 0 (no N/E swap)
- EAST command → dE +33.2 m, dN ≈ 0
- stable hover, no runaway

This proves the platform is a faithful drop-in for stock control before any controller question is asked.

## Result 2 — INDI under realistic actuator lag: BUZZES (the finding)

Flying the *shipped* Layer-B INDI (`CC_TYPE=3`, `CC3_OUTER_EN=1`, attitude-only) on the backend, **drag off**, over the trajectory battery. Engagement is proven, not assumed — the custom controller emits `"Custom controller is ON"` and the logged INDI increment is active (`du_active_rms ≈ 0.09`), so these are genuine INDI flights, not accidental stock.

Buzz metrics use the trustworthy 50 Hz IMU gyro and 400 Hz INDI message (the 10 Hz `RATE`/`ATT` log messages alias and are unreliable for this).

| case | shipped `OMG_FILT=80` | retuned `OMG_FILT=160` | benign-SITL baseline | verdict (omg160) |
|---|---|---|---|---|
| circle_slow track_rms | 10.5 m | **3.03 m** (6.3×) | 0.485 m | **fails** (> 1.5 m tol) |
| lemniscate_fast track_rms | 4.86 m | **1.15 m** (1.6×) | 0.711 m | clean-drop-in |
| saturation fraction | 7–13% | **2.9%** | 0% | improved, non-zero |
| IMU-gyro buzz (roll/pitch hp-rms) | — | **~50 / 50 °/s** | low | still buzzing |
| ω̇-inversion NRMSE (roll/pitch) | — | **~1.5** (r² < 0) | — | not real inversion |

**Reading:** the 30 ms actuator lag adds feedback group delay that the C1 rate loop — whose `OMG_FILT=80` cutoff was tuned for *benign* SITL — cannot absorb, producing a ~limit-cycle under trajectory tracking. Raising `OMG_FILT` to 160 restores some phase margin (circle_slow 10.5→3.03 m, saturation halved, INDI still active — *not* the degenerate G1-inflation/inert regime), and `lemniscate_fast` becomes acceptable, but `circle_slow` still limit-cycles at 6.3× baseline and the ω̇-inversion still does not track (NRMSE ~1.5, r² < 0). Lowering the rate-loop gain also quiets the buzz but drives the INDI increment inert — the wrong fix.

Inner-loop retuning alone cannot make the shipped INDI a clean, *active* drop-in on realistic actuator dynamics.

## Conclusion — why the drag A/B is Phase 2

Linear rotor drag is a translational disturbance; INDI rejects it through the outer-loop specific-force increment, which only works if the inner loop faithfully delivers the commanded torque. On the realistic backend the inner loop does not — it limit-cycles — so a drag-rejection A/B cannot be cleanly demonstrated in Phase 1. This is not a dead end; it is the S4 thesis made concrete:

> The destabilizer is the **actuator-state side** of the INDI increment. Layer-A/B/C1 estimate actuator state from the *previous mixer command* with no lag model; benign SITL hid the cost because it has no real actuator lag. The realistic backend exposes it. **Layer-C C2/C3 — measured per-motor rotor speed feeding the increment, plus the G2 rotor-inertia term — is precisely the fix.**

**Phase 2** extends this same platform (add rotor inertia + synthetic bidi-eRPM) and implements C2/C3, then returns to the drag-rejection A/B with an inner loop that can actually fly it.

## Artifacts

- Backend: `indi_harness/sitl/jsonsim/` (`model.py`, `protocol.py`, `__main__.py`); tests `tests/test_jsonsim_*.py`.
- Engage helper + trustworthy buzz reader: `indi_harness/sitl/baseline_cc.py` (LOW→HIGH + STATUSTEXT confirm), `read_imu_gyro`.
- NixOS-VM gates (anixpkgs `dev/indi-s4-drag`): `indi-drag-rejection.nix` (stock frame gate, green), `indi-drag-rejection-indi.nix` + `-omg160.nix` (INDI-on-backend diagnostic + scorer encoding the finding).
- Retune sweep data: `scratchpad/s4_retune_sweep.csv`.
