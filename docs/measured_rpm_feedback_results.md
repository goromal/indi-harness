# Measured-RPM feedback: implementation and historical flight results

The contribution is a rotor-inertia-aware JSON simulator, a per-motor RPM
telemetry path, measured-actuator feedback in the INDI rate controller, and
effectiveness identification and flight-scoring tools. It does not include
native rotor-speed allocation or per-motor speed control.

For current acceptance results and remaining work, read the
[controller audit](2026-09-12-controller-audit.md).

## Implemented behavior

- `QuadJsonModel` includes rotor-inertia yaw reaction. Against the independent
  NumPy model, attitude RMSE improved from 6.4e-4 to 3.3e-6 rad. A deliberately
  increased-inertia test distinguishes the implemented reaction from omission.
- The JSON reply publishes mechanical RPM in `rpm_1` through `rpm_4`.
  `SIM_JSON` updates each motor's `AP_ESC_Telem` sample independently.
- `CC3_USE_RPM=1` reconstructs normalized actuator state by projecting
  squared rotor speeds onto the stock mixer's factors and dividing by their
  squared norm. `CC3_G2_YAW` adds rotor-inertia compensation. Unhealthy or stale
  RPM causes fallback to the current stock PID output.
- `INDC` logs measured rotor state and reconstructed feedback; `INDU` separates
  current PID, previous custom output, and new custom output.
- `sysid.py` distinguishes physical torque effectiveness (`1/J`) from
  normalized firmware-command effectiveness and fits recorded simulator state.

## August flight measurements

These experiments used measured RPM, drag disabled, `CC3_G2_YAW=0`, and
`CC3_OMG_FILT=80`. They establish failed flights, not a unique cause.

| Configuration | Circle tracking RMS | Peak altitude | Saturation | Early roll reconstruction |
|---|---:|---:|---:|---|
| Legacy PID-baseline INDI | 3–10.5 m | ~10 m | 3–13% | — |
| Measured RPM, corrected projection | 138 m | 865 m | 16–21% | slope 0.73, correlation 0.92 |
| Measured RPM, linearized thrust map | 2444 m | 11000 m | 96–100% | slope 0.99, correlation 0.96 |

The initial interpretation was that measured-feedback loop structure required
a native rotor-speed allocator. The September audit found that interpretation
was confounded: the diagnostic compared against current PID rather than the
previous custom command, yaw effectiveness used incompatible scaling, and RPM
freshness and units needed correction. A controlled old-firmware experiment
flies with normalized yaw effectiveness 28.8 and diverges when only that value
is changed to 1000. Therefore the August results do not establish that native
rotor-speed allocation is necessary.

## Reproduction and acceptance

From anixpkgs:

```sh
nix-build pkgs/nixos/sitl-envs/indi-actuator-probe.nix -A flight
# Larger trajectory experiment; requires temporary local-build=true:
nix-build pkgs/nixos/sitl-envs/indi-measured-rpm-tracking.nix
```

The small flight is a passing regression, not a substitute for trajectory
acceptance. The latest larger attempt had an unstable stock handover before
custom control engaged. Establish a settled stock GUIDED hold on the linearized
map before repeating trajectories and the PID/INDI drag comparison.
