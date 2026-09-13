# JSON rotor-dynamics simulator: validation and controller measurements

The simulator provides the non-ideal actuator dynamics needed to evaluate INDI:
first-order motor lag, rotor mixing, rotor-inertia yaw reaction, and optional
linear rotor drag. It speaks ArduPilot's JSON SITL protocol over UDP port 9002,
with NED world coordinates and FRD body coordinates.

This page records the initial August experiments. See the
[current controller audit](2026-09-12-controller-audit.md) for the later
mechanical-RPM and freshness fixes, normalized gain checks, and passing
small-flight regression. A trajectory-level drag-rejection advantage remains
unproven.

## Simulator implementation

`indi_harness.sitl.jsonsim` composes a body wrench for the pysignals rigid-body
integrator from four motor commands. Motor time constant is 30 ms; rotor drag
is `F_drag = -R diag(kx,ky,kz) R^T v`, enabled with `--drag`.

- Identical rotor-speed inputs reproduce the independent NumPy `QuadSim`
  within 2e-2 m position RMSE and 1e-2 rad attitude error, with drag on or off.
- Lockstep stepping makes the physics deterministic.
- Newline-terminated replies match the JSON parser's framing.
- An optional ground-contact constraint prevents a disarmed vehicle from
  free-falling before takeoff.
- The backend's hover throttle is approximately 0.30; tests configure this
  explicitly and disable hover learning.

## Stock-controller frame and sign validation

The NixOS VM test `indi-drag-rejection.nix` passes:

- Takeoff to 10 m reports local NED z approximately -10 m.
- A north command gives +33.2 m north displacement with negligible east motion.
- An east command gives +33.2 m east displacement with negligible north motion.
- Hover remains stable with no runaway.

## Historical INDI actuator-lag measurements

These flights used the legacy stock-PID-baseline INDI controller with its
flatness outer loop, stock collective control, and drag disabled. Controller
engagement was confirmed by status text and nonzero INDI increments.

| Metric | Estimator cutoff 80 Hz | Estimator cutoff 160 Hz | Ideal-actuator baseline |
|---|---:|---:|---:|
| Circle tracking RMS | 10.5 m | 3.03 m | 0.485 m |
| Fast figure-eight tracking RMS | 4.86 m | 1.15 m | 0.711 m |
| Saturation fraction | 7–13% | 2.9% | 0% |
| High-pass roll/pitch gyro RMS | — | ~50/50 deg/s | Low |
| Roll/pitch acceleration NRMSE | — | ~1.5 | — |

Raising the estimator cutoff improved tracking but did not close the circle
tracking or acceleration-inversion gates. The August interpretation attributed
this to the actuator-state feedback path. Later ownership and gain-unit
findings mean that interpretation is not a unique, established cause.

## Reproduction

From anixpkgs, with temporary `dependencies.nix local-build=true`:

```sh
nix-build pkgs/nixos/sitl-envs/indi-drag-rejection.nix
nix-build pkgs/nixos/sitl-envs/indi-drag-rejection-indi.nix
nix-build pkgs/nixos/sitl-envs/indi-drag-rejection-indi-omg160.nix
```

The first is a green simulator frame check. The other two retain diagnostic
measurements, not passing controller acceptance gates. Restore
`local-build=false` before committing.
