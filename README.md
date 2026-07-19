# indi-harness

Quaternion INDI prototype and trajectory harness for ArduPilot. This package implements the S0/S1 deliverable of the ArduPilot INDI plan: an offline control-law prototype using quaternion-based attitude control with a trajectory harness for testing and validation.

## Conventions

- **World frame:** North-East-Down (NED)
- **Body frame:** Forward-Right-Down (FRD)
- **Quaternions:** Hamilton scalar-first `[w, x, y, z]`, representing rotations from body frame to world frame
- **Dependencies:** Pure NumPy (no ROS/ArduPilot required for this stage)

## Running Tests

```bash
nix-shell --run "pytest tests/ -v"
```

All tests use Hamilton quaternion conventions and NED/FRD coordinate frames established in `indi_harness.quat`.

## S1 stock-controller SITL baseline

Tracking RMSE of the **stock** ArduPilot controller flying the S1 battery in
the headless SITL VM (EKF XKF1 positions from the `.BIN` dataflash, scored on
the trajectory timeline). These are baselines to compare the INDI controller
against in S3 — not pass/fail bars. Repeatability-checked across two fresh VM
runs with `scripts/compare_baselines.py` (per-case tolerance
`max(0.15 m, 30%)`).

| Case | Period [s] | RMSE run 1 [m] | RMSE run 2 [m] |
|------|-----------:|---------------:|---------------:|
| hover_step | — | 0.615 | 0.608 |
| circle_slow | 12 | 0.271 | 0.246 |
| circle_fast | 8 | 0.636 | 0.730 |
| lemniscate_slow | 12 | 5.072 | 5.053 |
| lemniscate_fast | 8 | 1.787 | 1.634 |

Committed artifact: `baselines/s1_stock_sitl.json` (run 1).

```
Reproduce: cd anixpkgs && nix-build pkgs/nixos/sitl-envs/s1-baseline.nix
```
