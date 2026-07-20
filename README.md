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

## S2 offboard SITL results

Tracking RMSE of the **offboard** ROS 2 outer loop (`indi_harness.offboard`)
flying the same battery through the stock inner loop via `SET_ATTITUDE_TARGET`
(MAVLink 5790, `GUID_OPTIONS 8`), consuming `/ap/*` DDS state. Scored by the
**same** XKF1 `.BIN` evaluator as S1, so the columns are directly comparable.
Repeatability-checked across two fresh VM runs (`compare_baselines.py`,
exit 0).

| Case | S1 stock [m] | S2 offboard run 1 [m] | S2 run 2 [m] | Δ vs S1 |
|------|-------------:|----------------------:|-------------:|--------:|
| hover_step | 0.615 | 0.540 | 0.488 | −16% |
| circle_slow | 0.271 | 0.757 | 0.756 | +179% |
| circle_fast | 0.636 | 0.756 | 0.799 | +22% |
| lemniscate_slow | 5.072 | 0.462 | 0.443 | −91% |
| lemniscate_fast | 1.787 | 0.769 | 0.803 | −56% |

The offboard loop flies stably at a consistent 0.44–0.80 m across all cases
(vs S1's 0.27–5.07 m swing): the flatness feed-forward wins big on aggressive
trajectories, while the ~27.7 Hz / ~35 ms offboard command cadence imposes a
~0.5–0.8 m floor that costs accuracy on gentle ones. The battery flies with
the flatness PD+ff path (`--no-indi`); the INDI acceleration increment is
unstable offboard under command-path latency (carried to S3). Full analysis:
[`docs/s2_latency_attribution.md`](docs/s2_latency_attribution.md).

Committed artifact: `baselines/s2_offboard_sitl.json` (run 1).

```
Reproduce: cd anixpkgs && nix-build pkgs/nixos/sitl-envs/s2-offboard.nix
```
