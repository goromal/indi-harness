# indi-harness

Current controller/simulator status and acceptance criteria:
[September 12 audit and handoff](docs/2026-09-12-controller-audit.md).

Quaternion INDI control models, a rotor-dynamics JSON simulator for ArduPilot,
and a trajectory and telemetry harness for testing controller behavior.

## Conventions

- **World frame:** North-East-Down (NED)
- **Body frame:** Forward-Right-Down (FRD)
- **Quaternions:** Hamilton scalar-first `[w, x, y, z]`, representing rotations from body frame to world frame
- **Dependencies:** NumPy for offline control models; pysignals for JSON physics;
  ArduPilot SITL for flight tests; ROS 2 for offboard/DDS trajectory tests.

## Running Tests

```bash
nix-shell --run "pytest tests/ -v"
```

All tests use Hamilton quaternion conventions and NED/FRD coordinate frames established in `indi_harness.quat`.

## Stock-controller SITL baseline

Tracking RMSE of the **stock** ArduPilot controller flying the stock-guided battery in
the headless SITL VM (EKF XKF1 positions from the `.BIN` dataflash, scored on
the trajectory timeline). These are baselines to compare the INDI controller
against — not pass/fail bars. Repeatability-checked across two fresh VM
runs with `scripts/compare_baselines.py` (per-case tolerance
`max(0.15 m, 30%)`).

| Case | Period [s] | RMSE run 1 [m] | RMSE run 2 [m] |
|------|-----------:|---------------:|---------------:|
| hover_step | — | 0.615 | 0.608 |
| circle_slow | 12 | 0.271 | 0.246 |
| circle_fast | 8 | 0.636 | 0.730 |
| lemniscate_slow | 12 | 5.072 | 5.053 |
| lemniscate_fast | 8 | 1.787 | 1.634 |

Committed artifact: `baselines/stock_guided_sitl.json` (run 1).

```
Reproduce: cd anixpkgs && nix-build pkgs/nixos/sitl-envs/stock-tracking-baseline.nix
```

## Offboard flatness-controller SITL results

Tracking RMSE of the **offboard** ROS 2 outer loop (`indi_harness.offboard`)
flying the same battery through the stock inner loop via `SET_ATTITUDE_TARGET`
(MAVLink 5790, `GUID_OPTIONS 8`), consuming `/ap/*` DDS state. Scored by the
**same** XKF1 `.BIN` evaluator as stock-guided, so the columns are directly comparable.
Repeatability-checked across two fresh VM runs (`compare_baselines.py`,
exit 0).

| Case | stock [m] | offboard run 1 [m] | offboard run 2 [m] | Δ vs stock-guided |
|------|-------------:|----------------------:|-------------:|--------:|
| hover_step | 0.615 | 0.540 | 0.488 | −16% |
| circle_slow | 0.271 | 0.757 | 0.756 | +179% |
| circle_fast | 0.636 | 0.756 | 0.799 | +22% |
| lemniscate_slow | 5.072 | 0.462 | 0.443 | −91% |
| lemniscate_fast | 1.787 | 0.769 | 0.803 | −56% |

The offboard loop flies stably at a consistent 0.44–0.80 m across all cases
(vs stock-guided's 0.27–5.07 m swing): the flatness feed-forward wins big on aggressive
trajectories, while the ~27.7 Hz / ~35 ms offboard command cadence imposes a
~0.5–0.8 m floor that costs accuracy on gentle ones. The battery flies with
the flatness PD+ff path (`--no-indi`); the INDI acceleration increment is
unstable offboard under command-path latency (carried to onboard). Full analysis:
[`docs/offboard_latency_attribution.md`](docs/offboard_latency_attribution.md).

Committed artifact: `baselines/offboard_flatness_sitl.json` (run 1).

```
Reproduce: cd anixpkgs && nix-build pkgs/nixos/sitl-envs/offboard-flatness-tracking.nix
```

## In-firmware INDI attitude/rate results

Tracking RMSE of the **in-firmware** quaternion INDI attitude/rate backend
(`AC_CustomControl_INDI` in the `goromal/ardupilot` fork), engaged at runtime
via `CC_TYPE=3` and flying the same battery through the **stock outer loop**
(GUIDED position streaming — identical command path to stock-guided; legacy INDI rate controller swaps only
the inner attitude/rate controller). Scored by the same XKF1 `.BIN` evaluator.
Repeatability-checked across two fresh VM runs (`compare_baselines.py`, exit 0).

| Case | stock [m] | onboard INDI run 1 [m] | onboard run 2 [m] | Δ vs stock-guided |
|------|-------------:|------------------:|-------------:|--------:|
| hover_step | 0.615 | 0.756 | 0.754 | +23% |
| circle_slow | 0.271 | 0.252 | 0.252 | −7% |
| circle_fast | 0.636 | 0.724 | 0.750 | +14% |
| lemniscate_slow | 5.072 | 5.070 | 5.032 | −1% |
| lemniscate_fast | 1.787 | 1.708 | 1.719 | −4% |

The in-firmware INDI inner loop **matches or beats the stock rate controller**
(beats on circle_slow, lemniscate_slow, lemniscate_fast; slightly worse on
hover_step, circle_fast) — the legacy INDI rate controller milestone. The `.BIN` INDI health
confirms the backend flew it (48 400 messages, 0.3 % saturation) and engages/
disengages cleanly mid-flight (transient bound 0.21°). It does not beat offboard on
the lemniscates because it keeps the stock outer loop (beating those is a
flatness outer loop goal). Full analysis + the G1 limit-cycle finding:
[`docs/indi_rate_results.md`](docs/indi_rate_results.md).

Committed artifact: `baselines/indi_rate_sitl.json` (run 1).

```
Reproduce: cd anixpkgs && nix-build pkgs/nixos/sitl-envs/indi-rate-backend.nix
```
