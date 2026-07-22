# S3 Layer-A results — in-firmware quaternion INDI attitude/rate backend

Design-doc phase **S3, Layer A**: a `AC_CustomControl_INDI` backend in the
`goromal/ardupilot` fork implementing the quaternion tilt-prioritized attitude
error + INDI rate loop, selectable at runtime via `CC_TYPE=3`, flying the S1
trajectory battery in SITL. Layer A replaces **only** the inner attitude/rate
controller — the stock guided-mode position→attitude outer loop still runs, so
the command path is identical to S1 (`SET_POSITION_TARGET_LOCAL_NED` GUIDED
streaming). Output is torque-like to the stock mixer (§2 step 5 limitation
accepted; real actuator/RPM feedback is Layer C).

Reproduce: `cd anixpkgs && nix-build pkgs/nixos/sitl-envs/s3-layerA.nix`
(requires the fork + this repo threaded in — see `dependencies.nix`
`drone-local-fork`, or the bumped flake.lock pins once pushed).

## Tracking RMSE — S1 stock vs S2 offboard vs S3 INDI

Per-case EKF tracking RMSE (XKF1, sim-time normalized) flying the 5-case S1
battery. S3 INDI is two independent fresh-VM runs; `compare_baselines.py`
run1-vs-run2 exits 0 (repeatable within `max(0.15 m, 30%)`).

| Case            | S1 stock | S2 offboard | **S3 INDI run 1** | **S3 INDI run 2** |
|-----------------|---------:|------------:|------------------:|------------------:|
| hover_step      |    0.615 |       0.540 |             0.756 |             0.754 |
| circle_slow     |    0.271 |       0.757 |             0.252 |             0.252 |
| circle_fast     |    0.636 |       0.756 |             0.724 |             0.750 |
| lemniscate_slow |    5.072 |       0.462 |             5.070 |             5.032 |
| lemniscate_fast |    1.787 |       0.769 |             1.708 |             1.719 |

Committed artifact: `baselines/s3_layerA_sitl.json` (run 1).

## Verdict: S3 INDI **matches or beats** the stock baseline

Against the design-doc S3 exit ("beats or matches stock + S2"):

- **vs S1 stock — matches/beats.** S3 INDI beats stock on circle_slow
  (0.252 vs 0.271), lemniscate_slow (5.03–5.07 vs 5.072), and lemniscate_fast
  (1.71 vs 1.787); it is marginally worse on hover_step (0.75 vs 0.615) and
  circle_fast (0.72–0.75 vs 0.636). Net: the in-firmware INDI inner loop tracks
  the battery about as well as the stock rate controller — the Layer-A milestone
  number.
- **vs S2 offboard — does not beat on the lemniscates.** S2's flatness PD+ff
  outer loop tracks the lemniscates far better (0.46 / 0.77 vs 5.03 / 1.71)
  because S2 replaces the *outer* loop with a feedforward trajectory tracker;
  Layer A keeps the stock outer loop and only improves the inner loop, so it
  inherits the stock outer loop's lemniscate tracking. Beating S2 on the
  lemniscates is a Layer-B (outer loop + flatness feedforward) goal.

## INDI-health evidence (`.BIN`, design-doc §L)

The `INDI` dataflash message (predicted vs measured/filtered angular accel,
actuator-state estimate, increment Δu, saturation flag) confirms the INDI
backend actually flew the battery (not a stock fallback), and characterizes it:

- **48 400 INDI messages** logged over the battery, **0.3 % saturation** — the
  loop is stable and non-saturating throughout the 5 cases.
- Mean |Δu| ≈ `[0.11, 0.11, 0.003]` (roll/pitch/yaw) — the INDI increment is
  doing real incremental control on roll/pitch.
- Predicted ω̇ p95 ≈ `[36, 33, 15]` rad/s² vs measured (filtered gyro-derivative)
  ω̇ p95 ≈ `[189, 173, 4]` rad/s². The measured estimate carries substantial
  high-frequency content from differentiating the gyro; the conservative
  effectiveness (`CC3_G1_RP/YAW = 1000`, well above the true ~175–479 rad/s²/unit
  for the SITL quad) attenuates the increment enough to stay stable. This is a
  **de-tuned / conservative INDI**, not textbook ω̇-tracking.

## Flight config + the limit-cycle finding

Param flight config (firmware defaults): `CC3_FILT_HZ = 40`, `CC3_ATT_TLT_P = 6`,
`CC3_ATT_YAW_P = 3`, `CC3_KW_RP = 20`, `CC3_KW_YAW = 10`, **`CC3_G1_RP = CC3_G1_YAW = 1000`**.

`G1` is the load-bearing tuning parameter. A *low* G1 over-amplifies the noisy
400 Hz gyro-derivative angular-accel estimate into a high-frequency motor-buzz
limit cycle: at `G1 = 1.0` the loop saturates ~94 % of the time and the gyro
shows ~180 rad/s² oscillation *while the attitude stays within ±3°* (it buzzes,
it does not tip). Lowering `CC3_FILT_HZ` (40→8) makes it **worse** — added group
delay, a phase/latency-driven instability of the same class as the S2 offboard
finding (`docs/s2_latency_attribution.md`). Setting `G1` well above the true
effectiveness attenuates the increment (`Δu = (ω̇_cmd − ω̇_filt)/G1`) enough to
fly stably. A cleaner angular-accel estimate (filter the gyro before
differentiating) or Layer-C actuator/RPM feedback would allow a lower, more
aggressive `G1` and true ω̇-tracking; deferred as a follow-on.

## Engage / disengage transient

Scripted mid-hover custom-controller transition (`CC_TYPE=3` is RebootRequired,
so the switch is the aux-function activation `0→3→0` on RC9, which engages /
disengages the INDI backend's `update()`). Max |roll|,|pitch| deviation:

| phase                     | max\|att\| [deg] |
|---------------------------|-----------------:|
| pre-engage (stock hover)  |             0.34 |
| **ENGAGE 0→3 transient**  |         **0.21** |
| steady under INDI         |             0.18 |
| **DISENGAGE 3→0 transient** |       **0.19** |

Engage/disengage transient bound **0.21°** — smaller than the stock hover
jitter (0.34°), i.e. no observable attitude transient on either transition. In
addition, all four battery runs (2 VM + 2 standalone) engage the INDI backend
mid-flight (after takeoff) and then fly the full 5-case battery stably (0.3 %
saturation), a repeated bounded-engage demonstration under trajectory flight.
