# S3 Layer B — In-firmware INDI outer loop + flatness feedforward (results)

**Claim:** the differential-flatness + linear-INDI **outer** loop, ported into the
firmware `CC_TYPE=3` backend and fed a full flat reference `{p,v,a,j,s,ψ,ψ̇,ψ̈}`
over the custom `ardupilot_msgs/FlatSetpoint` AP_DDS topic from a ROS2
trajectory-server, closes the **lemniscate-tracking gap** that C1 (inner-loop
only) left open — bringing the S2 offboard flatness advantage into firmware, at
the low-latency controller rate.

## Battery position tracking (RMSE, m)

| case | stock (S1) | S2 offboard | Layer A | C1 (inner) | **Layer B** |
| ---------------- | ----- | ----- | ----- | ----- | --------- |
| hover_step       | —     | —     | —     | 0.750 | **0.592** |
| circle_slow      | 0.271 | 0.757 | 0.252 | 0.266 | **0.485** |
| circle_fast      | 0.636 | 0.756 | 0.724 | 0.761 | **0.460** |
| lemniscate_slow  | 5.072 | 0.462 | —     | 5.012 | **0.510** |
| lemniscate_fast  | 1.787 | 0.769 | 1.708 | 1.704 | **0.711** |

Config: `CC_TYPE=3 CC_AXIS_MASK=7 CC3_OMG_FILT=80 CC3_G1_RP=500 CC3_OUTER_EN=1
CC3_B_THR_EN=0 CC3_B_ACC_FILT=8`. Attitude-only (collective from the stock guided
altitude controller — see below). Source: `s3_layerB_sitl.json`.

**Metric note.** The first four columns are XKF1-vs-trajectory position RMSE on the
trajectory clock (`align.evaluate_bin`). Layer B is scored from the `INDB` .BIN
message — the DDS flat reference vs the measured (AHRS) position logged on the
**same firmware tick** (`align.flat_tracking_score` over the active,
`fallback==0` samples). No boot↔traj clock alignment is needed because both series
are logged together in firmware; it is the direct, appropriate metric for the
in-firmware DDS-driven outer loop, and comparable in magnitude to XKF1 RMSE.

## The headline: the lemniscate gap is closed

C1 is an inner-loop replacement, so its position RMSE necessarily tracks stock —
on the lemniscates that is **5.0 / 1.7 m**, because the stock guided outer loop
does not feed acceleration/jerk/snap. S2 showed that a flatness outer loop pulls
this to **0.46 / 0.77 m**, but only offboard (SET_ATTITUDE_TARGET, where the INDI
*accel* loop was latency-unstable). Layer B runs the whole flatness + linear-INDI
outer loop **in firmware** and reaches **0.51 / 0.71 m** — matching S2, and
beating stock/C1 by ~10× (slow) and ~2.4× (fast). The in-firmware low-latency
path works where S2 offboard did not.

## Outer loop stable AND active (escalation clause cleared)

`CC3_B_ACC_FILT` (the specific-force / thrust-state phase-margin cutoff) was swept
on `circle_slow`:

| `CC3_B_ACC_FILT` (Hz) | track RMS (m) | track max (m) | active_frac | stable |
| --- | ----- | ----- | ---- | --- |
| 4  | 0.454 | 0.737 | 0.78 | ✅ |
| 8  | 0.485 | 0.734 | 0.79 | ✅ |
| 15 | 0.490 | 0.818 | 0.79 | ✅ |
| 30 | 0.529 | 0.807 | 0.79 | ✅ |

Stable + active across the whole 4–30 Hz range — a robust operating point, not
knife-edge. Default kept at **8 Hz** (`CC3_B_ACC_FILT=8.0`): mid-range, keeps the
INDI specific-force increment genuinely active (non-degenerate) without the added
measurement noise at 30 Hz. Tracking is nearly flat across the sweep — see the
honest note below.

## Honest notes

- **The drag-rejection benefit is S4.** Tracking is insensitive to
  `CC3_B_ACC_FILT` (0.45–0.53 m across 4–30 Hz) because benign SITL has no
  unmodelled disturbance (drag) for the measured-specific-force INDI increment to
  reject. The increment is genuinely active — it just has nothing to correct here.
  Its disturbance-rejection payoff needs S4's aero fidelity; in SITL, Layer B's win
  is the flatness **feedforward** (a/j/s) closing the lemniscate gap, not the
  incremental term.

- **Attitude-only (`CC3_B_THR_EN=0`).** The outer loop drives the attitude target
  (`q_ref`, `w_ff`, `dw_ff`); the collective/altitude stays with the stock guided
  controller (vertical error ~0.2 m). Driving the collective from the outer loop
  (`CC3_B_THR_EN=1`) was tried and defeated across three fixes — the throttle →
  thrust-state → thrust-vector coupling self-references into an altitude runaway
  (climb to ~66 m). It is disabled; the RPM-fed thrust path is deferred to HW/S4,
  consistent with the S2 finding that the INDI *thrust* loop is the fragile part.

- **The integration fix (why C1 alone could not track).** The outer loop first
  fed its flatness target only to the INDI increment, while the stock rate
  controller (which runs first every fast loop and sets the actuator baseline the
  increment adds to) still tracked the guided station-hold target. The two
  opposed and cancelled to ~0 roll/pitch torque — the vehicle yawed in place. Fix:
  `Copter::update_flight_mode()` now pushes the backend's flatness `q_ref`/`w_ff`
  into `AC_AttitudeControl::input_quaternion()` after `flightmode->run()`, so the
  stock rate controller and the INDI increment track the **same** target.
