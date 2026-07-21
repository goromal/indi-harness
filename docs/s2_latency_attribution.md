# S2 offboard vs S1 stock-guided: latency attribution

Design-doc S2 exit gate: *"offboard controller flies the S1 trajectory set
stably; degradation vs stock guided mode understood and attributable to
latency."* This documents the S1↔S2 comparison, the measured command-path
rate/latency, and the attribution.

Both phases fly the identical 5-case battery and are scored by the **same**
evaluator (`sitl.align.evaluate_bin` on XKF1 from the `.BIN`), so the numbers
are apples-to-apples. Reproduce (in the drone VM checkout, `local-build = true`):

```
nix-build pkgs/nixos/sitl-envs/s2-offboard.nix   # anixpkgs, branch dev/indi-s2
```

## Per-case RMSE (m)

Two independent fresh-VM S2 runs (run-1, run-2); S1 from
`baselines/s1_stock_sitl.json`.

| case            |    S1 |  S2 run-1 |  S2 run-2 | S2 avg |  Δ (S2−S1) |  Δ %   |
|-----------------|------:|----------:|----------:|-------:|-----------:|-------:|
| hover_step      | 0.615 |     0.540 |     0.488 |  0.514 |     −0.101 |  −16.5 |
| circle_slow     | 0.271 |     0.757 |     0.756 |  0.756 |     +0.485 | +179.0 |
| circle_fast     | 0.636 |     0.756 |     0.799 |  0.778 |     +0.142 |  +22.4 |
| lemniscate_slow | 5.072 |     0.462 |     0.443 |  0.452 |     −4.620 |  −91.1 |
| lemniscate_fast | 1.787 |     0.769 |     0.803 |  0.786 |     −1.001 |  −56.0 |

Repeatability: `scripts/compare_baselines.py run1 run2` → **exit 0** (every
case within `max(0.15 m, 30%)`).

## Measured command path (from the run-1 mcap/sqlite3 bag, `latency.json`)

| topic                | rate    | inter-arrival p50 | p95     | max      |
|----------------------|---------|-------------------|---------|----------|
| `/ap/pose/filtered`  | 27.7 Hz | 35.4 ms           | 41.4 ms | 45.9 ms  |
| `/indi/cmd_attitude` | 8.6 Hz  | 100 ms            | 100 ms  | 4105 ms  |

`/ap/pose/filtered` is the state-feedback rate **and** the command rate: the
node is event-driven on each pose and sends exactly one `SET_ATTITUDE_TARGET`
per pose (design-doc T.1). So the offboard loop closes at ~27.7 Hz with a
~35 ms state-to-command period. `/indi/cmd_attitude` is a 10 Hz **diagnostic**
republish (the node's internals timer), not the command path — its 8.6 Hz
figure and 4.1 s max gap (idle between cases) are artifacts of that timer, not
control latency.

End-to-end command latency (pose sensed → attitude realized in the `.BIN`) is
not directly correlated here, but the ~27.7 Hz / ~35 ms update period is the
dominant bound: the offboard outer loop can only observe and correct at that
cadence, versus S1's stock controller running the full guided
position→attitude chain onboard at the firmware loop rate.

## Attribution

**Stable? Yes.** The offboard controller flies all five cases with RMSE
0.44–0.80 m across both runs — no divergence, no case lost. Repeatable to
within tolerance.

**What actually flew.** The battery uses the flatness **PD + feed-forward**
outer loop (`--no-indi`), *not* the INDI acceleration increment. The INDI
accel loop is unstable offboard: at the 27.7 Hz pose rate with the DDS+MAVLink
command-path latency, the previous-command specific force (`f_state`) and the
IMU-measured specific force (`f_meas`) no longer cancel in the increment
`f_cmd = f_state + (a_cmd − g) − f_meas`, so `f_cmd` winds up — observed
commanded tilt >130° and thrust surrogate →1e4 before the vehicle flipped and
flew off (>100 m). The flatness attitude+rate+thrust feed-forward is retained;
only the accel increment is dropped.

**Root cause is de-synchronization, not "too much latency" (corrected).** An
earlier draft of this doc concluded INDI-offboard "needs a faster/lower-latency
inner path (S3)." An offline study (`tests/test_offboard_bridge.py::
test_indi_needs_delay_alignment_under_latency`, and the `OffboardGains.
cmd_delay_ticks` knob it exercises) shows that framing is imprecise. The INDI
increment cancels *only if `f_state` and `f_meas` refer to the same instant's
thrust*. Currently `f_state` is the command from **one tick** ago, but `f_meas`
reflects the command actually realized ~(command-path latency) ticks ago — so
the two are misaligned by the whole delay and the residual accumulates. Sweeping
a modelled command-path delay: plain INDI is stable to ~75 ms, winds up by
~110 ms, and diverges past ~185 ms, while PD+ff is flat throughout. **Delaying
`f_state` to match the latency (`cmd_delay_ticks` ≈ latency in ticks) restores
cancellation and recovers INDI to PD+ff quality (0.10–0.15 m) at every latency
tested.** Anti-windup / clamping the fed-back command does **not** help (it only
bounds the runaway into ~2–3 m tracking). Alignment tolerates *under*-estimating
the latency but diverges if you *over*-estimate — so bias the estimate low.

This is offline evidence (fixed-delay model, instant attitude, no jitter); it
is *not* yet VM-confirmed, and the offboard alignment buffer is throwaway
scaffolding regardless. The transferable point is for the **eventual onboard
INDI** (ArduPilot firmware, design-doc S3+): the same synchronization
requirement applies there — you match motor spin-up lag (~30 ms) and the IMU
filter delay instead of a comms round-trip — so "low latency" alone does not
make INDI safe; the actuator/sensor signals must be time-aligned before the
increment. That is the real lesson, and it is what a firmware implementation
must get right.

**Is the S2↔S1 delta attributable to latency?** Partly, and the honest
reading is more interesting than "uniform degradation":

- S2's RMSE is remarkably **flat** across trajectories (0.45–0.79 m) while S1
  swings 0.27→5.07 m. The offboard loop imposes a ~0.5–0.8 m **tracking floor**
  set by the 27.7 Hz / ~35 ms command cadence — it cannot correct faster than
  it sees state.
- That floor is a **large win** on the aggressive trajectories, where S1's
  stock position controller tracks poorly (lemniscate_slow 5.07→0.45, −91%;
  lemniscate_fast 1.79→0.79, −56%). The flatness feed-forward supplies the
  exact attitude/thrust for the reference, so S2 tracks tightly regardless of
  how hard the trajectory is.
- That same floor is a **regression** on the gentle trajectories, where S1 is
  already well below it (circle_slow 0.27→0.76, +179%; circle_fast 0.64→0.78,
  +22%). S2 cannot beat a 0.27 m stock result from behind a 0.5–0.8 m
  latency-limited floor.

**Honest gradient test.** If latency were the dominant error source, the
fastest-moving references (circle_fast, lemniscate_fast) would degrade *most*.
They do not: circle_**slow** degrades more than circle_**fast** (+179% vs
+22%), and both lemniscates *improve*. So the per-case delta is **not**
dominated by a speed-proportional latency effect. It is dominated by the
**control-strategy difference** — flatness feed-forward (S2) vs stock guided
position tracking (S1) — with the offboard command cadence contributing a
roughly constant ~0.5–0.8 m floor. The one unambiguous latency-attributable
degradation is circle_slow, where S2 hits that floor while S1 does not.

**Bottom line for the S2 exit.** The offboard ROS 2 outer loop flies the S1
battery stably and repeatably; the differences vs stock guided are understood
and attributed — a constant offboard-cadence floor (~0.5–0.8 m, the latency
signature) plus a flatness-feed-forward advantage that dominates on aggressive
trajectories. Absolute-number parity with onboard control was never the S2
goal (design-doc "known limitation"); consistent, stable, feed-forward-driven
offboard tracking was, and it is met. The INDI-offboard instability is
understood (command/measurement de-synchronization, not irreducible latency)
and its transferable lesson — time-align the actuator/sensor signals before the
increment — is carried to the onboard S3 work.
