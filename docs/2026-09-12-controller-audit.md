# Controller audit and current handoff — 2026-09-12

## Established results

The measured-RPM feedback platform is useful, but the August conclusion that measured
torque-space feedback is inherently unstable was too strong. The old firmware
(`7ee4de5`, binary SHA256 `f3b2630fc8c3f86769bb97011d998cfe034e19bb7bf3511f3f2bd5b06e16c36e`)
completed hover and +/-5 degree roll/pitch steps with a linearized actuator map
and `CC3_G1_YAW=28.8`. Holding that setup fixed and changing ONLY G1_YAW to 1000
produced a yaw runaway (>8 rad/s) during hover. This isolates a real gain issue;
it does not establish that the original DDS-driven trajectory failure has only
one cause.

The stock mixer normalizes every axis to +/-0.5. For the JSON plant's 900 rad/s
maximum rotor speed, normalized angular-acceleration effectiveness is
`[343.6539, 343.6539, 28.8]`, not the physical torque effectiveness `1/J`.
The old yaw gain was ~34.7 times the true normalized effectiveness. The new
host regression includes the attitude loop and 30 ms motor lag: the correctly
scaled yaw cell converges, while the 1000-gain cell diverges.

## Controller and telemetry corrections

- Stock PID runs immediately BEFORE custom control. The legacy `u_act` is
  current stock PID output, not the previous custom command. `INDC.Cx/Cy/Cz`
  retain that historical meaning; new `INDU` logs distinguish current PID,
  previous custom output, new output, and desired/measured body rates.
- The JSON adapter publishes mechanical RPM, matching AP_ESC_Telem's hardware
  contract. Motor pole pairs are already handled by DShot/BLHeli drivers.
- The real RPM-source adapter now reads unslewed telemetry with a 20 ms age
  bound. Monitoring's interpolation and one-second validity are inappropriate
  for this controller.
- Missing JSON motor fields age independently. A missing/stale source cannot
  become healthy by replaying the shim's cached truth. Synthetic dropout,
  latency, missing-source and recovery tests cover the boundary.
- `USE_RPM=0` retains the legacy PID-baseline behavior. The measured path and
  fallback still use the stock mixer; full rotor-speed allocation has not been built.

## Small-flight gate

`python -m indi_harness.sitl.rate_probe` runs fresh standalone JSON SITL with
hover, attitude steps, four seconds of missing motor-0 telemetry, and recovery.
It owns and stops its processes and saves exact parameters, binary hash,
simulator truth, time windows and the DataFlash log. `probe_score` checks the
logged parameters, all-axis gain ceiling, bounded altitude/rates, saturation,
excited roll/pitch acceleration tracking, fallback and recovery, command-history
continuity, and identification from the recorded flight.

Corrected local firmware flight `probe-corrected-01`:

- All four phases completed; saturation fraction 0 throughout.
- Altitude stayed approximately 4.95–5.01 m; peak body rate <0.62 rad/s.
- Step angular-acceleration NRMSE ~0.81/0.82, r² ~0.34/0.33. This meets the
  separately declared small-flight gate (<0.9, r²>0.2); it does NOT meet the
  old provisional trajectory threshold <0.6. Hover has too little excitation
  to interpret NRMSE as an inversion test.
- Motor-0 omission triggers fallback; recovery returns to measured feedback.
- Previous-output log continuity error is zero during flight. The scorer
  excludes the intentional ground/disarm resets and trims phase boundaries.
- Recorded-flight identification (no J, kf or Ir supplied to the regression)
  recovered G1 `[343.6325, 343.6674, 28.8]`, normalized G2 `1.929012e-6`.
  These are backend truth measurements during a real SITL run, not a
  hardware/IMU-noise identification result.

Artifacts are under the workspace `data/`: `probe-old-linear-04`,
`probe-old-yaw1000`, `probe-corrected-01`; earlier numbered attempts retain
startup/arming diagnostics. Do not commit the large flight logs into source.

The remote-pinned Nix package also passes: 136 Python tests, 25 non-empty
firmware math tests, and the same four-phase flight on DDS-enabled firmware.
Packaged flight artifact: `data/indi-packaged-probe` (a Nix store result).
Step NRMSE is 0.815/0.818, saturation is zero, peak rate 0.614 rad/s;
fallback is 100% in the trimmed dropout window and 0% in all other windows.
The CI flight job now includes this small regression. Reproduce from anixpkgs:

```sh
nix-build pkgs/nixos/sitl-envs/indi-actuator-probe.nix -A flight
```

This expression always uses the checkout's lock pins. The larger VM expression
still needs temporary `dependencies.nix local-build=true`; restore it to false
before committing. Firmware and harness changes are coordinated through their
dependency PRs; the hub pins their exact revisions.

## Corrected DDS trajectory attempt

The VM battery was also run with the corrected linear map and logged gains
`[500, 500, 28.8]`; RPM fallback was 0%, and the normalized gain check passed.
The trajectory gate nevertheless failed: heavy saturation, five active blocks
for two cases, and an EKF-reported peak altitude of 1344 m. Do not interpret its
per-case RMS as valid comparisons: the engaged blocks no longer map one-to-one
to the cases and the estimator was failing.

Crucially, this is not a clean measured-RPM feedback-onset test. At t=69.53 s, BEFORE custom control
engaged at t=70.83 s, stock RATE already logged roll 238 deg/s. At the first
INDU tick the gyro was `[2.78, 3.64, -1.36]` rad/s. The first active DDS outer
sample was later, at t=76.43 s, after the EKF failsafe. Thus neither measured-RPM feedback nor DDS
caused the onset of this run's instability. The scorer now explicitly rejects
an unsettled pre-engagement handover (>1 rad/s roll/pitch norm in the preceding
two seconds). This is an additional validity check, not a relaxed acceptance
threshold.

Artifacts: `data/indi-measured-rpm-trajectory-vm/{flight.BIN,indi_score.json}`,
`data/indi-measured-rpm-onset.log`, `data/indi-measured-rpm-engagement.log`.
Next isolate the stock GUIDED hold/takeoff on the linearized map, then verify a
settled handover before enabling DDS. The small probe uses a shorter takeoff
settle and attitude targets; it does not validate that longer position hold.

## Outstanding acceptance criteria

1. Establish a stable stock GUIDED hold and handover on the linear map, then
   repeat the DDS-driven trajectory battery; retain the
   stock collective controller until its separate thrust-state problem is solved.
2. Only after clean trajectory flight, run PID/INDI x drag-off/drag-on comparisons.
3. rotor-speed allocation is a candidate for allocation, saturation and RPM-loop benefits; it is not
   an established prerequisite from the August failure. Decide its scope from
   these controlled results. robustness robustness and hardware validation remain open.

The August results and spec reframes are historical records. This handoff
supersedes their causal certainty and the stale August 25 next-steps brief.
