"""Boot-clock <-> trajectory-clock alignment (design doc L.3).

The FlightRecord's (traj_t, boot_ms) pairs define a linear map that absorbs
both the boot-time offset and any SITL sim-time slowdown; .BIN TimeUS data
is then scored on the trajectory timeline via evalmetrics.rmse_position.
"""
import numpy as np
from ..evalmetrics import rmse_position


def boot_to_traj_map(traj_t, boot_ms):
    """Least-squares linear fit: traj_t ~= k * boot_ms + b."""
    A = np.vstack([np.asarray(boot_ms, float),
                   np.ones(len(boot_ms))]).T
    k, b = np.linalg.lstsq(A, np.asarray(traj_t, float), rcond=None)[0]
    return float(k), float(b)


def evaluate_bin(time_us, p_ned, k, b, traj, origin_offset, trim_s=2.0):
    """RMSE of BIN EKF positions against the reference on the traj clock.

    origin_offset = p_origin - traj.ref(0).p, so that
    p_ref(t) = traj.ref(t).p + origin_offset = origin + (p(t) - p(0)).
    """
    t_traj = k * (np.asarray(time_us, float) / 1000.0) + b
    keep = t_traj >= 0.0
    t_traj, p_ned = t_traj[keep], np.asarray(p_ned, float)[keep]
    p_ref = np.array([traj.ref(t).p for t in t_traj]) + np.asarray(origin_offset)
    return rmse_position(t_traj, p_ned, p_ref, trim_s=trim_s)


def omega_tracking_score(health):
    """Per-axis predicted-vs-measured omega_dot tracking, from an INDI
    health dict (see sitl.binlog.read_indi_health: domega_pred, domega_meas,
    [n,3] rad/s^2). Returns per axis {nrmse, r2, exc_rms}.

    NRMSE is normalized by RMS(meas), NOT the peak-to-peak range: the gate must
    reject Layer A's attenuated predictor (pred ~ 0.2*meas -> NRMSE ~ 0.9) and
    accept true tracking (pred ~ meas -> NRMSE ~ 0). Range-normalization would
    score Layer A's failure mode at ~0.3 and let it slip through the gate.

    exc_rms = RMS(measured omega_dot) is the per-axis EXCITATION level. NRMSE
    is only meaningful where exc_rms is substantial: on a near-quiescent axis
    (an axis the trajectory never commands) measured omega_dot ~ noise, so NRMSE
    divides by ~0 and is meaningless (~inf/large). Gate with omega_gate_ok(),
    which skips unexcited axes -- otherwise a quiescent yaw/roll produces a
    phantom failure (learned the hard way: circle_slow barely excites yaw, and a
    forward step barely excites roll)."""
    pred = np.asarray(health["domega_pred"], float)
    meas = np.asarray(health["domega_meas"], float)
    out = {}
    for ax in range(3):
        m, p = meas[:, ax], pred[:, ax]
        rms_m = float(np.sqrt(np.mean(m ** 2)))
        rmse = float(np.sqrt(np.mean((p - m) ** 2)))
        nrmse = rmse / rms_m if rms_m > 1e-9 else float("inf")
        ss_res = float(np.sum((m - p) ** 2))
        ss_tot = float(np.sum((m - m.mean()) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
        out[ax] = {"nrmse": nrmse, "r2": r2, "exc_rms": rms_m}
    return out


def omega_gate_ok(health, axes=(0, 1), nrmse_max=0.75, exc_floor=0.08):
    """Excitation-aware omega_dot-tracking gate, scoped to the axes the C1
    torque-space INDI is responsible for. Returns (ok, per_axis) where per_axis[ax]
    adds 'excited' and 'enforced'.

    ENFORCED AXES (`axes`, default (0,1) = roll,pitch): each must be excited
    (exc_rms >= exc_floor) AND track (nrmse < nrmse_max); the gate requires at
    least one enforced axis to actually be excited (else the maneuver didn't test
    what we claim). NON-enforced axes are scored and reported but never fail the
    gate.

    Why roll/pitch only: C1 delivers real omega_dot-inversion on roll/pitch
    (NRMSE ~0.48/0.49 in SITL, vs the ~1.1 not-tracking floor). YAW is
    deliberately NOT enforced -- its omega_dot-inversion needs the rotor-inertia
    (G2) term fed by measured RPM (design doc S3 sec.3.6), which SITL does not
    model (no rotor-inertia reaction; actuator "RPM" is ~the command) and is
    therefore deferred to HW/S4. Yaw still flies clean (it is not limit-cycling;
    pred/meas yaw accel correlate +0.9, merely under-scaled by weak yaw
    effectiveness) -- it just does not do full omega_dot-inversion in SITL, so
    gating it here would enforce physics we cannot simulate.

    nrmse_max default 0.75 is the honest in-SITL clean-flight floor for the C1
    torque-space INDI. A tighter <0.3 needs the measured-actuator-state fidelity
    SITL lacks (rotor inertia / actuator lag) -> HW/S4."""
    s = omega_tracking_score(health)
    per = {}
    ok = True
    any_enforced_excited = False
    for ax in range(3):
        excited = s[ax]["exc_rms"] >= exc_floor
        enforced = ax in axes
        per[ax] = {**s[ax], "excited": excited, "enforced": enforced}
        if enforced and excited:
            any_enforced_excited = True
            if not (s[ax]["nrmse"] < nrmse_max):
                ok = False
    return (ok and any_enforced_excited), per


def g1_ceiling_ok(g1, analytic, factor=3.0):
    """True if the fitted G1 stays within `factor`x the analytic seed --
    guards the Layer-C sysid regression against a runaway/unphysical fit."""
    return float(g1) < factor * float(analytic)
