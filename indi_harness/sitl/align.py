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
    [n,3] rad/s^2). NRMSE is the enforced Layer-C exit-gate metric (< 0.3);
    R^2 is reported alongside it."""
    pred = np.asarray(health["domega_pred"], float)
    meas = np.asarray(health["domega_meas"], float)
    out = {}
    for ax in range(3):
        m, p = meas[:, ax], pred[:, ax]
        rng = m.max() - m.min()
        nrmse = float(np.sqrt(np.mean((p - m) ** 2)) / rng) if rng > 1e-9 else float("inf")
        ss_res = float(np.sum((m - p) ** 2))
        ss_tot = float(np.sum((m - m.mean()) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
        out[ax] = {"nrmse": nrmse, "r2": r2}
    return out


def g1_ceiling_ok(g1, analytic, factor=3.0):
    """True if the fitted G1 stays within `factor`x the analytic seed --
    guards the Layer-C sysid regression against a runaway/unphysical fit."""
    return float(g1) < factor * float(analytic)
