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
