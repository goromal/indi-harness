"""Tilt-prioritized attitude error (Brescianini & D'Andrea; Sun et al.).

qe = q_red (x) q_yaw, where q_yaw is a pure z rotation and q_red carries tilt.
"""
import numpy as np
from . import quat


def tilt_yaw_decompose(qe):
    """Split error quaternion qe into (q_red, q_yaw) with qe = q_red (x) q_yaw.

    q_red carries the tilt error (zero z vector component); q_yaw is a pure
    z rotation. Near 180-degree tilt (hypot(w, z) -> 0) yaw is undefined, so
    all error is returned as tilt and q_yaw is identity.
    """
    w, x, y, z = qe
    n = np.hypot(w, z)
    if n < 1e-9:
        # 180-degree tilt: yaw is undefined; put all error in the tilt part.
        q_red = quat.qnormalize(np.array([0.0, x, y, 0.0]))
        q_yaw = np.array([1.0, 0.0, 0.0, 0.0])
        return q_red, q_yaw
    q_red = np.array([w * w + z * z, w * x - y * z, w * y + x * z, 0.0]) / n
    q_yaw = np.array([w, 0.0, 0.0, z]) / n
    return quat.qnormalize(q_red), quat.qnormalize(q_yaw)


def attitude_rate_ref(q, q_ref, kp_tilt, kp_yaw, w_ff):
    """P-on-rotation-vector attitude law with separate tilt/yaw gains.

    Returns a body-frame angular-rate command [rad/s]. q and q_ref are
    body-to-world quaternions; w_ff is the reference body rate.

    Note: adding w_ff directly assumes small attitude error (exact form
    would rotate it through qe). Fine for S0.
    """
    qe = quat.qerr(q, q_ref)
    q_red, q_yaw = tilt_yaw_decompose(qe)
    return kp_tilt * quat.qlog(q_red) + kp_yaw * quat.qlog(q_yaw) + np.asarray(w_ff, float)
