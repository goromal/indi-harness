"""Hamilton scalar-first quaternions [w, x, y, z]; q maps body -> world."""
import numpy as np


def qnormalize(q):
    q = np.asarray(q, float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        raise ValueError("zero-norm quaternion")
    return q / n


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def qrot(q, v):
    """Rotate body-frame v into world frame."""
    qv = np.array([0.0, v[0], v[1], v[2]])
    return qmul(qmul(q, qv), qconj(q))[1:]


def qrot_inv(q, v):
    """Rotate world-frame v into body frame."""
    return qrot(qconj(q), v)


def R_from_q(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def q_from_R(R):
    """Shepperd's method; returns scalar-part-positive quaternion."""
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s,
                      (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
                      0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
                      (R[1, 2] + R[2, 1]) / s, 0.25 * s])
    q = qnormalize(q)
    return q if q[0] >= 0 else -q


def qexp(rv):
    """Rotation vector (rad) -> quaternion."""
    rv = np.asarray(rv, float)
    th = np.linalg.norm(rv)
    if th < 1e-12:
        return np.array([1.0, 0.5 * rv[0], 0.5 * rv[1], 0.5 * rv[2]])
    ax = rv / th
    return np.concatenate([[np.cos(th / 2)], np.sin(th / 2) * ax])


def qlog(q):
    """Quaternion -> rotation vector (rad)."""
    q = qnormalize(q)
    vn = np.linalg.norm(q[1:])
    if vn < 1e-12:
        return 2.0 * q[1:]
    # Use atan2 to get angle in full range (not limited to shortest arc)
    theta = 2.0 * np.arctan2(vn, q[0])
    return theta * q[1:] / vn


def qerr(q, q_ref):
    """Error quaternion: rotation from current body frame to reference frame,
    expressed in body axes. Scalar part forced >= 0 (no unwinding)."""
    qe = qmul(qconj(q), q_ref)
    return qe if qe[0] >= 0 else -qe
