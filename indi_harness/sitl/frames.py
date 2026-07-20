"""NED <-> ENU conversion. This is THE one place it lives (design doc S1).

MAVLink local frames are NED; ROS 2 / REP-103 is ENU. The mapping
(N,E,D) <-> (E,N,U) is a swap of the first two axes and a z negation,
which is an involution — the same function converts both directions.
"""
import numpy as np


def ned_to_enu(v):
    v = np.asarray(v, float)
    return np.array([v[1], v[0], -v[2]])


def enu_to_ned(v):
    return ned_to_enu(v)


# Body-frame sibling of the world-frame pair above: ROS body is FLU
# (REP-103), MAVLink/ArduPilot body is FRD. Same one-place rule.
_R_ENU_NED = np.array([[0.0, 1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0]])
_R_FLU_FRD = np.diag([1.0, -1.0, -1.0])


def flu_to_frd(v):
    v = np.asarray(v, float)
    return np.array([v[0], -v[1], -v[2]])


def frd_to_flu(v):
    return flu_to_frd(v)


def pose_enu_to_ned(p_enu, q_enu):
    """(p, q) of an ENU/FLU pose -> the same physical pose in NED/FRD.

    R_ned_frd = R_enu->ned @ R_enu_flu @ R_flu->frd  (both maps involutive).
    Imported lazily to keep frames free of the quat dependency cycle.
    """
    from .. import quat
    R = quat.R_from_q(np.asarray(q_enu, float))
    R_ned = _R_ENU_NED @ R @ _R_FLU_FRD
    return ned_to_enu(p_enu), quat.q_from_R(R_ned)


def pose_ned_to_enu(p_ned, q_ned):
    from .. import quat
    R = quat.R_from_q(np.asarray(q_ned, float))
    R_enu = _R_ENU_NED @ R @ _R_FLU_FRD
    return ned_to_enu(p_ned), quat.q_from_R(R_enu)
