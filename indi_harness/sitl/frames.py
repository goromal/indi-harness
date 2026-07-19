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
