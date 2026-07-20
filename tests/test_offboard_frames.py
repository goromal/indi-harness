import numpy as np
from indi_harness import quat
from indi_harness.sitl.frames import (ned_to_enu, flu_to_frd,
                                      pose_enu_to_ned, pose_ned_to_enu)


def test_flu_frd_involution():
    v = np.array([0.7, -0.2, 1.1])
    assert np.allclose(flu_to_frd(flu_to_frd(v)), v)
    assert np.allclose(flu_to_frd([1.0, 2.0, 3.0]), [1.0, -2.0, -3.0])


def test_pose_identity_enu_faces_east():
    # ENU identity attitude = body x along East. In NED that is yaw +90 deg.
    p_ned, q_ned = pose_enu_to_ned(np.zeros(3), np.array([1.0, 0, 0, 0]))
    x_b_ned = quat.qrot(q_ned, np.array([1.0, 0, 0]))
    assert np.allclose(x_b_ned, [0.0, 1.0, 0.0], atol=1e-9)  # East in NED


def test_pose_round_trip():
    rng = np.random.default_rng(7)
    p = rng.normal(size=3)
    q = quat.qnormalize(rng.normal(size=4))
    p2, q2 = pose_ned_to_enu(*pose_enu_to_ned(p, q))
    assert np.allclose(p2, p, atol=1e-9)
    assert min(np.linalg.norm(q2 - q), np.linalg.norm(q2 + q)) < 1e-9
