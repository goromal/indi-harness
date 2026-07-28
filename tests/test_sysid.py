import numpy as np
from indi_harness.params import QuadParams
from indi_harness.sysid import analytic_seed


def test_seed_matches_mixer_roundtrip():
    P = QuadParams()
    s = analytic_seed(P)
    Om_h = P.hover_speed()
    Tt = s["M"] @ np.full(4, Om_h ** 2)
    assert np.isclose(Tt[0], P.m * P.g, rtol=1e-6)      # hover thrust == weight
    assert np.allclose(Tt[1:], 0.0, atol=1e-9)          # balanced hover -> zero torque


def test_g1_torque_in_true_effectiveness_band():
    s = analytic_seed(QuadParams())
    assert 100.0 < s["g1_torque"][0] < 600.0            # roll
    assert 100.0 < s["g1_torque"][2] < 600.0            # yaw


def test_throttle_rpm_affine_monotonic():
    s = analytic_seed(QuadParams())
    a, b = s["throttle_rpm_a"], s["throttle_rpm_b"]
    assert a > 0.0
    assert (a * 1.0 + b) > (a * 0.0 + b)
