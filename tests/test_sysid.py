import numpy as np
from indi_harness.params import QuadParams
from indi_harness.sysid import (
    analytic_seed, analytic_g1, analytic_g2, identify_g1, identify_g2,
    normalized_effectiveness,
)


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


def test_analytic_wrappers_match_seed():
    P = QuadParams()
    s = analytic_seed(P)
    assert np.allclose(analytic_g1(P), s["g1_torque"])
    assert np.isclose(analytic_g2(P), s["g2_yaw"])


def test_sysid_recovers_g2_within_tol():
    P = QuadParams()
    est = identify_g2(P)                 # doublet + regression on the sim
    true = analytic_g2(P)                # = P.Ir
    print(f"g2: est={est} true={true}")
    assert abs(est - true) <= 0.15 * abs(true), f"est={est} true={true}"


def test_sysid_recovers_g1_within_tol():
    P = QuadParams()
    est = identify_g1(P)                 # length-3, per axis
    true = analytic_g1(P)                # = 1/diag(J)
    print(f"g1: est={est} true={true}")
    assert np.all(np.abs(est - true) <= 0.15 * np.abs(true)), f"est={est} true={true}"


def test_normalized_effectiveness_matches_motor_perturbations():
    P = QuadParams()
    seed = normalized_effectiveness(P)
    hover = np.full(4, P.hover_speed() ** 2)
    for axis in range(3):
        delta = .01 * P.Omega_max ** 2 * seed["factors"][:, axis]
        tau = (P.mixer() @ (hover + delta))[1:]
        acceleration = np.linalg.solve(P.J, tau)
        assert np.isclose(acceleration[axis] / .01, seed["g1"][axis])
    assert np.allclose(seed["g1"], [343.65389566, 343.65389566, 28.8])
    assert not np.allclose(seed["g1"], analytic_g1(P))
    assert np.isclose(seed["g2_yaw"] * seed["torque_per_command"][2, 2], P.Ir)
