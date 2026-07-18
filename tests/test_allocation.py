import numpy as np
from indi_harness.params import QuadParams
from indi_harness.allocation import Allocator

P = QuadParams()
A = Allocator(P)

def test_static_hover_inverse():
    Om = A.static(T=P.m * P.g, tau=np.zeros(3))
    assert np.allclose(Om, P.hover_speed(), rtol=1e-9)

def test_increment_round_trip():
    Om_f = np.full(4, P.hover_speed())
    dT, dtau = 0.4, np.array([0.02, -0.01, 0.005])
    Om_sq_cmd, sat = A.indi_increment(dT, dtau, Om_f ** 2, np.zeros(4))
    u = P.mixer() @ Om_sq_cmd - P.mixer() @ (Om_f ** 2)
    assert not sat
    assert np.isclose(u[0], dT, atol=1e-9)
    assert np.allclose(u[1:], dtau, atol=1e-9)

def test_g2_yaw_compensation():
    Om_f = np.full(4, P.hover_speed()) ** 2
    _, d = P.layout()
    dOm = d * 5000.0  # CCW pair accelerating
    base, _ = A.indi_increment(0.0, np.zeros(3), Om_f, np.zeros(4))
    comp, _ = A.indi_increment(0.0, np.zeros(3), Om_f, dOm)
    u = P.mixer() @ comp - P.mixer() @ base
    assert u[3] > 0  # extra +yaw torque to cancel -yaw reaction

def test_saturation_sheds_yaw_first():
    Om_f = np.full(4, P.Omega_max * 0.98) ** 2
    dT, dtau = 2.0, np.array([0.0, 0.0, 0.05])
    Om_sq_cmd, sat = A.indi_increment(dT, dtau, Om_f, np.zeros(4))
    assert sat
    assert np.all(Om_sq_cmd <= P.Omega_max ** 2 + 1e-6)
    # yaw was shed exactly (shed path zeroes the yaw increment)
    u = P.mixer() @ Om_sq_cmd - P.mixer() @ Om_f
    assert abs(u[3]) < 1e-9

def test_static_clips_to_limits():
    Om = A.static(T=0.0, tau=np.zeros(3))          # zero thrust request
    assert np.allclose(Om, P.Omega_min)            # clipped to lower bound
    Om = A.static(T=100.0 * P.m * P.g, tau=np.zeros(3))
    assert np.allclose(Om, P.Omega_max)            # clipped to upper bound
