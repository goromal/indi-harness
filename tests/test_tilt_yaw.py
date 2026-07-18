import numpy as np
from indi_harness import quat
from indi_harness.tilt_yaw import tilt_yaw_decompose, attitude_rate_ref

RNG = np.random.default_rng(1)

def rand_q():
    q = quat.qnormalize(RNG.normal(size=4))
    return q if q[0] >= 0 else -q

def test_composition():
    for _ in range(50):
        qe = rand_q()
        q_red, q_yaw = tilt_yaw_decompose(qe)
        assert np.allclose(quat.qmul(q_red, q_yaw), qe, atol=1e-10)

def test_structure():
    qe = rand_q()
    q_red, q_yaw = tilt_yaw_decompose(qe)
    assert abs(q_red[3]) < 1e-10          # reduced error has no z component
    assert np.allclose(q_yaw[1:3], 0, atol=1e-12)  # yaw error is pure z

def test_singularity_finite():
    qe = np.array([0.0, 1.0, 0.0, 0.0])   # 180 deg tilt about x
    q_red, q_yaw = tilt_yaw_decompose(qe)
    assert np.all(np.isfinite(q_red)) and np.all(np.isfinite(q_yaw))
    assert np.allclose(np.linalg.norm(q_red), 1.0)
    assert np.allclose(quat.qmul(q_red, q_yaw), qe, atol=1e-12)

def test_pure_tilt_uses_tilt_gain():
    q = np.array([1.0, 0, 0, 0])
    q_ref = quat.qexp(np.array([0.4, 0, 0]))  # 0.4 rad roll error
    w = attitude_rate_ref(q, q_ref, kp_tilt=10.0, kp_yaw=3.0, w_ff=np.zeros(3))
    assert np.isclose(w[0], 10.0 * 0.4, atol=1e-6)
    assert np.allclose(w[1:], 0, atol=1e-10)

def test_rate_ref_zero_at_zero_error():
    q = rand_q()
    w_ff = np.array([0.1, -0.2, 0.3])
    w = attitude_rate_ref(q, q, kp_tilt=10.0, kp_yaw=3.0, w_ff=w_ff)
    assert np.allclose(w, w_ff, atol=1e-10)

def test_pure_yaw_uses_yaw_gain():
    q = np.array([1.0, 0, 0, 0])
    q_ref = quat.qexp(np.array([0, 0, 0.4]))  # 0.4 rad yaw error
    w = attitude_rate_ref(q, q_ref, kp_tilt=10.0, kp_yaw=3.0, w_ff=np.zeros(3))
    assert np.allclose(w[:2], 0, atol=1e-10)
    assert np.isclose(w[2], 3.0 * 0.4, atol=1e-6)
