import numpy as np
import pytest
from indi_harness import quat

RNG = np.random.default_rng(0)

def test_qnormalize_rejects_zero():
    with pytest.raises(ValueError):
        quat.qnormalize(np.zeros(4))

def rand_q():
    return quat.qnormalize(RNG.normal(size=4))

def test_qmul_identity():
    q = rand_q()
    qi = np.array([1.0, 0, 0, 0])
    assert np.allclose(quat.qmul(qi, q), q)
    assert np.allclose(quat.qmul(q, quat.qconj(q)), qi, atol=1e-12)

def test_qrot_matches_matrix():
    for _ in range(20):
        q, v = rand_q(), RNG.normal(size=3)
        assert np.allclose(quat.qrot(q, v), quat.R_from_q(q) @ v, atol=1e-11)

def test_qrot_inv_is_inverse():
    q, v = rand_q(), RNG.normal(size=3)
    assert np.allclose(quat.qrot_inv(q, quat.qrot(q, v)), v, atol=1e-11)

def test_exp_log_roundtrip():
    for _ in range(20):
        rv = RNG.normal(size=3) * 2.0
        assert np.allclose(quat.qlog(quat.qexp(rv)), rv, atol=1e-9)
    assert np.allclose(quat.qexp(np.zeros(3)), [1, 0, 0, 0])

def test_q_from_R_roundtrip():
    for _ in range(20):
        q = rand_q()
        if q[0] < 0:
            q = -q
        assert np.allclose(quat.q_from_R(quat.R_from_q(q)), q, atol=1e-9)

def test_qerr_no_unwinding():
    q, qr = rand_q(), rand_q()
    e1, e2 = quat.qerr(q, qr), quat.qerr(q, -qr)
    assert e1[0] >= 0 and np.allclose(e1, e2)

def test_qerr_zero_when_aligned():
    q = rand_q()
    assert np.allclose(quat.qerr(q, q), [1, 0, 0, 0], atol=1e-12)
