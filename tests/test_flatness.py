import numpy as np
import pytest
from indi_harness import quat
from indi_harness.trajectory import Circle, Lemniscate, Hover, FlatOutput
from indi_harness.flatness import flat_reference

M, G = 1.0, 9.81
H = 1e-4

def test_hover():
    ref = flat_reference(Hover(point=np.array([0, 0, -5.0])), 1.0, M, G)
    assert np.allclose(ref.q, [1, 0, 0, 0], atol=1e-9)
    assert np.allclose(ref.w, 0, atol=1e-9)
    assert np.isclose(ref.T, M * G)

@pytest.mark.parametrize("traj", [
    Circle(radius=2.0, period=6.0, alt=5.0),
    Lemniscate(amplitude=2.0, period=6.0, alt=5.0),
])
def test_omega_matches_fd_of_attitude(traj):
    for t in np.linspace(0.5, 5.0, 8):
        r0 = flat_reference(traj, t, M, G)
        rp = flat_reference(traj, t + H, M, G)
        w_fd = quat.qlog(quat.qmul(quat.qconj(r0.q), rp.q)) / H
        assert np.allclose(r0.w, w_fd, atol=1e-3), f"t={t}: {r0.w} vs {w_fd}"

@pytest.mark.parametrize("traj", [
    Circle(radius=2.0, period=6.0, alt=5.0),
    Lemniscate(amplitude=2.0, period=6.0, alt=5.0),
])
def test_domega_matches_fd_of_omega(traj):
    for t in np.linspace(0.5, 5.0, 8):
        rm = flat_reference(traj, t - H, M, G)
        rp = flat_reference(traj, t + H, M, G)
        dw_fd = (rp.w - rm.w) / (2 * H)
        r0 = flat_reference(traj, t, M, G)
        assert np.allclose(r0.dw, dw_fd, atol=1e-2), f"t={t}: {r0.dw} vs {dw_fd}"

def test_yaw_rate_feedforward():
    class SpinHover(Hover):
        def ref(self, t):
            fo = super().ref(t)
            return FlatOutput(fo.p, fo.v, fo.a, fo.j, fo.s, 0.3 * t, 0.3, 0.0)
    r = flat_reference(SpinHover(point=np.zeros(3)), 2.0, M, G)
    assert np.isclose(r.w[2], 0.3, atol=1e-6)
