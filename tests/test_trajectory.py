import pathlib

import numpy as np
import pytest
from indi_harness.trajectory import (Circle, Lemniscate, Hover,
                                     PolynomialTrajectory, FlatOutput)

H = 1e-3
REPO = pathlib.Path(__file__).resolve().parent.parent

def fd_check(traj, t):
    """Each returned derivative must match central FD of the one below it."""
    fo = traj.ref(t)
    fp, fm = traj.ref(t + H), traj.ref(t - H)
    assert np.allclose((fp.p - fm.p) / (2 * H), fo.v, atol=1e-4)
    assert np.allclose((fp.v - fm.v) / (2 * H), fo.a, atol=1e-4)
    assert np.allclose((fp.a - fm.a) / (2 * H), fo.j, atol=1e-3)
    assert np.allclose((fp.j - fm.j) / (2 * H), fo.s, atol=1e-2)
    assert np.isclose((fp.psi - fm.psi) / (2 * H), fo.dpsi, atol=1e-4)
    assert np.isclose((fp.dpsi - fm.dpsi) / (2 * H), fo.ddpsi, atol=1e-3)

@pytest.mark.parametrize("traj", [
    Circle(radius=2.0, period=8.0, alt=5.0),
    Lemniscate(amplitude=2.0, period=6.0, alt=5.0),
])
def test_analytic_derivative_chain(traj):
    for t in np.linspace(0.5, 5.5, 7):
        fd_check(traj, t)

def test_hover_is_constant():
    h = Hover(point=np.array([1.0, 2.0, -5.0]))
    fo = h.ref(3.0)
    assert np.allclose(fo.p, [1, 2, -5])
    for term in (fo.v, fo.a, fo.j, fo.s):
        assert np.allclose(term, 0)

def test_polynomial_from_yaml():
    traj = PolynomialTrajectory.from_yaml(REPO / "trajectories/poly_test.yaml")
    assert traj.duration > 0
    for t in np.linspace(0.1, traj.duration - 0.1, 5):
        fd_check(traj, t)

def test_multi_segment_boundary():
    z = {"y": np.array([0.0]), "z": np.array([-5.0]), "yaw": np.array([0.0])}
    traj = PolynomialTrajectory([
        (2.0, {"x": np.array([0.0, 1.0]), **z}),   # x = t
        (2.0, {"x": np.array([2.0, 1.0]), **z}),   # x = 2 + t_local
    ])
    assert traj.duration == 4.0
    # position continuous across the segment boundary
    assert np.isclose(traj.ref(2.0 - 1e-6).p[0], traj.ref(2.0 + 1e-6).p[0],
                      atol=1e-5)
    # querying exactly at t == duration must not fall off the end
    assert np.isclose(traj.ref(4.0).p[0], 4.0)

def test_ned_altitude_is_negative_z():
    # alt=5 means z = -5 in NED
    assert Circle(radius=1.0, period=4.0, alt=5.0).ref(0.0).p[2] == -5.0
