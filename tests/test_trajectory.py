import pathlib

import numpy as np
import pytest
from indi_harness.trajectory import (Circle, Lemniscate, Hover,
                                     PolynomialTrajectory, FlatOutput,
                                     SmoothStart, SmoothStop)

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


STOP_PATHS = [Circle(radius=2.0, period=8.0, alt=10.0),
              Lemniscate(amplitude=2.0, period=8.0, alt=10.0)]


@pytest.mark.parametrize("path", STOP_PATHS)
def test_smooth_stop_follows_the_path_until_braking(path):
    stop = SmoothStop(path, end_time=16.0, brake_s=3.0)
    assert stop.duration == pytest.approx(17.5)
    for t in (0.0, 5.0, 14.49):
        a, b = stop.ref(t), path.ref(t)
        assert np.allclose(a.p, b.p) and np.allclose(a.v, b.v)
        assert np.allclose(a.s, b.s)


@pytest.mark.parametrize("path", STOP_PATHS)
def test_smooth_stop_derivatives_stay_consistent_through_braking(path):
    stop = SmoothStop(path, end_time=16.0, brake_s=3.0)
    for t in np.linspace(14.0, 18.0, 17):
        fd_check(stop, t)


@pytest.mark.parametrize("path", STOP_PATHS)
def test_smooth_stop_rests_at_the_path_end_point(path):
    # The reference must come to rest where the path ends rather than hold the
    # end point while still commanding the path's end velocity.
    stop = SmoothStop(path, end_time=16.0, brake_s=3.0)
    for t in (stop.duration, stop.duration + 5.0):
        fo = stop.ref(t)
        assert np.allclose(fo.p, path.ref(16.0).p, atol=1e-9)
        for d in (fo.v, fo.a, fo.j, fo.s):
            assert np.allclose(d, 0.0, atol=1e-9)


@pytest.mark.parametrize("path", STOP_PATHS)
def test_smooth_start_begins_at_rest_on_the_path_start_point(path):
    # The paths begin at full speed; a vehicle engaging from hover lagged the
    # reference by ~0.8 m before it caught up.
    start = SmoothStart(path, ramp_s=3.0)
    assert start.duration == pytest.approx(path.duration + 1.5)
    for t in (-1.0, 0.0):
        fo = start.ref(t)
        assert np.allclose(fo.p, path.ref(0.0).p, atol=1e-9)
        for d in (fo.v, fo.a, fo.j, fo.s):
            assert np.allclose(d, 0.0, atol=1e-9)


@pytest.mark.parametrize("path", STOP_PATHS)
def test_smooth_start_derivatives_stay_consistent_through_the_ramp(path):
    start = SmoothStart(path, ramp_s=3.0)
    for t in np.linspace(0.01, 4.0, 17):
        fd_check(start, t)


@pytest.mark.parametrize("path", STOP_PATHS)
def test_smooth_start_follows_the_path_after_the_ramp(path):
    start = SmoothStart(path, ramp_s=3.0)
    for t in (3.0, 5.0, 7.5):
        a, b = start.ref(t), path.ref(t - 1.5)
        assert np.allclose(a.p, b.p) and np.allclose(a.v, b.v)
        assert np.allclose(a.s, b.s)
