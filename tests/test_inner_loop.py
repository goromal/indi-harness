import numpy as np
from indi_harness import quat
from indi_harness.params import QuadParams
from indi_harness.simmodel import QuadSim
from indi_harness.inner_loop import InnerLoopINDI, InnerGains

P = QuadParams()
FS = 500.0

def run_roll_step(ctrl_params, angle=np.deg2rad(20), t_end=1.5):
    sim = QuadSim(P, drag_on=False)
    sim.Omega = np.full(4, P.hover_speed())
    ctrl = InnerLoopINDI(ctrl_params, InnerGains(), fs=FS)
    q_ref = quat.qexp(np.array([angle, 0, 0]))
    T_hover = P.m * P.g
    n_sub = int(round(1 / FS / sim.dt))
    roll, t = [], []
    for k in range(int(t_end * FS)):
        Om_cmd, diag = ctrl.update(1 / FS, sim.omega, sim.q, q_ref,
                                   np.zeros(3), np.zeros(3), T_hover, sim.Omega)
        for _ in range(n_sub):
            sim.step(Om_cmd)
        rv = quat.qlog(sim.q)
        roll.append(rv[0]); t.append(k / FS)
    return np.array(t), np.array(roll), quat.qlog(sim.q)

def test_roll_step_settles():
    t, roll, rv_end = run_roll_step(P)
    angle = np.deg2rad(20)
    settled = np.abs(roll - angle) < 0.05 * angle
    assert settled[t > 0.6].all(), "did not settle within 0.6 s"
    assert roll.max() < 1.15 * angle, "overshoot > 15%"
    assert abs(rv_end[2]) < np.deg2rad(2), "yaw disturbed"

def test_robust_to_effectiveness_error():
    t, roll, _ = run_roll_step(P.perturbed(kf_scale=1.3))
    angle = np.deg2rad(20)
    assert np.abs(roll[t > 1.0] - angle).max() < 0.1 * angle

def test_health_diagnostics_present():
    sim = QuadSim(P, drag_on=False)
    sim.Omega = np.full(4, P.hover_speed())
    ctrl = InnerLoopINDI(P, InnerGains(), fs=FS)
    _, diag = ctrl.update(1 / FS, sim.omega, sim.q, np.array([1.0, 0, 0, 0]),
                          np.zeros(3), np.zeros(3), P.m * P.g, sim.Omega)
    for key in ("domega_pred", "domega_meas_f", "sat"):
        assert key in diag
