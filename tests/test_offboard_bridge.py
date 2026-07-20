import numpy as np
from indi_harness import quat
from indi_harness.params import QuadParams
from indi_harness.trajectory import Hover, Circle
from indi_harness.offboard.bridge import OffboardOuterLoop, OffboardGains

P = QuadParams()
HOVER_THR = 0.35


def make(fs=50.0, indi_accel=True):
    return OffboardOuterLoop(P, OffboardGains(indi_accel=indi_accel),
                             fs=fs, hover_throttle=HOVER_THR)


def hover_state(p=np.array([0.0, 0.0, -10.0])):
    return dict(p=p, v=np.zeros(3), q=np.array([1.0, 0, 0, 0]),
                f_b=np.array([0.0, 0.0, -P.g]))  # accelerometer at hover


def test_hover_equilibrium():
    ctl = make()
    traj = Hover(point=np.array([0.0, 0.0, -10.0]))
    sp = None
    for k in range(200):  # let filters settle
        sp = ctl.tick(t=k / 50.0, traj=traj, **hover_state())
    tilt = 2.0 * np.degrees(np.arccos(min(1.0, abs(sp.q[0]))))
    assert tilt < 0.5
    assert abs(sp.thrust - HOVER_THR) < 0.02 * HOVER_THR


def test_north_error_pitches_forward():
    ctl = make()
    traj = Hover(point=np.array([1.0, 0.0, -10.0]))  # target 1 m north
    for k in range(200):
        sp = ctl.tick(t=k / 50.0, traj=traj, **hover_state())
    # pitch toward +N: body z tips so its NED-horizontal component is south
    z_b = quat.qrot(sp.q, np.array([0.0, 0.0, 1.0]))
    assert z_b[0] < -0.01  # z_b leans -N => thrust (-z_b) leans +N


def closed_loop_rmse(indi_accel, fs=50.0):
    ctl = make(fs=fs, indi_accel=indi_accel)
    traj = Circle(radius=2.0, period=8.0, alt=10.0)
    fo0 = traj.ref(0.0)
    p, v = fo0.p.copy(), fo0.v.copy()
    dt, errs = 1.0 / fs, []
    # "attitude tracked instantly" means the plant realizes the commanded
    # attitude sp.q each tick; a faithful state feedback must therefore return
    # that attitude AND the specific force sensed in it. Feeding a constant
    # level attitude + hover accel (as a hover-only plant would) hands the INDI
    # accel loop stale feedback on a tilted circle -> spurious ~0.4 m lag.
    q_prev = np.array([1.0, 0, 0, 0])
    sf_world_prev = np.array([0.0, 0.0, -P.g])  # specific force, world (NED)
    for k in range(int(16.0 * fs)):
        t = k * dt
        st = dict(p=p, v=v, q=q_prev,
                  f_b=quat.qrot_inv(q_prev, sf_world_prev))
        sp = ctl.tick(t=t, traj=traj, **st)
        # kinematic plant: attitude/thrust realized instantly
        T = sp.thrust / HOVER_THR * P.g          # mass-normalized
        a = np.array([0.0, 0.0, P.g]) + quat.qrot(sp.q, np.array([0.0, 0.0, -T]))
        sf_world_prev = a - np.array([0.0, 0.0, P.g])
        q_prev = sp.q
        v = v + a * dt
        p = p + v * dt
        if t > 2.0:
            errs.append(np.linalg.norm(p - traj.ref(t).p))
    return float(np.sqrt(np.mean(np.square(errs))))


def test_closed_loop_circle():
    assert closed_loop_rmse(indi_accel=True) < 0.15
    assert closed_loop_rmse(indi_accel=False) < 0.15
    assert closed_loop_rmse(indi_accel=True, fs=100.0) < 0.15
