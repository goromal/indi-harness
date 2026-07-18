import numpy as np
from indi_harness.params import QuadParams
from indi_harness.simmodel import QuadSim

P = QuadParams()

def test_hover_equilibrium():
    sim = QuadSim(P, drag_on=False)
    wh = P.hover_speed()
    sim.Omega = np.full(4, wh)   # start airborne: seed lagged motor state
    for _ in range(int(2.0 / sim.dt)):
        sim.step(np.full(4, wh))
    assert np.linalg.norm(sim.p) < 1e-3
    assert np.linalg.norm(sim.omega) < 1e-4

def test_free_fall():
    sim = QuadSim(P, drag_on=False)
    for _ in range(int(0.5 / sim.dt)):
        sim.step(np.zeros(4))
    assert np.isclose(sim.v[2], P.g * 0.5, rtol=1e-2)   # NED: +z down

def test_motor_first_order():
    sim = QuadSim(P, drag_on=False)
    target = 600.0
    for _ in range(int(P.tau_m / sim.dt)):
        sim.step(np.full(4, target))
    assert np.allclose(sim.Omega, 0.632 * target, rtol=0.02)

def test_accel_reads_minus_g_at_hover():
    sim = QuadSim(P, drag_on=False)
    sim.Omega = np.full(4, P.hover_speed())
    sim.step(np.full(4, P.hover_speed()))
    f_b = sim.specific_force()
    assert np.allclose(f_b, [0, 0, -P.g], atol=1e-2)

def test_yaw_reaction_sign():
    sim = QuadSim(P, drag_on=False)
    wh = P.hover_speed()
    sim.Omega = np.full(4, wh)
    # CCW pair (motors 0=FR, 1=BL, d=+1) faster -> positive yaw torque
    cmd = np.array([wh * 1.1, wh * 1.1, wh * 0.9, wh * 0.9])
    for _ in range(int(0.2 / sim.dt)):
        sim.step(cmd)
    assert sim.omega[2] > 0
