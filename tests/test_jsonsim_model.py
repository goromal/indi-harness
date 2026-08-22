import numpy as np

from indi_harness.params import QuadParams
from indi_harness.sitl.jsonsim.model import QuadJsonModel

P = QuadParams()
DT = 1.0 / 400.0


def test_hover_equilibrium():
    """AC1: four motor commands at hover -> stays put, attitude stays level."""
    m = QuadJsonModel(P, drag_on=True, dt=DT)
    wh = P.hover_speed()
    m.seed_omega(np.full(4, wh))
    for _ in range(int(1.0 / DT)):
        m.step_omega(np.full(4, wh))
    st = m.state()
    assert np.linalg.norm(st["position"]) < 1e-3
    q = np.asarray(st["quaternion"])
    assert np.linalg.norm(q - np.array([1.0, 0.0, 0.0, 0.0])) < 1e-4


def _build_with_velocity(drag_on, v_target=np.array([2.0, 0.0, 0.0]), t_build=0.2):
    """Fallback velocity-injection: accelerate via a direct lateral body force
    applied through step_wrench, which bypasses the mixer/drag path entirely
    (drag is only computed inside _wrench/step_omega), so the resulting body
    velocity is identical regardless of the model's drag_on flag."""
    m = QuadJsonModel(P, drag_on=drag_on, dt=DT)
    F_build = np.asarray(v_target, float) * P.m / t_build
    for _ in range(int(t_build / DT)):
        m.step_wrench(np.concatenate([F_build, np.zeros(3)]))
    return m


def test_drag_opposes_velocity_when_on():
    """AC2: nonzero body velocity + drag_on=True -> accel_body opposes
    velocity; drag_on=False -> it does not."""
    m_on = _build_with_velocity(drag_on=True)
    m_off = _build_with_velocity(drag_on=False)

    v_on = np.asarray(m_on._m.x(m_on.t).twist).ravel()[:3]
    v_off = np.asarray(m_off._m.x(m_off.t).twist).ravel()[:3]
    assert np.linalg.norm(v_on) > 0.5
    # identical buildup procedure (step_wrench bypasses drag) -> identical velocity
    assert np.allclose(v_on, v_off, atol=1e-9)

    wh = P.hover_speed()
    m_on.seed_omega(np.full(4, wh))
    m_off.seed_omega(np.full(4, wh))
    m_on.step_omega(np.full(4, wh))
    m_off.step_omega(np.full(4, wh))

    a_on = np.asarray(m_on.state()["accel_body"])
    a_off = np.asarray(m_off.state()["accel_body"])

    # drag_on: horizontal accel opposes horizontal velocity
    assert np.dot(a_on[:2], v_on[:2]) < 0.0
    # drag_off: no rotor drag -> ~zero horizontal accel
    assert np.allclose(a_off[:2], 0.0, atol=1e-6)
    assert not np.allclose(a_on[:2], a_off[:2])


def test_roll_direction_matches_mixer_sign():
    """AC3: a mixer-consistent +tau_x command rolls the body with +x body-rate.
    Per params.mixer(): tau_x = kf*a*(-O0^2+O1^2+O2^2-O3^2) for motor order
    [FR,BL,FL,BR]; speeding up BL/FL and slowing FR/BR gives +tau_x."""
    m = QuadJsonModel(P, drag_on=False, dt=DT)
    wh = P.hover_speed()
    m.seed_omega(np.full(4, wh))
    cmd = wh * np.array([0.9, 1.1, 1.1, 0.9])
    for _ in range(int(0.1 / DT)):
        m.step_omega(cmd)
    st = m.state()
    assert st["gyro"][0] > 0.0


def test_accel_equals_wrench_over_mass():
    """AC4: accel_body == F_body / m."""
    m = QuadJsonModel(P, drag_on=True, dt=DT)
    wh = P.hover_speed()
    m.seed_omega(np.full(4, wh))
    wrench = m._wrench(np.full(4, wh))
    m.step_wrench(wrench)
    st = m.state()
    assert np.allclose(st["accel_body"], np.asarray(wrench[:3]) / P.m)
