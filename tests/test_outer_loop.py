import numpy as np
from indi_harness.params import QuadParams
from indi_harness.runner import run_scenario
from indi_harness.trajectory import Hover

P = QuadParams()

def test_hover_hold():
    log = run_scenario(P, Hover(point=np.array([0, 0, -5.0])),
                       t_end=6.0, wind=np.zeros(3), drag_on=True,
                       p0=np.array([0, 0, -5.0]))
    err = np.linalg.norm(log["p"] - log["p_ref"], axis=1)
    assert err[log["t"] > 4.0].max() < 0.02

def test_large_lateral_step_stays_finite():
    # 8 m lateral step: transient tilts hard; quaternion must never go NaN
    log = run_scenario(P, Hover(point=np.array([8.0, 0, -5.0])),
                       t_end=4.0, drag_on=True, p0=np.array([0, 0, -5.0]))
    assert np.all(np.isfinite(log["q"]))
    assert np.all(np.isfinite(log["p"]))

def test_wind_rejection_no_integrator():
    log = run_scenario(P, Hover(point=np.array([0, 0, -5.0])),
                       t_end=8.0, wind=np.array([3.0, 0, 0]), drag_on=True,
                       p0=np.array([0, 0, -5.0]))
    err = np.linalg.norm(log["p"] - log["p_ref"], axis=1)
    assert err[log["t"] > 6.0].max() < 0.05
