"""S0 exit criterion (design doc: .claude/ardupilot-indi-plan.md, section S0).

Green here == S0 complete; Stage 2 (S1 SITL baseline) may begin.
"""
import numpy as np
import pytest
from indi_harness.params import QuadParams
from indi_harness.runner import run_scenario
from indi_harness.trajectory import Hover, Lemniscate
from indi_harness.evalmetrics import rmse_position

P = QuadParams()
LEM = dict(amplitude=2.0, period=6.0, alt=5.0)


def lem_run(wind, ff_on):
    log = run_scenario(P, Lemniscate(**LEM), t_end=12.0,
                       wind=wind, drag_on=True, ff_on=ff_on)
    _, total = rmse_position(log["t"], log["p"], log["p_ref"], trim_s=2.0)
    return total


def test_lemniscate_tracking_ff_on():
    assert lem_run(np.zeros(3), ff_on=True) < 0.15


def test_wind_degradation_bounded():
    calm = lem_run(np.zeros(3), ff_on=True)
    windy = lem_run(np.array([3.0, 0.0, 0.0]), ff_on=True)
    assert windy < 1.6 * calm, f"wind blew up tracking: {calm:.3f} -> {windy:.3f}"


def test_feedforward_reduces_error():
    ff = lem_run(np.zeros(3), ff_on=True)
    fb = lem_run(np.zeros(3), ff_on=False)
    assert ff < 0.6 * fb, f"feedforward not earning its keep: ff={ff:.3f} fb={fb:.3f}"


def test_disturbance_step_no_windup():
    pulse = lambda t: (np.array([4.0, 0.0, 0.0])
                       if 3.0 <= t < 4.0 else np.zeros(3))
    log = run_scenario(P, Hover(point=np.array([0, 0, -5.0])), t_end=8.0,
                       drag_on=True, f_ext=pulse, p0=np.array([0, 0, -5.0]))
    err = np.linalg.norm(log["p"] - log["p_ref"], axis=1)
    t = log["t"]
    assert err[(t > 6.0)].max() < 0.05, "did not recover within 2 s of pulse end"
    assert err[t > 7.0].max() < 0.02, "post-recovery offset (windup-like)"
