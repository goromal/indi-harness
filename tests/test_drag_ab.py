"""Drag A/B cell scoring and the relative-gap assertion (pure array math)."""
import numpy as np
import pytest

from indi_harness.offboard.traj_server import ramped_reference, reference_duration
from indi_harness.sitl.drag_ab import (
    CASES, drag_gap, score_custom_cases, score_stock_case)


def _stock_flight(case, origin, offset=np.zeros(3), speedup=2.0):
    """A stock case on a SITL clock running `speedup`x the trajectory clock,
    with the EKF flying the ramped reference plus a constant offset."""
    reference = ramped_reference(case)
    duration = reference_duration(case)
    traj_t = np.arange(0.0, duration, 0.02)
    boot_ms = 50_000.0 + traj_t * 1000.0 * speedup
    time_us = np.arange(boot_ms[0] - 2000.0, boot_ms[-1] + 2000.0, 10.0) * 1000.0
    t = (time_us / 1000.0 - 50_000.0) / (1000.0 * speedup)
    shift = origin - reference.ref(0.0).p
    p = np.array([reference.ref(min(max(x, 0.0), duration)).p for x in t])
    entry = {"case": case.name, "origin": list(origin),
             "traj_t": list(traj_t), "boot_ms": list(boot_ms)}
    return time_us, p + shift + offset, entry


def test_stock_case_zero_error_on_reference():
    case = CASES["circle_fast"]
    time_us, p, entry = _stock_flight(case, np.array([1.0, -2.0, -10.0]))
    s = score_stock_case(time_us, p, entry, case)
    assert s["rms_m"] < 1e-6 and s["n"] > 0


def test_stock_case_scores_constant_offset():
    case = CASES["lemniscate_fast"]
    time_us, p, entry = _stock_flight(case, np.array([0.0, 0.0, -10.0]),
                                      offset=np.array([0.3, 0.4, 0.0]))
    s = score_stock_case(time_us, p, entry, case)
    assert s["rms_m"] == pytest.approx(0.5, abs=1e-6)
    assert s["max_m"] == pytest.approx(0.5, abs=1e-6)


def test_custom_cases_split_by_engaged_block():
    fb = np.array([1] * 5 + [0] * 60 + [1] * 5 + [0] * 70 + [1] * 5)
    ref = np.zeros((fb.size, 3))
    meas = np.zeros((fb.size, 3))
    meas[5:65, 0] = 0.2
    meas[70:140, 1] = 1.0
    health = {"fallback": fb, "ref_p": ref, "meas_p": meas}
    stats = score_custom_cases(health, [{"case": "a"}, {"case": "b"}])
    assert [s["rms_m"] for s in stats] == pytest.approx([0.2, 1.0])
    with pytest.raises(ValueError):
        score_custom_cases(health, [{"case": "a"}])


def _cell(controller, **rms):
    return {"controller": controller,
            "cases": {k: {"rms_m": v} for k, v in rms.items()}}


def test_drag_gap_relative_assertion():
    result = drag_gap(_cell("stock", c=0.30), _cell("stock", c=0.70),
                      _cell("custom", c=0.10), _cell("custom", c=0.25),
                      alpha=0.5)
    row = result["cases"][0]
    assert row["stock_delta_m"] == pytest.approx(0.40)
    assert row["custom_delta_m"] == pytest.approx(0.15)
    assert result["passed"]
    # INDI degrading by more than alpha times PID's degradation fails.
    result = drag_gap(_cell("stock", c=0.30), _cell("stock", c=0.70),
                      _cell("custom", c=0.10), _cell("custom", c=0.35),
                      alpha=0.5)
    assert not result["passed"]


def test_drag_gap_requires_stock_degradation():
    result = drag_gap(_cell("stock", c=0.30), _cell("stock", c=0.30),
                      _cell("custom", c=0.10), _cell("custom", c=0.05),
                      alpha=0.5)
    assert not result["cases"][0]["stock_degraded"]
    assert not result["passed"]
