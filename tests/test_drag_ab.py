"""Drag A/B cell scoring and the drag-effect gate (pure array math)."""
import numpy as np
import pytest

from indi_harness.offboard.traj_server import ramped_reference, reference_duration
from indi_harness.sitl.drag_ab import (
    CASES, GRID_DT, drag_effect_gate, error_track, fit_reference_start,
    score_custom_cases, score_stock_case, track_distance)


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


def _received(case, origin, t0_us, rate_hz=50.0):
    """A received-reference stream that was at trajectory time 0 at t0_us."""
    reference = ramped_reference(case)
    shift = np.asarray(origin) - reference.ref(0.0).p
    t = np.arange(0.0, reference_duration(case), 1.0 / rate_hz)
    return t0_us + t * 1e6, np.array([reference.ref(x).p for x in t]) + shift


def test_fit_reference_start_recovers_stream_start():
    case = CASES["circle_fast"]
    origin = np.array([0.5, -0.2, -10.0])
    # The stream's first sample is 0.137 s after trajectory time 0.
    ref_t, ref_p = _received(case, origin, t0_us=40e6)
    keep = ref_t >= 40e6 + 0.137e6
    t0 = fit_reference_start(ref_t[keep], ref_p[keep], case, origin)
    assert t0 == pytest.approx(40e6, abs=2e3)


def test_error_track_on_trajectory_grid():
    case = CASES["lemniscate_fast"]
    origin = np.array([0.0, 0.0, -10.0])
    ref_t, ref_p = _received(case, origin, t0_us=12e6)
    meas_p = ref_p + np.array([0.1, 0.0, 0.0])
    track = error_track(ref_t, ref_p, ref_t, meas_p, case, origin)
    e = np.asarray(track["e"])
    assert track["dt"] == GRID_DT
    assert len(e) == int(round(reference_duration(case) / GRID_DT)) + 1
    finite = np.isfinite(e[:, 0])
    assert finite.mean() > 0.95
    assert np.allclose(e[finite], [0.1, 0.0, 0.0], atol=1e-6)


def test_track_distance_ignores_uncovered_samples():
    a = [[0.0, 0.0, 0.0], [0.3, 0.4, 0.0], [np.nan] * 3]
    b = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
    assert track_distance(a, b) == pytest.approx(np.sqrt(0.25 / 2))


def _cell(*errors, n=50):
    """A score_cell result whose one case has a constant error track."""
    return {"cases": {"c": {"rms_m": float(np.linalg.norm(errors)),
                            "error_track": {"e": [list(errors)] * n}}}}


def _runs(*xs):
    return [_cell(x, 0.0, 0.0) for x in xs]


def test_gate_passes_when_drag_moves_stock_far_more():
    result = drag_effect_gate(
        stock_off=_runs(0.10, 0.11, 0.12), stock_on=_runs(-0.20, -0.21, -0.19),
        custom_off=_runs(0.00, 0.01, 0.00), custom_on=_runs(0.02, 0.03, 0.02),
        alpha=0.5)
    row = result["cases"][0]
    assert row["measurable"]
    assert row["stock_effect_m"] == pytest.approx([0.29, 0.33])
    assert row["custom_effect_m"][1] == pytest.approx(0.03)
    assert result["passed"]


def test_gate_fails_when_stock_effect_is_within_repeat_noise():
    result = drag_effect_gate(
        stock_off=_runs(0.00, 0.20), stock_on=_runs(0.10, 0.12),
        custom_off=_runs(0.00, 0.00), custom_on=_runs(0.00, 0.00), alpha=0.5)
    assert not result["cases"][0]["measurable"]
    assert not result["passed"]


def test_gate_fails_when_custom_moves_too_much():
    result = drag_effect_gate(
        stock_off=_runs(0.10, 0.10), stock_on=_runs(-0.10, -0.10),
        custom_off=_runs(0.00, 0.00), custom_on=_runs(0.15, 0.10), alpha=0.5)
    assert result["cases"][0]["measurable"]
    assert not result["passed"]


def test_gate_needs_repeats_to_be_measurable():
    result = drag_effect_gate(
        stock_off=_runs(0.10), stock_on=_runs(-0.10),
        custom_off=_runs(0.00), custom_on=_runs(0.01), alpha=0.5)
    assert not result["cases"][0]["measurable"]
    assert not result["passed"]
