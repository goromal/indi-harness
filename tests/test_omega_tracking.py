import numpy as np
from indi_harness.sitl.align import (
    omega_tracking_score, omega_gate_ok, g1_ceiling_ok)


def _health(pred, meas):
    return {"domega_pred": pred, "domega_meas": meas}


def test_tracking_scores_low_nrmse():
    t = np.linspace(0, 10, 2000)
    meas = np.stack([np.sin(t), np.cos(t), 0.3*np.sin(2*t)], axis=1)
    pred = meas + 0.02 * np.random.default_rng(0).standard_normal(meas.shape)
    s = omega_tracking_score(_health(pred, meas))
    assert all(s[ax]["nrmse"] < 0.3 for ax in range(3))


def test_divergence_scores_high_nrmse():
    # Layer A's failure mode: an attenuated predictor (pred ~ 0.1*meas). Under
    # RMS-normalization this scores NRMSE ~ 0.9 -- correctly rejected by the
    # gate. (Range-normalization would score it ~0.3 and let it slip through,
    # which is exactly the metric bug this convention avoids.)
    t = np.linspace(0, 10, 2000)
    meas = np.stack([5*np.sin(t)]*3, axis=1)
    pred = 0.1 * meas
    s = omega_tracking_score(_health(pred, meas))
    assert all(s[ax]["nrmse"] > 0.5 for ax in range(3))


def test_gate_skips_unexcited_axis():
    # roll+pitch excited & tracking; yaw quiescent (measured omega_dot ~ 0) with a
    # meaningless/huge NRMSE. The gate must PASS -- skipping the unexcited yaw --
    # not fire a phantom failure on it (the circle_slow-yaw / step-roll artifact).
    t = np.linspace(0, 10, 2000)
    rng = np.random.default_rng(0)
    excited = np.sin(t)                      # exc_rms ~ 0.7 >> floor
    quiescent = 1e-3 * rng.standard_normal(t.shape)   # exc_rms ~ 0 << floor
    meas = np.stack([excited, excited, quiescent], axis=1)
    pred = np.stack([excited + 0.05*rng.standard_normal(t.shape),
                     excited + 0.05*rng.standard_normal(t.shape),
                     -quiescent], axis=1)    # yaw pred anti-correlated -> NRMSE huge
    ok, per = omega_gate_ok(_health(pred, meas))
    assert per[2]["excited"] is False and per[2]["nrmse"] > 1.0   # yaw: bad but skipped
    assert per[0]["excited"] and per[1]["excited"]
    assert ok                                # passes on the excited axes alone


def test_gate_does_not_enforce_yaw_even_when_excited():
    # The real SITL-battery case: yaw IS excited (the trajectory commands a
    # continuous facing-turn) but its omega_dot-inversion is deferred to HW/S4
    # (needs the G2/RPM term SITL can't model). Roll/pitch track; yaw is excited
    # with a bad NRMSE. The default gate (axes=(0,1)) must PASS -- reporting yaw
    # but not enforcing it -- which mirrors run1: roll 0.48, pitch 0.49, yaw 2.46.
    t = np.linspace(0, 10, 2000)
    rng = np.random.default_rng(1)
    rp = np.sin(t)
    yaw = 1.4 * np.sin(0.5 * t)              # genuinely excited (exc_rms >> floor)
    meas = np.stack([rp, rp, yaw], axis=1)
    pred = np.stack([rp + 0.05*rng.standard_normal(t.shape),
                     rp + 0.05*rng.standard_normal(t.shape),
                     0.2 * yaw], axis=1)     # yaw under-scaled -> NRMSE ~0.8 (>0.75)
    ok, per = omega_gate_ok(_health(pred, meas))
    assert per[2]["excited"] and not per[2]["enforced"]   # yaw excited but not gated
    assert per[2]["nrmse"] > 0.75                          # would fail if enforced
    assert per[0]["enforced"] and per[1]["enforced"]
    assert ok                                              # passes: roll/pitch only


def test_gate_fails_when_enforced_axis_untracked():
    # If an ENFORCED axis (roll/pitch) fails to track, the gate must fail.
    t = np.linspace(0, 10, 2000)
    meas = np.stack([np.sin(t)]*3, axis=1)   # all excited
    pred = 0.1 * meas                        # Layer-A attenuation -> NRMSE ~0.9
    ok, per = omega_gate_ok(_health(pred, meas))
    assert per[0]["excited"] and per[0]["enforced"]
    assert not ok


def test_g1_ceiling():
    assert g1_ceiling_ok(400.0, analytic=200.0, factor=3.0)
    assert not g1_ceiling_ok(1000.0, analytic=200.0, factor=3.0)
