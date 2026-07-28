import numpy as np
from indi_harness.sitl.align import omega_tracking_score, g1_ceiling_ok


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


def test_g1_ceiling():
    assert g1_ceiling_ok(400.0, analytic=200.0, factor=3.0)
    assert not g1_ceiling_ok(1000.0, analytic=200.0, factor=3.0)
