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
    # Range-normalized NRMSE only crosses 0.5 for a genuinely diverging
    # (e.g. anti-correlated) predictor, not merely an attenuated one -- a
    # 0.1x-scaled prediction still tracks the shape and stays well under
    # 0.5. Use a sign-inverted "prediction" as the strongly-diverging case.
    t = np.linspace(0, 10, 2000)
    meas = np.stack([5*np.sin(t)]*3, axis=1)
    pred = -meas
    s = omega_tracking_score(_health(pred, meas))
    assert s[0]["nrmse"] > 0.5


def test_g1_ceiling():
    assert g1_ceiling_ok(400.0, analytic=200.0, factor=3.0)
    assert not g1_ceiling_ok(1000.0, analytic=200.0, factor=3.0)
