"""Unit tests for the pure scoring helpers in sitl.binscore (rms, active_runs,
and the windowed extractors). Pure array math -- no .BIN fixture required."""
import numpy as np

from indi_harness.sitl.binscore import (
    rms,
    active_runs,
    gyro_window,
    omega_window,
    rate_window,
)


def test_rms_basic():
    assert rms([]) == 0.0
    assert rms([0.0, 0.0, 0.0]) == 0.0
    assert abs(rms([2.0, 2.0, 2.0]) - 2.0) < 1e-12
    # rms of [3,4] = sqrt((9+16)/2) = sqrt(12.5)
    assert abs(rms([3.0, 4.0]) - np.sqrt(12.5)) < 1e-12


def test_active_runs_segments_and_min_len():
    # two long fallback==0 runs separated by a fallback==1 gap, plus a short run
    # that must be dropped by min_len.
    fb = np.array([1, 1] + [0] * 60 + [1, 1, 1] + [0] * 55 + [1] + [0] * 10)
    runs = active_runs(fb, min_len=50)
    assert len(runs) == 2
    (a0, a1), (b0, b1) = runs
    assert a0 == 2 and a1 == 62
    assert b0 == 65 and b1 == 120
    # the trailing 10-long run is below min_len -> excluded
    assert all(j - i >= 50 for i, j in runs)


def test_active_runs_all_active():
    fb = np.zeros(100, int)
    assert active_runs(fb, min_len=50) == [(0, 100)]


def test_active_runs_none_active():
    fb = np.ones(100, int)
    assert active_runs(fb, min_len=50) == []


def test_gyro_window_empty():
    out = gyro_window(np.array([]), np.zeros((0, 3)), 0.0, 1.0)
    assert out["n"] == 0
    assert out["roll"]["rms"] == 0.0 and out["pitch"]["buzz_hp_rms"] == 0.0


def test_gyro_window_flags_buzz():
    # roll axis: smooth (near-constant) -> low buzz_hp_rms; pitch: alternating
    # high-frequency (limit-cycle) -> high buzz_hp_rms.
    n = 200
    gt = np.arange(n, dtype=float)
    gyro = np.zeros((n, 3))
    gyro[:, 0] = np.deg2rad(10.0)                      # steady roll rate
    gyro[:, 1] = np.deg2rad(30.0) * (-1.0) ** np.arange(n)  # buzzing pitch
    out = gyro_window(gt, gyro, 0.0, float(n))
    assert out["n"] == n
    assert out["roll"]["buzz_hp_rms"] < 1.0
    assert out["pitch"]["buzz_hp_rms"] > 10.0


def test_omega_window_perfect_and_windowed():
    n = 100
    it = np.arange(n, dtype=float)
    meas = np.zeros((n, 3))
    meas[:, 0] = np.sin(it / 5.0)     # excited roll
    meas[:, 1] = np.cos(it / 5.0)     # excited pitch
    ih = {
        "time_us": it,
        "domega_pred": meas.copy(),   # perfect prediction -> nrmse ~ 0
        "domega_meas": meas,
        "du": np.ones((n, 3)) * 0.5,
        "sat": np.zeros(n, int),
    }
    out = omega_window(ih, 0.0, float(n))
    assert out["roll"]["nrmse"] < 1e-9
    assert out["pitch"]["nrmse"] < 1e-9
    assert abs(out["du_rms"][0] - 0.5) < 1e-9
    assert out["sat_frac"] == 0.0


def test_rate_window_buzz_proxy():
    n = 100
    rt = np.arange(n, dtype=float)
    rdes = np.zeros(n)
    ract = 5.0 * (-1.0) ** np.arange(n)   # alternating -> big diff() rms
    pdes = np.zeros(n)
    pact = np.zeros(n)
    out = rate_window(rt, rdes, ract, pdes, pact, 0.0, float(n))
    assert out["roll"]["buzz_ddt_rms"] > 5.0
    assert out["pitch"]["buzz_ddt_rms"] == 0.0
