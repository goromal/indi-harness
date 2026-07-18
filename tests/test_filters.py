import numpy as np
import pytest
from indi_harness.filters import Butter2

FS = 500.0

def test_rejects_bad_cutoff():
    for bad in (0.0, -5.0, FS / 2, FS):
        with pytest.raises(ValueError):
            Butter2(cutoff_hz=bad, fs_hz=FS, n_ch=1)

def test_rejects_wrong_shape():
    f = Butter2(20.0, FS, 2)
    with pytest.raises(ValueError):
        f.update(np.zeros(3))

def test_channel_independence():
    f = Butter2(20.0, FS, 2)
    t = np.arange(0, 2.0, 1 / FS)
    for ti in t:
        y = f.update(np.array([np.sin(2 * np.pi * 3.0 * ti), 0.0]))
    assert abs(y[1]) < 1e-12  # quiet channel stays exactly quiet

def test_dc_gain_unity():
    f = Butter2(cutoff_hz=20.0, fs_hz=FS, n_ch=1)
    y = 0.0
    for _ in range(3000):
        y = f.update(np.array([1.0]))[0]
    assert abs(y - 1.0) < 1e-6

def test_attenuation_at_5x_cutoff():
    fc = 20.0
    f = Butter2(fc, FS, 1)
    t = np.arange(0, 4.0, 1 / FS)
    x = np.sin(2 * np.pi * 5 * fc * t)
    y = np.array([f.update(np.array([xi]))[0] for xi in x])
    gain = np.abs(y[len(y) // 2:]).max()
    assert gain < 10 ** (-20 / 20)

def test_phase_match_derivative_commutes():
    """Same filter applied to x and to dx/dt: filtered derivative must equal
    derivative of filtered signal (identical phase lag)."""
    fc = 15.0
    fx, fdx = Butter2(fc, FS, 1), Butter2(fc, FS, 1)
    t = np.arange(0, 6.0, 1 / FS)
    x = np.sin(2 * np.pi * 3.0 * t)
    dx = 2 * np.pi * 3.0 * np.cos(2 * np.pi * 3.0 * t)
    yx = np.array([fx.update(np.array([v]))[0] for v in x])
    ydx = np.array([fdx.update(np.array([v]))[0] for v in dx])
    dyx = np.gradient(yx, 1 / FS)
    half = len(t) // 2
    assert np.allclose(ydx[half:], dyx[half:], atol=0.02 * np.abs(ydx).max())

def test_reset_no_transient():
    f = Butter2(20.0, FS, 2)
    f.reset(np.array([3.0, -1.0]))
    y = f.update(np.array([3.0, -1.0]))
    assert np.allclose(y, [3.0, -1.0], atol=1e-9)
