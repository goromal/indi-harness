"""Parse the Layer-B INDI outer-loop health message (INDB) from a fixture .BIN
captured from a CC3_OUTER_EN=1 attitude-only flight, and score flat-tracking
(design doc L: the .BIN is source of truth)."""
import pathlib
import numpy as np
import pytest
from indi_harness.sitl.binlog import read_outer_health
from indi_harness.sitl.align import flat_tracking_score

FIXTURE = pathlib.Path(__file__).parent / "fixtures" / "indi_layerB.BIN"


@pytest.mark.skipif(not FIXTURE.exists(), reason="Layer-B fixture .BIN not present")
def test_read_outer_health_shapes():
    h = read_outer_health(FIXTURE)
    n = len(h["time_us"])
    assert n > 0, "no INDB messages parsed from the fixture"

    # every field is a time series aligned to the message count
    assert h["ref_p"].shape == (n, 3)
    assert h["meas_p"].shape == (n, 3)
    assert h["t_cmd"].shape == (n,)
    assert h["fallback"].shape == (n,)

    # loop-consistent, monotonic timestamps
    assert np.all(np.diff(h["time_us"]) >= 0)

    # positions are finite (a diverging outer loop would show NaN/inf)
    for key in ("ref_p", "meas_p", "t_cmd"):
        assert np.all(np.isfinite(h[key])), f"{key} has non-finite values"

    # fallback flag is boolean-valued
    assert set(np.unique(h["fallback"])).issubset({0, 1})


@pytest.mark.skipif(not FIXTURE.exists(), reason="Layer-B fixture .BIN not present")
def test_flat_tracking_score_active():
    h = read_outer_health(FIXTURE)
    s = flat_tracking_score(h)

    # the outer loop actually engaged for a meaningful fraction of the flight
    assert s["n"] > 0
    assert 0.0 < s["active_frac"] <= 1.0
    assert s["n_active"] > 0

    # score fields are finite and ordered (max >= rms >= 0)
    assert np.isfinite(s["rms_m"]) and np.isfinite(s["max_m"])
    assert s["max_m"] >= s["rms_m"] >= 0.0

    # this fixture is a stable, tracking attitude-only flight: bounded error,
    # nowhere near the ~35 m collective-runaway or ~2.2 m parked-at-origin modes
    assert s["rms_m"] < 1.5, f"unexpectedly large tracking RMS {s['rms_m']}"


def test_flat_tracking_score_synthetic():
    """Unit-level: perfect tracking scores ~0; a known offset scores its norm.
    Runs without the fixture (pure array math)."""
    n = 50
    ref = np.zeros((n, 3))
    ref[:, 0] = np.linspace(0.0, 4.0, n)
    # measured trails ref by a constant 0.3 m in east; first 5 ticks fall back
    meas = ref.copy()
    meas[:, 1] -= 0.3
    fb = np.zeros(n, int)
    fb[:5] = 1
    health = {"ref_p": ref, "meas_p": meas,
              "t_cmd": np.zeros(n), "fallback": fb}

    s = flat_tracking_score(health)
    assert s["n"] == n
    assert s["n_active"] == n - 5
    assert abs(s["rms_m"] - 0.3) < 1e-9
    assert abs(s["max_m"] - 0.3) < 1e-9
    assert abs(s["active_frac"] - (n - 5) / n) < 1e-9

    # including the fallback ticks (which are also offset here) is still 0.3
    s_all = flat_tracking_score(health, active_only=False)
    assert abs(s_all["rms_m"] - 0.3) < 1e-9
    assert s_all["n_active"] == n
