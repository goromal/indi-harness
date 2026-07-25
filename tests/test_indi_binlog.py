"""Parse the Layer-A INDI health message from a fixture .BIN captured from a
CC_TYPE=INDI SITL hover (design doc L: the .BIN is source of truth)."""
import pathlib
import numpy as np
import pytest
from indi_harness.sitl.binlog import read_indi_health

FIXTURE = pathlib.Path(__file__).parent / "fixtures" / "indi_health.BIN"


@pytest.mark.skipif(not FIXTURE.exists(), reason="INDI fixture .BIN not present")
def test_read_indi_health_shapes_and_health():
    h = read_indi_health(FIXTURE)
    n = len(h["time_us"])
    assert n > 0, "no INDI messages parsed from the fixture"

    # every field is a time series aligned to the message count
    assert h["domega_pred"].shape == (n, 3)
    assert h["domega_meas"].shape == (n, 3)
    assert h["u_act"].shape == (n, 3)
    assert h["du"].shape == (n, 3)
    assert h["sat"].shape == (n,)

    # loop-consistent, monotonic timestamps
    assert np.all(np.diff(h["time_us"]) >= 0)

    # health signals are finite (a diverging INDI loop would show NaN/inf)
    for key in ("domega_pred", "domega_meas", "u_act", "du"):
        assert np.all(np.isfinite(h[key])), f"{key} has non-finite values"

    # saturation flag is boolean-valued
    assert set(np.unique(h["sat"])).issubset({0, 1})
