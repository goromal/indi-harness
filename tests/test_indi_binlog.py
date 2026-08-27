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


from indi_harness.sitl import binlog


class _FakeMsg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class _FakeLog:
    def __init__(self, msgs):
        self._msgs = list(msgs)
        self._i = 0

    def recv_match(self, type=None):
        # yield only the INDC messages, then None
        while self._i < len(self._msgs):
            m = self._msgs[self._i]
            self._i += 1
            if getattr(m, "_type", "INDC") == type:
                return m
        return None


def test_read_indc_health_shapes(monkeypatch):
    msgs = [
        _FakeMsg(_type="INDC", TimeUS=1000, O0=500.0, O1=501.0, O2=502.0, O3=503.0,
                 D0=1.0, D1=2.0, D2=3.0, D3=4.0, Ux=0.1, Uy=-0.2, Uz=0.05, FB=0),
        _FakeMsg(_type="INDC", TimeUS=2000, O0=510.0, O1=511.0, O2=512.0, O3=513.0,
                 D0=1.1, D1=2.1, D2=3.1, D3=4.1, Ux=0.11, Uy=-0.21, Uz=0.06, FB=1),
    ]
    monkeypatch.setattr(binlog.DFReader, "DFReader_binary", lambda p: _FakeLog(msgs))
    h = binlog.read_indc_health("ignored.BIN")
    n = len(h["time_us"])
    assert n == 2
    assert h["omega"].shape == (n, 4)
    assert h["omega_dot"].shape == (n, 4)
    assert h["u_act"].shape == (n, 3)
    assert h["fallback"].shape == (n,)
    assert list(h["fallback"]) == [0, 1]
    assert h["omega"][0][0] == 500.0 and h["u_act"][1][2] == 0.06
