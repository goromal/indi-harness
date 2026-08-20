"""Pure-logic tests for the Layer-B trajectory server (no rclpy): origin
shifting, trajectory-clock latching, battery case-switching, and the
ready-file protocol that drives battery mode / the DDS-staleness fallback."""
import json
import numpy as np
import pytest
from indi_harness.offboard.traj_server import TrajServer, read_ready
from indi_harness.sitl.baseline import BATTERY

CASES = {c.name: c for c in BATTERY}


def test_elapsed_latches_and_clamps():
    s = TrajServer(CASES["circle_slow"])
    # first call latches t0; time is relative and clamped to [0, duration]
    assert s.elapsed(100.0) == 0.0
    assert s.elapsed(105.0) == 5.0
    assert s.elapsed(100.0 + s.case.duration + 50.0) == s.case.duration


def test_sample_shifts_to_origin():
    s = TrajServer(CASES["circle_slow"])
    origin = np.array([10.0, -5.0, -9.7])
    s.set_origin(origin)
    # _Shifted convention: ref(t).p = origin + (traj(t) - traj(0))
    base0 = s.case.traj.ref(0.0).p
    fo = s.sample(0.0)
    assert np.allclose(fo.p, origin, atol=1e-9)
    fo3 = s.sample(3.0)
    expected = origin + (s.case.traj.ref(3.0).p - base0)
    assert np.allclose(fo3.p, expected, atol=1e-9)


def test_set_case_relatches_clock():
    s = TrajServer(CASES["circle_slow"])
    s.elapsed(100.0)                      # latch t0 at 100
    assert s.elapsed(110.0) == 10.0
    s.set_case(CASES["lemniscate_slow"])  # switch -> clock must re-latch
    assert s.case is CASES["lemniscate_slow"]
    assert s.elapsed(200.0) == 0.0        # new t0
    assert s.elapsed(203.0) == 3.0


def test_set_case_noop_when_same():
    s = TrajServer(CASES["circle_slow"])
    s.elapsed(100.0)
    s.set_case(CASES["circle_slow"])      # same object -> no re-latch
    assert s.elapsed(104.0) == 4.0


def test_read_ready_roundtrip_and_missing(tmp_path):
    p = tmp_path / "lb_ready"
    assert read_ready(p) is None          # absent -> None (fallback)
    p.write_text(json.dumps({"case": "lemniscate_fast",
                             "origin": [1.0, 2.0, -9.7]}))
    name, origin = read_ready(p)
    assert name == "lemniscate_fast"
    assert np.allclose(origin, [1.0, 2.0, -9.7])
    # malformed / partial payloads degrade to None rather than raising
    p.write_text("{not json")
    assert read_ready(p) is None
    p.write_text(json.dumps({"case": "x"}))   # missing origin
    assert read_ready(p) is None


def test_battery_switching_via_ready_file(tmp_path):
    """Simulate the battery-mode tick: follow the ready-file case-by-case,
    re-latching origin + clock on each switch, falling back when absent."""
    p = tmp_path / "lb_ready"
    s = TrajServer()

    def tick(sim_time):
        r = read_ready(p)
        if r is None:
            return None                    # no publish -> backend fallback
        name, origin = r
        s.set_case(CASES[name])
        s.set_origin(origin)
        return s.sample(s.elapsed(sim_time))

    assert tick(0.0) is None               # nothing yet -> fallback

    p.write_text(json.dumps({"case": "circle_slow", "origin": [0, 0, -9.7]}))
    fo = tick(10.0)
    assert s.case is CASES["circle_slow"]
    assert np.allclose(fo.p, [0, 0, -9.7], atol=1e-9)   # t=0 at origin

    p.unlink()
    assert tick(15.0) is None              # between cases -> fallback

    p.write_text(json.dumps({"case": "lemniscate_slow", "origin": [2, 0, -9.7]}))
    tick(20.0)
    assert s.case is CASES["lemniscate_slow"]
    assert s.elapsed(21.0) == 1.0          # clock re-latched at the switch
