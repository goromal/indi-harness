from types import SimpleNamespace

import numpy as np
import pytest
from pymavlink import mavutil
from indi_harness.trajectory import Circle
from indi_harness.sitl.streamer import GuidedStreamer, FlightRecord


class FakeMav:
    def __init__(self, parent):
        self.parent = parent
        self.setpoints = []
        self.commands = []

    def set_position_target_local_ned_send(self, *args):
        self.setpoints.append(args)

    def command_long_send(self, *args):
        self.commands.append(args)

    def set_mode_send(self, *args):
        pass


class FakeConn:
    """Duck-typed mavutil connection. Scripted heartbeat arming behavior."""
    target_system, target_component = 1, 1

    def __init__(self, arm_after_attempts=1):
        self.mav = FakeMav(self)
        self.arm_after = arm_after_attempts
        self._pos_t = 0.0

    def recv_match(self, type=None, blocking=False, timeout=None):
        if type == "HEARTBEAT":
            armed = len(self.mav.commands) >= self.arm_after
            base_mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0
            return SimpleNamespace(base_mode=base_mode, custom_mode=4)
        if type == "LOCAL_POSITION_NED":
            self._pos_t += 0.02
            return SimpleNamespace(
                time_boot_ms=int(self._pos_t * 1000) + 5000,
                x=1.0, y=2.0, z=-10.0, vx=0.0, vy=0.0, vz=0.0)
        return None


def test_fly_sends_setpoints_and_records():
    conn = FakeConn()
    s = GuidedStreamer(conn, rate_hz=50.0, realtime=False)
    traj = Circle(radius=2.0, period=8.0, alt=10.0)
    origin = np.array([1.0, 2.0, -10.0])
    rec = s.fly(traj, duration=2.0, origin=origin)
    assert len(conn.mav.setpoints) >= 0.9 * 50 * 2.0
    traj_t, boot_ms, p, p_ref = rec.arrays()
    assert len(traj_t) == len(boot_ms) == len(p) == len(p_ref)
    assert np.all(np.diff(traj_t) >= 0)
    # relative displacement: first reference equals the origin
    assert np.allclose(p_ref[0], origin, atol=1e-9)


def test_arm_retries_then_succeeds():
    conn = FakeConn(arm_after_attempts=3)
    s = GuidedStreamer(conn, rate_hz=50.0, realtime=False)
    s.arm(timeout=10.0, retry_s=0.0)
    assert len(conn.mav.commands) >= 3


def test_arm_timeout_raises():
    conn = FakeConn(arm_after_attempts=10 ** 9)
    s = GuidedStreamer(conn, rate_hz=50.0, realtime=False)
    with pytest.raises(TimeoutError):
        s.arm(timeout=0.2, retry_s=0.0)
