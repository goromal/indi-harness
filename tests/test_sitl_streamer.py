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
            if not blocking:
                return None  # drained: no further queued heartbeat
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


def test_fly_checks_each_received_position():
    conn = FakeConn()
    positions = []
    s = GuidedStreamer(conn, rate_hz=2.0, realtime=False)
    s.fly(Circle(radius=2.0, period=8.0, alt=10.0), duration=2.0,
          origin=np.array([1.0, 2.0, -10.0]),
          on_position=lambda message: positions.append(message.z))
    assert positions == [-10.0] * 4


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


ARM_DISARM = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
NAV_TAKEOFF = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF


class Msg(SimpleNamespace):
    def get_type(self):
        return self.mtype


def heartbeat(armed):
    return Msg(mtype="HEARTBEAT", custom_mode=4,
               base_mode=mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)


class QueuedAutopilot:
    """Like a pymavlink connection, recv_match returns the OLDEST queued
    message, so a backlog gives the caller a stale view. A blocking read with
    nothing queued lets the autopilot publish its current state. Takeoff is
    rejected while disarmed, as ArduCopter does."""
    target_system, target_component = 1, 1

    def __init__(self, armed=False, backlog=()):
        self.mav = self
        self.armed, self.climbing, self.alt = armed, False, 0.0
        self.queue = list(backlog)
        self.commands = []

    def command_long_send(self, system, component, command, confirmation, *params):
        self.commands.append(command)
        if command == ARM_DISARM:
            self.armed = params[0] == 1
        elif command == NAV_TAKEOFF:
            self.climbing = self.armed

    def set_mode_send(self, *args):
        pass

    def publish(self):
        if self.climbing:
            self.alt = min(self.alt + 2.5, 10.0)
        self.queue += [heartbeat(self.armed),
                       Msg(mtype="LOCAL_POSITION_NED", x=0.0, y=0.0, z=-self.alt)]

    def recv_match(self, type=None, blocking=False, timeout=None):
        while True:
            while self.queue:
                m = self.queue.pop(0)
                if type is None or m.get_type() == type:
                    return m
            if not blocking:
                return None
            self.publish()


def test_arm_acts_on_the_newest_heartbeat_not_a_stale_backlog():
    # Armed already, but ten older disarmed heartbeats are still queued: reading
    # them one per retry left the runner ~10 s behind the autopilot.
    ap = QueuedAutopilot(armed=True,
                         backlog=[heartbeat(False)] * 10 + [heartbeat(True)])
    GuidedStreamer(ap, realtime=False).arm(timeout=10.0, retry_s=0.0)
    assert ap.commands == [ARM_DISARM]


def test_takeoff_rearms_and_retries_when_the_vehicle_auto_disarmed():
    # ArduCopter auto-disarms a landed vehicle DISARM_DELAY after arming and
    # rejects a takeoff sent after that; a single unchecked command never climbs.
    ap = QueuedAutopilot(armed=False)
    GuidedStreamer(ap, realtime=False).takeoff(10.0, timeout=5.0)
    assert ap.alt >= 9.0
    assert ap.commands.count(NAV_TAKEOFF) == 2
    assert ARM_DISARM in ap.commands
