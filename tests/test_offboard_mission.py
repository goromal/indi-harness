import numpy as np
from types import SimpleNamespace
from indi_harness.params import QuadParams
from indi_harness.sitl.baseline import Case
from indi_harness.trajectory import Circle
from indi_harness.offboard.node import OffboardMission


class FakeMav:
    def __init__(self):
        self.att_targets = []

    def set_attitude_target_send(self, *args):
        self.att_targets.append(args)


class FakeConn:
    target_system, target_component = 1, 1

    def __init__(self):
        self.mav = FakeMav()
        self._t = 0.0

    def recv_match(self, type=None, blocking=False, timeout=None):
        if type == "LOCAL_POSITION_NED":
            self._t += 0.02
            return SimpleNamespace(time_boot_ms=int(self._t * 1000) + 7000,
                                   x=0.0, y=0.0, z=-10.0,
                                   vx=0.0, vy=0.0, vz=0.0)
        return None


def pose_msg(t_s):
    # duck-typed /ap/pose/filtered sample: ENU identity pose at 10 m up
    stamp = SimpleNamespace(sec=int(t_s), nanosec=int((t_s % 1.0) * 1e9))
    pose = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0, z=10.0),
        orientation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0))
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp), pose=pose)


def test_mission_streams_and_joins():
    conn = FakeConn()
    P = QuadParams()
    case = Case("circle_fast", Circle(radius=2.0, period=8.0, alt=10.0), 4.0)
    m = OffboardMission(conn, P, hover_throttle=0.35, rate_hz=50.0)
    m.start_case(case, origin=np.array([0.0, 0.0, -10.0]))
    k = 0
    while not m.done:
        m.on_pose(pose_msg(100.0 + k / 50.0))
        k += 1
    assert len(conn.mav.att_targets) >= 0.9 * 50 * case.duration
    traj_t = np.array(m.record.traj_t)
    assert len(traj_t) > 0 and np.all(np.diff(traj_t) >= 0)
    assert len(m.record.boot_ms) == len(traj_t)
