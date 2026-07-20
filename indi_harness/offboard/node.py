"""S2 offboard node: event-driven on /ap/pose/filtered, commands
SET_ATTITUDE_TARGET on the mavlink-router link (design doc T.1 route).

OffboardMission is pure logic (duck-typed messages + conn) so the whole
control path is offline-testable; the rclpy shell at the bottom only wires
subscriptions and republishes internals for the mcap bag (§L).
Trajectory time comes from message stamps — never wall clock (§T timing
note: sim-time slowdown must not skew feedforward phasing).
"""
import numpy as np
from .. import quat
from ..sitl.frames import pose_enu_to_ned, enu_to_ned
from ..sitl.streamer import FlightRecord
from .bridge import OffboardOuterLoop, OffboardGains


class OffboardMission:
    def __init__(self, conn, params, hover_throttle, rate_hz=50.0,
                 gains=None, indi_accel=True):
        self.conn, self.P = conn, params
        g = gains or OffboardGains(indi_accel=indi_accel)
        self.ctl = OffboardOuterLoop(params, g, fs=rate_hz,
                                     hover_throttle=hover_throttle)
        self.case = self.origin = self._t0 = None
        self.record = FlightRecord()
        self.f_b = np.array([0.0, 0.0, -params.g])  # updated by on_imu
        self._vel_ned = None                         # updated by on_twist
        self.done = False
        self.last_sp = None

    def start_case(self, case, origin):
        self.case, self.origin, self._t0 = case, np.asarray(origin, float), None
        self.record = FlightRecord()
        self.done = False

    def on_imu(self, msg):
        # /ap/imu/experimental/data is already FRD (frame_id base_link_ned):
        # linear_acceleration is the specific force in the autopilot body
        # frame, matching the bridge convention (hover -> [0,0,-g]). Verified
        # empirically in the VM (docs/apdds_audit.md): rest-state
        # linear_acceleration.z = -9.8. No FLU->FRD flip.
        a = msg.linear_acceleration
        self.f_b = np.array([a.x, a.y, a.z])

    def on_twist(self, msg):
        tw = msg.twist.linear
        self._vel_ned = enu_to_ned([tw.x, tw.y, tw.z])

    def on_pose(self, msg):
        if self.case is None or self.done:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = stamp
        t = stamp - self._t0
        if t > self.case.duration:
            self.done = True
            return
        o, p_msg = msg.pose.orientation, msg.pose.position
        p_ned, q_ned = pose_enu_to_ned(
            np.array([p_msg.x, p_msg.y, p_msg.z]),
            np.array([o.w, o.x, o.y, o.z]))
        v_ned = self._vel_ned if self._vel_ned is not None else np.zeros(3)
        traj = _Shifted(self.case.traj, self.origin)
        sp = self.ctl.tick(t=t, traj=traj, p=p_ned, v=v_ned, q=q_ned,
                           f_b=self.f_b)
        f = sp.fields()
        self.conn.mav.set_attitude_target_send(
            f["time_boot_ms"], self.conn.target_system,
            self.conn.target_component, f["type_mask"], f["q"],
            f["body_roll_rate"], f["body_pitch_rate"], f["body_yaw_rate"],
            f["thrust"])
        self.last_sp = sp
        m = self.conn.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if m is not None:
            self.record.traj_t.append(t)
            self.record.boot_ms.append(m.time_boot_ms)
            self.record.p.append([m.x, m.y, m.z])
            self.record.p_ref.append(traj.ref(t).p.tolist())


class _Shifted:
    """Relative-displacement wrapper: ref(t).p = origin + (p(t) - p(0)) —
    the same convention the S1 streamer applied, so align/evaluate_bin's
    origin_offset math is reused unchanged."""

    def __init__(self, traj, origin):
        self._traj, self._d = traj, np.asarray(origin) - traj.ref(0.0).p
        self.duration = getattr(traj, "duration", None)

    def ref(self, t):
        fo = self._traj.ref(t)
        return type(fo)(fo.p + self._d, fo.v, fo.a, fo.j, fo.s,
                        fo.psi, fo.dpsi, fo.ddpsi)


def run_node(mission, imu_topic, spin_until_done):
    """rclpy shell. Imported lazily — host test envs have no rclpy."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped, TwistStamped

    class IndiNode(Node):
        def __init__(self):
            super().__init__("indi_offboard")
            self.create_subscription(PoseStamped, "/ap/pose/filtered",
                                     mission.on_pose, qos_profile_sensor_data)
            self.create_subscription(TwistStamped, "/ap/twist/filtered",
                                     mission.on_twist, qos_profile_sensor_data)
            if imu_topic:
                from sensor_msgs.msg import Imu
                self.create_subscription(Imu, imu_topic, mission.on_imu,
                                         qos_profile_sensor_data)
            self.ref_pub = self.create_publisher(PoseStamped,
                                                 "/indi/ref_pose", 10)
            self.cmd_pub = self.create_publisher(PoseStamped,
                                                 "/indi/cmd_attitude", 10)
            self.create_timer(0.1, self._publish_internals)

        def _publish_internals(self):
            if mission.last_sp is None or mission.case is None:
                return
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            q = mission.last_sp.q
            msg.pose.orientation.w = float(q[0])
            msg.pose.orientation.x = float(q[1])
            msg.pose.orientation.y = float(q[2])
            msg.pose.orientation.z = float(q[3])
            msg.pose.position.z = float(mission.last_sp.thrust)  # thrust in z
            self.cmd_pub.publish(msg)

    rclpy.init()
    node = IndiNode()
    try:
        spin_until_done(rclpy, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
