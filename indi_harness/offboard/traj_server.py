"""S3 Layer-B trajectory server: publish ardupilot_msgs/FlatSetpoint over the
custom AP_DDS topic (rt/ap/flat_setpoint) so the in-firmware INDI outer loop
(AC_CustomControl_INDI, CC_TYPE=3, CC3_OUTER_EN=1) can fly a minimum-snap /
analytic flat-output reference.

Design notes
------------
* The message carries the full flat reference {p,v,a,j,s, yaw,yaw_rate,
  yaw_accel} in the **NED world frame** -- the firmware FlatRef cache consumes
  the fields verbatim (no ENU->NED flip on the wire; the trajectory-server owns
  the frame). This mirrors the AP_DDS FlatSetpoint IDL added in Task B3.
* Trajectory time comes from the SITL clock (/ap/clock), never wall clock, so
  sim-time slowdown does not skew the feedforward phasing (design doc T timing
  note). The origin is captured from the first /ap/pose/filtered so the
  trajectory is engaged relative to the vehicle's position (the _Shifted
  convention reused from the S2 offboard node).
* Pure logic (TrajServer) is duck-typed and offline-testable; the rclpy shell
  at the bottom is imported lazily (host test envs have no rclpy).
"""
import numpy as np

from ..trajectory import FlatOutput
from .node import _Shifted


class TrajServer:
    """Pure flat-reference sampler. Holds a trajectory Case and, once an origin
    is set, produces FlatSetpoint field dicts (NED) at arbitrary trajectory
    time. Frame conversions and message construction live in the rclpy shell."""

    def __init__(self, case):
        self.case = case
        self.origin = None
        self._t0 = None

    def set_origin(self, p_ned):
        self.origin = np.asarray(p_ned, float)

    def elapsed(self, t_now):
        """Trajectory time from an absolute clock reading (latched on first
        call). Held at the trajectory duration after it completes so the last
        setpoint is a valid hover-at-end rather than an extrapolation."""
        if self._t0 is None:
            self._t0 = t_now
        t = t_now - self._t0
        return min(max(t, 0.0), self.case.duration)

    def sample(self, t):
        """FlatOutput (NED) at trajectory time t, shifted to the origin."""
        traj = _Shifted(self.case.traj, self.origin) if self.origin is not None \
            else self.case.traj
        return traj.ref(t)


def _fill_vec3(v3, arr):
    v3.x, v3.y, v3.z = float(arr[0]), float(arr[1]), float(arr[2])


def build_msg(FlatSetpoint, fo, stamp):
    """Construct an ardupilot_msgs/FlatSetpoint from a FlatOutput (NED)."""
    msg = FlatSetpoint()
    msg.stamp = stamp
    _fill_vec3(msg.position, fo.p)
    _fill_vec3(msg.velocity, fo.v)
    _fill_vec3(msg.acceleration, fo.a)
    _fill_vec3(msg.jerk, fo.j)
    _fill_vec3(msg.snap, fo.s)
    msg.yaw = float(fo.psi)
    msg.yaw_rate = float(fo.dpsi)
    msg.yaw_accel = float(fo.ddpsi)
    return msg


def run_traj_server(server, rate_hz=50.0, spin=None):
    """rclpy shell. Publishes FlatSetpoint on /ap/flat_setpoint (DDS
    rt/ap/flat_setpoint) at rate_hz, timed off /ap/clock, origin latched from
    /ap/pose/filtered. Imported lazily -- host test envs have no rclpy."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseStamped
    from ardupilot_msgs.msg import FlatSetpoint
    from ..sitl.frames import enu_to_ned

    class TrajNode(Node):
        def __init__(self):
            super().__init__("indi_traj_server")
            self.sim_time = None
            self.pub = self.create_publisher(FlatSetpoint, "/ap/flat_setpoint", 10)
            self.create_subscription(Clock, "/ap/clock", self._on_clock,
                                     qos_profile_sensor_data)
            self.create_subscription(PoseStamped, "/ap/pose/filtered",
                                     self._on_pose, qos_profile_sensor_data)
            self.create_timer(1.0 / rate_hz, self._tick)

        def _on_clock(self, msg):
            self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

        def _on_pose(self, msg):
            if server.origin is None:
                p = msg.pose.position
                server.set_origin(enu_to_ned([p.x, p.y, p.z]))

        def _tick(self):
            # Need both a clock and an origin before streaming setpoints.
            if self.sim_time is None or server.origin is None:
                return
            t = server.elapsed(self.sim_time)
            fo = server.sample(t)
            self.pub.publish(build_msg(FlatSetpoint, fo,
                                       self.get_clock().now().to_msg()))

    rclpy.init()
    node = TrajNode()
    try:
        (spin or (lambda r, n: r.spin(n)))(rclpy, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
