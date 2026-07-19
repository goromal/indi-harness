"""Guided-mode MAVLink trajectory streamer (design doc T.1 — deliberately thin,
no abstraction layer). Talks to mavlink-router's GCS TCP port (default 5790).

The FlightRecord's (traj_t, boot_ms) pairs are the ONLY bridge between the
trajectory clock and ArduPilot's boot clock — align.py fits the linear map
that lets .BIN TimeUS data be scored on the trajectory timeline even when
SITL runs off wall-clock speed (design doc: sim-time-normalized, always).
"""
import time
from dataclasses import dataclass, field
import numpy as np
from pymavlink import mavutil
from .setpoints import Setpoint, PVA_TYPEMASK, MAV_FRAME_LOCAL_NED

GUIDED_CUSTOM_MODE = 4  # ArduCopter GUIDED


@dataclass
class FlightRecord:
    traj_t: list = field(default_factory=list)
    boot_ms: list = field(default_factory=list)
    p: list = field(default_factory=list)      # received EKF local NED
    p_ref: list = field(default_factory=list)  # commanded reference

    def arrays(self):
        return (np.asarray(self.traj_t, float), np.asarray(self.boot_ms, float),
                np.asarray(self.p, float), np.asarray(self.p_ref, float))


class GuidedStreamer:
    def __init__(self, conn, rate_hz=50.0, realtime=True):
        """conn: mavutil connection (or duck-typed fake). realtime=False skips
        sleeps so offline tests run instantly."""
        self.conn, self.rate_hz, self.realtime = conn, rate_hz, realtime

    @classmethod
    def connect(cls, url="tcp:127.0.0.1:5790", rate_hz=50.0):
        conn = mavutil.mavlink_connection(url)
        conn.wait_heartbeat(timeout=120)
        return cls(conn, rate_hz=rate_hz)

    def _sleep(self, dt):
        if self.realtime and dt > 0:
            time.sleep(dt)

    def set_mode_guided(self, timeout=60.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.conn.mav.set_mode_send(
                self.conn.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                GUIDED_CUSTOM_MODE)
            hb = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb is not None and hb.custom_mode == GUIDED_CUSTOM_MODE:
                return
            self._sleep(0.5)
        raise TimeoutError("GUIDED mode not confirmed")

    def arm(self, timeout=300.0, retry_s=2.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
            hb = self.conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if hb is not None and (hb.base_mode
                                   & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                return
            self._sleep(retry_s)
        raise TimeoutError("arming failed (prearm checks may still be settling)")

    def takeoff(self, alt_m, timeout=120.0):
        self.conn.mav.command_long_send(
            self.conn.target_system, self.conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt_m)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            m = self.conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                                     timeout=5)
            if m is not None and -m.z > 0.9 * alt_m:
                return np.array([m.x, m.y, m.z])
        raise TimeoutError("takeoff altitude not reached")

    def local_position(self, timeout=10.0):
        m = self.conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                                 timeout=timeout)
        if m is None:
            raise TimeoutError("no LOCAL_POSITION_NED")
        return np.array([m.x, m.y, m.z])

    def fly(self, traj, duration, origin, record=None):
        """Stream ref(t) = origin + (traj.p(t) - traj.p(0)) at rate_hz."""
        rec = record or FlightRecord()
        fo0 = traj.ref(0.0)
        dt = 1.0 / self.rate_hz
        n_steps = int(round(duration * self.rate_hz))
        for k in range(n_steps):
            t = k * dt
            fo = traj.ref(t)
            p_ref = origin + (fo.p - fo0.p)
            sp = Setpoint(p=p_ref, v=fo.v, a=fo.a)
            f = sp.fields()
            self.conn.mav.set_position_target_local_ned_send(
                f["time_boot_ms"], self.conn.target_system,
                self.conn.target_component, f["coordinate_frame"],
                f["type_mask"], f["x"], f["y"], f["z"],
                f["vx"], f["vy"], f["vz"], f["afx"], f["afy"], f["afz"],
                f["yaw"], f["yaw_rate"])
            m = self.conn.recv_match(type="LOCAL_POSITION_NED", blocking=False)
            if m is not None:
                rec.traj_t.append(t)
                rec.boot_ms.append(m.time_boot_ms)
                rec.p.append([m.x, m.y, m.z])
                rec.p_ref.append(p_ref.tolist())
            self._sleep(dt)
        return rec
