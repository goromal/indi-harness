"""ArduPilot SITL JSON protocol: servo-packet parse + state-JSON format + lockstep."""
import json
import struct
from dataclasses import dataclass

import numpy as np

MAGIC_16 = 18458
_FMT = '<HHI16H'
_SIZE = struct.calcsize(_FMT)


@dataclass
class ServoPacket:
    frame_rate: int
    frame_count: int
    pwm: list


def parse_servo(data):
    if len(data) < _SIZE:
        return None
    fields = struct.unpack(_FMT, data[:_SIZE])
    if fields[0] != MAGIC_16:
        return None
    return ServoPacket(frame_rate=fields[1], frame_count=fields[2], pwm=list(fields[3:]))


def format_state(st):
    # ArduPilot SIM_JSON.cpp frames sensor messages on '\n' (it converts the
    # newline to a nul terminator and only parses up to the last terminator).
    # Without a trailing newline it never parses the reply and spins on
    # "No JSON sensor message received, resending servos". Terminate every reply.
    return json.dumps({
        "timestamp": st["timestamp"],
        "imu": {"gyro": st["gyro"], "accel_body": st["accel_body"]},
        "position": st["position"],
        "velocity": st["velocity"],
        "quaternion": st["quaternion"],
    }) + "\n"


def pwm_to_omega(pwm4, omega_max):
    """Normalised thrust-linear map: u=(pwm-1000)/1000 in [0,1]; Omega=sqrt(u)*Omega_max."""
    u = np.clip((np.asarray(pwm4, float) - 1000.0) / 1000.0, 0.0, 1.0)
    return np.sqrt(u) * omega_max


class LockstepDriver:
    """One physics step per servo packet; resets on frame_count regression."""
    def __init__(self, model):
        self.model = model
        self._last_count = None

    def on_packet(self, data):
        pkt = parse_servo(data)
        if pkt is None:
            return None
        if self._last_count is not None and pkt.frame_count < self._last_count:
            self.model.__init__(self.model.P, drag_on=self.model.drag_on,
                                 dt=self.model.dt, ground=self.model.ground)
            self.model.seed_omega(self.model.P.hover_speed() * np.ones(4))
        self._last_count = pkt.frame_count
        self.model.dt = 1.0 / max(pkt.frame_rate, 1)
        omega_cmd = pwm_to_omega(pkt.pwm[:4], self.model.P.Omega_max)
        self.model.step_omega(omega_cmd)
        return format_state(self.model.state())
