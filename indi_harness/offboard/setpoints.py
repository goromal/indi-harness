"""SET_ATTITUDE_TARGET packing. Pure data — no link I/O here.

MAVLink q order is [w,x,y,z] (scalar-first) — identical to indi_harness.quat.
type_mask 0 = attitude + body rates + throttle all active. Requires
GUID_OPTIONS bit 3 (value 8) on the vehicle so `thrust` means thrust (0..1
normalized), not climb rate.
"""
from dataclasses import dataclass
import numpy as np

ATT_TYPEMASK_FULL = 0x00
THRUST_MIN, THRUST_MAX = 0.05, 0.95


@dataclass
class AttitudeSetpoint:
    q: np.ndarray       # NED/FRD reference attitude [w,x,y,z]
    w: np.ndarray       # body-rate feedforward [rad/s]
    thrust: float       # normalized 0..1

    def fields(self, boot_ms=0):
        return dict(
            time_boot_ms=int(boot_ms), type_mask=ATT_TYPEMASK_FULL,
            q=[float(x) for x in self.q],
            body_roll_rate=float(self.w[0]), body_pitch_rate=float(self.w[1]),
            body_yaw_rate=float(self.w[2]),
            thrust=float(np.clip(self.thrust, THRUST_MIN, THRUST_MAX)))
