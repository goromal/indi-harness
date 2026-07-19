"""SET_POSITION_TARGET_LOCAL_NED packing. Pure data — no link I/O here."""
from dataclasses import dataclass
import numpy as np

MAV_FRAME_LOCAL_NED = 1
TYPEMASK_YAW_IGNORE = 0x400
TYPEMASK_YAW_RATE_IGNORE = 0x800
# Position + velocity + acceleration all active; yaw channels ignored.
PVA_TYPEMASK = TYPEMASK_YAW_IGNORE | TYPEMASK_YAW_RATE_IGNORE


@dataclass
class Setpoint:
    p: np.ndarray  # NED position [m]
    v: np.ndarray  # NED velocity [m/s]
    a: np.ndarray  # NED acceleration [m/s^2]

    def fields(self, boot_ms=0):
        return dict(
            time_boot_ms=int(boot_ms), coordinate_frame=MAV_FRAME_LOCAL_NED,
            type_mask=PVA_TYPEMASK,
            x=float(self.p[0]), y=float(self.p[1]), z=float(self.p[2]),
            vx=float(self.v[0]), vy=float(self.v[1]), vz=float(self.v[2]),
            afx=float(self.a[0]), afy=float(self.a[1]), afz=float(self.a[2]),
            yaw=0.0, yaw_rate=0.0)
