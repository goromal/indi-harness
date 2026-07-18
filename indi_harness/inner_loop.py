"""Layer A(+C) inner loop: quaternion tilt-prioritized attitude + INDI rate
loop on rotor-speed-squared increments.

Phase-matching rule enforced structurally: the angular-accel filter and the
actuator-state (Omega^2, Omega_dot) filters share one (cutoff, fs) pair.
"""
from dataclasses import dataclass, field
import numpy as np
from .allocation import Allocator
from .filters import Butter2
from .tilt_yaw import attitude_rate_ref


@dataclass
class InnerGains:
    kp_tilt: float = 10.0
    kp_yaw: float = 5.0
    kw: np.ndarray = field(default_factory=lambda: np.array([20.0, 20.0, 10.0]))
    cutoff_hz: float = 25.0   # shared by ALL inner-loop INDI filters


class InnerLoopINDI:
    def __init__(self, params, gains, fs):
        self.P, self.G = params, gains
        self.alloc = Allocator(params)
        fc = gains.cutoff_hz
        self.f_domega = Butter2(fc, fs, 3)    # angular accel estimate
        self.f_omsq = Butter2(fc, fs, 4)      # actuator state Omega^2
        self.f_domdt = Butter2(fc, fs, 4)     # rotor accel (G2 term)
        self.prev_gyro = None
        self.prev_Om = None

    def update(self, dt, gyro, q, q_ref, w_ff, dw_ff, T_cmd, Omega_meas):
        P, G = self.P, self.G
        gyro = np.asarray(gyro, float)
        if self.prev_gyro is None:
            self.prev_gyro = gyro.copy()
            self.prev_Om = np.asarray(Omega_meas, float).copy()
            self.f_omsq.reset(np.asarray(Omega_meas, float) ** 2)
        domega_raw = (gyro - self.prev_gyro) / dt
        self.prev_gyro = gyro.copy()
        domega_f = self.f_domega.update(domega_raw)

        Om = np.asarray(Omega_meas, float)
        Om_sq_f = self.f_omsq.update(Om ** 2)
        dOm_f = self.f_domdt.update((Om - self.prev_Om) / dt)
        self.prev_Om = Om.copy()

        w_ref = attitude_rate_ref(q, q_ref, G.kp_tilt, G.kp_yaw, w_ff)
        dw_cmd = G.kw * (w_ref - gyro) + dw_ff
        dtau = P.J @ (dw_cmd - domega_f)
        dT = T_cmd - P.kf * float(np.sum(Om_sq_f))
        Om_sq_cmd, sat = self.alloc.indi_increment(dT, dtau, Om_sq_f, dOm_f)
        Om_cmd = np.sqrt(np.clip(Om_sq_cmd, 0.0, None))
        diag = {"domega_pred": dw_cmd, "domega_meas_f": domega_f,
                "w_ref": w_ref, "sat": sat, "Om_cmd": Om_cmd}
        return Om_cmd, diag
