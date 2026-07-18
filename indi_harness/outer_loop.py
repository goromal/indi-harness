"""Layer B: cascaded linear INDI outer loop (Smeur 2018 structure).

Thrust-vector increment on filtered measured specific force:
    f_cmd = f_state_f + (a_cmd - a_meas_f)        [mass-normalized, world]
where f_state_f is the current thrust-vector state estimate (from filtered
attitude + filtered rotor speeds) and a_meas_f = g e3 + filtered accel.
Drag never enters a model — it lives in the measurement, which is the point.
"""
from dataclasses import dataclass, field
import numpy as np
from . import quat
from .filters import Butter2

E3 = np.array([0.0, 0.0, 1.0])


@dataclass
class OuterGains:
    kp: np.ndarray = field(default_factory=lambda: np.array([6.0, 6.0, 6.0]))
    kv: np.ndarray = field(default_factory=lambda: np.array([4.0, 4.0, 4.0]))
    cutoff_hz: float = 8.0    # shared by accel + thrust-state filters


class OuterLoopINDI:
    def __init__(self, params, gains, fs):
        self.P, self.G = params, gains
        self.f_accel = Butter2(gains.cutoff_hz, fs, 3)   # measured specific force (world)
        self.f_state = Butter2(gains.cutoff_hz, fs, 3)   # thrust-vector state (world)

    def update(self, p, v, q, f_b_meas, Omega_meas, fo):
        """fo: FlatOutput reference. Returns (z_b_des, T_cmd [N])."""
        P, G = self.P, self.G
        a_cmd = fo.a + G.kp * (fo.p - p) + G.kv * (fo.v - v)
        f_w_meas = quat.qrot(q, f_b_meas)
        f_meas_f = self.f_accel.update(f_w_meas)
        # thrust-vector state: current body z, current filtered rotor thrust
        T_state = P.kf * float(np.sum(np.asarray(Omega_meas) ** 2)) / P.m
        f_state = quat.qrot(q, np.array([0.0, 0.0, -T_state]))
        f_state_f = self.f_state.update(f_state)
        f_cmd = f_state_f + (a_cmd - E3 * P.g) - f_meas_f
        T_bar = np.linalg.norm(f_cmd)
        z_b_des = -f_cmd / T_bar
        return z_b_des, P.m * T_bar


def attitude_from_thrust_dir(z_b_des, psi):
    """Reference quaternion from desired body-z and yaw (same triad as flatness)."""
    x_c = np.array([np.cos(psi), np.sin(psi), 0.0])
    y_b = np.cross(z_b_des, x_c)
    n = np.linalg.norm(y_b)
    if n < 1e-6:
        # z_b_des ~ parallel to x_c (~90 deg pitch along heading): yaw triad is
        # degenerate; fall back to y_c to keep the quaternion finite.
        y_b = np.array([-np.sin(psi), np.cos(psi), 0.0])
    else:
        y_b = y_b / n
    x_b = np.cross(y_b, z_b_des)
    return quat.q_from_R(np.column_stack([x_b, y_b, z_b_des]))
