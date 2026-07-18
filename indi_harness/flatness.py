"""Differential-flatness map (NED/FRD, thrust along -z_b).

alpha := g*e3 - a  (mass-normalized thrust vector, points along +z_b)
T_bar := |alpha|, z_b := alpha / T_bar
x_c := (cos psi, sin psi, 0); y_b := z_b x x_c (normalized); x_b := y_b x z_b

Body rates from the jerk chain, angular accel from the snap chain.
omega_z closed-form derivation: wz = y_b . dx_b = -(x_b . dy_b).
Expanding dy_b = (d(z_b x x_c) - dn*y_b)/n and projecting onto x_b yields:
  wz = (wx * (z_b . x_c) + dpsi * (y_b . y_c)) / n
omega_z-dot is obtained by central difference of closed-form omega_z —
exact to ~1e-8 on analytic trajectories.
"""
from dataclasses import dataclass
import numpy as np
from . import quat

E3 = np.array([0.0, 0.0, 1.0])


@dataclass
class RefState:
    q: np.ndarray    # body->world reference attitude
    w: np.ndarray    # body-rate feedforward [rad/s]
    dw: np.ndarray   # body angular-accel feedforward [rad/s^2]
    T: float         # thrust magnitude [N], along -z_b


def _core(fo, m, g):
    alpha = g * E3 - fo.a
    dalpha, ddalpha = -fo.j, -fo.s
    T_bar = np.linalg.norm(alpha)
    z_b = alpha / T_bar
    dT = z_b @ dalpha
    dz = (dalpha - dT * z_b) / T_bar
    ddT = dz @ dalpha + z_b @ ddalpha
    ddz = (ddalpha - ddT * z_b - 2.0 * dT * dz) / T_bar

    x_c = np.array([np.cos(fo.psi), np.sin(fo.psi), 0.0])
    y_c = np.array([-np.sin(fo.psi), np.cos(fo.psi), 0.0])
    zxc = np.cross(z_b, x_c)
    n = np.linalg.norm(zxc)
    y_b = zxc / n
    x_b = np.cross(y_b, z_b)
    R = np.column_stack([x_b, y_b, z_b])

    w_x = -y_b @ dz
    w_y = x_b @ dz
    w_z = (w_x * (z_b @ x_c) + fo.dpsi * (y_b @ y_c)) / n
    w = np.array([w_x, w_y, w_z])

    dw_x = -y_b @ ddz + w_y * w_z
    dw_y = x_b @ ddz - w_x * w_z
    return quat.q_from_R(R), w, dw_x, dw_y, m * T_bar, w_z


def flat_reference(traj, t, m, g, h=1e-4):
    fo = traj.ref(t)
    q, w, dw_x, dw_y, T, _ = _core(fo, m, g)
    # omega_z-dot via central difference of closed-form omega_z (see docstring)
    wz_p = _core(traj.ref(t + h), m, g)[5]
    wz_m = _core(traj.ref(t - h), m, g)[5]
    dw = np.array([dw_x, dw_y, (wz_p - wz_m) / (2.0 * h)])
    return RefState(q=q, w=w, dw=dw, T=T)
