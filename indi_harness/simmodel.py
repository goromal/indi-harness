"""Toy quadrotor: NED world, FRD body. Semi-implicit Euler at dt=5e-4.

Thrust and torques are driven by the *lagged* rotor speed self.Omega
(first-order lag, time constant tau_m) — the actuator dynamics the INDI
controller must contend with. Scenarios that start airborne must seed
self.Omega (e.g. at hover_speed()); motors spin up from zero otherwise.
"""
import numpy as np
from . import quat


class QuadSim:
    def __init__(self, params, dt=5e-4, drag_on=True, wind=np.zeros(3), q0=None,
                 p0=np.zeros(3), v0=np.zeros(3)):
        self.P, self.dt, self.drag_on = params, dt, drag_on
        self.wind = np.asarray(wind, float).copy()
        self.p = np.asarray(p0, float).copy()
        self.v = np.asarray(v0, float).copy()
        self.q = np.array([1.0, 0, 0, 0]) if q0 is None else np.asarray(q0, float)
        self.omega = np.zeros(3)
        self.Omega = np.zeros(4)
        self.f_ext = lambda t: np.zeros(3)   # external force hook, world [N]
        self.t = 0.0
        self._a_world = np.zeros(3)
        self.M = params.mixer()
        _, self.d = params.layout()

    def _forces_torques(self):
        P = self.P
        u = self.M @ (self.Omega ** 2)       # [T_up, tau_x, tau_y, tau_z]
        F_thrust_b = np.array([0.0, 0.0, -u[0]])
        tau = u[1:].copy()
        F_b = F_thrust_b
        if self.drag_on:
            v_air_b = quat.qrot_inv(self.q, self.v - self.wind)
            F_b = F_b - P.drag_D @ v_air_b
        return F_b, tau

    def step(self, Omega_cmd):
        P, dt = self.P, self.dt
        Omega_cmd = np.clip(Omega_cmd, 0.0, P.Omega_max)
        dOmega = (Omega_cmd - self.Omega) / P.tau_m
        F_b, tau = self._forces_torques()
        # rotor-inertia yaw reaction: accelerating a CCW rotor (+d) torques body -z
        tau[2] -= P.Ir * float(self.d @ dOmega)
        F_w = quat.qrot(self.q, F_b) + self.f_ext(self.t)
        a = np.array([0.0, 0.0, P.g]) + F_w / P.m
        self._a_world = a
        # semi-implicit Euler
        self.v += a * dt
        self.p += self.v * dt
        self.omega += np.linalg.solve(P.J, tau - np.cross(self.omega, P.J @ self.omega)) * dt
        self.q = quat.qnormalize(quat.qmul(self.q, quat.qexp(self.omega * dt)))
        self.Omega = np.clip(self.Omega + dOmega * dt, 0.0, P.Omega_max)
        self.t += dt

    def specific_force(self):
        """Body-frame accelerometer output (no noise in S0)."""
        return quat.qrot_inv(self.q, self._a_world - np.array([0.0, 0.0, self.P.g]))
