"""pysignals-based quad physics for the ArduPilot SITL JSON backend.

NED world, FRD body. State integrated by pysignals RigidBody6DOFModel; the
quad-specific physics (rotor mixer, first-order motor lag, linear rotor drag)
compose the body wrench u=[F;tau] fed to the integrator each step.
Validated against the S0 numpy QuadSim (simmodel.py) in a later task.

NOTE on the installed pysignals build: empirically (see tests/test_jsonsim_model.py
history), RigidBody6DOFModel integrates v_dot = F/m - g_param, i.e. the g_param
field of RigidBodyParams3D is SUBTRACTED, not added. To reproduce the standard
NED convention used elsewhere in this repo (simmodel.py: thrust-up is a
negative-z body force, gravity accelerates +z), g_param must be passed as
NEGATIVE g, not +g. Do not "fix" this to np.array([0,0,params.g]) without
re-verifying hover equilibrium -- that sign was checked against the actual
installed binary, not assumed from documentation.
"""
import numpy as np
import pysignals as ps
import geometry as geo  # noqa: F401  (registers SE3/SO3 return types)
from indi_harness import quat


class QuadJsonModel:
    def __init__(self, params, drag_on=True, dt=1.0 / 400.0):
        self.P, self.drag_on, self.dt = params, drag_on, dt
        self.M = params.mixer()
        self._omega = np.zeros(4)
        self._F_body = np.zeros(3)
        self.t = 0.0
        rp = ps.RigidBodyParams3D()
        rp.m = float(params.m)
        rp.J = np.asarray(params.J, float)
        rp.g = np.array([0.0, 0.0, -params.g])  # see NOTE above
        self._m = ps.RigidBody6DOFModel()
        self._m.reset()
        self._m.setParams(rp)
        self._m.x.update(self.t, ps.SE3State.identity())
        self._u = ps.Vector6Signal()

    def seed_omega(self, omega):
        self._omega = np.clip(np.asarray(omega, float), 0.0, self.P.Omega_max)

    def _wrench(self, omega_cmd):
        P = self.P
        omega_cmd = np.clip(np.asarray(omega_cmd, float), 0.0, P.Omega_max)
        self._omega += (omega_cmd - self._omega) * (self.dt / P.tau_m)
        u = self.M @ (self._omega ** 2)
        F_body = np.array([0.0, 0.0, -u[0]])
        tau = u[1:].copy()
        if self.drag_on:
            v_body = np.asarray(self._m.x(self.t).twist).ravel()[:3]
            F_body = F_body - P.drag_D @ v_body
        return np.concatenate([F_body, tau])

    def step_omega(self, omega_cmd):
        self.step_wrench(self._wrench(omega_cmd))

    def step_wrench(self, wrench6):
        wrench6 = np.asarray(wrench6, float)
        self._F_body = wrench6[:3].copy()
        tn = self.t + self.dt
        self._u.update(self.t, wrench6)
        if not self._m.simulateEuler(self._u, tn, self.dt):
            raise RuntimeError("pysignals simulateEuler failed")
        self.t = tn

    def state(self):
        s = self._m.x(self.t)
        pos = np.asarray(s.pose.t()).ravel()
        q = np.asarray(s.pose.q().array()).ravel()
        tw = np.asarray(s.twist).ravel()
        vel_ned = quat.qrot(q, tw[:3])
        return {
            "timestamp": self.t,
            "position": pos.tolist(),
            "velocity": vel_ned.tolist(),
            "quaternion": q.tolist(),
            "gyro": tw[3:].tolist(),
            "accel_body": (self._F_body / self.P.m).tolist(),
        }
