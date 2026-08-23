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
    def __init__(self, params, drag_on=True, dt=1.0 / 400.0, ground=False):
        # ground: enable a flat ground plane at the NED origin (D=0). A disarmed
        # copter has motors off (thrust 0), so without ground contact it
        # free-falls forever and the pysignals state / the downstream ArduPilot
        # EKF diverge into NaN -> SITL's feenableexcept trap aborts with a
        # floating-point exception before the vehicle can ever arm. Ground
        # contact is required for the SITL smoke/gate (server sets ground=True)
        # but must stay OFF for the airborne parity/drag/accel unit tests, whose
        # trajectories sit at or pass through D=0 and would be perturbed by a
        # rest clamp -- hence opt-in rather than always-on.
        self.P, self.drag_on, self.dt = params, drag_on, dt
        self.ground = ground
        self.M = params.mixer()
        self._omega = np.zeros(4)
        self._F_body = np.zeros(3)
        self._on_ground = ground  # starts resting on the ground plane
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
        # Flat ground plane at the NED origin (D=0). While resting on the ground
        # we HOLD the integrator state (skip simulateEuler): the vehicle sits
        # level at rest, so pose/twist stay at their last values and the reported
        # accelerometer reads -g (see state()). We can't pin via x.update -- the
        # SE3StateSignal ignores an update at an already-sampled time, and
        # x.reset()+reseed loses a step to integrator priming, which would
        # deadlock liftoff if done every step. Holding avoids the integrator
        # entirely until thrust overcomes weight, then releases cleanly.
        if self.ground and self._on_ground:
            # Level on the ground -> world-frame lift is just the body -z thrust;
            # release once it exceeds weight (net upward). wrench6[2] is the body
            # z force (down +), so -wrench6[2] is upward thrust.
            if -wrench6[2] <= self.P.m * self.P.g:
                # Rest on the surface: pin pose to D=0 at zero velocity and
                # advance the signal's sample time to keep it in lockstep with
                # self.t. Skipping the update (stale sample time) makes the next
                # real simulateEuler integrate over the whole held interval ->
                # huge jump; updating at the new (strictly increasing) time is
                # honoured (unlike an update at an already-sampled time).
                tn = self.t + self.dt
                s = self._m.x(self.t)
                p = np.asarray(s.pose.t()).ravel()
                s.pose = geo.SE3.fromVecAndQuat(
                    np.array([p[0], p[1], 0.0]), s.pose.q())
                s.twist = np.zeros(6)
                self._m.x.update(tn, s)
                self.t = tn
                return
            self._on_ground = False  # liftoff: fall through and integrate
        tn = self.t + self.dt
        self._u.update(self.t, wrench6)
        if not self._m.simulateEuler(self._u, tn, self.dt):
            raise RuntimeError("pysignals simulateEuler failed")
        self.t = tn
        if self.ground:
            pos_d = float(np.asarray(self._m.x(self.t).pose.t()).ravel()[2])
            if pos_d >= 0.0:
                self._on_ground = True  # touched down / never left

    def state(self):
        s = self._m.x(self.t)
        pos = np.asarray(s.pose.t()).ravel()
        q = np.asarray(s.pose.q().array()).ravel()
        tw = np.asarray(s.twist).ravel()
        vel_ned = quat.qrot(q, tw[:3])
        # Specific force (what the accelerometer reads), body FRD. Airborne this
        # is thrust/m; resting on the ground the surface normal balances gravity,
        # so the accelerometer reads -g in body z (level) rather than the 0 that
        # motors-off F_body/m would give -- a free-fall reading on the ground
        # would prevent the EKF from ever settling / the vehicle from prearming.
        if self.ground and self._on_ground:
            accel_body = np.array([0.0, 0.0, -self.P.g])
        else:
            accel_body = self._F_body / self.P.m
        return {
            "timestamp": self.t,
            "position": pos.tolist(),
            "velocity": vel_ned.tolist(),
            "quaternion": q.tolist(),
            "gyro": tw[3:].tolist(),
            "accel_body": accel_body.tolist(),
        }
