"""INDI control allocation on rotor-speed-squared increments (Layer C math).

Delta(Omega^2) = M^-1 [DeltaT; Delta_tau + G2 term]
G2 term: measured rotor accel Om_dot reacts on body yaw as -Ir*(d . Om_dot);
the allocator adds +Ir*(d . Om_dot_f) to the yaw request to cancel it.
"""
import numpy as np


class Allocator:
    def __init__(self, params):
        self.P = params
        self.M = params.mixer()
        self.Minv = np.linalg.inv(self.M)
        _, self.d = params.layout()

    def static(self, T, tau):
        """Static inverse (feedforward init): returns Omega [rad/s]."""
        Om_sq = self.Minv @ np.concatenate([[T], tau])
        return np.sqrt(np.clip(Om_sq, self.P.Omega_min ** 2, self.P.Omega_max ** 2))

    def _solve(self, dT, dtau, Om_sq_f):
        return Om_sq_f + self.Minv @ np.concatenate([[dT], dtau])

    def indi_increment(self, dT, dtau, Om_sq_f, dOm_f):
        """Allocate increments onto rotor speeds.

        dT: thrust increment [N]; dtau: body torque increment [N m];
        Om_sq_f: filtered rotor speeds squared [rad^2/s^2] (actuator state);
        dOm_f: filtered rotor angular accel [rad/s^2] (drives G2 term).
        Returns (Omega^2 commands clipped to [Omega_min^2, Omega_max^2], sat).
        """
        P = self.P
        dtau = np.asarray(dtau, float).copy()
        dtau[2] += P.Ir * float(self.d @ dOm_f)          # G2 compensation
        lo, hi = P.Omega_min ** 2, P.Omega_max ** 2
        Om_sq = self._solve(dT, dtau, Om_sq_f)
        sat = bool(np.any(Om_sq < lo) or np.any(Om_sq > hi))
        if sat:
            # prioritized shedding: drop yaw increment first, keep tilt+thrust
            Om_sq = self._solve(dT, np.array([dtau[0], dtau[1], 0.0]), Om_sq_f)
            Om_sq = np.clip(Om_sq, lo, hi)
        return Om_sq, sat
