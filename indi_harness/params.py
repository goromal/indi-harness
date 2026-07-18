"""Quad parameters shared by the sim (truth) and the controller (model).
Controller model error is injected explicitly via .perturbed() — never by
maintaining two hand-edited copies."""
from dataclasses import dataclass, replace
import numpy as np


@dataclass
class QuadParams:
    m: float = 1.0            # kg
    g: float = 9.81
    J: np.ndarray = None      # inertia diag [kg m^2]
    arm: float = 0.15         # m, center to rotor
    kf: float = 1.0e-5        # N / (rad/s)^2
    km: float = 1.6e-7        # N m / (rad/s)^2
    tau_m: float = 0.03       # motor time constant [s]
    # Rotor inertia must be physically consistent with the prop class implied
    # by kf: the instantaneous yaw reaction to a commanded yaw torque has
    # magnitude ratio Ir/(2*Omega_hover*km*tau_m); above ~1 the yaw loop is
    # non-minimum-phase-unstable (observed as a wind-triggered limit cycle).
    # 5e-7 gives ratio ~0.2 for this small-fast-prop parameter set.
    Ir: float = 5.0e-7        # rotor inertia [kg m^2]
    Omega_min: float = 100.0  # rad/s
    Omega_max: float = 900.0
    drag_D: np.ndarray = None  # rotor-drag coeffs, body frame [N/(m/s)]

    def __post_init__(self):
        if self.J is None:
            self.J = np.diag([5.0e-3, 5.0e-3, 9.0e-3])
        if self.drag_D is None:
            self.drag_D = np.diag([0.35, 0.35, 0.15])

    def hover_speed(self):
        return np.sqrt(self.m * self.g / (4.0 * self.kf))

    # Motor layout: X config, FRD body, order [FR, BL, FL, BR] (ArduPilot X).
    # d = +1: CCW prop (viewed from above) -> +yaw reaction in NED.
    def layout(self):
        a = self.arm / np.sqrt(2.0)
        pos = np.array([[a, a, 0], [-a, -a, 0], [a, -a, 0], [-a, a, 0]], float)
        d = np.array([1.0, 1.0, -1.0, -1.0])
        return pos, d

    def mixer(self):
        """M (4x4): [T_up; tau_x; tau_y; tau_z] = M @ Omega^2."""
        pos, d = self.layout()
        M = np.zeros((4, 4))
        for i in range(4):
            M[0, i] = self.kf                       # total upward thrust [N]
            M[1, i] = -self.kf * pos[i, 1]          # tau_x = -kf*Omega^2*ry
            M[2, i] = self.kf * pos[i, 0]           # tau_y = +kf*Omega^2*rx
            M[3, i] = d[i] * self.km                # yaw drag torque
        return M

    def perturbed(self, kf_scale=1.0, km_scale=1.0, tau_scale=1.0):
        """Controller-side model error injection (S5-style robustness tests)."""
        return replace(self, kf=self.kf * kf_scale, km=self.km * km_scale,
                       tau_m=self.tau_m * tau_scale, J=self.J.copy(),
                       drag_D=self.drag_D.copy())
