"""Flat-output reference trajectories (NED world frame).

YAML polynomial format (single source of truth — the S1 onboard-Lua and DDS
trajectory-server evaluators must consume these same files):

    segments:
      - duration: 4.0
        # coeffs[axis][k] multiplies t^k, t local to the segment, axes x,y,z,yaw
        coeffs:
          x:   [0.0, 0.0, 0.0, 0.3125, -0.1171875, 0.01171875]
          y:   [0.0]
          z:   [-5.0]
          yaw: [0.0]
"""
from dataclasses import dataclass
import numpy as np
import yaml


@dataclass
class FlatOutput:
    p: np.ndarray   # position (3,) NED [m]
    v: np.ndarray
    a: np.ndarray
    j: np.ndarray
    s: np.ndarray
    psi: float      # yaw [rad]
    dpsi: float
    ddpsi: float


def _sinusoid(amp, w, phase, t, order):
    """order-th derivative of amp*sin(w t + phase)."""
    return amp * w ** order * np.sin(w * t + phase + order * np.pi / 2)


# Trajectory contract (duck-typed): every class below exposes .duration [s]
# and .ref(t) -> FlatOutput; consumers rely on nothing else.
class Hover:
    def __init__(self, point, psi=0.0):
        self.point = np.asarray(point, float)
        self.psi = psi
        self.duration = np.inf

    def ref(self, t):
        z3 = np.zeros(3)
        return FlatOutput(self.point.copy(), z3, z3.copy(), z3.copy(), z3.copy(),
                          self.psi, 0.0, 0.0)


class Circle:
    """x = R cos(wt), y = R sin(wt), z = -alt; fixed yaw."""
    def __init__(self, radius, period, alt, psi=0.0):
        self.R, self.w, self.alt, self.psi = radius, 2 * np.pi / period, alt, psi
        self.duration = period

    def ref(self, t):
        d = [np.array([
            _sinusoid(self.R, self.w, np.pi / 2, t, k),   # cos = sin(+pi/2)
            _sinusoid(self.R, self.w, 0.0, t, k),
            0.0]) for k in range(5)]
        d[0][2] = -self.alt
        return FlatOutput(*d, self.psi, 0.0, 0.0)


class Lemniscate:
    """Gerono figure-eight: x = A sin(wt), y = (A/2) sin(2wt), z = -alt."""
    def __init__(self, amplitude, period, alt, psi=0.0):
        self.A, self.w, self.alt, self.psi = amplitude, 2 * np.pi / period, alt, psi
        self.duration = period

    def ref(self, t):
        d = [np.array([
            _sinusoid(self.A, self.w, 0.0, t, k),
            _sinusoid(self.A / 2, 2 * self.w, 0.0, t, k),
            0.0]) for k in range(5)]
        d[0][2] = -self.alt
        return FlatOutput(*d, self.psi, 0.0, 0.0)


class PolynomialTrajectory:
    def __init__(self, segments):
        # segments: list of (duration, {axis: np.ndarray coeffs low->high})
        self.segments = segments
        self.duration = sum(s[0] for s in segments)

    @classmethod
    def from_yaml(cls, path):
        with open(path) as f:
            spec = yaml.safe_load(f)
        segs = []
        for seg in spec["segments"]:
            coeffs = {ax: np.asarray(seg["coeffs"].get(ax, [0.0]), float)
                      for ax in ("x", "y", "z", "yaw")}
            segs.append((float(seg["duration"]), coeffs))
        return cls(segs)

    @staticmethod
    def _polyval_derivs(c, t, n_derivs=5):
        out = []
        for k in range(n_derivs):
            if len(c) == 0:
                out.append(0.0)
                continue
            # Horner on the k-th derivative coefficients
            dc = c.copy()
            for _ in range(k):
                dc = dc[1:] * np.arange(1, len(dc))
            out.append(float(np.polyval(dc[::-1], t)) if len(dc) else 0.0)
        return out

    def ref(self, t):
        # t_local <= dur holds on break for any t in [0, duration], including
        # t == duration exactly (left-continuous at segment boundaries).
        t_local = min(max(float(t), 0.0), self.duration)
        for dur, coeffs in self.segments:
            if t_local <= dur:
                break
            t_local -= dur
        vals = {ax: self._polyval_derivs(coeffs[ax], t_local) for ax in coeffs}

        def arr(k):
            return np.array([vals["x"][k], vals["y"][k], vals["z"][k]])

        return FlatOutput(arr(0), arr(1), arr(2), arr(3), arr(4),
                          vals["yaw"][0], vals["yaw"][1], vals["yaw"][2])
