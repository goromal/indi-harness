"""Discrete 2nd-order Butterworth low-pass via pre-warped bilinear transform.

INDI phase-matching rule: every signal pair whose difference forms an INDI
increment (angular accel vs actuator state; accel vs thrust state) must pass
through filters constructed with identical (cutoff_hz, fs_hz).
"""
import numpy as np


class Butter2:
    def __init__(self, cutoff_hz, fs_hz, n_ch):
        if not 0.0 < cutoff_hz < fs_hz / 2.0:
            raise ValueError(
                f"cutoff_hz={cutoff_hz} must lie in (0, fs_hz/2={fs_hz / 2.0})")
        wc = np.tan(np.pi * cutoff_hz / fs_hz)
        k1 = np.sqrt(2.0) * wc
        k2 = wc * wc
        a0 = 1.0 + k1 + k2
        self.b = np.array([k2, 2 * k2, k2]) / a0
        self.a = np.array([2.0 * (k2 - 1.0), 1.0 - k1 + k2]) / a0
        self.n_ch = n_ch
        self.reset(np.zeros(n_ch))

    def reset(self, x0):
        x0 = np.asarray(x0, float)
        if x0.shape != (self.n_ch,):
            raise ValueError(f"expected shape ({self.n_ch},), got {x0.shape}")
        self.x1 = x0.copy()
        self.x2 = x0.copy()
        self.y1 = x0.copy()
        self.y2 = x0.copy()

    def update(self, x):
        x = np.asarray(x, float)
        if x.shape != (self.n_ch,):
            raise ValueError(f"expected shape ({self.n_ch},), got {x.shape}")
        y = (self.b[0] * x + self.b[1] * self.x1 + self.b[2] * self.x2
             - self.a[0] * self.y1 - self.a[1] * self.y2)
        self.x2, self.x1 = self.x1, x.copy()
        self.y2, self.y1 = self.y1, y.copy()
        return y
