"""Offboard outer loop (design doc S2): INDI linear-accel loop + flatness
attitude reference, emitting stock-inner-loop attitude+rate+thrust targets.

Differences from the S0 in-sim OuterLoopINDI (deliberate, offboard realities):
- Actuator state: no rotor speeds offboard -> the thrust-vector state is the
  *previously commanded* thrust vector passed through the same filter as the
  accel measurement (phase-matching rule, design doc #1 warning).
- Output: normalized throttle via the learned hover-throttle linear map
  (MOT_THST_HOVER); expo refinement deliberately omitted — S2 embraces
  absolute-number mismatch (design doc S2 'known limitation').
- indi_accel=False degrades to PD+ff on the reference (no accel measurement
  needed) — the fallback if the audit found no usable IMU topic.
"""
from collections import deque
from dataclasses import dataclass, field
import numpy as np
from .. import quat
from ..filters import Butter2
from ..flatness import flat_reference
from ..outer_loop import attitude_from_thrust_dir
from .setpoints import AttitudeSetpoint

E3 = np.array([0.0, 0.0, 1.0])


@dataclass
class OffboardGains:
    kp: np.ndarray = field(default_factory=lambda: np.array([2.5, 2.5, 3.0]))
    kv: np.ndarray = field(default_factory=lambda: np.array([2.5, 2.5, 3.0]))
    cutoff_hz: float = 4.0     # accel + thrust-state pair (shared, always)
    indi_accel: bool = True
    # Ticks by which the fed-back commanded thrust vector (f_state) is delayed
    # before it is differenced against the measured specific force (f_meas).
    # The INDI increment f_state - f_meas only cancels if both refer to the
    # SAME instant's thrust. f_meas reflects the command actually realized
    # ~(command-path latency) ticks ago; f_state must be delayed to match, or
    # the residual accumulates and f_cmd winds up (the offboard-flight failure
    # mode). 1 = "previous tick" (correct only when the actuator responds
    # within a tick, e.g. the in-sim S0 loop); set ~= measured command-path
    # latency in ticks for a latency-laden path. NOTE: robust to
    # under-estimating the latency, unstable if you over-estimate — bias low.
    cmd_delay_ticks: int = 1


class OffboardOuterLoop:
    def __init__(self, params, gains, fs, hover_throttle):
        self.P, self.G, self.fs = params, gains, fs
        self.hover_throttle = hover_throttle
        self.f_accel = Butter2(gains.cutoff_hz, fs, 3)
        self.f_state = Butter2(gains.cutoff_hz, fs, 3)
        # ring of past commanded thrust vectors; [0] is the oldest (the one
        # delayed by cmd_delay_ticks). maxlen=1 reproduces "previous tick".
        depth = max(1, int(getattr(gains, "cmd_delay_ticks", 1)))
        self._f_cmd_hist = deque([-self.P.g * E3] * depth, maxlen=depth)

    def tick(self, t, traj, p, v, q, f_b):
        """All inputs NED/FRD. Returns AttitudeSetpoint (NED/FRD)."""
        P, G = self.P, self.G
        fo = traj.ref(t)
        ref = flat_reference(traj, t, P.m, P.g)
        a_cmd = fo.a + G.kp * (fo.p - p) + G.kv * (fo.v - v)
        if G.indi_accel:
            f_meas_f = self.f_accel.update(quat.qrot(q, f_b))
            f_state_f = self.f_state.update(self._f_cmd_hist[0])  # delay-aligned
            f_cmd = f_state_f + (a_cmd - P.g * E3) - f_meas_f
        else:
            f_cmd = a_cmd - P.g * E3
        self._f_cmd_hist.append(f_cmd)
        T_bar = float(np.linalg.norm(f_cmd))
        z_b_des = -f_cmd / T_bar
        q_cmd = attitude_from_thrust_dir(z_b_des, fo.psi)
        thrust = self.hover_throttle * T_bar / P.g
        return AttitudeSetpoint(q=q_cmd, w=ref.w, thrust=thrust)
