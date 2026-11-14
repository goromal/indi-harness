"""Expected logged angular-accel response to the INDI angular-accel command.

The INDI dataflash row logs Px (the commanded angular accel nu at tick k) and
Mx (the filtered measured angular accel at tick k) with one timestamp. They are
NOT time-aligned by construction: with an exact inversion the true angular
accel follows nu through the actuator dynamics, and Mx then sees it through the
gyro and estimator filters:

    Mx ~= LPF_estimator * LPF_gyro * motor_lag * z^-frame_delay * nu

On the JSON plant this chain is ~30-50 ms (motor lag tau_m 30 ms, stock
INS_GYRO_FILTER 20 Hz), so a raw Px-vs-Mx NRMSE cannot reach 0.6 above ~2 Hz
content however good the inversion is (Vikunja #168-#170). Passing Px through
this known chain before scoring tests the inversion itself; nothing is fitted.

Filters are the firmware's 2nd-order Butterworth (filters.Butter2), run at the
control-loop rate. The stock gyro filter actually runs at the IMU backend rate,
which has the same low-frequency group delay.
"""
from dataclasses import dataclass
from typing import Optional

import numpy as np

from ..filters import Butter2


@dataclass(frozen=True)
class ResponseChain:
    sample_hz: float
    motor_tau_s: Optional[float]           # first-order actuator lag; None = none
    gyro_filter_hz: Optional[float]        # stock INS_GYRO_FILTER; None = off
    estimator_filter_hz: Optional[float]   # CC3 angular-accel estimator cutoff
    frame_delay: int = 1                   # ticks before a command reaches the gyro


def response_chain_from_parameters(parameters, motor_tau_s, frame_delay=1):
    """Build the chain from logged PARM values (raises KeyError if missing).

    CC3_OMG_FILT > 0 is the filter-then-differentiate estimator at that
    cutoff; 0 is the legacy differentiate-then-filter estimator at CC3_FILT_HZ.
    INS_GYRO_FILTER 0 disables the stock gyro filter."""
    sample_hz = float(parameters["SCHED_LOOP_RATE"])
    gyro_hz = float(parameters["INS_GYRO_FILTER"])
    omega_hz = float(parameters["CC3_OMG_FILT"])
    estimator_hz = omega_hz if omega_hz > 0 else float(parameters["CC3_FILT_HZ"])
    return ResponseChain(
        sample_hz=sample_hz,
        motor_tau_s=motor_tau_s,
        gyro_filter_hz=gyro_hz if gyro_hz > 0 else None,
        estimator_filter_hz=estimator_hz if estimator_hz > 0 else None,
        frame_delay=int(frame_delay),
    )


def _segment(nu, chain):
    """Apply the chain to one contiguous block, starting from steady state."""
    y = np.asarray(nu, float)
    if chain.frame_delay > 0:
        d = min(chain.frame_delay, len(y))
        y = np.concatenate([np.repeat(y[:1], d, axis=0), y[:len(y) - d]])
    if chain.motor_tau_s:
        k = (1.0 / chain.sample_hz) / chain.motor_tau_s
        out = np.empty_like(y)
        state = y[0].copy()
        for i in range(len(y)):
            state = state + k * (y[i] - state)
            out[i] = state
        y = out
    for cutoff in (chain.gyro_filter_hz, chain.estimator_filter_hz):
        if cutoff:
            f = Butter2(cutoff, chain.sample_hz, y.shape[1])
            f.reset(y[0])
            y = np.array([f.update(row) for row in y])
    return y


def expected_measured_accel(nu, chain, time_us=None, gap_frames=5):
    """Expected logged measured angular accel for commanded nu [n,3].

    If time_us is given, the chain restarts from steady state wherever rows are
    more than gap_frames ticks apart (INDI rows are only written while the
    custom controller runs, so cases are separated by logging gaps)."""
    nu = np.asarray(nu, float)
    if len(nu) == 0:
        return nu.copy()
    starts = [0]
    if time_us is not None:
        gap_us = gap_frames * 1e6 / chain.sample_hz
        starts += list(np.flatnonzero(np.diff(np.asarray(time_us, float)) > gap_us) + 1)
    bounds = starts + [len(nu)]
    return np.concatenate([_segment(nu[a:b], chain)
                           for a, b in zip(bounds[:-1], bounds[1:])])
