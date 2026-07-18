"""Tracking metrics. Reused verbatim by the S1 harness evaluator — keep free
of sim-specific assumptions (takes plain arrays)."""
import numpy as np


def rmse_position(t, p, p_ref, trim_s=1.0):
    """Per-axis + total position RMSE after trimming the initial transient."""
    t = np.asarray(t, float)
    keep = t >= (t[0] + trim_s)
    e = np.asarray(p)[keep] - np.asarray(p_ref)[keep]
    per_axis = np.sqrt(np.mean(e ** 2, axis=0))
    total = float(np.sqrt(np.mean(np.sum(e ** 2, axis=1))))
    return per_axis, total
