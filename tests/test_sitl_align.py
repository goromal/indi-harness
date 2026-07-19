import numpy as np
from indi_harness.trajectory import Circle
from indi_harness.sitl.align import boot_to_traj_map, evaluate_bin

def make_pairs(rate=0.5, offset_ms=5000.0, n=200):
    """SITL running at `rate` x real time: boot_ms advances slower."""
    traj_t = np.linspace(0.0, 20.0, n)
    boot_ms = offset_ms + traj_t * 1000.0 * rate
    return traj_t, boot_ms

def test_map_recovery_half_speed():
    traj_t, boot_ms = make_pairs(rate=0.5)
    k, b = boot_to_traj_map(traj_t, boot_ms)
    assert np.isclose(k, 1.0 / (1000.0 * 0.5), atol=1e-9)
    assert np.allclose(k * boot_ms + b, traj_t, atol=1e-6)

def test_evaluate_bin_zero_error_on_reference():
    traj = Circle(radius=2.0, period=8.0, alt=10.0)
    origin_offset = np.array([1.0, 2.0, 0.0])
    traj_t, boot_ms = make_pairs(rate=1.0)
    k, b = boot_to_traj_map(traj_t, boot_ms)
    time_us = boot_ms * 1000.0
    p_bin = np.array([traj.ref(t).p for t in traj_t]) + origin_offset
    per_axis, total = evaluate_bin(time_us, p_bin, k, b, traj,
                                   origin_offset, trim_s=1.0)
    assert total < 1e-9

def test_evaluate_bin_detects_offset():
    traj = Circle(radius=2.0, period=8.0, alt=10.0)
    traj_t, boot_ms = make_pairs(rate=1.0)
    k, b = boot_to_traj_map(traj_t, boot_ms)
    time_us = boot_ms * 1000.0
    p_bin = (np.array([traj.ref(t).p for t in traj_t])
             + np.array([0.3, 0.0, 0.0]))
    per_axis, total = evaluate_bin(time_us, p_bin, k, b, traj,
                                   np.zeros(3), trim_s=1.0)
    assert np.isclose(total, 0.3, atol=1e-6)
    assert np.isclose(per_axis[0], 0.3, atol=1e-6)
