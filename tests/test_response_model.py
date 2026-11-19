"""Delay-compensated omega_dot scoring: the commanded angular accel is passed
through the known actuator/sensor chain before it is compared with the logged
measured angular accel (Vikunja #168-#170)."""
import numpy as np
import pytest

from indi_harness.filters import Butter2
from indi_harness.sitl.response_model import (
    ResponseChain,
    expected_measured_accel,
    response_chain_from_parameters,
)
from indi_harness.sitl.align import (
    delay_compensated_tracking_score,
    excited_axis_nrmse,
)

FS = 400.0
CHAIN = ResponseChain(sample_hz=FS, motor_tau_s=0.03, gyro_filter_hz=20.0,
                      estimator_filter_hz=80.0, frame_delay=1)


def _command(n=4000, seed=0):
    """Band-limited random commanded angular accel, [n,3]."""
    rng = np.random.default_rng(seed)
    raw = rng.standard_normal((n, 3))
    smooth = Butter2(4.0, FS, 3)
    return np.array([smooth.update(r) for r in raw]) * 5.0


def test_step_is_delayed_one_frame_then_rises_with_motor_lag():
    chain = ResponseChain(sample_hz=FS, motor_tau_s=0.03, gyro_filter_hz=None,
                          estimator_filter_hz=None, frame_delay=1)
    nu = np.zeros((100, 3))
    nu[10:] = 1.0
    y = expected_measured_accel(nu, chain)
    assert np.all(y[:11] == 0.0)
    k = (1.0 / FS) / 0.03
    assert y[11, 0] == pytest.approx(k)
    # after one time constant (12 ticks) a first-order lag reaches ~63 %
    assert y[11 + 12, 0] == pytest.approx(1.0 - (1.0 - k) ** 13, rel=1e-9)
    assert 0.6 < y[11 + 12, 0] < 0.7


def test_filters_match_firmware_butterworth():
    chain = ResponseChain(sample_hz=FS, motor_tau_s=None, gyro_filter_hz=20.0,
                          estimator_filter_hz=None, frame_delay=0)
    nu = _command()
    ref = Butter2(20.0, FS, 3)
    expected = np.array([ref.update(r) for r in nu - nu[0]]) + nu[0]
    np.testing.assert_allclose(expected_measured_accel(nu, chain), expected,
                               atol=1e-12)


def test_ideal_inversion_scores_near_zero_after_compensation():
    nu = _command()
    meas = expected_measured_accel(nu, CHAIN)
    health = {"time_us": np.arange(len(nu)) * 2500.0,
              "domega_pred": nu, "domega_meas": meas}
    s = delay_compensated_tracking_score(health, CHAIN)
    for ax in range(3):
        assert s[ax]["nrmse"] < 1e-9
        # the uncompensated command-vs-response NRMSE stays visible
        assert s[ax]["nrmse_raw"] > 0.6


def test_compensation_does_not_hide_a_wrong_effectiveness():
    # An attenuated response (inversion with the wrong G1) must still fail.
    nu = _command()
    meas = 0.3 * expected_measured_accel(nu, CHAIN)
    health = {"time_us": np.arange(len(nu)) * 2500.0,
              "domega_pred": nu, "domega_meas": meas}
    s = delay_compensated_tracking_score(health, CHAIN)
    assert all(s[ax]["nrmse"] > 0.6 for ax in range(3))


def test_chain_restarts_at_logging_gaps():
    # Rows are only logged while the custom controller runs; a gap between
    # cases must not carry filter state across it.
    nu = _command(2000)
    t = np.arange(2000) * 2500.0
    t[1000:] += 5e6
    whole = expected_measured_accel(nu, CHAIN, time_us=t)
    second = expected_measured_accel(nu[1000:], CHAIN)
    np.testing.assert_allclose(whole[1000:], second, atol=1e-12)


def test_excited_axis_nrmse_skips_quiet_axes_and_requires_one_excited():
    per = {0: {"nrmse": 0.3, "exc_rms": 1.0},
           1: {"nrmse": 5.0, "exc_rms": 0.01},
           2: {"nrmse": 9.0, "exc_rms": 3.0}}
    assert excited_axis_nrmse(per, axes=(0, 1), exc_floor=0.08) == 0.3
    quiet = {0: {"nrmse": 2.0, "exc_rms": 0.01},
             1: {"nrmse": 2.0, "exc_rms": 0.02}}
    assert excited_axis_nrmse(quiet, axes=(0, 1), exc_floor=0.08) is None


def test_chain_from_logged_parameters():
    params = {"SCHED_LOOP_RATE": 400.0, "INS_GYRO_FILTER": 20.0,
              "CC3_OMG_FILT": 80.0}
    chain = response_chain_from_parameters(params, motor_tau_s=0.03)
    assert chain == CHAIN
    # OMG_FILT 0 selects the legacy differentiate-then-filter estimator, which
    # filters at CC3_FILT_HZ instead.
    legacy = response_chain_from_parameters(
        {**params, "CC3_OMG_FILT": 0.0, "CC3_FILT_HZ": 40.0}, motor_tau_s=0.03)
    assert legacy.estimator_filter_hz == 40.0
    # INS_GYRO_FILTER 0 disables the stock gyro filter.
    assert response_chain_from_parameters(
        {**params, "INS_GYRO_FILTER": 0.0}, motor_tau_s=0.03).gyro_filter_hz is None
    with pytest.raises(KeyError):
        response_chain_from_parameters({"SCHED_LOOP_RATE": 400.0}, motor_tau_s=0.03)


def test_compensated_window_uses_whole_flight_chain():
    from indi_harness.sitl.binscore import compensated_omega_window
    nu = _command()
    meas = expected_measured_accel(nu, CHAIN)
    ih = {"time_us": np.arange(len(nu)) * 2500.0,
          "domega_pred": nu, "domega_meas": meas}
    out = compensated_omega_window(ih, meas, CHAIN, 2500.0 * 1000, 2500.0 * 3000)
    for nm in ("roll", "pitch", "yaw"):
        assert out[nm]["nrmse"] < 1e-9
        assert out[nm]["nrmse_raw"] > 0.6
