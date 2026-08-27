"""Buzz-closes scorer unit tests (S4 Phase 2 spec §5): pure decision logic,
no I/O or fixture .BIN needed."""
from indi_harness.buzz_score import score


def test_buzz_closed_all_pass():
    r = score(track_rms=0.55, benign_rms=0.485, sat_frac=0.0,
              omega_dot_nrmse=0.4, tol=1.5, sat_tol=0.02, nrmse_tol=0.6)
    assert r["closed"] is True
    assert r["track_ok"] and r["sat_ok"] and r["nrmse_ok"]


def test_buzz_open_track_fails():
    r = score(track_rms=3.03, benign_rms=0.485, sat_frac=0.029,
              omega_dot_nrmse=1.5, tol=1.5, sat_tol=0.02, nrmse_tol=0.6)
    assert r["closed"] is False
    assert not r["track_ok"]


def test_buzz_open_when_only_nrmse_fails():
    r = score(track_rms=0.5, benign_rms=0.485, sat_frac=0.0,
              omega_dot_nrmse=0.9, tol=1.5, sat_tol=0.02, nrmse_tol=0.6)
    assert r["closed"] is False
    assert r["track_ok"] and r["sat_ok"] and not r["nrmse_ok"]
