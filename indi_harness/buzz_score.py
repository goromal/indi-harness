"""Buzz-closes scorer: the C2 primary success gate (S4 Phase 2 spec §5).

All thresholds are explicit args so the calibrate-then-freeze values live in
the CI env (Task 8), never hardcoded here.
"""


def score(track_rms, benign_rms, sat_frac, omega_dot_nrmse,
          tol, sat_tol, nrmse_tol):
    """Score a single flight against the buzz-closes gate.

    track_rms/benign_rms: tracking-error RMS under disturbance vs. benign
        baseline (m or rad, whatever the caller measured).
    sat_frac: fraction of ticks with actuator saturation active.
    omega_dot_nrmse: normalized RMSE of measured vs. predicted angular
        accel (the INDI model-fit quality signal).
    tol/sat_tol/nrmse_tol: explicit pass/fail thresholds for the three
        criteria above (no hardcoded magic numbers here).

    Returns a dict with the raw inputs, per-criterion bool flags, and the
    overall `closed` verdict (True only if all three criteria pass).
    """
    track_ok = track_rms <= benign_rms * tol
    sat_ok = sat_frac <= sat_tol
    nrmse_ok = omega_dot_nrmse <= nrmse_tol
    return {
        "track_rms": track_rms,
        "benign_rms": benign_rms,
        "sat_frac": sat_frac,
        "omega_dot_nrmse": omega_dot_nrmse,
        "track_ok": bool(track_ok),
        "sat_ok": bool(sat_ok),
        "nrmse_ok": bool(nrmse_ok),
        "closed": bool(track_ok and sat_ok and nrmse_ok),
    }
