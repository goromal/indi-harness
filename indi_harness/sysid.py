"""Layer-C sysid: analytic effectiveness seed from known constants + a
doublet-regression confirm (added in a later task). The analytic seed is BOTH
the param seed and the regression oracle."""
import numpy as np


def analytic_seed(params):
    """Derive Layer-C effectiveness params from the quad constants.
    See indi_harness.params.QuadParams (mixer(), layout(), J, kf, km, arm, Ir,
    hover_speed(), Omega_min/max).

    g1_torque is the *torque-space* effectiveness the Layer-A fallback path
    uses (design-doc: 'map Delta-omega_dot -> Delta-tau via diagonal G1
    (torque-space)', i.e. Delta-tau = Delta-omega_dot / G1). Since the rigid
    body EOM is omega_dot = J^-1 @ tau, that ratio is exactly J^-1 -- it does
    NOT depend on kf/km/arm, which only govern the *actuator* allocation
    (M/Minv, below) that later converts a torque request into per-rotor
    Omega^2 commands for the Layer-C RPM path. Numerically 1/J = [200, 200,
    111] rad/s^2 per N*m for the default SITL quad, matching the design
    doc's cited true-effectiveness ballpark (~175-479 rad/s^2/unit) for
    roll/pitch; yaw is a bit lower (weaker yaw authority: J_yaw > J_roll)
    but still comfortably inside the loose acceptance band.
    """
    P = params
    M = P.mixer()
    Minv = np.linalg.inv(M)
    Jd = np.diag(P.J)
    g1_torque = 1.0 / Jd

    # Throttle -> RPM feedforward (ESC characteristic): assume a linear ESC
    # response spanning the full commandable rotor-speed range, throttle in
    # [0, 1] -> Omega in [Omega_min, Omega_max]. This is independent of kf
    # (kf maps Omega -> thrust; this map is throttle command -> Omega).
    a = P.Omega_max - P.Omega_min
    b = P.Omega_min

    return {
        "M": M, "Minv": Minv,
        "g1_torque": g1_torque,
        "g2_yaw": P.Ir,
        "throttle_rpm_a": a, "throttle_rpm_b": b,
        "omega_min": P.Omega_min, "omega_max": P.Omega_max,
    }
