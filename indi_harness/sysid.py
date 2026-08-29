"""Layer-C sysid: analytic effectiveness seed from known constants + a
doublet-regression confirm. The analytic seed is BOTH the param seed and the
regression oracle: identify_g1/identify_g2 excite the oracle QuadSim and
regress the resulting torque/omega_dot trajectories to recover G1 and G2
independently of the analytic formulas below, then the tests assert the two
agree within ~15%."""
import numpy as np

from .simmodel import QuadSim


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


def analytic_g1(params):
    """Thin wrapper: the analytic torque-space effectiveness (1/J, length-3)."""
    return analytic_seed(params)["g1_torque"]


def analytic_g2(params):
    """Thin wrapper: the analytic rotor-inertia yaw-reaction gain (physical Ir)."""
    return analytic_seed(params)["g2_yaw"]


# X-quad allocation patterns (+/-1 per rotor) that excite one torque axis to
# first order while leaving the other two near zero, derived from
# QuadParams.layout(): pos = [[a,a,0],[-a,-a,0],[a,-a,0],[-a,a,0]], d =
# [1,1,-1,-1]. E.g. the roll pattern [1,-1,-1,1] has zero net rx-weighted and
# d-weighted sum (so tau_y, tau_z stay ~0 to first order in the perturbation)
# but a nonzero net ry-weighted sum (so tau_x is excited). Verified by
# construction, not fit -- this is a fixed geometric property of the layout.
_AXIS_PATTERNS = {
    0: np.array([1.0, -1.0, -1.0, 1.0]),   # roll
    1: np.array([1.0, -1.0, 1.0, -1.0]),   # pitch
    2: np.array([1.0, 1.0, -1.0, -1.0]),   # yaw
}


def identify_g1(params, T=0.4, amp=40.0, freq=2.5, dt=5e-4):
    """Doublet-regression recovery of G1 (torque-space effectiveness, 1/J).

    For each axis, drive the oracle QuadSim (indi_harness.simmodel.QuadSim,
    the S0 reference physics) near hover with a sinusoidal per-rotor
    perturbation from the single-axis allocation pattern above, so only that
    axis's torque is meaningfully excited. At each step we reconstruct the
    torque tau_k driving the sim from the KNOWN ACTUATOR MODEL only (mixer M
    built from kf/km/arm, plus the known rotor-inertia reaction term Ir) --
    NOT from J, which is what we are trying to recover, so this is not
    circular. We finite-difference the sim's own body rate to get the
    measured angular acceleration omega_dot_k. Per axis, G1_k is the
    origin-constrained least-squares slope of omega_dot_k on tau_k (the EOM
    omega_dot = J^-1 @ tau has zero intercept when the cross-coupling term
    omega x (J omega) is negligible, which it is here: amplitudes are kept
    modest so body rates stay small near hover).
    """
    P = params
    M = P.mixer()
    _, d = P.layout()
    Om_h = P.hover_speed()
    g1 = np.zeros(3)
    for k, pat in _AXIS_PATTERNS.items():
        sim = QuadSim(P, dt=dt, drag_on=False)
        sim.Omega = Om_h * np.ones(4)
        num = den = 0.0
        t = 0.0
        while t < T:
            oc = Om_h * np.ones(4) + amp * pat * np.sin(2 * np.pi * freq * t)
            Omega_prev = sim.Omega.copy()
            omega_prev = sim.omega.copy()
            dOmega = (oc - Omega_prev) / P.tau_m
            u = M @ (Omega_prev ** 2)
            tau = u[1:].copy()
            tau[2] -= P.Ir * float(d @ dOmega)      # known rotor-inertia reaction
            sim.step(oc)
            t += dt
            omega_dot_k = (sim.omega[k] - omega_prev[k]) / dt
            num += omega_dot_k * tau[k]
            den += tau[k] ** 2
        g1[k] = num / den
    return g1


def identify_g2(params, T=1.0, amp=60.0, freq=3.0, dt=5e-4):
    """Doublet-regression recovery of G2 (rotor-inertia yaw-reaction gain Ir).

    Drives the oracle QuadSim near hover with the yaw allocation pattern
    (differential CCW-vs-CW rotor speed), so the yaw axis carries two
    physically distinct torque contributions: a quasi-steady aerodynamic
    drag term proportional to d.Omega^2 (the km term) and a transient
    rotor-inertia reaction proportional to d.Omega_dot (the Ir term, see
    QuadSim.step). We reconstruct the TOTAL yaw torque actually applied at
    each step from J_zz * (measured, finite-differenced omega_dot_z) -- a
    known mass property, NOT the unknown Ir -- then regress that total
    torque on the two regressors [d.Omega^2, d.Omega_dot] via ordinary least
    squares. The second coefficient isolates the transient reaction from the
    steady drag term and recovers Ir without ever assuming its value.
    """
    P = params
    _, d = P.layout()
    Jzz = P.J[2, 2]
    Om_h = P.hover_speed()
    pat = _AXIS_PATTERNS[2]
    sim = QuadSim(P, dt=dt, drag_on=False)
    sim.Omega = Om_h * np.ones(4)
    rows, y = [], []
    t = 0.0
    while t < T:
        oc = Om_h * np.ones(4) + amp * pat * np.sin(2 * np.pi * freq * t)
        Omega_prev = sim.Omega.copy()
        omega_prev = sim.omega.copy()
        dOmega = (oc - Omega_prev) / P.tau_m
        r1 = float(d @ (Omega_prev ** 2))          # quasi-steady (km) regressor
        r2 = float(d @ dOmega)                      # transient (Ir) regressor
        sim.step(oc)
        t += dt
        omega_dot_z = (sim.omega[2] - omega_prev[2]) / dt
        tau_z_total = Jzz * omega_dot_z             # reconstructed from J, not Ir
        rows.append([r1, r2])
        y.append(tau_z_total)
    A = np.array(rows)
    coef, *_ = np.linalg.lstsq(A, np.array(y), rcond=None)
    return -float(coef[1])          # tau_z = km*r1 - Ir*r2 -> coef[1] = -Ir
