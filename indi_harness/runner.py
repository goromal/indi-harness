"""Closed-loop scenario runner: sim + outer INDI + inner INDI + flat feedforward."""
import numpy as np
from . import quat
from .flatness import flat_reference
from .inner_loop import InnerLoopINDI, InnerGains
from .outer_loop import OuterLoopINDI, OuterGains, attitude_from_thrust_dir
from .simmodel import QuadSim


def run_scenario(params, traj, t_end, wind=None, drag_on=True,
                 fs=500.0, ff_on=True, p0=None, f_ext=None,
                 inner_gains=None, outer_gains=None, ctrl_params=None):
    """ctrl_params: controller-side model (defaults to truth; pass
    params.perturbed(...) for robustness runs)."""
    wind = np.zeros(3) if wind is None else np.asarray(wind, float)
    cp = params if ctrl_params is None else ctrl_params
    ref0 = flat_reference(traj, 0.0, params.m, params.g)
    fo0 = traj.ref(0.0)
    sim = QuadSim(params, drag_on=drag_on, wind=wind,
                  p0=fo0.p if p0 is None else p0, v0=fo0.v, q0=ref0.q)
    sim.Omega = np.full(4, params.hover_speed())
    if f_ext is not None:
        sim.f_ext = f_ext
    inner = InnerLoopINDI(cp, inner_gains or InnerGains(), fs)
    outer = OuterLoopINDI(cp, outer_gains or OuterGains(), fs)
    n_sub = int(round(1 / fs / sim.dt))
    log = {k: [] for k in ("t", "p", "p_ref", "q", "sat", "domega_pred",
                           "domega_meas_f")}
    for k in range(int(t_end * fs)):
        t = k / fs
        fo = traj.ref(t)
        ref = flat_reference(traj, t, params.m, params.g)
        z_b_des, T_cmd = outer.update(sim.p, sim.v, sim.q, sim.specific_force(),
                                      sim.Omega, fo)
        q_ref = attitude_from_thrust_dir(z_b_des, fo.psi)
        w_ff = ref.w if ff_on else np.zeros(3)
        dw_ff = ref.dw if ff_on else np.zeros(3)
        Om_cmd, diag = inner.update(1 / fs, sim.omega, sim.q, q_ref,
                                    w_ff, dw_ff, T_cmd, sim.Omega)
        for _ in range(n_sub):
            sim.step(Om_cmd)
        log["t"].append(t); log["p"].append(sim.p.copy())
        log["p_ref"].append(fo.p.copy()); log["q"].append(sim.q.copy())
        log["sat"].append(diag["sat"])
        log["domega_pred"].append(diag["domega_pred"].copy())
        log["domega_meas_f"].append(diag["domega_meas_f"].copy())
    return {k: np.array(v) for k, v in log.items()}
