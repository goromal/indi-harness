"""Drag-rejection A/B: score one {controller x drag} cell, then compare cells.

Every cell flies the same ramped reference (offboard.traj_server.ramped_reference)
from the same takeoff and settled handover. Per case, a cell's error track is

    e(t) = EKF position - reference as received by the vehicle

on the trajectory clock t, sampled on a fixed grid. The received reference is
INDB's DDS reference for custom (INDI) cells and GUIP's GUIDED target for stock
cells; each run's trajectory time is fitted by matching that reference to the
known ramped reference.

Drag's effect on a controller is the distance between its drag-on and drag-off
error tracks, d = RMS_t |e_on(t) - e_off(t)|. Unlike a difference of RMS
values, this cannot cancel when drag moves an error through zero. With several
runs per cell, per case:

- measurable: min stock on/off distance > max stock repeat distance (two runs
  of the same cell), i.e. drag changes stock tracking beyond run-to-run noise;
- passed: measurable and max custom on/off distance < alpha * min stock
  on/off distance.

Per-case tracking RMS (score) is reported alongside as a diagnostic.

Usage:
    python3 -m indi_harness.sitl.drag_ab score --controller stock|custom \\
        flight.BIN flown.json --out cell_score.json
    python3 -m indi_harness.sitl.drag_ab gate --alpha 0.5 \\
        --stock-off a.json b.json --stock-on ... --custom-off ... --custom-on ...
"""
import argparse
import itertools
import json
import pathlib
import sys

import numpy as np
from pymavlink import DFReader

from ..offboard.traj_server import ramped_reference, reference_duration
from .align import boot_to_traj_map
from .baseline import BATTERY
from .binlog import read_ekf_pos, read_outer_health
from .binscore import active_runs

CASES = {case.name: case for case in BATTERY}
GRID_DT = 0.1  # s, error-track sample period (XKF1 logs at ~10 Hz)


def _error_stats(err):
    err = np.asarray(err, float)
    if not err.size:
        return {"rms_m": float("nan"), "max_m": float("nan"), "n": 0}
    return {"rms_m": float(np.sqrt(np.mean(err ** 2))),
            "max_m": float(err.max()), "n": int(err.size)}


def score_stock_case(time_us, p_ned, entry, case):
    """Reference-vs-XKF1 error for one stock case. Returns _error_stats."""
    k, b = boot_to_traj_map(entry["traj_t"], entry["boot_ms"])
    reference = ramped_reference(case)
    duration = entry["traj_t"][-1]
    t_traj = k * (np.asarray(time_us, float) / 1000.0) + b
    window = (t_traj >= 0.0) & (t_traj <= duration)
    offset = np.asarray(entry["origin"], float) - reference.ref(0.0).p
    p_ref = np.array([reference.ref(t).p for t in t_traj[window]]) + offset
    return _error_stats(np.linalg.norm(np.asarray(p_ned)[window] - p_ref,
                                       axis=1))


def score_custom_cases(health, flown):
    """Reference-vs-measured error per engaged INDB block, in flight order."""
    fallback = np.asarray(health["fallback"], int)
    runs = active_runs(fallback)
    if len(runs) != len(flown):
        raise ValueError(f"{len(runs)} engaged INDB blocks for "
                         f"{len(flown)} flown cases")
    ref, meas = health["ref_p"], health["meas_p"]
    return [_error_stats(np.linalg.norm(ref[i:j] - meas[i:j], axis=1))
            for i, j in runs]


def fit_reference_start(time_us, ref_p, case, origin):
    """Boot time [us] at which the received reference was at trajectory time
    0: the shift that best overlays it on the ramped reference."""
    reference = ramped_reference(case)
    offset = np.asarray(origin, float) - reference.ref(0.0).p
    duration = reference_duration(case)
    time_us = np.asarray(time_us, float)
    ref_p = np.asarray(ref_p, float)
    step = max(1, len(time_us) // 400)
    ts, rs = time_us[::step], ref_p[::step]

    def cost(t0):
        t = np.clip((ts - t0) / 1e6, 0.0, duration)
        expected = np.array([reference.ref(x).p for x in t]) + offset
        return float(np.mean(np.sum((rs - expected) ** 2, axis=1)))

    best = time_us[0]
    for span, res in ((2e6, 20e3), (40e3, 1e3)):
        candidates = best + np.arange(-span, span + res, res)
        best = candidates[int(np.argmin([cost(c) for c in candidates]))]
    return float(best)


def error_track(ref_t_us, ref_p, meas_t_us, meas_p, case, origin):
    """Error track e(t) [n,3] on the GRID_DT trajectory grid (see module)."""
    t0 = fit_reference_start(ref_t_us, ref_p, case, origin)
    grid = np.arange(0.0, reference_duration(case) + 1e-9, GRID_DT)
    boot = t0 + grid * 1e6
    lo = max(ref_t_us[0], meas_t_us[0])
    hi = min(ref_t_us[-1], meas_t_us[-1])
    keep = (boot >= lo) & (boot <= hi)
    e = np.full((grid.size, 3), np.nan)
    for axis in range(3):
        e[keep, axis] = (np.interp(boot[keep], meas_t_us, meas_p[:, axis])
                         - np.interp(boot[keep], ref_t_us, ref_p[:, axis]))
    return {"dt": GRID_DT, "t0_us": t0, "e": e.tolist()}


def _read(path, typ, fields):
    log = DFReader.DFReader_binary(str(path))
    rows = []
    while (m := log.recv_match(type=typ)) is not None:
        rows.append([m.TimeUS] + [getattr(m, f) for f in fields])
    return np.array(rows, float)


def received_guided_blocks(bin_path, n_cases, gap_s=1.0):
    """GUIP (received GUIDED target) rows split into the last n_cases
    streaming bursts: [(time_us, p_ned)], in flight order."""
    guip = _read(bin_path, "GUIP", ["pX", "pY", "pZ"])
    breaks = np.where(np.diff(guip[:, 0]) > gap_s * 1e6)[0] + 1
    blocks = np.split(guip, breaks)[-n_cases:]
    if len(blocks) != n_cases:
        raise ValueError(f"{len(blocks)} GUIP bursts for {n_cases} cases")
    return [(b[:, 0], b[:, 1:4]) for b in blocks]


def score_cell(bin_path, flown, controller):
    names = [entry["case"] for entry in flown]
    tracks = {}
    if controller == "stock":
        time_us, p_ned = read_ekf_pos(bin_path)
        stats = [score_stock_case(time_us, p_ned, entry, CASES[entry["case"]])
                 for entry in flown]
        for entry, (rt, rp) in zip(flown, received_guided_blocks(
                bin_path, len(flown))):
            tracks[entry["case"]] = error_track(
                rt, rp, time_us, p_ned, CASES[entry["case"]], entry["origin"])
    elif controller == "custom":
        health = read_outer_health(bin_path)
        stats = score_custom_cases(health, flown)
        runs = active_runs(np.asarray(health["fallback"], int))
        t = health["time_us"]
        for entry, (i, j) in zip(flown, runs):
            tracks[entry["case"]] = error_track(
                t[i:j], health["ref_p"][i:j], t[i:j], health["meas_p"][i:j],
                CASES[entry["case"]], entry["origin"])
    else:
        raise ValueError(f"unknown controller {controller!r}")
    return {"controller": controller,
            "cases": {name: dict(s, error_track=tracks[name])
                      for name, s in zip(names, stats)}}


def track_distance(a, b):
    """RMS over the common grid of |e_a(t) - e_b(t)|."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = min(len(a), len(b))
    d = np.linalg.norm(a[:n] - b[:n], axis=1)
    d = d[np.isfinite(d)]
    return float(np.sqrt(np.mean(d ** 2))) if d.size else float("nan")


def _tracks(cells, case):
    return [cell["cases"][case]["error_track"]["e"] for cell in cells]


def _cross(on, off):
    return [track_distance(a, b) for a in on for b in off]


def _repeat(*cells):
    return [track_distance(a, b) for runs in cells
            for a, b in itertools.combinations(runs, 2)]


def drag_effect_gate(stock_off, stock_on, custom_off, custom_on, alpha):
    """Per-case drag-effect gate over lists of score_cell results (module
    docstring). Cases are those flown in every run."""
    groups = (stock_off, stock_on, custom_off, custom_on)
    names = [n for n in stock_off[0]["cases"]
             if all(n in cell["cases"] for g in groups for cell in g)]
    rows = []
    for name in names:
        so, sn, co, cn = (_tracks(g, name) for g in groups)
        stock_cross, custom_cross = _cross(sn, so), _cross(cn, co)
        stock_repeat, custom_repeat = _repeat(so, sn), _repeat(co, cn)
        rms = [float(np.mean([c["cases"][name]["rms_m"] for c in g]))
               for g in groups]
        measurable = bool(stock_repeat) and min(stock_cross) > max(stock_repeat)
        rows.append({
            "case": name,
            "stock_effect_m": [min(stock_cross), max(stock_cross)],
            "stock_repeat_max_m": max(stock_repeat) if stock_repeat else None,
            "custom_effect_m": [min(custom_cross), max(custom_cross)],
            "custom_repeat_max_m": max(custom_repeat) if custom_repeat else None,
            "mean_rms_m": dict(zip(("stock_off", "stock_on", "custom_off",
                                    "custom_on"), rms)),
            "measurable": measurable,
            "passed": bool(measurable
                           and max(custom_cross) < alpha * min(stock_cross)),
        })
    return {"alpha": alpha, "runs_per_cell": [len(g) for g in groups],
            "cases": rows,
            "passed": bool(rows) and all(r["passed"] for r in rows)}


def _main_score(args):
    flown = json.loads(pathlib.Path(args.flown).read_text())
    result = score_cell(args.bin, flown, args.controller)
    for name, s in result["cases"].items():
        print(f"  {args.controller:<6} {name:<16} rms={s['rms_m']:.3f} m "
              f"max={s['max_m']:.3f} m n={s['n']}", flush=True)
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(result))
    return 0


def _load(paths):
    return [json.loads(pathlib.Path(p).read_text()) for p in paths]


def _main_gate(args):
    result = drag_effect_gate(_load(args.stock_off), _load(args.stock_on),
                              _load(args.custom_off), _load(args.custom_on),
                              alpha=args.alpha)
    print(f"runs per cell (stock off/on, custom off/on): "
          f"{result['runs_per_cell']}")
    print(f"{'case':<16} {'PID effect':>15} {'PID repeat':>10} "
          f"{'INDI effect':>15} {'INDI repeat':>11}  measurable passed")
    for r in result["cases"]:
        rep = lambda x: f"{x:10.3f}" if x is not None else f"{'-':>10}"
        print(f"{r['case']:<16} {r['stock_effect_m'][0]:7.3f}-"
              f"{r['stock_effect_m'][1]:<7.3f} {rep(r['stock_repeat_max_m'])} "
              f"{r['custom_effect_m'][0]:7.3f}-{r['custom_effect_m'][1]:<7.3f} "
              f"{rep(r['custom_repeat_max_m'])} "
              f" {str(r['measurable']):<10} {r['passed']}")
        m = r["mean_rms_m"]
        print(f"{'':<16} mean RMS m: PID {m['stock_off']:.3f}->{m['stock_on']:.3f}"
              f"  INDI {m['custom_off']:.3f}->{m['custom_on']:.3f}")
    print(f"alpha={args.alpha} passed={result['passed']}")
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(result, indent=1))
    return 0 if result["passed"] else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)
    score = sub.add_parser("score")
    score.add_argument("--controller", choices=("stock", "custom"),
                       required=True)
    score.add_argument("bin")
    score.add_argument("flown")
    score.add_argument("--out")
    gate = sub.add_parser("gate")
    gate.add_argument("--alpha", type=float, default=0.5)
    for flag in ("--stock-off", "--stock-on", "--custom-off", "--custom-on"):
        gate.add_argument(flag, nargs="+", required=True)
    gate.add_argument("--out")
    args = ap.parse_args(argv)
    return _main_score(args) if args.command == "score" else _main_gate(args)


if __name__ == "__main__":
    sys.exit(main())
