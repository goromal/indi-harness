"""Drag-rejection A/B: score one {controller x drag} cell, then compare cells.

Every cell flies the same ramped reference (offboard.traj_server.ramped_reference)
from the same takeoff and settled handover. Tracking error is the per-sample
distance between that reference and the EKF position over the whole reference:

- custom (INDI) cells read both from INDB, one block per case (fallback == 0);
- stock cells read XKF1 and evaluate the reference on the trajectory clock via
  the runner's recorded (traj_t, boot_ms) pairs.

The gap compares each controller only against itself, drag on versus off, so a
fixed offset between the two scoring sources cancels:

    RMSE_INDI(drag) - RMSE_INDI(no drag) < alpha * [RMSE_PID(drag) - RMSE_PID(no drag)]

Usage:
    python3 -m indi_harness.sitl.drag_ab score --controller stock|custom \\
        flight.BIN flown.json --out cell_score.json
    python3 -m indi_harness.sitl.drag_ab gap --alpha 0.5 \\
        stock_off.json stock_on.json custom_off.json custom_on.json
"""
import argparse
import json
import pathlib
import sys

import numpy as np

from ..offboard.traj_server import ramped_reference
from .align import boot_to_traj_map
from .baseline import BATTERY
from .binlog import read_ekf_pos, read_outer_health
from .binscore import active_runs

CASES = {case.name: case for case in BATTERY}


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


def score_cell(bin_path, flown, controller):
    if controller == "stock":
        time_us, p_ned = read_ekf_pos(bin_path)
        stats = [score_stock_case(time_us, p_ned, entry, CASES[entry["case"]])
                 for entry in flown]
    elif controller == "custom":
        stats = score_custom_cases(read_outer_health(bin_path), flown)
    else:
        raise ValueError(f"unknown controller {controller!r}")
    return {"controller": controller,
            "cases": {entry["case"]: s for entry, s in zip(flown, stats)}}


def drag_gap(stock_off, stock_on, custom_off, custom_on, alpha):
    """Per-case drag-induced RMSE increase for each controller and the
    relative assertion. Inputs are score_cell results; cases are those flown
    in every cell."""
    cells = (stock_off, stock_on, custom_off, custom_on)
    names = [n for n in stock_off["cases"]
             if all(n in cell["cases"] for cell in cells)]
    rows = []
    for name in names:
        rms = [cell["cases"][name]["rms_m"] for cell in cells]
        stock_delta, custom_delta = rms[1] - rms[0], rms[3] - rms[2]
        rows.append({
            "case": name,
            "stock_off_m": rms[0], "stock_on_m": rms[1],
            "custom_off_m": rms[2], "custom_on_m": rms[3],
            "stock_delta_m": stock_delta, "custom_delta_m": custom_delta,
            # Drag must measurably hurt the stock controller for the
            # comparison to mean anything.
            "stock_degraded": bool(stock_delta > 0.0),
            "passed": bool(stock_delta > 0.0
                           and custom_delta < alpha * stock_delta),
        })
    return {"alpha": alpha, "cases": rows,
            "passed": bool(rows) and all(r["passed"] for r in rows)}


def _main_score(args):
    flown = json.loads(pathlib.Path(args.flown).read_text())
    result = score_cell(args.bin, flown, args.controller)
    for name, s in result["cases"].items():
        print(f"  {args.controller:<6} {name:<16} rms={s['rms_m']:.3f} m "
              f"max={s['max_m']:.3f} m n={s['n']}", flush=True)
    if args.out:
        pathlib.Path(args.out).write_text(json.dumps(result, indent=1))
    return 0


def _main_gap(args):
    cells = [json.loads(pathlib.Path(p).read_text()) for p in args.cells]
    result = drag_gap(*cells, alpha=args.alpha)
    print(f"{'case':<16} {'PID off':>8} {'PID on':>8} {'dPID':>7} "
          f"{'INDI off':>8} {'INDI on':>8} {'dINDI':>7}  pass")
    for r in result["cases"]:
        print(f"{r['case']:<16} {r['stock_off_m']:8.3f} {r['stock_on_m']:8.3f} "
              f"{r['stock_delta_m']:7.3f} {r['custom_off_m']:8.3f} "
              f"{r['custom_on_m']:8.3f} {r['custom_delta_m']:7.3f}  "
              f"{r['passed']}")
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
    gap = sub.add_parser("gap")
    gap.add_argument("--alpha", type=float, default=0.5)
    gap.add_argument("cells", nargs=4,
                     metavar=("STOCK_OFF", "STOCK_ON", "CUSTOM_OFF", "CUSTOM_ON"))
    gap.add_argument("--out")
    args = ap.parse_args(argv)
    return _main_score(args) if args.command == "score" else _main_gap(args)


if __name__ == "__main__":
    sys.exit(main())
