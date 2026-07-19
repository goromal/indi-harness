"""S1 baseline battery: fly canned trajectories in GUIDED against SITL and
score EKF tracking RMSE from the .BIN dataflash (design doc S1 exit).

Usage (from inside the drone VM, or anywhere that reaches the router):
    python3 -m indi_harness.sitl.baseline \
        --url tcp:127.0.0.1:5790 --logs-dir /data/drone/ardusitl/logs \
        --out /tmp/s1_baseline
"""
import argparse
import json
import pathlib
import time
from dataclasses import dataclass
import numpy as np
from ..trajectory import Circle, Lemniscate, Hover, FlatOutput
from .align import boot_to_traj_map, evaluate_bin
from .binlog import read_ekf_pos
from .streamer import GuidedStreamer

ALT_M = 10.0


class HoverStep(Hover):
    """Hover with a position step at step_t — the hover-perturbation case."""

    def __init__(self, point, step_t, step, psi=0.0):
        super().__init__(point, psi)
        self.step_t, self.step = step_t, np.asarray(step, float)

    def ref(self, t):
        fo = super().ref(t)
        p = fo.p + (self.step if t >= self.step_t else 0.0)
        return FlatOutput(p, fo.v, fo.a, fo.j, fo.s, fo.psi, fo.dpsi, fo.ddpsi)


@dataclass
class Case:
    name: str
    traj: object
    duration: float


BATTERY = [
    Case("hover_step",
         HoverStep(point=np.array([0.0, 0.0, -ALT_M]), step_t=5.0,
                   step=np.array([2.0, 0.0, 0.0])), 20.0),
    Case("circle_slow", Circle(radius=2.0, period=12.0, alt=ALT_M), 24.0),
    Case("circle_fast", Circle(radius=2.0, period=8.0, alt=ALT_M), 16.0),
    Case("lemniscate_slow", Lemniscate(amplitude=2.0, period=12.0, alt=ALT_M), 24.0),
    Case("lemniscate_fast", Lemniscate(amplitude=2.0, period=8.0, alt=ALT_M), 16.0),
]


def results_json(results):
    return json.dumps(results, indent=1)


def newest_bin(logs_dir):
    bins = sorted(pathlib.Path(logs_dir).glob("*.BIN"),
                  key=lambda p: p.stat().st_mtime)
    if not bins:
        raise FileNotFoundError(f"no .BIN logs under {logs_dir}")
    return bins[-1]


def run_battery(url, logs_dir, out_dir, cases=None):
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    s = GuidedStreamer.connect(url)
    print("connected; entering GUIDED", flush=True)
    s.set_mode_guided()
    print("arming (retries until prearm passes)...", flush=True)
    s.arm()
    print(f"taking off to {ALT_M} m", flush=True)
    s.takeoff(ALT_M)
    time.sleep(5.0)  # settle

    flown = []
    for case in cases or BATTERY:
        origin = s.local_position()
        print(f"flying {case.name} for {case.duration:.0f} s", flush=True)
        rec = s.fly(case.traj, case.duration, origin)
        flown.append((case, origin, rec))
        time.sleep(3.0)  # let it settle between cases

    bin_path = newest_bin(logs_dir)
    print(f"evaluating against {bin_path}", flush=True)
    time_us, p_ned = read_ekf_pos(bin_path)
    results = []
    for case, origin, rec in flown:
        traj_t, boot_ms, p_live, p_ref = rec.arrays()
        k, b = boot_to_traj_map(traj_t, boot_ms)
        origin_offset = origin - case.traj.ref(0.0).p
        # only rows inside this case's flight window
        t_traj_all = k * (time_us / 1000.0) + b
        win = (t_traj_all >= 0.0) & (t_traj_all <= case.duration)
        per_axis, total = evaluate_bin(time_us[win], p_ned[win], k, b,
                                       case.traj, origin_offset, trim_s=2.0)
        results.append({"case": case.name, "rmse_total": float(total),
                        "rmse_axes": [float(x) for x in per_axis],
                        "n_bin_samples": int(win.sum()), "source": "XKF1"})
        print(f"  {case.name:<16} RMSE {total:.3f} m "
              f"({int(win.sum())} BIN samples)", flush=True)

    (out / "s1_baseline.json").write_text(results_json(results))
    print(f"wrote {out / 's1_baseline.json'}", flush=True)
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="tcp:127.0.0.1:5790")
    ap.add_argument("--logs-dir", default="/data/drone/ardusitl/logs")
    ap.add_argument("--out", default="/tmp/s1_baseline")
    args = ap.parse_args()
    run_battery(args.url, args.logs_dir, args.out)


if __name__ == "__main__":
    main()
