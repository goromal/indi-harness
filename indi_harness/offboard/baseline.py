"""S2 offboard battery: same 5 cases, flown by the offboard outer loop
through the stock inner loop; scored identically to S1 (same evaluator,
same .BIN source of truth) so the comparison is apples-to-apples.

Usage (in the drone VM):
    python3 -m indi_harness.offboard.baseline \
        --url tcp:127.0.0.1:5790 --logs-dir /data/drone/ardusitl/logs \
        --out /tmp/s2_offboard [--no-indi] [--imu-topic /ap/imu/experimental/data]
"""
import argparse
import json
import pathlib
import time
import numpy as np
from ..params import QuadParams
from ..sitl.baseline import ALT_M, BATTERY, evaluate_flown, results_json
from ..sitl.setpoints import Setpoint
from ..sitl.streamer import GuidedStreamer


def fetch_hover_throttle(conn, default=0.35, timeout=10.0):
    conn.mav.param_request_read_send(conn.target_system,
                                     conn.target_component,
                                     b"MOT_THST_HOVER", -1)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if m is not None and str(m.param_id).strip("\x00") == "MOT_THST_HOVER":
            return float(m.param_value)
    print(f"MOT_THST_HOVER fetch timed out; using {default}", flush=True)
    return default


def hold_position(s, p_ned, seconds):
    """Re-stabilize between cases with plain guided position setpoints."""
    for _ in range(int(seconds * 10)):
        sp = Setpoint(p=p_ned, v=np.zeros(3), a=np.zeros(3))
        f = sp.fields()
        s.conn.mav.set_position_target_local_ned_send(
            f["time_boot_ms"], s.conn.target_system, s.conn.target_component,
            f["coordinate_frame"], f["type_mask"], f["x"], f["y"], f["z"],
            f["vx"], f["vy"], f["vz"], f["afx"], f["afy"], f["afz"],
            f["yaw"], f["yaw_rate"])
        time.sleep(0.1)


def run_battery(url, logs_dir, out_dir, indi_accel, imu_topic):
    from .node import OffboardMission, run_node
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    s = GuidedStreamer.connect(url)
    print("connected; GUIDED + arm + takeoff", flush=True)
    s.set_mode_guided()
    s.arm()
    s.takeoff(ALT_M)
    time.sleep(5.0)
    hover_thr = fetch_hover_throttle(s.conn)
    print(f"hover throttle {hover_thr:.3f}", flush=True)

    P = QuadParams()
    mission = OffboardMission(s.conn, P, hover_throttle=hover_thr,
                              indi_accel=indi_accel)
    flown = []

    def spin_until_done(rclpy, node):
        for case in BATTERY:
            origin = s.local_position()
            print(f"offboard {case.name} for {case.duration:.0f} s", flush=True)
            mission.start_case(case, origin)
            while not mission.done:
                rclpy.spin_once(node, timeout_sec=0.5)
            flown.append((case, origin, mission.record))
            hold_position(s, origin, 4.0)

    run_node(mission, imu_topic, spin_until_done)
    results = evaluate_flown(flown, logs_dir)
    (out / "s2_offboard.json").write_text(results_json(results))
    print(f"wrote {out / 's2_offboard.json'}", flush=True)
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="tcp:127.0.0.1:5790")
    ap.add_argument("--logs-dir", default="/data/drone/ardusitl/logs")
    ap.add_argument("--out", default="/tmp/s2_offboard")
    ap.add_argument("--no-indi", action="store_true",
                    help="PD+ff fallback (no IMU topic available)")
    ap.add_argument("--imu-topic", default="/ap/imu/experimental/data")
    args = ap.parse_args()
    run_battery(args.url, args.logs_dir, args.out,
                indi_accel=not args.no_indi, imu_topic=args.imu_topic)


if __name__ == "__main__":
    main()
