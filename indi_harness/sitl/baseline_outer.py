"""S3 Layer-B battery: fly the trajectory battery with the in-firmware INDI
outer loop (CC_TYPE=3, CC3_OUTER_EN=1) driven by the DDS FlatSetpoint reference
from the ROS2 trajectory-server (design doc S3 Layer-B exit).

Unlike Layer A/C (baseline_cc streams GUIDED position targets and the stock
outer loop tracks them), here the *outer loop itself* is in firmware: this
runner only takes off, engages the custom controller (RC aux 109), and holds
station in GUIDED while `traj_server` streams the flat reference over DDS -- the
INDI outer loop flies the figure-8, ignoring the (held) GUIDED target. The .BIN
INDB message logs the reference vs measured position directly, so flat-tracking
is scored from the log (read_outer_health / flat_tracking_score), not from a
streamed reference here.

Coordination with traj_server (separate rclpy process): this runner takes off +
engages, then writes {case, origin} to --ready-file and holds for the requested
duration; the caller starts traj_server once the ready file exists so the
trajectory origin is latched at the hover point.

Usage (inside the drone VM):
    python3 -m indi_harness.sitl.baseline_outer \
        --url tcp:127.0.0.1:5790 --out /tmp/s3_layerB --engage-rc 9 \
        --ready-file /tmp/lb_ready --cases circle_slow
"""
import argparse
import json
import pathlib
import time
import numpy as np
from pymavlink import mavutil
from ..trajectory import Hover
from .baseline import BATTERY, ALT_M
from .baseline_cc import engage_custom_controller, _wait_gps_fix
from .streamer import GuidedStreamer, FlightRecord


def _hold(s, duration, origin):
    """Hold the GUIDED position target at `origin` for `duration` s (keeps
    GUIDED happy; the CC3_OUTER_EN backend ignores it and flies the DDS
    reference). Records LOCAL_POSITION_NED as a coarse sanity track -- the .BIN
    INDB is the authoritative reference-vs-measured record."""
    rec = FlightRecord()
    hover = Hover(point=np.asarray(origin, float))
    s.fly(hover, duration, origin, record=rec)
    return rec


def run_battery_outer(url, out_dir, engage_rc=9, ready_file=None,
                      cases=None, settle_s=4.0):
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    s = GuidedStreamer.connect(url)
    c = s.conn
    c.mav.request_data_stream_send(c.target_system, c.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    print("waiting for GPS fix / EKF settle", flush=True)
    _wait_gps_fix(c)
    time.sleep(10.0)
    print("connected; entering GUIDED", flush=True)
    s.set_mode_guided()
    print("arming (retries until prearm passes)...", flush=True)
    s.arm()
    print(f"taking off to {ALT_M} m", flush=True)
    s.takeoff(ALT_M)
    time.sleep(5.0)
    print(f"engaging custom controller via RC{engage_rc} (aux 109)", flush=True)
    engaged = engage_custom_controller(c, engage_rc)
    if not engaged:
        # Not fatal to the run (STATUSTEXT can be missed under load); the .BIN du
        # is the authoritative engagement check. Surface it loudly regardless.
        print("WARNING: 'Custom controller is ON' STATUSTEXT not confirmed; "
              "relying on .BIN du for engagement proof", flush=True)
    time.sleep(settle_s)

    flown = []
    for case in cases or BATTERY:
        origin = s.local_position()
        if ready_file is not None:
            pathlib.Path(ready_file).write_text(
                json.dumps({"case": case.name,
                            "origin": [float(x) for x in origin]}))
        print(f"holding for {case.name} ({case.duration:.0f} s) -- "
              f"outer loop flies via DDS", flush=True)
        rec = _hold(s, case.duration, origin)
        flown.append({"case": case.name, "origin": [float(x) for x in origin],
                      "n_samples": len(rec.p)})
        if ready_file is not None:
            pathlib.Path(ready_file).unlink(missing_ok=True)
        time.sleep(3.0)

    ch = [65535] * 9
    c.mav.rc_channels_override_send(c.target_system, c.target_component, *ch)
    (out / "s3_layerB_flown.json").write_text(json.dumps(flown, indent=1))
    print(f"wrote {out / 's3_layerB_flown.json'}", flush=True)
    return flown


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="tcp:127.0.0.1:5790")
    ap.add_argument("--out", default="/tmp/s3_layerB")
    ap.add_argument("--engage-rc", type=int, default=9)
    ap.add_argument("--ready-file", default=None,
                    help="written with {case,origin} once hovering+engaged")
    ap.add_argument("--cases", default=None,
                    help="comma-separated case names (default: full battery)")
    args = ap.parse_args()
    cases = None
    if args.cases:
        names = set(args.cases.split(","))
        cases = [c for c in BATTERY if c.name in names]
    run_battery_outer(args.url, args.out, engage_rc=args.engage_rc,
                      ready_file=args.ready_file, cases=cases)


if __name__ == "__main__":
    main()
