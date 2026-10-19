"""flatness outer-loop controller battery: fly the trajectory battery with the in-firmware INDI
outer loop (CC_TYPE=3, CC3_OUTER_EN=1) driven by the DDS FlatSetpoint reference
from the ROS2 trajectory-server .

Unlike legacy INDI rate controller/C (baseline_cc streams GUIDED position targets and the stock
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
        --url tcp:127.0.0.1:5790 --out /tmp/indi_flatness --engage-rc 9 \
        --ready-file /tmp/trajectory_ready --cases circle_slow
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


def _hold(s, duration, origin, on_position=None):
    """Hold the GUIDED position target at `origin` for `duration` s (keeps
    GUIDED happy; the CC3_OUTER_EN backend ignores it and flies the DDS
    reference). Records LOCAL_POSITION_NED as a coarse sanity track -- the .BIN
    INDB is the authoritative reference-vs-measured record."""
    rec = FlightRecord()
    hover = Hover(point=np.asarray(origin, float))
    s.fly(hover, duration, origin, record=rec, on_position=on_position)
    return rec


def _wait_for_settled_handover(conn, duration=5.0, history=2.0,
                               rate_limit=1.0):
    """Wait under stock control and reject an unsettled handover live."""
    deadline = time.monotonic() + duration
    rates = []
    while time.monotonic() < deadline:
        message = conn.recv_match(type="ATTITUDE", blocking=True, timeout=.25)
        if message is not None:
            rates.append((message.time_boot_ms / 1000.0,
                          float(np.hypot(message.rollspeed,
                                         message.pitchspeed))))
    if not rates:
        raise RuntimeError("no ATTITUDE samples for handover check")
    end = rates[-1][0]
    recent = [rate for timestamp, rate in rates if timestamp >= end - history]
    peak = max(recent)
    if peak >= rate_limit:
        raise RuntimeError(f"unsettled stock handover: peak roll/pitch rate "
                           f"{peak:.3f} rad/s >= {rate_limit:.3f}")
    return {"end_boot_s": end, "history_s": history,
            "peak_roll_pitch_rate_rad_s": peak, "limit_rad_s": rate_limit,
            "n_samples": len(recent)}


def run_battery_outer(url, out_dir, engage_rc=9, ready_file=None,
                      cases=None, settle_s=4.0):
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    evidence = {
        "bounds": {"handover_rate_rad_s": 1.0, "handover_history_s": 2.0,
                   "min_altitude_m": 2.0, "max_altitude_m": 14.0,
                   "max_horizontal_displacement_m": 5.0,
                   "runner_timeout_s": 600},
        "success": False,
    }
    try:
        s = GuidedStreamer.connect(url)
        c = s.conn
        c.mav.request_data_stream_send(c.target_system, c.target_component,
                                       mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
        print("waiting for GPS fix / EKF settle", flush=True)
        _wait_gps_fix(c)
        time.sleep(10.0)
        print("connected; entering GUIDED", flush=True)
        s.set_mode_guided()
        print("arming (retries until prearm passes)...", flush=True)
        s.arm()
        print(f"taking off to {ALT_M} m", flush=True)
        s.takeoff(ALT_M)
        evidence["handover"] = _wait_for_settled_handover(c)
        print(f"settled handover: {evidence['handover']}", flush=True)
        print(f"engaging custom controller via RC{engage_rc} (aux 109)", flush=True)
        engaged = engage_custom_controller(c, engage_rc)
        if not engaged:
            # The .BIN du is authoritative, but do not publish DDS readiness
            # when live engagement itself could not be confirmed.
            raise RuntimeError("'Custom controller is ON' not confirmed")
        evidence["indi_settle"] = _wait_for_settled_handover(
            c, duration=settle_s)
        print(f"settled INDI before DDS: {evidence['indi_settle']}", flush=True)

        flown = []
        for case in cases or BATTERY:
            origin = s.local_position()

            def check_position(message):
                altitude = -message.z
                horizontal = float(np.hypot(message.x - origin[0],
                                            message.y - origin[1]))
                if not (evidence["bounds"]["min_altitude_m"] <= altitude
                        <= evidence["bounds"]["max_altitude_m"]):
                    raise RuntimeError(f"altitude bound exceeded: {altitude:.3f} m")
                if horizontal > evidence["bounds"]["max_horizontal_displacement_m"]:
                    raise RuntimeError(
                        f"horizontal position bound exceeded: {horizontal:.3f} m")

            if ready_file is not None:
                pathlib.Path(ready_file).write_text(
                    json.dumps({"case": case.name,
                                "origin": [float(x) for x in origin]}))
            print(f"holding for {case.name} ({case.duration:.0f} s) -- "
                  f"outer loop flies via DDS", flush=True)
            try:
                rec = _hold(s, case.duration, origin,
                            on_position=check_position)
            finally:
                if ready_file is not None:
                    pathlib.Path(ready_file).unlink(missing_ok=True)
            flown.append({"case": case.name,
                          "origin": [float(x) for x in origin],
                          "n_samples": len(rec.p)})
            time.sleep(3.0)

        ch = [65535] * 9
        c.mav.rc_channels_override_send(c.target_system, c.target_component, *ch)
        (out / "indi_flatness_flown.json").write_text(json.dumps(flown, indent=1))
        evidence["success"] = True
        print(f"wrote {out / 'indi_flatness_flown.json'}", flush=True)
        return flown
    except BaseException as error:
        evidence["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        (out / "handover.json").write_text(json.dumps(evidence, indent=2))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="tcp:127.0.0.1:5790")
    ap.add_argument("--out", default="/tmp/indi_flatness")
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
