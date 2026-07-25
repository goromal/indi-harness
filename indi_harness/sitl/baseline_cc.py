"""S3 Layer-A battery: fly the S1 trajectory battery in GUIDED with the
in-firmware custom controller (CC_TYPE=INDI) engaged, scoring EKF tracking
RMSE from the .BIN (design doc S3 Layer-A exit).

Identical command path to S1 (stock GUIDED position streaming) -- Layer A keeps
the stock outer loop and only swaps the inner attitude/rate controller. The
custom controller is engaged mid-flight via the RC aux function (CUSTOM_
CONTROLLER=109) on --engage-rc, which the SITL params wire to a channel.

Usage (inside the drone VM or anywhere reaching the router):
    python3 -m indi_harness.sitl.baseline_cc \
        --url tcp:127.0.0.1:5790 --logs-dir /data/drone/ardusitl/logs \
        --out /tmp/s3_layerA --engage-rc 9
"""
import argparse
import json
import pathlib
import time
from pymavlink import mavutil
from .baseline import BATTERY, ALT_M, evaluate_flown, results_json
from .streamer import GuidedStreamer


def _rc_override(conn, chan, value):
    """Override one RC channel (1-8 via the base fields, 9 via chan9_raw);
    others 65535 = 'ignore'. Used to flip the custom-controller aux switch."""
    ch = [65535] * 9
    ch[chan - 1] = value
    conn.mav.rc_channels_override_send(conn.target_system,
                                       conn.target_component, *ch)


def engage_custom_controller(conn, chan, value=2000, repeats=8, dt=0.2):
    for _ in range(repeats):
        _rc_override(conn, chan, value)
        time.sleep(dt)


def _wait_gps_fix(conn, timeout=90.0):
    """Wait for a sim GPS 3D fix so the EKF has an origin before arming --
    otherwise arm() may pass prearm before the estimate is ready and the
    GUIDED takeoff will not climb. No-op cost when a fix is already present."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        g = conn.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if g is not None and g.fix_type >= 3:
            return True
    return False


def run_battery_cc(url, logs_dir, out_dir, engage_rc=9, cases=None):
    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    s = GuidedStreamer.connect(url)
    c = s.conn
    c.mav.request_data_stream_send(c.target_system, c.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1)
    print("waiting for GPS fix / EKF settle", flush=True)
    _wait_gps_fix(c)
    time.sleep(10.0)  # let the EKF converge on the origin
    print("connected; entering GUIDED", flush=True)
    s.set_mode_guided()
    print("arming (retries until prearm passes)...", flush=True)
    s.arm()
    print(f"taking off to {ALT_M} m", flush=True)
    s.takeoff(ALT_M)
    time.sleep(5.0)  # settle under the stock controller first
    print(f"engaging custom controller via RC{engage_rc} (aux 109)", flush=True)
    engage_custom_controller(c, engage_rc)
    time.sleep(3.0)  # settle under INDI before the battery

    flown = []
    for case in cases or BATTERY:
        origin = s.local_position()
        print(f"flying {case.name} for {case.duration:.0f} s", flush=True)
        rec = s.fly(case.traj, case.duration, origin)
        flown.append((case, origin, rec))
        time.sleep(3.0)

    # release the override (leave the vehicle in a clean state)
    _rc_override(c, engage_rc, 65535)
    results = evaluate_flown(flown, logs_dir)
    (out / "s3_layerA.json").write_text(results_json(results))
    print(f"wrote {out / 's3_layerA.json'}", flush=True)
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--url", default="tcp:127.0.0.1:5790")
    ap.add_argument("--logs-dir", default="/data/drone/ardusitl/logs")
    ap.add_argument("--out", default="/tmp/s3_layerA")
    ap.add_argument("--engage-rc", type=int, default=9,
                    help="RC channel wired to CUSTOM_CONTROLLER aux (option 109)")
    args = ap.parse_args()
    run_battery_cc(args.url, args.logs_dir, args.out, engage_rc=args.engage_rc)


if __name__ == "__main__":
    main()
