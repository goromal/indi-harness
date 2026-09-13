"""Bounded standalone JSON-SITL hover/attitude-step experiment.

Uses a fresh EEPROM and owned processes per run. Artifacts include the exact
parameters, binary SHA256, simulator truth, time windows, and DataFlash log.
Run from the harness Nix shell; --out must be a new directory in workspace data.
"""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import threading
import time

import numpy as np
from pymavlink import mavutil

from . import mavflight
from .baseline_cc import engage_custom_controller, _rc_override
from .jsonsim.__main__ import Server
from .streamer import GuidedStreamer


def run(args):
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    binary = Path(args.binary).resolve()
    params = {
        "FRAME_CLASS": 1, "FRAME_TYPE": 1,
        "MOT_THST_HOVER": .303, "MOT_HOVER_LEARN": 0,
        "MOT_THST_EXPO": 0, "MOT_SPIN_ARM": 0, "MOT_SPIN_MIN": 0, "MOT_SPIN_MAX": 1,
        "MOT_BAT_VOLT_MIN": 0, "MOT_BAT_VOLT_MAX": 0,
        "CC_TYPE": 3, "CC_AXIS_MASK": 7, "RC9_OPTION": 109,
        "CC3_USE_RPM": 1, "CC3_OUTER_EN": 0, "CC3_B_THR_EN": 0,
        "CC3_G1_RP": 500, "CC3_G1_YAW": 28.8, "CC3_G2_YAW": 0,
        "CC3_OMG_FILT": 80, "INS_GYRO_FILTER": 20,
        "LOG_DISARMED": 1, "LOG_BITMASK": 65535,
        "FS_THR_ENABLE": 0, "FS_GCS_ENABLE": 0,
        "RC_OVERRIDE_TIME": 60,
    }
    for setting in args.param:
        key, value = setting.split("=", 1)
        params[key] = float(value)
    param_file = out / "probe.parm"
    param_file.write_text("".join(f"{k} {v}\n" for k, v in params.items()))
    manifest = {"binary": str(binary), "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
                "parameters": params, "windows": [], "success": False}
    # Refuse to share ports with another run.
    with socket.socket() as check:
        check.bind(("127.0.0.1", 5760))
    server = Server(port=9002, drag_on=False)
    truth = []
    original = server.driver.on_packet
    missing = []

    def record(data):
        reply = original(data)
        if reply is not None:
            state = json.loads(reply)
            state["omega"] = server.model._omega.tolist()
            truth.append(state)
            if missing:
                packet = json.loads(reply)
                for motor in missing:
                    packet.pop(f"rpm_{motor + 1}", None)
                reply = json.dumps(packet) + "\n"
        return reply

    server.driver.on_packet = record
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    proc = None
    conn = None
    try:
        with (out / "sitl.log").open("w") as log:
            proc = subprocess.Popen([str(binary), "--model=JSON:127.0.0.1",
                "--home=34.381441,-118.580861,417,0", "--speedup=1", "-I0",
                "--config=undulation:0.0",
                "--defaults", f"{Path(args.defaults).resolve()},{param_file}"],
                cwd=out, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                try:
                    conn = mavflight.connect("tcp:127.0.0.1:5760", timeout=5)
                    break
                except ConnectionRefusedError:
                    if proc.poll() is not None:
                        raise RuntimeError("SITL exited; see sitl.log")
                    time.sleep(.2)
            if conn is None or conn.target_system == 0:
                raise RuntimeError("No autopilot heartbeat")
            mavflight.request_data_stream(conn, 20)
            mavflight.wait_gps(conn)
            GuidedStreamer(conn).set_mode_guided()
            assert mavflight.arm_with_retry(conn, timeout=90, verbose=True), "arming failed"
            assert mavflight.takeoff(conn, 5, timeout=90, settle_window=2) is not None, "takeoff failed"
            print("takeoff settled", flush=True)
            if not args.stock:
                assert engage_custom_controller(conn, 9), "custom engagement not confirmed"

            for label, duration in [("hover", 8), ("steps", 16), ("dropout", 4), ("recovery", 6)]:
                missing[:] = [0] if label == "dropout" else []
                t0 = server.model.t
                deadline = time.monotonic() + duration * 4 + 10
                while server.model.t - t0 < duration:
                    if time.monotonic() > deadline:
                        raise TimeoutError("simulator stopped advancing")
                    t = server.model.t - t0
                    angles = [0., 0.]
                    if label == "steps":
                        half = int(t / 2)
                        angles[(half // 2) % 2] = np.deg2rad(5) * (1 if half % 2 == 0 else -1)
                    r, p = np.array(angles) / 2
                    q = [np.cos(r)*np.cos(p), np.sin(r)*np.cos(p),
                         np.cos(r)*np.sin(p), -np.sin(r)*np.sin(p)]
                    # GUID_OPTIONS=0: thrust=.5 requests zero climb, preserving
                    # the stock altitude loop while exciting inner control.
                    conn.mav.set_attitude_target_send(0, conn.target_system,
                        conn.target_component, 7, q, 0, 0, 0, .5)
                    if not args.stock:
                        _rc_override(conn, 9, 2000)
                    while conn.recv_match(blocking=False) is not None:
                        pass
                    st = server.model.state()
                    if np.linalg.norm(st["gyro"]) > 8 or abs(st["position"][2]) > 20:
                        raise RuntimeError(f"flight bound exceeded in {label}: {st}")
                    time.sleep(.02)
                manifest["windows"].append({"case": label, "start_s": t0, "end_s": server.model.t})
                print(f"{label} finished at sim t={server.model.t:.2f}", flush=True)
            manifest["success"] = True
    except Exception as error:
        manifest["error"] = str(error)
        raise
    finally:
        if conn is not None:
            mavflight.disarm(conn, force=True)
            time.sleep(1)
            conn.close()
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        server.shutdown()
        thread.join(timeout=2)
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
        (out / "truth.json").write_text(json.dumps(truth))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True)
    parser.add_argument("--defaults", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--stock", action="store_true")
    parser.add_argument("--param", action="append", default=[])
    run(parser.parse_args())


if __name__ == "__main__":
    main()
