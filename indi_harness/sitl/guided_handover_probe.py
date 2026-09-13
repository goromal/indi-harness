"""Bounded stock GUIDED takeoff and hold experiment for JSON SITL.

The startup timing matches ``baseline_outer`` without engaging custom control
or publishing DDS data.  Each run owns its simulator and firmware process and
writes all evidence under a newly-created output directory.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import re
import socket
import subprocess
import threading
import time

import numpy as np
from pymavlink import DFReader, mavutil

from . import mavflight
from .baseline_cc import engage_custom_controller, _rc_override
from .jsonsim.__main__ import Server
from .setpoints import Setpoint


ALTITUDE_M = 10.0
MAP_PARAMETERS = {
    "linearized": {
        "MOT_THST_EXPO": 0,
        "MOT_SPIN_ARM": 0,
        "MOT_SPIN_MIN": 0,
        "MOT_SPIN_MAX": 1,
        "MOT_BAT_VOLT_MIN": 0,
        "MOT_BAT_VOLT_MAX": 0,
    },
    "normal": {},
}
BASE_PARAMETERS = {
    "FRAME_CLASS": 1,
    "FRAME_TYPE": 1,
    "MOT_THST_HOVER": 0.30,
    "MOT_HOVER_LEARN": 0,
    "CC_TYPE": 3,
    "CC_AXIS_MASK": 7,
    "RC9_OPTION": 109,
    "CC3_OMG_FILT": 80,
    "CC3_G1_RP": 500,
    "CC3_G1_YAW": 28.8,
    "CC3_OUTER_EN": 1,
    "CC3_B_THR_EN": 0,
    "CC3_B_ACC_FILT": 8,
    "CC3_USE_RPM": 1,
    "CC3_G2_YAW": 0,
    "LOG_DISARMED": 1,
    "LOG_BITMASK": 65535,
    "FS_THR_ENABLE": 0,
    "FS_GCS_ENABLE": 0,
}
MAP_PARAMETER_NAMES = set(MAP_PARAMETERS["linearized"])


def requested_parameters(actuator_map, extra_parameters=()):
    params = {**BASE_PARAMETERS, **MAP_PARAMETERS[actuator_map]}
    for setting in extra_parameters:
        key, value = setting.split("=", 1)
        params[key] = float(value)
    return params


def _write_log_evidence(log_path, out, manifest):
    log = DFReader.DFReader_binary(str(log_path))
    actual, messages, rate_rows, imu_rows = {}, [], [], []
    while (msg := log.recv_match(type=["PARM", "MSG", "RATE", "IMU"])) is not None:
        kind = msg.get_type()
        if kind == "PARM":
            actual[msg.Name] = msg.Value
        elif kind == "MSG":
            messages.append(msg.Message)
        elif kind == "RATE":
            rate_rows.append([msg.TimeUS, msg.RDes, msg.R, msg.PDes, msg.P,
                              msg.YDes, msg.Y])
        elif getattr(msg, "I", 0) == 0:
            imu_rows.append([msg.TimeUS, msg.GyrX, msg.GyrY, msg.GyrZ,
                             msg.AccX, msg.AccY, msg.AccZ])

    names = set(manifest["requested_parameters"]) | MAP_PARAMETER_NAMES
    manifest["logged_parameters"] = {
        name: actual.get(name) for name in sorted(names)
    }
    version = next((message for message in messages
                    if message.startswith("ArduCopter V")), None)
    manifest["firmware_version"] = version
    match = re.search(r"\(([0-9a-f]{7,40})\)", version or "")
    manifest["firmware_git_hash"] = match.group(1) if match else None
    manifest["controller_messages"] = [
        message for message in messages if "Custom controller" in message
    ]

    with (out / "rate.csv").open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["time_us", "roll_des_deg_s", "roll_deg_s",
                         "pitch_des_deg_s", "pitch_deg_s", "yaw_des_deg_s",
                         "yaw_deg_s"])
        writer.writerows(rate_rows)
    with (out / "imu.csv").open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["time_us", "gyro_x_rad_s", "gyro_y_rad_s",
                         "gyro_z_rad_s", "accel_x_m_s2", "accel_y_m_s2",
                         "accel_z_m_s2"])
        writer.writerows(imu_rows)


def _send_position_hold(conn, origin):
    fields = Setpoint(p=np.asarray(origin, float), v=np.zeros(3),
                      a=np.zeros(3)).fields()
    conn.mav.set_position_target_local_ned_send(
        fields["time_boot_ms"], conn.target_system, conn.target_component,
        fields["coordinate_frame"], fields["type_mask"], fields["x"],
        fields["y"], fields["z"], fields["vx"], fields["vy"], fields["vz"],
        fields["afx"], fields["afy"], fields["afz"], fields["yaw"],
        fields["yaw_rate"])


def _send_attitude_hold(conn):
    conn.mav.set_attitude_target_send(
        0, conn.target_system, conn.target_component, 7,
        [1.0, 0.0, 0.0, 0.0], 0, 0, 0, 0.5)


def run(args):
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    binary = Path(args.binary).resolve()
    params = requested_parameters(args.actuator_map, args.param)
    parameter_file = out / "guided-handover.parm"
    parameter_file.write_text("".join(f"{key} {value}\n"
                                      for key, value in params.items()))
    bounds = {
        "max_roll_pitch_rate_rad_s": args.max_rate,
        "min_altitude_m_after_climb": args.min_altitude,
        "max_altitude_m": args.max_altitude,
        "max_horizontal_displacement_m": args.max_horizontal_displacement,
        "handover_rate_limit_rad_s": 1.0,
        "handover_history_s": 2.0,
    }
    manifest = {
        "actuator_map": args.actuator_map,
        "target_type": args.target,
        "engage_indi": args.engage_indi,
        "binary": str(binary),
        "binary_sha256": hashlib.sha256(binary.read_bytes()).hexdigest(),
        "defaults": str(Path(args.defaults).resolve()),
        "requested_parameters": params,
        "bounds": bounds,
        "events": [],
        "success": False,
    }
    truth = []
    server = proc = conn = thread = None
    wall_start = time.monotonic()
    climb_complete = False
    reference_xy = np.zeros(2)

    def mark(name):
        event = {"name": name, "sim_time_s": server.model.t,
                 "wall_elapsed_s": time.monotonic() - wall_start}
        manifest["events"].append(event)
        print(f"{name}: sim={event['sim_time_s']:.2f}s "
              f"wall={event['wall_elapsed_s']:.2f}s", flush=True)

    def check_bounds():
        if not truth:
            return
        state = truth[-1]
        gyro = np.asarray(state["imu"]["gyro"], float)
        position = np.asarray(state["position"], float)
        rate = float(np.linalg.norm(gyro[:2]))
        altitude = -float(position[2])
        horizontal = float(np.linalg.norm(position[:2] - reference_xy))
        if rate > bounds["max_roll_pitch_rate_rad_s"]:
            raise RuntimeError(f"roll/pitch rate bound exceeded: {rate:.3f} rad/s")
        if altitude > bounds["max_altitude_m"]:
            raise RuntimeError(f"altitude upper bound exceeded: {altitude:.3f} m")
        if climb_complete and altitude < bounds["min_altitude_m_after_climb"]:
            raise RuntimeError(f"altitude lower bound exceeded: {altitude:.3f} m")
        if climb_complete and horizontal > bounds["max_horizontal_displacement_m"]:
            raise RuntimeError(f"horizontal position bound exceeded: {horizontal:.3f} m")

    def recent_roll_pitch_peak(history_s):
        cutoff = server.model.t - history_s
        recent = [np.linalg.norm(np.asarray(row["imu"]["gyro"], float)[:2])
                  for row in truth if row["timestamp"] >= cutoff]
        return float(max(recent)) if recent else float("inf")

    def wait_phase(duration, target=None):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if proc.poll() is not None:
                raise RuntimeError("SITL exited; see sitl.log")
            if target == "position":
                _send_position_hold(conn, hold_origin)
            elif target == "attitude":
                _send_attitude_hold(conn)
            while conn.recv_match(blocking=False) is not None:
                pass
            check_bounds()
            time.sleep(0.02)

    try:
        with socket.socket() as check:
            check.bind(("127.0.0.1", 5760))
        server = Server(port=9002, drag_on=False)
        original = server.driver.on_packet

        def record(data):
            reply = original(data)
            if reply is not None:
                state = json.loads(reply)
                state["omega"] = server.model._omega.tolist()
                truth.append(state)
            return reply

        server.driver.on_packet = record
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        with (out / "sitl.log").open("w") as log:
            proc = subprocess.Popen([
                str(binary), "--model=JSON:127.0.0.1",
                "--home=34.381441,-118.580861,417,0", "--speedup=1", "-I0",
                "--config=undulation:0.0", "--defaults",
                f"{Path(args.defaults).resolve()},{parameter_file}",
            ], cwd=out, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 30
            while time.monotonic() < deadline:
                try:
                    conn = mavflight.connect("tcp:127.0.0.1:5760", timeout=5)
                    break
                except ConnectionRefusedError:
                    if proc.poll() is not None:
                        raise RuntimeError("SITL exited; see sitl.log")
                    time.sleep(0.2)
            if conn is None or conn.target_system == 0:
                raise RuntimeError("no autopilot heartbeat")
            mark("connected")
            mavflight.request_data_stream(conn, 20)
            mavflight.wait_gps(conn)
            mark("gps_ekf_warmup_complete")
            from .streamer import GuidedStreamer
            GuidedStreamer(conn).set_mode_guided()
            mark("guided_confirmed")
            if not mavflight.arm_with_retry(conn, timeout=90, verbose=True):
                raise RuntimeError("arming failed")
            mark("armed")
            conn.mav.command_long_send(
                conn.target_system, conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
                0, 0, 0, 0, 0, 0, ALTITUDE_M)
            mark("takeoff_commanded")
            takeoff_deadline = time.monotonic() + 120
            while time.monotonic() < takeoff_deadline:
                position = conn.recv_match(type="LOCAL_POSITION_NED", blocking=True,
                                           timeout=1)
                check_bounds()
                if position is not None and -position.z > 0.9 * ALTITUDE_M:
                    climb_complete = True
                    reference_xy[:] = [position.x, position.y]
                    break
            if not climb_complete:
                raise TimeoutError("takeoff altitude not reached")
            mark("takeoff_90_percent")
            wait_phase(5.0)
            mark("nominal_handover")
            pre_engagement_peak = recent_roll_pitch_peak(
                bounds["handover_history_s"])
            manifest["pre_engagement_peak_roll_pitch_rate_rad_s"] = pre_engagement_peak
            if pre_engagement_peak >= bounds["handover_rate_limit_rad_s"]:
                raise RuntimeError(
                    f"handover precondition failed: {pre_engagement_peak:.3f} rad/s")
            if args.engage_indi:
                if not engage_custom_controller(conn, 9):
                    raise RuntimeError("custom controller engagement not confirmed")
                check_bounds()
                mark("indi_engaged")
            wait_phase(4.0)
            mark("nominal_dds_ready")
            hold = mavflight.get_local_pos(conn)
            if hold is None:
                raise RuntimeError("no position at hold start")
            hold_origin = np.asarray(hold, float)
            reference_xy[:] = hold_origin[:2]
            mark("target_hold_start")
            wait_phase(args.hold_seconds, args.target)
            mark("target_hold_end")
            manifest["success"] = True
    except BaseException as error:
        manifest["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        if conn is not None:
            try:
                if args.engage_indi:
                    _rc_override(conn, 9, 65535)
                mavflight.disarm(conn, force=True)
                time.sleep(1)
                conn.close()
            except Exception as error:
                manifest["cleanup_error"] = str(error)
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        if server is not None:
            server.shutdown()
        if thread is not None:
            thread.join(timeout=2)
        (out / "truth.json").write_text(json.dumps(truth))
        logs = sorted((out / "logs").glob("*.BIN"))
        if logs:
            try:
                _write_log_evidence(logs[-1], out, manifest)
            except Exception as error:
                manifest["evidence_error"] = str(error)
        else:
            manifest["evidence_error"] = "no DataFlash log found"
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", required=True)
    parser.add_argument("--defaults", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--actuator-map", choices=MAP_PARAMETERS, required=True)
    parser.add_argument("--target", choices=("position", "attitude"), required=True)
    parser.add_argument("--engage-indi", action="store_true",
                        help="engage measured-RPM INDI after the pre-handover gate")
    parser.add_argument("--hold-seconds", type=float, default=12.0)
    parser.add_argument("--max-rate", type=float, default=2.0)
    parser.add_argument("--min-altitude", type=float, default=2.0)
    parser.add_argument("--max-altitude", type=float, default=14.0)
    parser.add_argument("--max-horizontal-displacement", type=float, default=5.0)
    parser.add_argument("--param", action="append", default=[])
    run(parser.parse_args())


if __name__ == "__main__":
    main()
