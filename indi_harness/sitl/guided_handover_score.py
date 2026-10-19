"""Score a bounded stock GUIDED handover probe from saved simulator truth."""
import argparse
import json
from pathlib import Path

import numpy as np

from .binlog import read_indc_health, read_indi_health


def score(directory):
    root = Path(directory)
    manifest = json.loads((root / "manifest.json").read_text())
    truth = json.loads((root / "truth.json").read_text())
    events = {event["name"]: event["sim_time_s"] for event in manifest["events"]}
    errors = []
    required_events = {
        "connected", "gps_ekf_warmup_complete", "guided_confirmed", "armed",
        "takeoff_commanded", "takeoff_90_percent", "nominal_handover",
        "nominal_dds_ready", "target_hold_start", "target_hold_end",
    }
    if manifest["engage_indi"]:
        required_events.add("indi_engaged")
    if not manifest["success"]:
        errors.append(manifest.get("error", "flight did not finish"))
    missing = required_events - set(events)
    if missing:
        errors.append(f"missing events: {sorted(missing)}")
    if not truth:
        errors.append("simulator truth is empty")
        result = {"passed": False, "errors": errors}
        (root / "score.json").write_text(json.dumps(result, indent=2))
        return result

    time_s = np.asarray([row["timestamp"] for row in truth], float)
    position = np.asarray([row["position"] for row in truth], float)
    gyro = np.asarray([row["imu"]["gyro"] for row in truth], float)
    roll_pitch_rate = np.linalg.norm(gyro[:, :2], axis=1)
    altitude = -position[:, 2]

    def window(start_name, end_name):
        if start_name not in events or end_name not in events:
            return np.zeros(len(time_s), dtype=bool)
        return ((time_s >= events[start_name]) & (time_s <= events[end_name]))

    handover = np.zeros(len(time_s), dtype=bool)
    if "nominal_handover" in events:
        t1 = events["nominal_handover"]
        handover = (time_s >= t1 - 2.0) & (time_s <= t1)
    hold = window("target_hold_start", "target_hold_end")
    airborne = window("takeoff_90_percent", "target_hold_end")

    def peak(values, mask):
        return float(np.max(values[mask])) if mask.any() else None

    handover_peak = peak(roll_pitch_rate, handover)
    hold_peak = peak(roll_pitch_rate, hold)
    airborne_peak = peak(roll_pitch_rate, airborne)
    if handover_peak is None or handover_peak >= 1.0:
        errors.append(f"unsettled handover: peak={handover_peak} rad/s")
    bounds = manifest["bounds"]
    if airborne_peak is None or airborne_peak > bounds["max_roll_pitch_rate_rad_s"]:
        errors.append(f"airborne rate bound exceeded: peak={airborne_peak} rad/s")
    if airborne.any() and (altitude[airborne].min() < bounds["min_altitude_m_after_climb"]
                           or altitude[airborne].max() > bounds["max_altitude_m"]):
        errors.append("airborne altitude bound exceeded")

    engagement = None
    if manifest["engage_indi"]:
        logs = sorted((root / "logs").glob("*.BIN"))
        if len(logs) != 1 or "indi_engaged" not in events:
            errors.append(f"expected one DataFlash log for engagement, got {logs}")
        else:
            indi = read_indi_health(logs[0])
            indc = read_indc_health(logs[0])
            start_us = events["indi_engaged"] * 1e6
            indi_mask = indi["time_us"] >= start_us
            indc_mask = indc["time_us"] >= start_us
            du_rms = (float(np.sqrt(np.mean(indi["du"][indi_mask] ** 2)))
                      if indi_mask.any() else 0.0)
            saturation_fraction = (float(np.mean(indi["sat"][indi_mask]))
                                   if indi_mask.any() else 1.0)
            fallback_fraction = (float(np.mean(indc["fallback"][indc_mask]))
                                 if indc_mask.any() else 1.0)
            mean_abs_omega = (float(np.mean(np.abs(indc["omega"][indc_mask])))
                              if indc_mask.any() else 0.0)
            messages = manifest.get("controller_messages", [])
            engagement = {
                "controller_messages": messages,
                "du_rms": du_rms,
                "saturation_fraction": saturation_fraction,
                "rpm_fallback_fraction": fallback_fraction,
                "mean_abs_omega_rad_s": mean_abs_omega,
            }
            if not any("Custom controller is ON" in message for message in messages):
                errors.append("DataFlash did not confirm custom controller engagement")
            if du_rms <= 1e-6:
                errors.append(f"custom controller increment is inactive: du_rms={du_rms}")
            if saturation_fraction > 0.02:
                errors.append(f"INDI saturation exceeds 2%: {saturation_fraction}")
            if fallback_fraction > 0.01 or mean_abs_omega <= 0:
                errors.append("measured-RPM channel is not healthy after engagement")

    first_growth = None
    growing = np.flatnonzero(airborne & (roll_pitch_rate >= 1.0))
    if growing.size:
        index = int(growing[0])
        first_growth = {
            "sim_time_s": float(time_s[index]),
            "roll_pitch_rate_rad_s": float(roll_pitch_rate[index]),
            "altitude_m": float(altitude[index]),
        }
    report = {
        "passed": not errors,
        "errors": errors,
        "configuration": {
            "actuator_map": manifest["actuator_map"],
            "target_type": manifest["target_type"],
            "engage_indi": manifest["engage_indi"],
        },
        "handover": {
            "peak_roll_pitch_rate_rad_s": handover_peak,
            "limit_rad_s": 1.0,
        },
        "flight": {
            "peak_roll_pitch_rate_rad_s": airborne_peak,
            "hold_peak_roll_pitch_rate_rad_s": hold_peak,
            "altitude_range_m": ([float(altitude[airborne].min()),
                                  float(altitude[airborne].max())]
                                 if airborne.any() else None),
            "first_rate_growth": first_growth,
        },
        "engagement": engagement,
        "firmware_git_hash": manifest.get("firmware_git_hash"),
        "binary_sha256": manifest["binary_sha256"],
        "logged_parameters": manifest.get("logged_parameters", {}),
        "events": manifest["events"],
    }
    (root / "score.json").write_text(json.dumps(report, indent=2,
                                                allow_nan=False))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    report = score(parser.parse_args().directory)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
