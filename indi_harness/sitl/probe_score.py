"""Score a rate_probe run using simulator truth and firmware DataFlash.

This small-flight gate is deliberately separate from the deferred realistic-physics trajectory
gate. Freeze thresholds here before validating the corrected firmware.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from pymavlink import DFReader

from .binlog import read_indi_health, read_indc_health
from .binscore import omega_window
from ..params import QuadParams
from ..sysid import normalized_effectiveness, identify_trace


def score(directory):
    root = Path(directory)
    manifest = json.loads((root / "manifest.json").read_text())
    truth = json.loads((root / "truth.json").read_text())
    logs = sorted((root / "logs").glob("*.BIN"))
    if len(logs) != 1:
        raise ValueError(f"Expected exactly one flight log, got {logs}")
    ih, ic = read_indi_health(logs[0]), read_indc_health(logs[0])
    log = DFReader.DFReader_binary(str(logs[0]))
    params, ownership, messages = {}, [], []
    while (msg := log.recv_match(type=["PARM", "INDU", "MSG"])) is not None:
        if msg.get_type() == "PARM":
            params[msg.Name] = msg.Value
        elif msg.get_type() == "MSG":
            messages.append(msg.Message)
        else:
            ownership.append(msg.to_dict())
    ts = np.array([r["timestamp"] for r in truth])
    rates = np.array([r["imu"]["gyro"] for r in truth])
    altitude = -np.array([r["position"][2] for r in truth])
    rows, errors = [], []
    identification = None
    required = {"hover", "steps", "dropout", "recovery"}
    if not manifest["success"] or {w["case"] for w in manifest["windows"]} != required:
        errors.append("flight did not finish all required windows")
    if not any("Custom controller is ON" in s for s in messages):
        errors.append("DataFlash did not record controller engagement")
    for key, expected in manifest["parameters"].items():
        if key not in params or not np.isclose(params[key], expected, atol=1e-5):
            errors.append(f"parameter {key}: log={params.get(key)} requested={expected}")
    linear = {"MOT_THST_EXPO": 0, "MOT_SPIN_MIN": 0, "MOT_SPIN_MAX": 1,
              "MOT_BAT_VOLT_MIN": 0, "MOT_BAT_VOLT_MAX": 0}
    if any(not np.isclose(params.get(k, np.nan), v) for k, v in linear.items()):
        errors.append("normalized gain check requires the linearized actuator map")
    seed = normalized_effectiveness(QuadParams())
    gains = np.array([params.get("CC3_G1_RP", np.nan)] * 2 + [params.get("CC3_G1_YAW", np.nan)])
    ratios = gains / seed["g1"]
    if not np.all(np.isfinite(ratios) & (ratios > 0) & (ratios < 3)):
        errors.append(f"normalized G1 ratios must be positive and below 3: {ratios}")
    for window in manifest["windows"]:
        name, start, end = window["case"], window["start_s"], window["end_s"]
        # Trim 0.1 s at phase boundaries for command receipt/clock rounding;
        # this still includes the internal 2-second step transients.
        mask = (ts >= start + .1) & (ts <= end - .1)
        rpm_mask = (ic["time_us"] >= (start + .1)*1e6) & (ic["time_us"] <= (end - .1)*1e6)
        if not mask.any() or not rpm_mask.any():
            errors.append(f"{name}: missing truth or RPM samples")
            continue
        om = omega_window(ih, (start + .1)*1e6, (end - .1)*1e6)
        fb = float(np.mean(ic["fallback"][rpm_mask]))
        row = {"case": name, "omega": om, "fallback_frac": fb,
               "peak_rate_rad_s": float(np.linalg.norm(rates[mask], axis=1).max()),
               "altitude_range_m": [float(altitude[mask].min()), float(altitude[mask].max())]}
        if row["peak_rate_rad_s"] >= 1 or not (2 < altitude[mask].min() <= altitude[mask].max() < 9):
            errors.append(f"{name}: rate or altitude bound exceeded")
        if om["sat_frac"] > .02:
            errors.append(f"{name}: saturation above 2%")
        if (name == "dropout" and fb < .95) or (name != "dropout" and fb > .01):
            errors.append(f"{name}: unexpected fallback fraction {fb}")
        if name == "steps":
            identification = identify_trace(ts[mask],
                np.array([r["omega"] for r in truth])[mask], rates[mask])
            if not np.allclose(identification["g1"], seed["g1"], rtol=.15):
                errors.append("recorded SITL G1 differs from normalized seed by >15%")
            if not np.isclose(identification["g2_yaw"], seed["g2_yaw"], rtol=.15):
                errors.append("recorded SITL G2 differs from normalized seed by >15%")
            for axis in ("roll", "pitch"):
                if om[axis]["exc_rms"] < .2 or om[axis]["nrmse"] >= .9 or om[axis]["r2"] <= .2:
                    errors.append(f"steps: {axis} acceleration tracking failed")
        rows.append(row)
    # Log ownership evidence without mistaking current PID for last output.
    own = {}
    if ownership and manifest["windows"]:
        # Disarming intentionally resets _last_output every ground tick.
        # Restrict this continuity check to the flight, excluding that reset.
        first = manifest["windows"][0]["start_s"] + .1
        last = manifest["windows"][-1]["end_s"] - .1
        ownership = [r for r in ownership if first < r["TimeUS"] / 1e6 < last]
        if len(ownership) < 2:
            errors.append("insufficient INDU samples during flight")
        else:
            previous = np.array([[r[f"L{k}"] for k in "xyz"] for r in ownership[1:]])
            output = np.array([[r[f"O{k}"] for k in "xyz"] for r in ownership[:-1]])
            own["previous_output_max_error"] = float(np.max(np.abs(previous-output)))
            if own["previous_output_max_error"] > 1e-6:
                errors.append("previous custom output does not match preceding INDU output")
    else:
        errors.append("INDU ownership diagnostic is missing")
    result = {"passed": not errors, "errors": errors, "per_case": rows,
              "g1_normalized_seed": seed["g1"].tolist(), "g1_ratios": ratios.tolist(),
              "ownership": own, "identification": identification}
    (root / "score.json").write_text(json.dumps(result, indent=2, allow_nan=False))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    result = score(parser.parse_args().directory)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
