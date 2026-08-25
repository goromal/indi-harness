"""Pure scoring + low-rate .BIN reader helpers for the INDI-on-backend battery.

Factored out of the (previously inline) anixpkgs INDI-on-JSON-backend scorer:
RMS, active-run segmentation, the low-rate RATE reader + engagement-MSG reader,
and the windowed inner-loop buzz / omega-inversion / rate extractors. Keeping
them here makes them importable and unit-testable; the extractors take their data
arrays explicitly (no module-level state) so they stay pure.
"""
import numpy as np
from pymavlink import DFReader

from .align import omega_tracking_score

RAD2DEG = 180.0 / np.pi


def rms(x):
    """Root-mean-square of an array (0.0 for empty)."""
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x ** 2))) if x.size else 0.0


def active_runs(fb, min_len=50):
    """Contiguous fallback==0 runs (each = one case's engaged tracking block).
    The runner unlinks the ready-file between cases -> the DDS reference goes
    stale -> fallback rises, cleanly separating cases in the INDB stream."""
    fb = np.asarray(fb, int)
    runs = []
    i = 0
    n = len(fb)
    while i < n:
        if fb[i] == 0:
            j = i
            while j < n and fb[j] == 0:
                j += 1
            if j - i >= min_len:
                runs.append((i, j))
            i = j
        else:
            i += 1
    return runs


def read_rate_msgs(path):
    """Roll/pitch rate desired-vs-actual (deg/s) from the RATE message.

    WARNING: RATE logs at only ~10 Hz and ALIASES the rate-loop limit cycle --
    it reads a near-constant ~57 deg/s everywhere (including hover), so it is
    GARBAGE for buzz detection. Kept only as a low-rate desired-vs-actual sanity
    trace; the trustworthy buzz signal is the ~50 Hz IMU gyro (binlog.read_imu_gyro
    / gyro_window below) plus the 400 Hz INDI-message saturation + du."""
    log = DFReader.DFReader_binary(str(path))
    t, rd, r, pd, p = [], [], [], [], []
    while True:
        m = log.recv_match(type="RATE")
        if m is None:
            break
        t.append(m.TimeUS)
        rd.append(m.RDes); r.append(m.R)
        pd.append(m.PDes); p.append(m.P)
    return (np.asarray(t, float), np.asarray(rd, float), np.asarray(r, float),
            np.asarray(pd, float), np.asarray(p, float))


def read_engage_msgs(path):
    """Return the list of 'Custom controller is ON/OFF' STATUSTEXT strings the
    firmware logged to the MSG dataflash message -- the concrete proof the RC
    aux LOW->HIGH edge actually engaged the custom controller (vs flying stock)."""
    log = DFReader.DFReader_binary(str(path))
    msgs = []
    while True:
        m = log.recv_match(type="MSG")
        if m is None:
            break
        txt = getattr(m, "Message", "")
        if "custom controller" in txt.lower():
            msgs.append(txt.strip())
    return msgs


def gyro_window(gt, gyro_all, t0, t1):
    """Trustworthy inner-loop buzz over an IMU-gyro TimeUS window (deg/s).

    gt/gyro_all are (time_us, gyro_rad_s[n,3]) from binlog.read_imu_gyro. For roll
    (GyrX) and pitch (GyrY): rms is the overall body-rate magnitude; buzz_hp_rms
    is the high-pass residual (gyro minus a ~9-sample moving average) -- a
    rate-loop limit cycle shows up as large high-frequency body-rate motion even
    where the commanded rate is smooth, so buzz_hp_rms spikes under buzz and stays
    small under clean tracking. n is the sample count (guards aliasing)."""
    if gt.size == 0:
        return {"roll": {"rms": 0.0, "buzz_hp_rms": 0.0},
                "pitch": {"rms": 0.0, "buzz_hp_rms": 0.0}, "n": 0}
    w = (gt >= t0) & (gt <= t1)
    g = gyro_all[w] * RAD2DEG

    def axis(x):
        if x.size < 3:
            return {"rms": rms(x), "buzz_hp_rms": 0.0}
        k = min(9, x.size)
        lp = np.convolve(x, np.ones(k) / k, mode="same")
        return {"rms": rms(x), "buzz_hp_rms": rms(x - lp)}

    return {"roll": axis(g[:, 0]), "pitch": axis(g[:, 1]), "n": int(g.shape[0])}


def omega_window(ih, t0, t1):
    """Per-axis omega_dot-inversion score over an INDI TimeUS window.

    ih is the full dict from binlog.read_indi_health; every array (including
    time_us) is masked to the [t0, t1] window before scoring."""
    it = ih["time_us"]
    w = (it >= t0) & (it <= t1)
    ihw = {k: (v[w] if getattr(v, "ndim", 1) else v) for k, v in ih.items()}
    om = omega_tracking_score(ihw)
    du = np.asarray(ihw["du"], float)
    sat = np.asarray(ihw["sat"], int)
    return {
        "roll": om[0], "pitch": om[1], "yaw": om[2],
        "du_rms": [rms(du[:, ax]) for ax in range(3)] if du.size else [0.0] * 3,
        "sat_frac": float(sat.mean()) if sat.size else 0.0,
    }


def rate_window(rt, rdes, ract, pdes, pact, t0, t1):
    """Roll/pitch rate tracking + buzz proxy over a RATE TimeUS window (arrays
    from read_rate_msgs). The buzz proxy is RMS of the sample-to-sample change of
    the ACTUAL rate: a limit cycle (inner-loop buzz) shows up as large
    high-frequency actual-rate motion even where the desired rate is smooth, so
    d(actual)/sample RMS spikes."""
    w = (rt >= t0) & (rt <= t1)
    rd, ra, pd, pa = rdes[w], ract[w], pdes[w], pact[w]

    def axis(des, act):
        return {"des_rms": rms(des), "act_rms": rms(act),
                "err_rms": rms(act - des),
                "buzz_ddt_rms": rms(np.diff(act)) if act.size > 1 else 0.0}

    return {"roll": axis(rd, ra), "pitch": axis(pd, pa)}
