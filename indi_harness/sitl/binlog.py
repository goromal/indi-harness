"""ArduPilot .BIN dataflash adapter (design doc L.1: BIN is the on-vehicle
source of truth; this reads the EKF position solution, XKF1.PN/PE/PD)."""
import numpy as np
from pymavlink import DFReader


def read_ekf_pos(path, core=0):
    """Return (time_us, p_ned) arrays from XKF1 rows of one EKF core."""
    log = DFReader.DFReader_binary(str(path))
    t, p = [], []
    while True:
        m = log.recv_match(type="XKF1")
        if m is None:
            break
        if getattr(m, "C", 0) != core:
            continue
        t.append(m.TimeUS)
        p.append([m.PN, m.PE, m.PD])
    return np.asarray(t, float), np.asarray(p, float)


def read_indi_health(path):
    """Return the Layer-A INDI health time series from the INDI dataflash
    message written by AC_CustomControl_INDI::update() (design doc L: the .BIN
    is source of truth). Keys map to arrays over time:
        time_us                              [n]
        domega_pred, domega_meas   (rad/s^2) [n,3]  predicted vs filtered accel
        u_act                                [n,3]  filtered actuator-state est
        du                                   [n,3]  INDI torque increment
        sat                                  [n]    saturation flag (0/1)
    """
    log = DFReader.DFReader_binary(str(path))
    t, pred, meas, uact, du, sat = [], [], [], [], [], []
    while True:
        m = log.recv_match(type="INDI")
        if m is None:
            break
        t.append(m.TimeUS)
        pred.append([m.Px, m.Py, m.Pz])
        meas.append([m.Mx, m.My, m.Mz])
        uact.append([m.Ax, m.Ay, m.Az])
        du.append([m.Dx, m.Dy, m.Dz])
        sat.append(m.S)
    return {
        "time_us": np.asarray(t, float),
        "domega_pred": np.asarray(pred, float),
        "domega_meas": np.asarray(meas, float),
        "u_act": np.asarray(uact, float),
        "du": np.asarray(du, float),
        "sat": np.asarray(sat, int),
    }
