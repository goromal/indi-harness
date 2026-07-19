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
