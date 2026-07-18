"""Human-facing S0 summary: RMSE table + tracking plots -> out/."""
import pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from indi_harness.params import QuadParams
from indi_harness.runner import run_scenario
from indi_harness.trajectory import Hover, Circle, Lemniscate
from indi_harness.evalmetrics import rmse_position

P = QuadParams()
OUT = pathlib.Path("out"); OUT.mkdir(exist_ok=True)
WIND = np.array([3.0, 0.0, 0.0])

CASES = {
    "hover": Hover(point=np.array([0, 0, -5.0])),
    "circle": Circle(radius=2.0, period=8.0, alt=5.0),
    "lemniscate": Lemniscate(amplitude=2.0, period=6.0, alt=5.0),
}

print(f"{'case':<12} {'ff':<6} {'wind':<6} {'RMSE [m]':>9}")
for name, traj in CASES.items():
    for ff_on in (True, False):
        for wind in (np.zeros(3), WIND):
            log = run_scenario(P, traj, t_end=12.0, wind=wind,
                               drag_on=True, ff_on=ff_on)
            _, total = rmse_position(log["t"], log["p"], log["p_ref"], trim_s=2.0)
            print(f"{name:<12} {str(ff_on):<6} {str(bool(wind.any())):<6} {total:>9.4f}")
            if ff_on and not wind.any():
                fig, ax = plt.subplots()
                ax.plot(log["p"][:, 1], log["p"][:, 0], label="flown")
                ax.plot(log["p_ref"][:, 1], log["p_ref"][:, 0], "--", label="ref")
                ax.set_xlabel("E [m]"); ax.set_ylabel("N [m]")
                ax.set_title(name); ax.legend(); ax.axis("equal")
                fig.savefig(OUT / f"{name}.png", dpi=120)
                plt.close(fig)
print(f"plots -> {OUT}/")
