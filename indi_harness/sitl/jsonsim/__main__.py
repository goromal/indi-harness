"""CLI physics backend for `arducopter --model=JSON:<host>`.

    python -m indi_harness.sitl.jsonsim --drag        # drag on
    python -m indi_harness.sitl.jsonsim --no-drag     # clean A/B baseline
"""
import argparse
import socket

import numpy as np

from indi_harness.params import QuadParams
from indi_harness.sitl.jsonsim.model import QuadJsonModel
from indi_harness.sitl.jsonsim.protocol import LockstepDriver


class Server:
    def __init__(self, port=9002, drag_on=True, drag=(0.35, 0.35, 0.15)):
        P = QuadParams()
        P.drag_D = np.diag(np.asarray(drag, float))
        # ground=True: the SITL vehicle must rest on a ground plane before it
        # arms/takes off (a disarmed free-fall diverges the EKF -> SITL FPE).
        self.model = QuadJsonModel(P, drag_on=drag_on, dt=1.0 / 400.0, ground=True)
        self.model.seed_omega(P.hover_speed() * np.ones(4))
        self.driver = LockstepDriver(self.model)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.5)
        self.port = self.sock.getsockname()[1]
        self._run = True

    def serve_forever(self):
        while self._run:
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            reply = self.driver.on_packet(data)
            if reply is not None:
                self.sock.sendto(reply.encode("ascii"), addr)

    def shutdown(self):
        self._run = False
        self.sock.close()


def build_parser():
    ap = argparse.ArgumentParser(prog="indi_harness.sitl.jsonsim")
    ap.add_argument("--port", type=int, default=9002)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--drag", dest="drag_on", action="store_true", default=True)
    g.add_argument("--no-drag", dest="drag_on", action="store_false")
    ap.add_argument("--dragx", type=float, default=0.35)
    ap.add_argument("--dragy", type=float, default=0.35)
    ap.add_argument("--dragz", type=float, default=0.15)
    return ap


def main():
    a = build_parser().parse_args()
    print(f"[jsonsim] port={a.port} drag_on={a.drag_on} D=({a.dragx},{a.dragy},{a.dragz})", flush=True)
    Server(a.port, a.drag_on, (a.dragx, a.dragy, a.dragz)).serve_forever()


if __name__ == "__main__":
    main()
