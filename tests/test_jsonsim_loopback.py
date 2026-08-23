"""Loopback smoke test for the jsonsim CLI UDP backend (indi_harness.sitl.jsonsim.__main__)."""
import json
import socket
import struct
import threading

import numpy as np
import pytest

from indi_harness.sitl.jsonsim.__main__ import Server, build_parser


def _pack(frame_rate, frame_count, pwm):
    return struct.pack("<HHI16H", 18458, frame_rate, frame_count, *pwm)


def test_help_lists_expected_options():
    parser = build_parser()
    help_text = parser.format_help()
    for opt in ("--drag", "--no-drag", "--port", "--dragx", "--dragy", "--dragz"):
        assert opt in help_text


@pytest.mark.parametrize("drag_on", [True, False])
def test_drag_flag_reflected_on_model(drag_on):
    srv = Server(port=0, drag_on=drag_on)
    try:
        assert srv.model.drag_on is drag_on
    finally:
        srv.shutdown()


def test_loopback_hover_stays_near_origin():
    srv = Server(port=0, drag_on=True)
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    try:
        wh = srv.model.P.hover_speed()
        omega_max = srv.model.P.Omega_max
        u_hover = (wh / omega_max) ** 2
        pwm_val = int(1000 + 1000 * u_hover)
        pwm_hover = [pwm_val] * 16

        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.settimeout(2.0)

        n_steps = int(2.0 / (1.0 / 400.0))  # ~2s of sim time at 400Hz
        last_state = None
        for fc in range(1, n_steps + 1):
            client.sendto(_pack(400, fc, pwm_hover), ("127.0.0.1", srv.port))
            data, _ = client.recvfrom(4096)
            last_state = json.loads(data.decode("ascii"))

        assert last_state is not None
        pos = np.asarray(last_state["position"])
        assert np.linalg.norm(pos) < 0.5
    finally:
        client.close()
        srv.shutdown()
        thread.join(timeout=2.0)
