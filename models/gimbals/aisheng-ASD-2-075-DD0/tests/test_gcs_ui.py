#!/usr/bin/env python3
"""导引头上位机底部按钮和虚拟摇杆几何单元测试。"""
from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "aisheng_gcs", ROOT / "ground_station" / "main.py")
GCS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GCS)


class GcsUiTest(unittest.TestCase):
    def test_button_layout_fits_panel(self):
        buttons = GCS.button_layout()
        self.assertEqual(len(buttons), 14)
        self.assertEqual([action for _, action, _ in buttons], [
            "unlock", "lock", "recenter", "cancel", "auto", "auto_off",
            "send_angle", "wide", "tele", "zoom_in", "zoom_out",
            "zero_yaw", "zero_pitch", "stop",
        ])
        for _label, _action, (x, y, width, height) in buttons:
            self.assertGreaterEqual(x, 0)
            self.assertGreaterEqual(y, GCS.DISPLAY_H)
            self.assertLessEqual(x + width, GCS.DISPLAY_W)
            self.assertLessEqual(y + height, GCS.DISPLAY_H + GCS.PANEL_H)

    def test_hit_button(self):
        buttons = GCS.button_layout()
        label, action, _rect = buttons[0]
        hit = GCS.hit_button(buttons[0][2][0] + 2, buttons[0][2][1] + 2, buttons)
        self.assertEqual((hit[0], hit[1]), (label, action))
        self.assertIsNone(GCS.hit_button(0, 0, buttons))

    def test_joystick_deadzone_and_limit(self):
        self.assertEqual(GCS.joystick_axes(GCS.DISPLAY_W / 2, GCS.JOYSTICK_Y),
                         (0.0, 0.0))
        x, y = GCS.joystick_axes(GCS.DISPLAY_W / 2 + GCS.JOYSTICK_RADIUS,
                                 GCS.JOYSTICK_Y)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        x, y = GCS.joystick_axes(GCS.DISPLAY_W / 2,
                                 GCS.JOYSTICK_Y - GCS.JOYSTICK_RADIUS)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 1.0)
        x, y = GCS.joystick_axes(GCS.DISPLAY_W / 2 + 2 * GCS.JOYSTICK_RADIUS,
                                 GCS.JOYSTICK_Y)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)

    def test_button_actions_send_expected_commands(self):
        class Client:
            def __init__(self):
                self.sent = []
            def send(self, command, **kwargs):
                self.sent.append((command, kwargs))
        class Manual:
            def __init__(self):
                self.stopped = 0
            def stop(self):
                self.stopped += 1
        station = GCS.GCS.__new__(GCS.GCS)
        class Video:
            def restart(self):
                pass
        station.client = Client()
        station.manual = Manual()
        station.video = Video()
        station.active = "wide"
        station.joystick = False
        station.joystick_values = (0.0, 0.0)
        station.last_cmd = ""
        station._activate("tele")
        self.assertEqual(station.active, "tele")
        self.assertEqual(station.client.sent[-1], (GCS.proto.CMD_DETECTOR_SWITCH, {"p1": 2}))
        station._activate("auto")
        self.assertEqual(station.client.sent[-1][0], GCS.proto.CMD_AUTO_LOCK_ON)
        station._activate("stop")
        self.assertEqual(station.manual.stopped, 1)
        self.assertEqual(station.client.sent[-1][0], GCS.proto.CMD_CANCEL_LOCK)

    def test_double_click_unlocks_and_point_locks_in_protocol_pixels(self):
        import numpy as np
        class Client:
            def __init__(self):
                self.sent = []
                self.latest = GCS.proto.TelemetryFrame(
                    status_control=1 << 5, status_control2=0x01)
            def send(self, command, **kwargs):
                self.sent.append((command, kwargs))
        class Video:
            def get(self):
                return np.zeros((540, 960, 3), dtype=np.uint8)
        station = GCS.GCS.__new__(GCS.GCS)
        station.client = Client()
        station.video = Video()
        station.active = "wide"
        station.buttons = []
        station.joystick = False
        station.joystick_values = (0.0, 0.0)
        station.view_crop = (0, 0, 960, 540)
        station.view_scale_x = 1.0
        station.view_scale_y = 1.0
        station.source_size = (960, 540)
        station.last_cmd = ""
        GCS.GCS._on_mouse(station, GCS.cv2.EVENT_LBUTTONDBLCLK, 480, 270, 0, None)
        self.assertEqual(station.client.sent[0][0], GCS.proto.CMD_FRAME_UNLOCK)
        self.assertEqual(station.client.sent[1], (
            GCS.proto.CMD_POINT_LOCK, {"p1": 960, "p2": 540}))

    def test_send_angle_uses_input_values_only_on_button(self):
        from unittest import mock
        class Client:
            def __init__(self):
                self.sent = []
                self.latest = GCS.proto.TelemetryFrame(status_control=4 << 5)
            def send(self, command, **kwargs):
                self.sent.append((command, kwargs))
        class Manual:
            def stop(self):
                pass
        class Video:
            def restart(self):
                pass
        station = GCS.GCS.__new__(GCS.GCS)
        station.client = Client()
        station.manual = Manual()
        station.video = Video()
        station.active = "wide"
        station.joystick = False
        station.joystick_values = (0.0, 0.0)
        station.last_cmd = ""
        station.angle_inputs = {"yaw": "12.3", "pitch": "-45.6"}
        station._activate("send_angle")
        self.assertEqual(station.client.sent[-1], (
            GCS.proto.CMD_SPECIFIED_ANGLE, {"p1": 1230, "p2": -4560}))
        self.assertIn("yaw 12.3", station.last_cmd)

    def test_json_receiver_and_manual_forward(self):
        import json
        import socket
        import time
        receiver = GCS.JsonTelemetryReceiver(0)
        try:
            port = receiver.rx.getsockname()[1]
            message = {
                "timestamp": 1, "track_status": 0, "pitch_angle": 0.0,
                "yaw_angle": 0.0, "pitch_misalignment": 0.0,
                "yaw_misalignment": 0.0, "tracking_confidence": 0.0,
                "target_confidence": 0.8, "gyro_pitch_rate": 0.0,
                "gyro_yaw_rate": 0.0, "pitch_los_rate": 0.0,
                "yaw_los_rate": 0.0, "los_omega_n": 0.0,
                "los_omega_e": 0.0, "los_omega_d": 0.0,
                "tracking_gate_w": 0.0, "tracking_gate_h": 0.0,
                "gate_x": 0, "gate_y": 0,
            }
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            tx.sendto(json.dumps(message).encode(), ("127.0.0.1", port))
            deadline = time.monotonic() + 1.0
            while receiver.latest is None and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(receiver.latest, message)
            out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            out.bind(("127.0.0.1", 0))
            out.settimeout(2.0)
            station = GCS.GCS.__new__(GCS.GCS)
            station.json_rx = receiver
            station.forward_inputs = {"ip": "127.0.0.1", "port": str(out.getsockname()[1])}
            station.forward_focus = None
            station.forward_selected = False
            station.forward_active = False
            station.forward_count = 0
            station.forward_error = ""
            station.forward_thread = None
            station.forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            station.running = True
            station._toggle_forward()
            payload, _ = out.recvfrom(8192)
            forwarded = json.loads(payload)
            self.assertEqual(len(forwarded), 19)
            self.assertIsInstance(forwarded["timestamp"], int)
            station.forward_active = False
            station.forward_thread.join(timeout=1.0)
            station.forward_sock.close()
            out.close()
            tx.close()
        finally:
            receiver.close()

    def test_single_axis_zero_keeps_the_other_axis(self):
        class Client:
            def __init__(self):
                self.sent = []
                self.latest = GCS.proto.TelemetryFrame(
                    status_control=4 << 5, yaw_deg=10.0, pitch_deg=20.0)
            def send(self, command, **kwargs):
                self.sent.append((command, kwargs))
        class Manual:
            def stop(self):
                pass
        class Video:
            def set_active(self, _active):
                pass
        station = GCS.GCS.__new__(GCS.GCS)
        station.client = Client()
        station.manual = Manual()
        station.wide = Video()
        station.tele = Video()
        station.active = "wide"
        station.joystick = False
        station.joystick_values = (0.0, 0.0)
        station.last_cmd = ""
        station._activate("zero_yaw")
        self.assertEqual(station.client.sent[-1], (
            GCS.proto.CMD_SPECIFIED_ANGLE, {"p1": 0, "p2": 2000}))
        station._activate("zero_pitch")
        self.assertEqual(station.client.sent[-1], (
            GCS.proto.CMD_SPECIFIED_ANGLE, {"p1": 1000, "p2": 0}))

    def test_labels_follow_pillow_availability(self):
        rows = GCS.BUTTON_ROWS if GCS._HAVE_PIL else GCS.BUTTON_ROWS_EN
        labels = [label for row in rows for label, _ in row]
        self.assertEqual(len(labels), 14)
        self.assertEqual(labels[0], "解锁" if GCS._HAVE_PIL else "UNLOCK")


if __name__ == "__main__":
    unittest.main(verbosity=2)
