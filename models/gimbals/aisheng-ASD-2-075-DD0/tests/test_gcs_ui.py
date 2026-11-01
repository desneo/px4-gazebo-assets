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
            def set_active(self, _active):
                pass
        station.client = Client()
        station.manual = Manual()
        station.wide = Video()
        station.tele = Video()
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
                return np.zeros((720, 1280, 3), dtype=np.uint8)
        station = GCS.GCS.__new__(GCS.GCS)
        station.client = Client()
        station.wide = Video()
        station.tele = Video()
        station.active = "wide"
        station.buttons = []
        station.joystick = False
        station.joystick_values = (0.0, 0.0)
        station.view_crop = (0, 0, 1280, 720)
        station.view_scale_x = 1.0
        station.view_scale_y = 1.0
        station.source_size = (1280, 720)
        station.last_cmd = ""
        GCS.GCS._on_mouse(station, GCS.cv2.EVENT_LBUTTONDBLCLK, 640, 360, 0, None)
        self.assertEqual(station.client.sent[0][0], GCS.proto.CMD_FRAME_UNLOCK)
        self.assertEqual(station.client.sent[1], (
            GCS.proto.CMD_POINT_LOCK, {"p1": 640, "p2": 360}))

    def test_send_angle_uses_trackbar_values_only_on_button(self):
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
        station.trackbar_ready = True
        def position(name, _window):
            return 123 if name == "Yaw x0.1" else 900
        with mock.patch.object(GCS.cv2, "getTrackbarPos", side_effect=position):
            station._activate("send_angle")
        self.assertEqual(station.client.sent[-1], (
            GCS.proto.CMD_SPECIFIED_ANGLE, {"p1": -4770, "p2": 1000}))
        self.assertIn("yaw -47.7", station.last_cmd)

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
