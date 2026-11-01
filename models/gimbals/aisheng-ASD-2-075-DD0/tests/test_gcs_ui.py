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
        self.assertEqual(len(buttons), 11)
        self.assertEqual([action for _, action, _ in buttons], [
            "unlock", "lock", "recenter", "cancel", "auto", "auto_off",
            "wide", "tele", "zoom_in", "zoom_out", "stop",
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
        station.client = Client()
        station.manual = Manual()
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

    def test_labels_follow_pillow_availability(self):
        rows = GCS.BUTTON_ROWS if GCS._HAVE_PIL else GCS.BUTTON_ROWS_EN
        labels = [label for row in rows for label, _ in row]
        self.assertEqual(len(labels), 11)
        self.assertEqual(labels[0], "解锁" if GCS._HAVE_PIL else "UNLOCK")


if __name__ == "__main__":
    unittest.main(verbosity=2)
