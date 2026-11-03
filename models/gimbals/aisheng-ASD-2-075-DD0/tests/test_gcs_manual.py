#!/usr/bin/env python3
"""摇杆持续按住、键盘点动和松手停车回归测试。"""
from __future__ import annotations

import importlib.util
from pathlib import Path
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "aisheng_gcs_manual", ROOT / "ground_station" / "main.py")
GCS = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GCS)


class RecordingClient:
    def __init__(self):
        self.sent = []
        self.condition = threading.Condition()

    def send(self, command, **params):
        with self.condition:
            self.sent.append((time.monotonic(), command, params))
            self.condition.notify_all()

    def wait_for(self, predicate, timeout=1.0):
        with self.condition:
            return self.condition.wait_for(lambda: predicate(self.sent), timeout)


class ManualControllerTest(unittest.TestCase):
    def setUp(self):
        self.client = RecordingClient()
        self.manual = GCS.ManualController(self.client)
        self.addCleanup(self.manual.close)

    def test_stationary_drag_continues_past_mouse_event_timeout(self):
        station = GCS.GCS.__new__(GCS.GCS)
        station.manual = self.manual
        station.joystick = True
        station._update_joystick(
            GCS.DISPLAY_W / 2 + GCS.JOYSTICK_RADIUS, GCS.JOYSTICK_Y)
        started = time.monotonic()
        self.assertTrue(self.client.wait_for(
            lambda sent: sent and sent[-1][0] - started >= 0.6))
        self.assertTrue(all(params["p1"] == 33 and params["p2"] == 0
                            for _stamp, _command, params in self.client.sent))
        station._release_joystick()
        self.assertFalse(station.joystick)
        self.assertEqual(station.joystick_values, (0.0, 0.0))
        self.assertEqual(self.client.sent[-1][2], {"p1": 0, "p2": 0})
        stopped_count = len(self.client.sent)
        time.sleep(0.12)
        self.assertEqual(len(self.client.sent), stopped_count)

    def test_keyboard_jog_expires_and_stops(self):
        self.manual.set_velocity(60.0, -60.0, hold=0.15)
        self.assertTrue(self.client.wait_for(
            lambda sent: len(sent) >= 2 and sent[-1][2] == {"p1": 0, "p2": 0}))
        self.assertEqual(self.client.sent[0][2], {"p1": 100, "p2": -100})
        self.assertFalse(self.manual.active)

    def test_mouse_move_without_pressed_button_releases_drag(self):
        station = GCS.GCS.__new__(GCS.GCS)
        station.manual = self.manual
        station.joystick = True
        station.buttons = []
        station._update_joystick(
            GCS.DISPLAY_W / 2 + GCS.JOYSTICK_RADIUS, GCS.JOYSTICK_Y)
        station._on_mouse(GCS.cv2.EVENT_MOUSEMOVE, 0, 0, 0, None)
        self.assertFalse(station.joystick)
        self.assertFalse(self.manual.active)
        self.assertEqual(self.client.sent[-1][2], {"p1": 0, "p2": 0})

    def test_stop_cannot_be_overwritten_by_inflight_velocity(self):
        entered = threading.Event()
        release = threading.Event()
        original_send = self.client.send

        def blocking_send(command, **params):
            if params["p1"]:
                entered.set()
                release.wait(1.0)
            original_send(command, **params)

        self.client.send = blocking_send
        self.manual.set_velocity(60.0, 0.0, hold=None)
        self.assertTrue(entered.wait(1.0))
        stopped = threading.Event()
        worker = threading.Thread(target=lambda: (self.manual.stop(), stopped.set()))
        worker.start()
        try:
            self.assertFalse(stopped.wait(0.05))
        finally:
            release.set()
            worker.join(1.0)
        self.assertTrue(stopped.is_set())
        self.assertEqual(self.client.sent[-1][2], {"p1": 0, "p2": 0})

    def test_close_stops_held_drag_and_ends_worker(self):
        self.manual.set_velocity(60.0, 0.0, hold=None)
        self.assertTrue(self.client.wait_for(lambda sent: bool(sent)))
        self.manual.close()
        self.assertFalse(self.manual.thread.is_alive())
        self.assertEqual(self.client.sent[-1][2], {"p1": 0, "p2": 0})


if __name__ == "__main__":
    unittest.main(verbosity=2)
