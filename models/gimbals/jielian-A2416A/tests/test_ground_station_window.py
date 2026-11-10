#!/usr/bin/env python3
"""A2416A地面站窗口生命周期测试。"""
from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from ground_station import main


class FakeCvError(Exception):
    pass


class FakeCv:
    WND_PROP_VISIBLE = 4
    error = FakeCvError

    def __init__(self, result=None, exception=None):
        self.result = result
        self.exception = exception

    def getWindowProperty(self, _window, _property):
        if self.exception is not None:
            raise self.exception
        return self.result


class FakeRunCv:
    WINDOW_AUTOSIZE = 1
    WND_PROP_VISIBLE = 4
    error = FakeCvError

    def __init__(self, *, visible=(1.0, 0.0), keys=(-1, -1)):
        self.visible = iter(visible)
        self.keys = iter(keys)
        self.imshow_calls = 0

    def namedWindow(self, *_args):
        pass

    def imshow(self, *_args):
        self.imshow_calls += 1

    def waitKey(self, _delay):
        return next(self.keys)

    def getWindowProperty(self, *_args):
        return next(self.visible)

    def setMouseCallback(self, *_args):
        pass

    def setWindowTitle(self, *_args):
        pass


class WindowLifecycleTest(unittest.TestCase):
    def test_visible_window_keeps_running(self):
        with patch.object(main, "cv2", FakeCv(result=1.0)):
            self.assertTrue(main.window_is_visible("A2416A_GCS"))

    def test_closed_window_stops_before_imshow_can_recreate_it(self):
        with patch.object(main, "cv2", FakeCv(result=0.0)):
            self.assertFalse(main.window_is_visible("A2416A_GCS"))

    def test_destroyed_qt_window_is_treated_as_closed(self):
        with patch.object(main, "cv2", FakeCv(exception=FakeCvError("missing"))):
            self.assertFalse(main.window_is_visible("A2416A_GCS"))

    def test_run_does_not_recreate_window_after_close_button(self):
        backend = FakeRunCv()
        station = object.__new__(main.GroundStation)
        station.window = "A2416A_GCS"
        station.window_title = "A2416A 捷联导引头地面站"
        station.running = True
        station._frame = Mock(return_value=object())
        station._update_rates = Mock()
        station._handle_forward_key = Mock(return_value=False)
        station.close = Mock()
        with patch.object(main, "cv2", backend):
            station.run()
        self.assertEqual(backend.imshow_calls, 2)
        station.close.assert_called_once_with()
        station._handle_forward_key.assert_not_called()

    def test_q_and_escape_exit_even_when_forward_input_has_focus(self):
        for key in (ord("q"), 27):
            with self.subTest(key=key):
                backend = FakeRunCv(visible=(1.0, 1.0), keys=(-1, key))
                station = object.__new__(main.GroundStation)
                station.window = "A2416A_GCS"
                station.window_title = "A2416A 捷联导引头地面站"
                station.running = True
                station._frame = Mock(return_value=object())
                station._update_rates = Mock()
                station._handle_forward_key = Mock(return_value=True)
                station.close = Mock()
                with patch.object(main, "cv2", backend):
                    station.run()
                station._handle_forward_key.assert_not_called()
                station.close.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
