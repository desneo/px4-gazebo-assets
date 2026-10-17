#!/usr/bin/env python3
"""Windows 专用入口的配置和视频帧处理测试。"""
from pathlib import Path
import os
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

import numpy as np


GROUND_STATION = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(GROUND_STATION))

import ground_station as common  # noqa: E402
import ground_station_windows as windows  # noqa: E402
from stamp import seq_to_bits  # noqa: E402


class WindowsGroundStationTests(unittest.TestCase):
    def test_d3d11_capture_params_require_explicit_hardware_decode(self):
        params = windows.d3d11_capture_params()
        pairs = dict(zip(params[0::2], params[1::2]))
        self.assertEqual(
            pairs[windows.cv2.CAP_PROP_HW_ACCELERATION],
            windows.cv2.VIDEO_ACCELERATION_D3D11)

    def test_open_capture_rejects_software_decode_fallback(self):
        class SoftwareCapture:
            released = False

            @staticmethod
            def isOpened():
                return True

            @staticmethod
            def get(prop):
                if prop == windows.cv2.CAP_PROP_HW_ACCELERATION:
                    return windows.cv2.VIDEO_ACCELERATION_NONE
                return 0

            def release(self):
                self.released = True

        capture = SoftwareCapture()
        station = windows.WindowsGroundStation.__new__(windows.WindowsGroundStation)
        station.cfg = {
            "gimbal_host": "127.0.0.1", "rtsp_port": 8555,
            "rtsp_mount": "/two_axis_pod", "aircraft_id": 1,
        }
        station.rtsp_transport = "tcp"
        with mock.patch.object(windows.cv2, "VideoCapture", return_value=capture):
            with self.assertRaisesRegex(RuntimeError, "拒绝软件解码回退"):
                station._open_capture()
        self.assertTrue(capture.released)

    def test_pointer_guard_uses_process_id_instead_of_mojibake_title(self):
        class User32:
            @staticmethod
            def GetForegroundWindow():
                return 100

            @staticmethod
            def GetWindowThreadProcessId(hwnd, process_id):
                process_id._obj.value = os.getpid()
                return 1

            @staticmethod
            def GetAsyncKeyState(key):
                return 0x8000

        guard = windows.WindowsPointerGuard.__new__(windows.WindowsPointerGuard)
        guard.title = "即使标题乱码也不参与判断"
        guard.user32 = User32()
        self.assertTrue(guard.held_in_window())

    def test_default_config_targets_aircraft_one(self):
        config = common.load_config(windows.WINDOWS_CONFIG)
        self.assertEqual(config["gimbal_host"], "192.168.1.56")
        self.assertEqual(config["udp_port"], 8770)
        self.assertEqual(config["aircraft_id"], 1)
        self.assertEqual(
            common.rtsp_url(config),
            "rtsp://192.168.1.56:8555/two_axis_pod_1")
        self.assertEqual(config["rtsp_transport"], "tcp")

    def test_publish_frame_updates_display_and_sequence_statistics(self):
        station = windows.WindowsGroundStation.__new__(windows.WindowsGroundStation)
        station.scale = 0.25
        station.lock = threading.Lock()
        station.latest_frame = None
        station.display_size = (0, 0)
        station.last_frame_mono = 0.0
        station.last_seq = None
        station.drops = 0
        station.expected = 0
        station.delays = []
        station.frame_times = []
        station.tele = SimpleNamespace(frame_delay_ms=lambda sequence, now: 12.5)

        frame = np.zeros((1080, 1920, 3), dtype=np.uint8)
        for index, bit in enumerate(seq_to_bits(123)):
            if bit:
                frame[1064:1080, index * 16:(index + 1) * 16] = 255
        station._publish_frame(frame)

        self.assertEqual(station.latest_frame.shape, (270, 480, 3))
        self.assertEqual(station.display_size, (480, 270))
        self.assertEqual(station.last_seq, 123)
        self.assertEqual(station.delays[-1][1], 12.5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
