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

    def test_binary_telemetry_and_manual_forward_share_control_socket(self):
        import json
        import socket
        import time
        gateway = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        gateway.bind(("127.0.0.1", 0))
        gateway.settimeout(2.0)
        out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        out.bind(("127.0.0.1", 0))
        out.settimeout(2.0)
        client = GCS.TelemetryClient("127.0.0.1", gateway.getsockname()[1])
        station = GCS.GCS.__new__(GCS.GCS)
        station.client = client
        station.forward_inputs = {"ip": "127.0.0.1", "port": str(out.getsockname()[1])}
        station.forward_focus = None
        station.forward_selected = False
        station.forward_active = False
        station.forward_count = 0
        station.forward_error = ""
        station.forward_thread = None
        station.forward_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        station.running = True
        foreign = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            heartbeat, peer = gateway.recvfrom(2048)
            self.assertEqual(GCS.proto.decode_control(heartbeat).command, 0)
            self.assertEqual(client.sock.getsockname(), peer)
            binary = GCS.proto.encode_telemetry(GCS.proto.TelemetryFrame(
                work_state=1, gate_x=960, gate_y=540, timestamp_ms=997,
                target_type=4, target_confidence=0.8,
                target_lon_deg=119.1058778, target_lat_deg=32.2178573))
            # 已连接 socket 只接受网关地址回包，拒绝其他来源。
            foreign.sendto(binary, peer)
            time.sleep(0.03)
            self.assertIsNone(client.latest)
            gateway.sendto(b"invalid", peer)
            gateway.sendto(binary, peer)
            deadline = time.monotonic() + 1.0
            while client.latest is None and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertEqual(client.count, 1)
            expected_timestamp = client.forward_snapshot[2]["timestamp"]
            self.assertEqual(client.latest.timestamp_ms, 997)
            client.send(GCS.proto.CMD_FRAME_UNLOCK)
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                data, command_peer = gateway.recvfrom(2048)
                if GCS.proto.decode_control(data).command == GCS.proto.CMD_FRAME_UNLOCK:
                    self.assertEqual(command_peer, peer)
                    break
            else:
                self.fail("未从同一端口收到控制命令")
            self.assertEqual(station.forward_count, 0)
            station._toggle_forward()
            payload, _ = out.recvfrom(8192)
            forwarded = json.loads(payload)
            self.assertEqual(len(forwarded), 37)
            self.assertEqual(forwarded["timestamp"], expected_timestamp)
            self.assertEqual(forwarded["device_timestamp"], 997)
            self.assertEqual(forwarded["target_confidence"], [0.8, 0.0, 0.0, 0.0])
            self.assertEqual(forwarded["target_lon"], [1191058778, 0, 0, 0])
            self.assertFalse(any(name.startswith("los_omega_") for name in forwarded))
            # 同一接收快照只转发一次，时间戳不会在转发线程中被重写。
            out.settimeout(0.05)
            with self.assertRaises(socket.timeout):
                out.recvfrom(8192)
            gateway.sendto(GCS.proto.encode_telemetry(
                GCS.proto.TelemetryFrame(timestamp_ms=998)), peer)
            out.settimeout(2.0)
            payload, _ = out.recvfrom(8192)
            second = json.loads(payload)
            self.assertEqual(second["device_timestamp"], 998)
            self.assertGreater(second["timestamp"], expected_timestamp)
            self.assertEqual(station.forward_count, 2)
        finally:
            station.forward_active = False
            if station.forward_thread is not None:
                station.forward_thread.join(timeout=1.0)
            station.forward_sock.close()
            foreign.close()
            out.close()
            client.close()
            gateway.close()

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

    def test_view_render_does_not_modify_decoded_frame(self):
        import numpy as np
        station = GCS.GCS.__new__(GCS.GCS)
        station.active = "wide"
        source = np.full((GCS.DISPLAY_H, GCS.DISPLAY_W, 3), 100, dtype=np.uint8)
        frame = GCS.proto.TelemetryFrame(status_control2=1)
        display = station._view_geometry(source, frame)
        GCS.cv2.rectangle(display, (10, 10), (100, 100), (0, 255, 0), 2)
        self.assertTrue(np.all(source == 100))
        # 没有新视频时，下一次绘制仍获得干净底图。
        self.assertTrue(np.all(station._view_geometry(source, frame) == 100))

    def test_rgba_overlay_preserves_color_in_bgr_image(self):
        import numpy as np
        image = np.zeros((1, 2, 3), dtype=np.uint8)
        overlay = np.array([[[255, 20, 10, 255], [0, 0, 255, 0]]], dtype=np.uint8)
        GCS.GCS._composite_rgba(image, overlay)
        self.assertEqual(tuple(image[0, 0]), (10, 20, 255))
        self.assertEqual(tuple(image[0, 1]), (0, 0, 0))

    def test_labels_follow_pillow_availability(self):
        rows = GCS.BUTTON_ROWS if GCS._HAVE_PIL else GCS.BUTTON_ROWS_EN
        labels = [label for row in rows for label, _ in row]
        self.assertEqual(len(labels), 14)
        self.assertEqual(labels[0], "解锁" if GCS._HAVE_PIL else "UNLOCK")


if __name__ == "__main__":
    unittest.main(verbosity=2)
