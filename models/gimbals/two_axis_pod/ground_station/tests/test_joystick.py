#!/usr/bin/env python3
"""离线验证鼠标输入、实际 JSON 编码的手动流及释放保护，无 GUI/网络依赖。"""
import ast
import json
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "ground_station.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
cv2 = SimpleNamespace(EVENT_LBUTTONDOWN=1, EVENT_MOUSEMOVE=0,
                      EVENT_LBUTTONUP=4, EVENT_FLAG_LBUTTON=1,
                      destroyAllWindows=lambda: None)
namespace = {"threading": threading, "time": time, "json": json, "math": math,
             "cv2": cv2, "MAX_DATAGRAM": 8192,
             "VISION_MODES": ("off", "detect", "auto_lock", "manual_track"),
             "VISION_MODE_CN": {"off": "关闭视觉", "detect": "目标检测",
                                "auto_lock": "自动锁定", "manual_track": "手动框选"}}
exec(compile(ast.Module(body=[node for node in tree.body
                             if isinstance(node, ast.ClassDef)
                             and node.name in ("UdpClient", "ManualController", "GroundStation")],
                        type_ignores=[]), str(SOURCE), "exec"), namespace)
UdpClient = namespace["UdpClient"]
ManualController = namespace["ManualController"]
GroundStation = namespace["GroundStation"]


class DatagramSink:
    def __init__(self):
        self.packets = []

    def sendto(self, data, target):
        self.packets.append((time.monotonic(), json.loads(data.decode("utf-8"))))


class JoystickTests(unittest.TestCase):
    def setUp(self):
        self.client = UdpClient.__new__(UdpClient)
        self.client.lock = threading.RLock()
        self.client._seq = 0
        self.client.sock = DatagramSink()
        self.client.target = ("127.0.0.1", 8766)
        self.client.on_log = lambda message: None
        self.client._stop = threading.Event()
        self.client._pending = {}
        self.client.hello = {"aircraft_id": 1, "version": 4, "angle_convention": "upright_joint"}
        self.client.telemetry = {"angle_convention": "upright_joint", "attitude_deg": {axis: {"pos": 0., "vel": 0., "acc": 0.} for axis in ("yaw", "pitch")}}
        self.client.last_rx_mono = self.client.last_telemetry_mono = time.monotonic()
        self.client.client_timeout = 5.0
        self.manual = ManualController(self.client)
        self.gs = GroundStation.__new__(GroundStation)
        self.gs.manual = self.manual
        self.gs.tele = self.client
        self.gs.display_size = (960, 540)
        self.gs.joystick = None
        self.gs.joystick_values = (0.0, 0.0)
        self.gs.joystick_event_at = 0.0
        self.held = True
        self.gs.pointer_guard = SimpleNamespace(held_in_window=lambda: self.held, close=lambda: None)
        self.gs.buttons = []
        self.gs.pressed = None
        self.gs.mouse_start = self.gs.mouse_end = None
        self.gs.drawing = False
        self.gs.pipeline = None
        self.gs.lock = threading.Lock()
        self.gs.frame_size = (1920, 1080)
        self.gs.last_cmd = ""

    def tearDown(self):
        self.manual.stop()
        if self.manual.is_alive():
            self.manual.join(timeout=1.0)

    def mouse(self, event, x, y, flags=0):
        self.gs.on_mouse(event, x, y, flags, None)

    def test_drag_produces_repeated_encoded_manual_then_release_zero(self):
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx, cy)
        self.mouse(cv2.EVENT_MOUSEMOVE, cx + radius, cy, cv2.EVENT_FLAG_LBUTTON)
        self.manual.start()
        end = time.monotonic() + 0.45
        while time.monotonic() < end:
            self.gs._check_joystick()  # 保持静止按住，同样续发。
            time.sleep(0.025)
        nonzero = [(stamp, packet) for stamp, packet in self.client.sock.packets if packet.get("yaw") == 30.0]
        self.assertGreaterEqual(len(nonzero), 4)
        for _, packet in nonzero:
            self.assertEqual(packet["cmd"], "manual")
            self.assertEqual(packet["pitch"], 0.0)
            self.assertNotIn("roll", packet)
            self.assertIsInstance(packet["seq"], int)
        self.assertLess(max(b[0] - a[0] for a, b in zip(nonzero, nonzero[1:])), 0.18)
        self.mouse(cv2.EVENT_LBUTTONUP, cx - radius, cy)
        count = len(self.client.sock.packets)
        self.assertEqual(self.client.sock.packets[-1][1]["yaw"], 0.0)
        time.sleep(0.15)
        self.assertTrue(all(p["yaw"] == p["pitch"] == 0.0
                            for _, p in self.client.sock.packets[count - 1:]))
        self.assertIsNone(self.gs.joystick)
        self.assertFalse(self.gs.drawing)

    def test_pitch_deadzone_and_clamped_range(self):
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx + 1, cy - 1)
        self.assertEqual(self.gs.joystick_values, (0.0, 0.0))
        self.mouse(cv2.EVENT_MOUSEMOVE, cx, cy - radius * 3, cv2.EVENT_FLAG_LBUTTON)
        self.assertEqual(self.manual.axes["pitch"][0], 20.0)
        self.assertEqual(len(self.gs._joystick_geometry()), 1)

    def test_yaw_left_is_negative_and_right_is_positive(self):
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx - radius, cy)
        self.assertEqual(self.manual.axes["yaw"][0], -30.0)
        self.mouse(cv2.EVENT_MOUSEMOVE, cx + radius, cy, cv2.EVENT_FLAG_LBUTTON)
        self.assertEqual(self.manual.axes["yaw"][0], 30.0)

    def test_old_protocol_disables_joystick_keys_and_axis_reset(self):
        self.client.hello = {"aircraft_id": 1, "version": 1}
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx - radius, cy)
        self.gs._manual("yaw", 30.0)
        self.gs._send_ctrl("reset_axis", axis="pitch")
        self.assertIsNone(self.gs.joystick)
        self.assertFalse(self.manual.axes)
        self.assertFalse(self.client.sock.packets)

    def test_protocol_downgrade_during_drag_cancels_motion(self):
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx + radius, cy)
        self.client.telemetry = {"joints_deg": {}}
        self.gs._check_joystick()
        self.assertIsNone(self.gs.joystick)
        self.assertEqual(self.client.sock.packets[-1][1]["yaw"], 0.0)

    def test_singular_view_preserves_yaw_control_and_reset(self):
        self.client.telemetry["singular"] = True
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx + radius, cy)
        self.assertEqual(self.manual.axes["yaw"][0], 30.0)
        self.gs._send_ctrl("reset_axis", axis="yaw")
        self.assertEqual(self.client.sock.packets[-1][1]["axis"], "yaw")

    def test_each_axis_button_uses_reliable_reset_axis_and_stops_manual(self):
        self.gs.buttons = self.gs._axis_reset_buttons()
        for label, (x, y, width, height), cmd, extra in self.gs.buttons:
            self.manual.hold("pitch", 20.0)
            self.mouse(cv2.EVENT_LBUTTONDOWN, x + width // 2, y + height // 2)
            self.mouse(cv2.EVENT_LBUTTONUP, x + width // 2, y + height // 2)
            zero, command = [p for _, p in self.client.sock.packets[-2:]]
            self.assertEqual(zero["pitch"], 0.0)
            self.assertEqual(command["cmd"], "reset_axis")
            self.assertEqual(command["axis"], extra["axis"])
            self.assertEqual(self.client._pending[command["seq"]]["payload"], command)
            self.assertFalse(self.gs.drawing)

    def test_lost_focus_or_outside_release_cancels(self):
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx - radius, cy)
        self.held = False
        self.gs._check_joystick()
        self.assertIsNone(self.gs.joystick)
        self.assertEqual(self.manual.axes, {})
        self.assertEqual(self.client.sock.packets[-1][1]["yaw"], 0.0)

    def test_unsupported_focus_monitor_has_finite_lease(self):
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx - radius, cy)
        self.held = None
        self.gs.joystick_event_at -= 0.31
        self.gs._check_joystick()
        self.assertIsNone(self.gs.joystick)
        self.assertEqual(self.manual.axes, {})

    def test_gui_stall_expires_manual_without_refresh(self):
        self.manual.hold("yaw", 30.0, duration=0.15)
        self.manual.start()
        time.sleep(0.35)
        packets = [p for _, p in self.client.sock.packets]
        self.assertTrue(any(p["yaw"] == 30.0 for p in packets))
        self.assertEqual(packets[-1]["yaw"], 0.0)

    def test_control_command_cancels_manual_first(self):
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx - radius, cy)
        self.client.send_reliable = self.client.send_once
        self.gs._send_ctrl("home")
        packets = [p for _, p in self.client.sock.packets]
        self.assertEqual(packets[-2]["cmd"], "manual")
        self.assertEqual(packets[-2]["yaw"], 0.0)
        self.assertEqual(packets[-1]["cmd"], "home")
        self.assertIsNone(self.gs.joystick)

    def test_video_drag_is_rejected_until_manual_track_mode(self):
        self.mouse(cv2.EVENT_LBUTTONDOWN, 100, 100)
        self.assertFalse(self.gs.drawing)
        self.assertIn("拒绝画框", self.gs.last_cmd)
        self.client.telemetry["vision"] = {"mode": "manual_track"}
        self.mouse(cv2.EVENT_LBUTTONDOWN, 100, 100)
        self.mouse(cv2.EVENT_MOUSEMOVE, 200, 180, cv2.EVENT_FLAG_LBUTTON)
        self.mouse(cv2.EVENT_LBUTTONUP, 200, 180)
        packet = self.client.sock.packets[-1][1]
        self.assertEqual(packet["cmd"], "track")
        self.assertEqual(packet["box"], [0.104167, 0.185185, 0.104167, 0.148148])

    def test_four_mode_buttons_send_one_reliable_mode_command(self):
        buttons = self.gs._vision_mode_buttons()
        self.assertEqual([extra["mode"] for _, _, _, extra in buttons],
                         ["off", "detect", "auto_lock", "manual_track"])
        self.gs.buttons = buttons
        label, (x, y, width, height), _, _ = buttons[2]
        self.mouse(cv2.EVENT_LBUTTONDOWN, x + width // 2, y + height // 2)
        self.mouse(cv2.EVENT_LBUTTONUP, x + width // 2, y + height // 2)
        packet = self.client.sock.packets[-1][1]
        self.assertEqual(packet["cmd"], "set_vision_mode")
        self.assertEqual(packet["mode"], "auto_lock")
        self.assertIn(packet["seq"], self.client._pending)

    def test_auto_lock_manual_control_sends_off_first(self):
        self.client.telemetry["vision"] = {"mode": "auto_lock"}
        self.gs._send_ctrl("home")
        packets = [packet for _, packet in self.client.sock.packets]
        self.assertEqual([packet["cmd"] for packet in packets],
                         ["set_vision_mode", "home"])
        self.assertEqual(packets[0]["mode"], "off")

    def test_manual_track_joystick_sends_off_before_motion(self):
        self.client.telemetry["vision"] = {"mode": "manual_track"}
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx + radius, cy)
        packets = [packet for _, packet in self.client.sock.packets]
        self.assertEqual(packets[0]["cmd"], "set_vision_mode")
        self.assertEqual(packets[0]["mode"], "off")
        self.assertEqual(self.manual.axes["yaw"][0], 30.0)

    def test_auto_lock_uses_single_target_overlay_after_selection(self):
        searching = {"vision": {"mode": "auto_lock", "locked_track_id": None}}
        selected = {"vision": {"mode": "auto_lock", "locked_track_id": 3}}
        manual = {"vision": {"mode": "manual_track"}}
        self.assertFalse(self.gs._single_target_overlay_active(searching))
        self.assertTrue(self.gs._single_target_overlay_active(selected))
        self.assertTrue(self.gs._single_target_overlay_active(manual))

    def test_detect_mode_keeps_detection_during_joystick(self):
        self.client.telemetry["vision"] = {"mode": "detect"}
        cx, cy, radius = self.gs._joystick_geometry()[0]
        self.mouse(cv2.EVENT_LBUTTONDOWN, cx + radius, cy)
        packets = [packet for _, packet in self.client.sock.packets]
        self.assertFalse(any(packet["cmd"] == "set_vision_mode" for packet in packets))
        self.assertEqual(self.manual.axes["yaw"][0], 30.0)

    def test_shutdown_is_idempotent_and_zeros_before_bye(self):
        self.manual.hold("yaw", 20.0)
        self.gs.shutdown()
        first = list(self.client.sock.packets)
        self.gs.shutdown()
        self.assertEqual(self.client.sock.packets, first)
        self.assertEqual(first[-2][1]["cmd"], "manual")
        self.assertEqual(first[-2][1]["yaw"], 0.0)
        self.assertEqual(first[-1][1]["cmd"], "bye")


if __name__ == "__main__":
    unittest.main(verbosity=2)
