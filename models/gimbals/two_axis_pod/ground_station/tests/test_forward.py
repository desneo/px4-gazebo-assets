"""艾晟转换、真实 UDP、身份边界、停止行为和输入焦点回归。"""
import copy
import configparser
import json
from pathlib import Path
import socket
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import ground_station as station
from telemetry_forward import ForwardInputs, TelemetryForwarder, aisheng_message, endpoint, save_destination


def telemetry():
    return {"type": "telemetry", "protocol_version": 7, "angle_convention": "motion_head_joint_right_positive",
            "aircraft_id": 1, "mode": "TRACK", "tracking": {"state": "TRACK"},
            "vision": {"mode": "manual_track", "state": "MANUAL_READY"},
            "attitude_deg": {"pitch": {"pos": -56.37, "vel": -1.23, "acc": 0},
                             "yaw": {"pos": 12.34, "vel": 2.34, "acc": 0}},
            "target": {"valid": True, "box_px": [95.0, 195.0, 50.0, 40.0],
                       "miss": {"valid": True, "offset_deg": [-.25, .5]}},
            "frame_table": [[42, 12.3]], "future_field": "不发送"}


class ForwardTests(unittest.TestCase):
    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(("127.0.0.1", 0))
        self.receiver.settimeout(.08)
        self.addCleanup(self.receiver.close)
        self.forwarder = TelemetryForwarder()
        self.addCleanup(self.forwarder.close)

    def start(self):
        self.forwarder.start(*self.receiver.getsockname())

    def receive(self):
        self.receiver.settimeout(1)
        data = self.receiver.recvfrom(8192)[0]
        self.receiver.settimeout(.08)
        return data

    def test_only_eleven_raw_aisheng_fields_without_wrapper(self):
        raw = json.dumps(telemetry(), ensure_ascii=False).encode()
        self.forwarder.offer(raw)
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(8192)
        self.start()
        self.forwarder.offer(raw)
        self.assertEqual(json.loads(self.receive()), {
            "track_status": 1, "pitch_angle": -56.37, "yaw_angle": 12.34,
            "pitch_misalignment": 0.5, "yaw_misalignment": -0.25,
            "tracking_confidence": 25.5, "target_confidence": 0.8,
            "pitch_los_rate": -1.23, "yaw_los_rate": 2.34,
            "tracking_gateW": 50.0, "tracking_gateH": 40.0})
        self.assertEqual(self.forwarder.snapshot()["sent"], 1)
        # 没有新遥测不定时重复发送上一次状态。
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(8192)

    def test_stop_and_restart_do_not_replay_old_packets(self):
        self.start()
        source = telemetry()
        self.forwarder.offer(json.dumps(source).encode())
        self.assertEqual(json.loads(self.receive())["yaw_angle"], 12.34)
        self.forwarder.stop()
        source["attitude_deg"]["yaw"]["pos"] = 22
        self.forwarder.offer(json.dumps(source).encode())
        self.start()
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(8192)
        source["attitude_deg"]["yaw"]["pos"] = -33
        self.forwarder.offer(json.dumps(source).encode())
        self.assertEqual(json.loads(self.receive())["yaw_angle"], -33.0)

    def test_tracking_gate_uses_final_float_pixel_box_and_invalid_state_zeroes_it(self):
        source = telemetry()
        source["target"]["box_px"] = [94.75, 194.25, 50.5, 40.25]
        packet = aisheng_message(source)
        self.assertEqual(packet["tracking_gateW"], 50.5)
        self.assertEqual(packet["tracking_gateH"], 40.25)
        source["tracking"]["state"] = "TEMP_LOST"
        source["target"]["box_px"] = "过期坏框"
        packet = aisheng_message(source)
        self.assertEqual(packet["tracking_gateW"], 0.0)
        self.assertEqual(packet["tracking_gateH"], 0.0)

    def test_only_identity_confirmed_telemetry_from_configured_peer_is_forwarded(self):
        self.start()
        client = station.UdpClient("127.0.0.1", 8766)
        self.addCleanup(client.stop)
        self.addCleanup(client.sock.close)
        client.on_telemetry = self.forwarder.offer
        raw = json.dumps(telemetry()).encode()
        client._on_datagram(raw, client.target)
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(8192)
        client._on_datagram(b'{"type":"hello","aircraft_id":1}', client.target)
        client._on_datagram(raw, ("127.0.0.1", 8767))
        client._on_datagram(b'{"type":"pong"}', client.target)
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(8192)
        client._on_datagram(raw, client.target)
        self.assertEqual(json.loads(self.receive()), aisheng_message(telemetry()))
        client._on_datagram(b'{"type":"hello","aircraft_id":2}', client.target)
        client._on_datagram(raw, client.target)
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(8192)

    def test_invalid_tracking_zeroes_miss_and_confidence_but_keeps_measured_attitude(self):
        original = telemetry()
        for path, value in ((('mode',), 'IDLE'), (('tracking', 'state'), 'TEMP_LOST'),
                            (('tracking', 'state'), 'INIT'), (('tracking', 'state'), 'LOST'),
                            (('target', 'valid'), False), (('target', 'miss', 'valid'), False)):
            source = copy.deepcopy(original)
            source['target_confidence'] = 99.0  # 固定值不取自输入，也不随丢失清零。
            node = source
            for key in path[:-1]:
                node = node[key]
            node[path[-1]] = value
            with self.subTest(path=path, value=value):
                packet = aisheng_message(source)
                self.assertEqual(packet['track_status'], 0)
                self.assertEqual(packet['tracking_confidence'], 0.0)
                self.assertEqual(packet['target_confidence'], 0.8)
                self.assertIs(type(packet['target_confidence']), float)
                self.assertEqual(packet['pitch_misalignment'], 0.0)
                self.assertEqual(packet['yaw_misalignment'], 0.0)
                self.assertEqual(packet['tracking_gateW'], 0.0)
                self.assertEqual(packet['tracking_gateH'], 0.0)
                self.assertEqual(packet['pitch_angle'], -56.37)
                self.assertEqual(packet['pitch_los_rate'], -1.23)

    def test_invalid_source_is_dropped_and_sender_recovers(self):
        self.start()
        self.forwarder.offer(b'{"type":"hello"}')
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(8192)
        self.assertEqual(self.forwarder.snapshot()['dropped'], 1)
        self.assertIn('转换失败', self.forwarder.snapshot()['error'])
        self.forwarder.offer(json.dumps(telemetry()).encode())
        self.assertEqual(json.loads(self.receive())['track_status'], 1)
        self.assertEqual(self.forwarder.snapshot()['error'], '')

    def test_vision_off_detect_search_or_error_overrides_residual_plugin_track(self):
        self.start()
        for vision in ({'mode': 'off', 'state': 'OFF'},
                       {'mode': 'detect', 'state': 'DETECTING'},
                       {'mode': 'auto_lock', 'state': 'AUTO_SEARCH'},
                       {'mode': 'auto_lock', 'state': 'AUTO_ACQUIRING', 'locked_track_id': 1},
                       {'mode': 'auto_lock', 'state': 'AUTO_LOCKED', 'locked_track_id': None},
                       {'mode': 'manual_track', 'error': '视觉错误'}, {}):
            source = telemetry()
            source['vision'] = vision
            self.forwarder.offer(json.dumps(source).encode())
            packet = json.loads(self.receive())
            with self.subTest(vision=vision):
                self.assertEqual(packet['track_status'], 0)
                self.assertEqual(packet['tracking_confidence'], 0.0)
                self.assertEqual(packet['pitch_misalignment'], 0.0)
                self.assertEqual(packet['yaw_misalignment'], 0.0)
                self.assertEqual(packet['target_confidence'], 0.8)
                self.assertEqual(packet['tracking_gateW'], 0.0)
                self.assertEqual(packet['tracking_gateH'], 0.0)

    def test_confirmed_auto_lock_and_manual_track_follow_actual_plugin_validity(self):
        for vision in ({'mode': 'manual_track', 'state': 'MANUAL_READY'},
                       {'mode': 'auto_lock', 'state': 'AUTO_LOCKED', 'locked_track_id': 2}):
            for state in ('INIT', 'TRACK', 'TEMP_LOST', 'LOST', 'IDLE', 'FAULT'):
                source = telemetry()
                source['vision'] = vision
                source['tracking']['state'] = state
                packet = aisheng_message(source)
                with self.subTest(vision=vision, state=state):
                    self.assertEqual(packet['track_status'], int(state == 'TRACK'))
                    self.assertEqual(packet['tracking_confidence'], 25.5 if state == 'TRACK' else 0.0)
                    self.assertEqual(packet['target_confidence'], 0.8)
                    self.assertEqual(packet['tracking_gateW'], 50.0 if state == 'TRACK' else 0.0)
                    self.assertEqual(packet['tracking_gateH'], 40.0 if state == 'TRACK' else 0.0)

    def test_rejects_old_protocol_missing_and_nonfinite_measurements(self):
        for path, value in ((('protocol_version',), 4), (('protocol_version',), 5),
                            (('protocol_version',), 6), (('angle_convention',), 'motion_head_joint'),
                            (('protocol_version',), True), (('angle_convention',), 'upright_joint'),
                            (('attitude_deg', 'pitch', 'pos'), None),
                            (('attitude_deg', 'pitch', 'vel'), float('nan')),
                            (('attitude_deg', 'yaw', 'pos'), float('inf')),
                            (('attitude_deg', 'yaw', 'pos'), 3.5e38),
                            (('target', 'miss', 'offset_deg'), [0]),
                            (('target', 'miss', 'offset_deg'), [True, 0]),
                            (('target', 'box_px'), None),
                            (('target', 'box_px'), [0, 0, 50]),
                            (('target', 'box_px'), [0, 0, 0, 40]),
                            (('target', 'box_px'), [0, 0, True, 40]),
                            (('target', 'box_px'), [0, 0, float('inf'), 40])):
            source = telemetry()
            node = source
            for key in path[:-1]:
                node = node[key]
            node[path[-1]] = value
            with self.subTest(path=path), self.assertRaises(ValueError):
                aisheng_message(source)

    def test_invalid_endpoint_is_rejected_before_socket_send(self):
        for ip, port in (("bad", "9000"), ("127.0.0.1", "0"), ("127.0.0.1", "65536"),
                         ("127.0.0.1", "1.5"), ("127.0.0.1", "９０００")):
            with self.subTest(ip=ip, port=port), self.assertRaises(ValueError):
                self.forwarder.start(ip, port)
        self.assertFalse(self.forwarder.snapshot()["active"])
        self.assertEqual(endpoint("::1", "9000"), ("::1", 9000))

    def test_destination_persistence_keeps_ini_comments_and_never_enables_sending(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "config.ini"
            path.write_text("# 原有配置说明\n[network]\nudp_port=8766\n\n"
                            "[forward]\n# 转发地址\nip = 127.0.0.1\nport = 9000\n"
                            "\n[session]\nreconnect = true\n", encoding="utf-8")
            save_destination(path, "127.0.0.2", "9010")
            text = path.read_text(encoding="utf-8")
            self.assertIn("# 原有配置说明", text)
            self.assertIn("# 转发地址", text)
            config = configparser.ConfigParser()
            config.read_string(text)
            self.assertEqual(config["forward"]["ip"], "127.0.0.2")
            self.assertEqual(config["forward"].getint("port"), 9010)
            self.assertNotIn("enabled", config["forward"])

    def test_editing_consumes_gimbal_hotkeys_and_escape_only_blurs(self):
        inputs = ForwardInputs("127.0.0.1", 9000)
        inputs.activate("ip")
        self.assertTrue(inputs.key(ord("h")))
        for char in "192.168.1.3":
            self.assertTrue(inputs.key(ord(char)))
        self.assertEqual(inputs.values["ip"], "192.168.1.3")
        self.assertTrue(inputs.key(9))
        for char in "9010":
            inputs.key(ord(char))
        self.assertEqual(inputs.values["port"], "9010")
        self.assertTrue(inputs.key(27))
        self.assertIsNone(inputs.focus)
        self.assertFalse(inputs.key(ord("h")))
        gui = station.GroundStation.__new__(station.GroundStation)
        gui.forward_inputs = inputs
        gui.forwarder = self.forwarder
        self.start()
        inputs.activate("port")
        self.assertTrue(gui._handle_forward_key(ord("h")))
        self.assertFalse(self.forwarder.snapshot()["active"])
        self.assertTrue(gui._handle_forward_key(ord("2")))
        self.assertEqual(inputs.values["port"], "2")
        self.assertTrue(gui._handle_forward_key(27))

    def test_clicking_address_pauses_send_and_edited_destination_is_used_after_send_click(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as new_receiver, tempfile.TemporaryDirectory() as folder:
            new_receiver.bind(('127.0.0.1', 0))
            new_receiver.settimeout(1)
            path = Path(folder) / 'station.ini'
            path.write_text('[forward]\nip = 127.0.0.1\nport = 9000\n', encoding='utf-8')
            gui = station.GroundStation.__new__(station.GroundStation)
            gui.display_size = (960, 540)
            gui.forwarder = self.forwarder
            gui.forward_inputs = ForwardInputs(*self.receiver.getsockname())
            gui._release_joystick = lambda: None
            gui.tele = SimpleNamespace(public_angles_ready=True, target=('127.0.0.1', 18766))
            gui.cfg = {}
            gui.args = SimpleNamespace(config=str(path))
            self.start()
            raw = json.dumps(telemetry()).encode()
            self.forwarder.offer(raw)
            self.receive()
            x, y, _, _ = gui._forward_geometry()['ip']
            gui._forward_mouse(station.cv2.EVENT_LBUTTONDOWN, x + 2, y + 2)
            gui._forward_mouse(station.cv2.EVENT_LBUTTONUP, x + 2, y + 2)
            self.assertFalse(self.forwarder.snapshot()['active'])
            self.assertEqual(gui.forward_inputs.focus, 'ip')
            for char in '127.0.0.1':
                gui._handle_forward_key(ord(char))
            gui._handle_forward_key(9)
            for char in str(new_receiver.getsockname()[1]):
                gui._handle_forward_key(ord(char))
            gui._handle_forward_key(13)
            self.forwarder.offer(raw)
            with self.assertRaises(socket.timeout):
                self.receiver.recvfrom(8192)
            self.assertEqual(gui.cfg, {})  # 编辑不提交配置，不发送半成品地址。
            x, y, _, _ = gui._forward_geometry()['toggle']
            gui._forward_mouse(station.cv2.EVENT_LBUTTONDOWN, x + 2, y + 2)
            gui._forward_mouse(station.cv2.EVENT_LBUTTONUP, x + 2, y + 2)
            self.assertTrue(self.forwarder.snapshot()['active'])
            self.forwarder.offer(raw)
            packet = json.loads(new_receiver.recvfrom(8192)[0])
            self.assertEqual(packet['target_confidence'], 0.8)
            self.assertEqual(packet['tracking_gateW'], 50.0)
            self.assertEqual(packet['tracking_gateH'], 40.0)
            self.assertEqual(gui.cfg['forward_port'], new_receiver.getsockname()[1])
            with self.assertRaises(socket.timeout):
                self.receiver.recvfrom(8192)

    def test_miss_display_uses_signed_right_down_coordinates_and_invalidates_stale(self):
        tele = {"target": {"valid": True, "miss": {"valid": True,
                "offset_px": [-20, 10], "offset_deg": [-.5, .25]}}}
        text = " ".join(station.GroundStation._miss_lines(tele, True))
        self.assertIn("X -20.0", text)
        self.assertIn("Y +10.0", text)
        self.assertIn("X -0.500", text)
        self.assertIn("右/下为正", text)
        self.assertIn("无效", station.GroundStation._miss_lines(tele, False)[0])
        tele["target"]["miss"]["valid"] = False
        self.assertIn("无效", station.GroundStation._miss_lines(tele, True)[0])
