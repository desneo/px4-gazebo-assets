#!/usr/bin/env python3
"""客户端协议与同机时延计算验证，仅依赖 Python 标准库。"""
import ast
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "ground_station.py"
tree = ast.parse(SOURCE.read_text(encoding="utf-8"), filename=str(SOURCE))
namespace = {"threading": threading, "time": time, "json": json,
             "MAX_DATAGRAM": 8192,
             "VISION_MODES": ("off", "detect", "auto_lock", "manual_track")}
exec(compile(ast.Module(body=[node for node in tree.body
                             if isinstance(node, ast.ClassDef)
                             and node.name == "UdpClient"], type_ignores=[]),
             str(SOURCE), "exec"), namespace)
UdpClient = namespace["UdpClient"]


class ClientTests(unittest.TestCase):
    def setUp(self):
        self.client = UdpClient.__new__(UdpClient)
        self.client.lock = threading.RLock()
        self.client.target = ("127.0.0.1", 8766)
        self.client.clock_domain = "local-boot"
        self.client.last_rx_mono = 0.
        self.client.last_telemetry_mono = 0.
        self.client.client_timeout = 5.0
        self.client._seq = 0
        self.packets = []
        self.client.sock = SimpleNamespace(sendto=lambda data, target: self.packets.append(json.loads(data)))
        self.client.telemetry = {}
        self.client.frame_t0 = {}
        self.client.messages = self.client.pongs = 0
        self.client.last_result = self.client.hello = None
        self.client.results = []
        self.client.errors = []
        self.client._pending = {}
        self.client.on_log = lambda message: None

    def receive(self, msg, addr=None):
        self.client._on_datagram(json.dumps(msg).encode(),
                                 self.client.target if addr is None else addr)

    def test_only_configured_peer_can_refresh_liveness(self):
        for addr in [("127.0.0.2", 8766), ("127.0.0.1", 8765)]:
            self.receive({"type": "pong"}, addr)
        self.assertEqual(self.client.last_rx_mono, 0.)
        self.assertEqual(self.client.pongs, 0)
        self.receive({"type": "pong"})
        self.assertGreater(self.client.last_rx_mono, 0.)
        self.assertEqual(self.client.pongs, 1)

    def test_invalid_messages_cannot_keep_connection_alive(self):
        for data in [b"", b"garbage", b"\xff", b"[]", b"null", b"{}",
                     b'{"type":[]}', b" " * 8193]:
            self.client._on_datagram(data, self.client.target)
        for msg in [{"type": "unknown"}, {"type": "telemetry", "tracking": []},
                    {"type": "telemetry", "frame_table": {}},
                    {"type": "telemetry", "frame_table": [[1, float("nan")]]},
                    {"type": "telemetry", "frame_table": [[1, float("inf")]]},
                    {"type": "telemetry", "frame_table": [[[], 1]]},
                    {"type": "command_result", "seq": [], "cmd": "home", "ok": True},
                    {"type": "command_result", "seq": 1, "cmd": "home", "ok": "false"},
                    {"type": "hello", "aircraft_id": 1, "clock_domain": []}]:
            self.receive(msg)
        self.assertEqual(self.client.last_rx_mono, 0.)
        self.assertEqual(self.client.messages, 0)

    def test_completed_requests_are_removed_but_results_remain_queryable(self):
        for seq in range(100):
            self.client._pending[seq] = {"payload": {"cmd": "home"}, "result": None}
            self.receive({"type": "command_result", "seq": seq,
                          "cmd": "home", "ok": seq % 2 == 0})
            self.assertEqual(self.client.find_result(seq=seq)["ok"], seq % 2 == 0)
        self.assertEqual(self.client._pending, {})
        self.assertLessEqual(len(self.client.results), 50)

    def test_wrong_command_ack_does_not_clear_pending(self):
        self.client._pending[5] = {"payload": {"cmd": "home"}, "result": None}
        self.receive({"type": "command_result", "seq": 5, "cmd": "stop", "ok": True})
        self.assertIn(5, self.client._pending)

    def test_axis_reset_result_completes_its_reliable_request(self):
        self.client._pending[8] = {"payload": {"cmd": "reset_axis", "axis": "pitch"}, "result": None}
        self.receive({"type": "command_result", "seq": 8, "cmd": "reset_axis", "ok": True})
        self.assertNotIn(8, self.client._pending)
        self.assertTrue(self.client.find_result(cmd="reset_axis", seq=8)["ok"])

    def test_vision_mode_is_reliable_and_rejects_unknown_mode(self):
        self.confirm_public_angles()
        seq = self.client.send_reliable("set_vision_mode", mode="auto_lock")
        self.assertGreater(seq, 0)
        self.assertEqual(self.packets[-1], {"cmd": "set_vision_mode", "seq": seq,
                                            "mode": "auto_lock"})
        self.assertEqual(self.client.send_reliable("set_vision_mode", mode="invalid"), -1)
        self.assertEqual(len(self.packets), 1)

    def test_vision_telemetry_validation(self):
        good = {"type": "telemetry", "vision": {
            "mode": "detect", "state": "running", "target_count": 2,
            "locked_track_id": None, "inference_ms": 8.4,
            "input_fps": 30.0, "output_fps": 30.0, "error": ""}}
        self.receive(good)
        self.assertEqual(self.client.telemetry["vision"]["target_count"], 2)
        count = self.client.messages
        for vision in ({"mode": "bad"}, {"target_count": -1},
                       {"locked_track_id": "7"}, {"inference_ms": float("nan")},
                       {"error": []}):
            self.receive({"type": "telemetry", "vision": vision})
        self.assertEqual(self.client.messages, count)

    def confirm_public_angles(self):
        self.receive({"type": "hello", "aircraft_id": 1, "version": 6, "angle_convention": "motion_head_joint"})
        self.receive({"type": "telemetry", "protocol_version": 6, "angle_convention": "motion_head_joint", "attitude_deg": {
            axis: {"pos": 0.0, "vel": 0.0, "acc": 0.0} for axis in ("yaw", "pitch")}})

    def test_public_control_requires_both_version_and_valid_public_telemetry(self):
        self.client.send_manual(yaw=10.0)
        self.assertEqual(self.client.send_reliable("reset_axis", axis="yaw"), -1)
        self.assertFalse(self.packets)
        self.confirm_public_angles()
        self.assertTrue(self.client.public_angles_ready)
        self.client.send_manual(yaw=10.0, pitch=-5.0)
        self.client.send_reliable("reset_axis", axis="pitch")
        self.assertEqual([packet["cmd"] for packet in self.packets], ["manual", "reset_axis"])
        self.assertNotIn("roll", self.packets[0])

    def test_stale_or_downgraded_protocol_blocks_even_retransmission_but_allows_zero(self):
        self.confirm_public_angles()
        seq = self.client.send_reliable("reset_axis", axis="yaw")
        self.receive({"type": "hello", "aircraft_id": 1, "version": 1})
        self.assertFalse(self.client._raw_send(self.client._pending[seq]["payload"]))
        self.client.send_manual(yaw=10.0)
        self.client.send_manual()
        self.assertEqual(len(self.packets), 2)
        self.assertEqual(self.packets[-1]["yaw"], 0.0)
        self.confirm_public_angles()
        self.client.last_telemetry_mono -= 6.0
        self.assertFalse(self.client.public_angles_ready)
        self.client.send_manual(yaw=10.0)
        self.assertEqual(len(self.packets), 2)

    def test_old_or_missing_telemetry_version_cannot_authorize_motion_head(self):
        self.confirm_public_angles()
        for version in (4, 5, None, True):
            self.client.telemetry['protocol_version'] = version
            self.assertFalse(self.client.public_angles_ready)
            self.assertEqual(self.client.send_manual(pitch=-20), -1)
            self.assertEqual(self.client.send_reliable('reset_axis', axis='pitch'), -1)
        self.assertFalse(self.packets)

    def test_old_hello_or_joint_marker_blocks_all_motion_but_allows_stop(self):
        for version, convention in ((5, "upright_joint"),
                                    (6, "upright_joint"),
                                    (5, "motion_head_joint")):
            self.confirm_public_angles()
            self.receive({"type": "hello", "aircraft_id": 1,
                          "version": version, "angle_convention": convention})
            with self.subTest(version=version, convention=convention):
                self.assertFalse(self.client.public_angles_ready)
                for cmd, extra in (("home", {}), ("reset", {}),
                                   ("reset_axis", {"axis": "yaw"}),
                                   ("track", {"box": [0, 0, .2, .2]}),
                                   ("set_vision_mode", {"mode": "auto_lock"})):
                    self.assertEqual(self.client.send_reliable(cmd, **extra), -1)
                self.assertEqual(self.client.send_manual(yaw=10), -1)
                self.assertGreater(self.client.send_reliable("stop"), 0)
                self.assertGreater(self.client.send_manual(), 0)
                self.assertGreater(self.client.send_reliable("set_vision_mode", mode="off"), 0)
        self.assertTrue(all(packet["cmd"] in ("stop", "manual", "set_vision_mode")
                            for packet in self.packets))

    def test_v6_with_old_telemetry_marker_does_not_enable_motion(self):
        self.confirm_public_angles()
        self.client.telemetry["angle_convention"] = "upright_joint"
        self.assertFalse(self.client.public_angles_ready)
        self.assertEqual(self.client.send_reliable("home"), -1)
        self.assertEqual(self.client.send_manual(pitch=-10), -1)
        self.assertFalse(self.packets)

    def test_marker_alone_or_malformed_public_attitude_cannot_enable_control(self):
        self.receive({"type": "hello", "aircraft_id": 1, "version": 6, "angle_convention": "motion_head_joint"})
        for attitude in ({}, {axis: {"pos": 0., "vel": 0., "acc": float("nan")} for axis in ("yaw", "pitch")}):
            self.receive({"type": "telemetry", "angle_convention": "motion_head_joint", "attitude_deg": attitude})
            self.assertFalse(self.client.public_angles_ready)
        self.receive({"type": "telemetry", "angle_convention": "motion_head_joint"})
        self.assertFalse(self.client.public_angles_ready)

    def test_singular_view_keeps_joint_control_valid(self):
        self.confirm_public_angles()
        self.client.telemetry["singular"] = True
        self.client.send_manual(yaw=20.0, pitch=-20.0)
        self.client.send_reliable("reset_axis", axis="yaw")
        self.assertEqual(len(self.packets), 2)
        self.assertEqual(self.packets[0]["yaw"], 20.0)
        self.assertNotIn("roll", self.packets[0])

    def test_unconfirmed_identity_blocks_all_control_but_allows_queries(self):
        commands = [("manual", {"yaw": 0, "pitch": 0}), ("manual", {"yaw": 10}),
                    ("home", {}), ("reset", {}), ("stop", {}), ("reset_fault", {}),
                    ("reset_axis", {"axis": "yaw"}), ("track", {"box": [0, 0, .2, .2]})]
        for bad in (None, "1", True, 1.0, -1, 2):
            with self.subTest(aircraft_id=bad):
                self.receive({"type": "hello", "aircraft_id": bad})
                self.assertFalse(self.client.identity_ready)
                for cmd, extra in commands:
                    self.assertEqual(self.client.send_once(cmd, **extra), -1)
                    self.assertEqual(self.client.send_reliable(cmd, **extra), -1)
                    self.assertFalse(self.client._raw_send(dict(cmd=cmd, seq=10, **extra)))
                self.assertFalse(self.client._pending)
        self.assertFalse(self.packets)
        for cmd in ("ping", "status", "bye"):
            self.assertGreater(self.client.send_once(cmd), 0)
        self.assertEqual([p["cmd"] for p in self.packets], ["ping", "status", "bye"])

    def test_matching_identity_unlocks_expected_aircraft(self):
        self.client.aircraft_id = 23
        self.receive({"type": "hello", "aircraft_id": 1})
        self.assertIn("期望 23", self.client.identity_error)
        self.assertEqual(self.client.send_reliable("home"), -1)
        self.receive({"type": "hello", "aircraft_id": 23})
        self.assertTrue(self.client.identity_ready)
        self.assertEqual(self.client.send_reliable("home"), -1)
        self.receive({"type": "hello", "aircraft_id": 23, "version": 6,
                      "angle_convention": "motion_head_joint"})
        self.receive({"type": "telemetry", "protocol_version": 6,
                      "angle_convention": "motion_head_joint", "attitude_deg": {
                          axis: {"pos": 0., "vel": 0., "acc": 0.}
                          for axis in ("yaw", "pitch")}})
        self.assertGreater(self.client.send_reliable("home"), 0)
        self.assertEqual(self.packets[-1]["cmd"], "home")

    def test_changed_identity_cancels_pending_and_blocks_retries(self):
        self.confirm_public_angles()
        seq = self.client.send_reliable("home")
        old_payload = self.client._pending[seq]["payload"]
        self.receive({"type": "hello", "aircraft_id": 2})
        self.assertFalse(self.client._pending)
        self.assertFalse(self.client._raw_send(old_payload))
        self.assertEqual(self.client.send_manual(), -1)
        self.assertEqual(len(self.packets), 1)

    def test_timeout_cannot_be_reauthorized_by_pong_or_telemetry(self):
        self.confirm_public_angles()
        self.client.last_rx_mono -= 6
        self.assertEqual(self.client.send_reliable("stop"), -1)
        self.receive({"type": "pong"})
        self.assertTrue(self.client.connected)
        self.assertFalse(self.client.identity_ready)
        self.receive({"type": "telemetry", "aircraft_id": 1})
        self.assertEqual(self.client.send_reliable("home"), -1)
        self.receive({"type": "hello", "aircraft_id": 1})
        self.assertTrue(self.client.identity_ready)
        self.assertEqual(self.client.send_reliable("home"), -1)
        self.confirm_public_angles()
        self.assertGreater(self.client.send_reliable("home"), 0)

    def test_other_peer_cannot_unlock_identity(self):
        self.receive({"type": "hello", "aircraft_id": 1}, ("127.0.0.2", 8766))
        self.assertFalse(self.client.identity_ready)
        self.assertEqual(self.client.send_reliable("home"), -1)

    def test_delay_requires_matching_boot_domain_and_nonnegative_age(self):
        self.receive({"type": "telemetry", "frame_table": [[42, 10.]]})
        self.assertEqual(self.client.lookup(42), 10.)
        self.assertIsNone(self.client.frame_delay_ms(42, 10.1))
        for domain in ["other-boot", ""]:
            self.receive({"type": "hello", "aircraft_id": 1, "clock_domain": domain})
            self.assertIsNone(self.client.frame_delay_ms(42, 10.1))
        self.receive({"type": "hello", "aircraft_id": 1, "clock_domain": "local-boot"})
        self.assertAlmostEqual(self.client.frame_delay_ms(42, 10.1), 100.)
        self.assertIsNone(self.client.frame_delay_ms(42, 9.9))
        self.assertIsNone(self.client.frame_delay_ms(42, 13.))
        self.assertIsNone(self.client.frame_delay_ms(100, 10.1))
        self.client.clock_domain = None
        self.assertIsNone(self.client.frame_delay_ms(42, 10.1))


if __name__ == "__main__":
    unittest.main(verbosity=2)
