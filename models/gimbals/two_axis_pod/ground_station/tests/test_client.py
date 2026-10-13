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
             "MAX_DATAGRAM": 8192}
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
                    {"type": "hello", "clock_domain": []}]:
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

    def confirm_public_angles(self):
        self.receive({"type": "hello", "version": 4, "angle_convention": "upright_joint"})
        self.receive({"type": "telemetry", "angle_convention": "upright_joint", "attitude_deg": {
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
        self.receive({"type": "hello", "version": 1})
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

    def test_marker_alone_or_malformed_public_attitude_cannot_enable_control(self):
        self.receive({"type": "hello", "version": 4, "angle_convention": "upright_joint"})
        for attitude in ({}, {axis: {"pos": 0., "vel": 0., "acc": float("nan")} for axis in ("yaw", "pitch")}):
            self.receive({"type": "telemetry", "angle_convention": "upright_joint", "attitude_deg": attitude})
            self.assertFalse(self.client.public_angles_ready)
        self.receive({"type": "telemetry", "angle_convention": "upright_joint"})
        self.assertFalse(self.client.public_angles_ready)

    def test_singular_view_keeps_joint_control_valid(self):
        self.confirm_public_angles()
        self.client.telemetry["singular"] = True
        self.client.send_manual(yaw=20.0, pitch=-20.0)
        self.client.send_reliable("reset_axis", axis="yaw")
        self.assertEqual(len(self.packets), 2)
        self.assertEqual(self.packets[0]["yaw"], 20.0)
        self.assertNotIn("roll", self.packets[0])

    def test_delay_requires_matching_boot_domain_and_nonnegative_age(self):
        self.receive({"type": "telemetry", "frame_table": [[42, 10.]]})
        self.assertEqual(self.client.lookup(42), 10.)
        self.assertIsNone(self.client.frame_delay_ms(42, 10.1))
        for domain in ["other-boot", ""]:
            self.receive({"type": "hello", "clock_domain": domain})
            self.assertIsNone(self.client.frame_delay_ms(42, 10.1))
        self.receive({"type": "hello", "clock_domain": "local-boot"})
        self.assertAlmostEqual(self.client.frame_delay_ms(42, 10.1), 100.)
        self.assertIsNone(self.client.frame_delay_ms(42, 9.9))
        self.assertIsNone(self.client.frame_delay_ms(42, 13.))
        self.assertIsNone(self.client.frame_delay_ms(100, 10.1))
        self.client.clock_domain = None
        self.assertIsNone(self.client.frame_delay_ms(42, 10.1))


if __name__ == "__main__":
    unittest.main(verbosity=2)
