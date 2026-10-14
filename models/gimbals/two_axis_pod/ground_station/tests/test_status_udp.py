"""额外状态上报的配置、真实 UDP 和 station 并发验证；不启动 Gazebo。"""
import contextlib
from collections import deque
import importlib.util
import io
import json
from pathlib import Path
import select
import socket
import sys
import tempfile
import threading
import time
import types
import unittest
from unittest.mock import Mock, patch

from test_protocol import BRIDGE


SERVICE = Path(__file__).resolve().parents[2] / "controller/service"
spec = importlib.util.spec_from_file_location("two_axis_status_udp", SERVICE / "status_udp.py")
status_udp = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = status_udp
spec.loader.exec_module(status_udp)
Config = status_udp.StatusUdpConfig
Publisher = status_udp.StatusUdpPublisher


class ConfigTests(unittest.TestCase):
    def test_default_disabled_and_twenty_hz(self):
        config = status_udp.load_status_config()
        self.assertFalse(config.enabled)
        self.assertEqual(config.hz, 20)

    def test_file_and_command_line_override(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "status.json"
            path.write_text(json.dumps(dict(enabled=True, ip="192.0.2.1", port=9999, hz=12)))
            config = status_udp.load_status_config(path)
            self.assertEqual((config.enabled, config.ip, config.port, config.hz),
                             (True, "192.0.2.1", 9999, 12))
            config = status_udp.load_status_config(path, ip="127.0.0.1", port=9001, hz=30)
            self.assertEqual((config.ip, config.port, config.hz), ("127.0.0.1", 9001, 30))
        self.assertTrue(status_udp.load_status_config(ip="::1", port=9000).enabled)

    def test_invalid_configuration_is_rejected(self):
        for values in ({"enabled": "true"}, {"ip": "bad-ip"}, {"port": True},
                       {"port": 0}, {"port": 65536}, {"hz": 0}, {"hz": 101},
                       {"hz": float("nan")}, {"hz": float("inf")},
                       {"hz": True}, {"stale_timeout_s": -1}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Config(**values)
        for args in ({"ip": "127.0.0.1"}, {"port": 9000}):
            with self.assertRaises(ValueError):
                status_udp.load_status_config(**args)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "status.json"
            for value in ([], {"unexpected": 1}, "invalid"):
                path.write_text(json.dumps(value) if value != "invalid" else "invalid")
                with self.assertRaises(ValueError):
                    status_udp.load_status_config(path)
            with self.assertRaises(ValueError):
                status_udp.load_status_config(Path(directory) / "missing.json")


class PublisherTests(unittest.TestCase):
    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(("127.0.0.1", 0))
        self.receiver.settimeout(.08)
        self.addCleanup(self.receiver.close)
        self.log = Mock()
        self.publisher = Publisher(Config(enabled=True, ip="127.0.0.1",
                                         port=self.receiver.getsockname()[1]), log=self.log)
        self.addCleanup(self.publisher.close)
        self.snapshot = {"type": "telemetry", "protocol_version": 4,
                         "angle_convention": "upright_joint", "mode": "IDLE",
                         "attitude_deg": {"yaw": {"pos": 12.3, "vel": 0, "acc": 0},
                                          "pitch": {"pos": 45, "vel": 0, "acc": 0}},
                         "tracking": {"state": "IDLE", "message": "测试状态"}}

    def receive(self):
        return json.loads(self.receiver.recvfrom(8192)[0].decode("utf-8"))

    def test_disabled_creates_no_socket_or_thread(self):
        with patch.object(status_udp.socket, "socket") as create:
            disabled = Publisher(Config())
            provider = Mock()
            disabled.start(provider)
            self.assertFalse(disabled.publish(self.snapshot, time.monotonic()))
            disabled.close()
        create.assert_not_called()
        provider.assert_not_called()

    def test_real_datagram_preserves_all_fields_without_registration(self):
        original = json.dumps(self.snapshot)
        self.assertTrue(self.publisher.publish(self.snapshot, time.monotonic()))
        self.assertEqual(self.receive(), self.snapshot)
        self.assertEqual(json.dumps(self.snapshot), original)

    def test_no_initial_or_stale_status_and_fresh_status_resumes(self):
        self.assertFalse(self.publisher.publish(None, None))
        self.assertFalse(self.publisher.publish(self.snapshot, time.monotonic() - 2))
        with self.assertRaises(socket.timeout):
            self.receive()
        self.assertTrue(self.publisher.publish(self.snapshot, time.monotonic()))
        self.assertEqual(self.receive(), self.snapshot)

    def test_bad_payloads_are_skipped_and_next_valid_packet_is_sent(self):
        self.assertFalse(self.publisher.publish({"invalid": float("nan")}, time.monotonic()))
        self.assertFalse(self.publisher.publish({"text": "中" * 4000}, time.monotonic()))
        self.assertEqual(self.log.call_count, 1)  # 错误日志限频，避免淹没 station 日志。
        self.assertTrue(self.publisher.publish(self.snapshot, time.monotonic()))
        self.assertEqual(self.receive(), self.snapshot)

    def test_send_failure_does_not_escape_or_prevent_later_sends(self):
        live_socket = self.publisher._socket
        failing = Mock()
        failing.sendto.side_effect = OSError("network unreachable")
        self.publisher._socket = failing
        self.assertFalse(self.publisher.publish(self.snapshot, time.monotonic()))
        self.publisher._socket = live_socket
        self.assertTrue(self.publisher.publish(self.snapshot, time.monotonic()))
        self.assertEqual(self.receive(), self.snapshot)

    def test_station_and_extra_output_have_independent_sockets_rates_and_sessions(self):
        station = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        station.bind(("127.0.0.1", 0))
        self.addCleanup(station.close)
        bridge = BRIDGE["GimbalBridge"].__new__(BRIDGE["GimbalBridge"])
        bridge.args = types.SimpleNamespace(telemetry_hz=50, client_timeout=5,
                                           mount="/two_axis_pod", udp_host="127.0.0.1", udp_port=0)
        bridge.lock = threading.RLock()
        bridge.clients_lock = threading.RLock()
        bridge.clients = {}
        bridge.retired_clients = {}
        bridge.pending_requests = {}
        bridge.clock_domain = "test-boot"
        bridge.frame_registry = deque()
        bridge.video = None
        bridge._latest_tel = self.snapshot
        bridge._latest_tel_received = time.monotonic()
        bridge._t_start = time.time()
        bridge._running = True
        bridge.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        bridge.sock.bind(("127.0.0.1", 0))
        bridge.sock.settimeout(.05)
        bridge.status_publisher = self.publisher
        self.assertIsNot(bridge.sock, self.publisher._socket)
        receive_thread = threading.Thread(target=bridge.udp_rx_loop, daemon=True)
        broadcast_thread = threading.Thread(target=bridge.telemetry_thread, daemon=True)
        counts = {"station": 0, "extra": 0}
        got_pong = False
        with contextlib.redirect_stdout(io.StringIO()):
            receive_thread.start()
            broadcast_thread.start()
            self.publisher.start(bridge.status_snapshot)
            # 接收端未登记也有额外状态；随后 station 正常 ping 登记。
            self.assertEqual(self.receive()["attitude_deg"], self.snapshot["attitude_deg"])
            self.assertFalse(bridge.clients)
            station.sendto(b'{"cmd":"ping","seq":7}', bridge.sock.getsockname())
            deadline = time.monotonic() + .8
            try:
                while time.monotonic() < deadline:
                    ready, _, _ = select.select([station, self.receiver], [], [], .05)
                    for sock in ready:
                        payload = json.loads(sock.recvfrom(8192)[0])
                        if sock is station and payload["type"] == "pong":
                            got_pong = payload["seq"] == 7
                        if payload["type"] == "telemetry":
                            counts["station" if sock is station else "extra"] += 1
                            self.assertEqual(payload["attitude_deg"], self.snapshot["attitude_deg"])
                self.assertTrue(got_pong)
                self.assertEqual(set(bridge.clients), {station.getsockname()})
                self.assertGreaterEqual(counts["station"], 25)
                self.assertGreaterEqual(counts["extra"], 10)
                self.assertLessEqual(counts["extra"], 22)
                self.assertGreater(counts["station"], counts["extra"] * 1.5)
                self.publisher.close()
                self.assertFalse(self.publisher._thread.is_alive())
                # 停止额外发送后，原 station 仍可请求 ping 并继续收到遥测。
                station.sendto(b'{"cmd":"ping","seq":8}', bridge.sock.getsockname())
                station.settimeout(.5)
                deadline = time.monotonic() + .5
                while time.monotonic() < deadline:
                    payload = json.loads(station.recvfrom(8192)[0])
                    if payload["type"] == "pong" and payload["seq"] == 8:
                        break
                else:
                    self.fail("额外发送停止后 station 连接受影响")
            finally:
                bridge.close()
                receive_thread.join(timeout=1)
                broadcast_thread.join(timeout=1)


if __name__ == "__main__":
    unittest.main()
