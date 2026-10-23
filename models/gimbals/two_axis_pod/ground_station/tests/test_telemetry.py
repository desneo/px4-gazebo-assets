#!/usr/bin/env python3
"""配置及新姿态驱动的遥测转发；不导入GUI、视频或Gazebo。"""
import contextlib
from collections import deque
import importlib.util
import io
import json
import math
from pathlib import Path
import socket
import tempfile
import threading
import time
import types
import unittest
from unittest import mock

from test_protocol import BRIDGE, CLIENT, ROOT


SERVICE = ROOT / "models/gimbals/two_axis_pod/controller/service"
SPEC = importlib.util.spec_from_file_location("pod_telemetry_config", SERVICE / "telemetry_config.py")
CONFIG = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CONFIG)


class TelemetryConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "telemetry.json"

    def load(self, value):
        self.path.write_text(json.dumps(value), encoding="utf-8")
        return CONFIG.load_telemetry_config(self.path)

    def test_default_and_numeric_frequency_boundaries(self):
        self.assertEqual(CONFIG.load_telemetry_config(), 50.0)
        for hz in (100, 50, .125, 1000):
            with self.subTest(hz=hz):
                self.assertEqual(self.load({"hz": hz}), float(hz))

    def test_invalid_frequency_and_object_types_raise_value_error(self):
        invalid = [None, True, False, "100", [], {}, 0, -1, 1000.01,
                   float("nan"), float("inf"), -float("inf"), 10**400]
        for hz in invalid:
            with self.subTest(hz=str(hz)):
                with self.assertRaises(ValueError):
                    self.load({"hz": hz})
        for value in ({}, [], [100], 100, "100", None):
            with self.subTest(root=value), self.assertRaises(ValueError):
                self.load(value)

    def test_unreadable_and_malformed_config_raise_value_error(self):
        with self.assertRaises(ValueError):
            CONFIG.load_telemetry_config(self.path)
        self.path.write_text("{broken", encoding="utf-8")
        with self.assertRaises(ValueError):
            CONFIG.load_telemetry_config(self.path)


class TelemetryForwardingTests(unittest.TestCase):
    def setUp(self):
        bridge_type = BRIDGE["GimbalBridge"]
        self.bridge = bridge_type.__new__(bridge_type)
        bridge = self.bridge
        bridge.args = types.SimpleNamespace(telemetry_hz=100., client_timeout=60.,
                                           aircraft_id=7, mount="/two_axis_pod_7")
        bridge.lock = threading.RLock()
        bridge._telemetry_ready = threading.Condition(bridge.lock)
        bridge._telemetry_pending = deque(maxlen=1000)
        bridge._last_queued_stamp = None
        bridge._last_telemetry_warning = -math.inf
        bridge._latest_tel = None
        bridge._plugin_protocol_compatible = False
        bridge._latest_tel_received = None
        bridge._running = True
        bridge._t_start = time.time()
        bridge.frame_registry = deque(maxlen=600)
        bridge.video = None
        bridge._vision_mode = "off"
        bridge._vision_state = "OFF"
        bridge._vision_target_count = 0
        bridge._locked_track_id = None
        bridge._vision_error = None
        bridge._vision_metrics = {}
        bridge._vision_times = deque(maxlen=120)
        bridge._vision = None
        bridge._last_tracking_stop_at = -math.inf
        bridge.clients_lock = threading.RLock()
        bridge.retired_clients = {}
        bridge.pending_requests = {}
        bridge.clock_domain = "test-clock"
        bridge.sock = mock.Mock()
        bridge._cmd_pub = mock.Mock()
        self.client = BRIDGE["_Client"](("127.0.0.1", 12345))
        bridge.clients = {self.client.addr: self.client}
        self.sent = []
        self.sent_ready = threading.Condition()
        self.threads = []
        bridge._send_json = self.record_send

    def tearDown(self):
        self.bridge.close()
        for thread in self.threads:
            thread.join(1.)
            self.assertFalse(thread.is_alive(), "遥测发送线程未退出")

    def record_send(self, addr, payload):
        value = json.loads(payload) if isinstance(payload, str) else dict(payload)
        with self.sent_ready:
            self.sent.append((addr, value))
            self.sent_ready.notify_all()
        return True

    def telemetry(self):
        with self.sent_ready:
            return [value for _, value in self.sent if value.get("type") == "telemetry"]

    def wait_for_count(self, expected):
        with self.sent_ready:
            complete = self.sent_ready.wait_for(
                lambda: sum(value.get("type") == "telemetry" for _, value in self.sent) >= expected,
                timeout=2.)
        self.assertTrue(complete, f"未收到{expected}份姿态，实际{len(self.telemetry())}")

    def start_sender(self):
        thread = threading.Thread(target=self.bridge.telemetry_thread, daemon=True)
        self.threads.append(thread)
        thread.start()
        return thread

    def produce(self, stamp, hz=None, **extra):
        payload = {"protocol_version": 7, "angle_convention": "motion_head_joint_right_positive", "mode": "IDLE", "ts_mono": stamp,
                   "telemetry_hz": self.bridge.args.telemetry_hz if hz is None else hz,
                   "telemetry_clock": "steady",
                   "yaw": stamp, "pitch": -stamp if isinstance(stamp, (int, float)) else 0,
                   **extra}
        self.bridge._on_telemetry(types.SimpleNamespace(data=json.dumps(payload)))

    def test_100_and_50_hz_production_forwards_each_new_attitude_once(self):
        self.start_sender()
        expected = []
        for hz in (100., 50.):
            self.bridge.args.telemetry_hz = hz
            started = time.monotonic()
            for index in range(24):
                stamp = hz + index/hz
                expected.append(stamp)
                self.produce(stamp)
                time.sleep(max(0., started+(index+1)/hz-time.monotonic()))
            self.wait_for_count(len(expected))
        samples = self.telemetry()
        self.assertEqual([sample["ts_mono"] for sample in samples], expected)
        self.assertEqual(len({sample["ts_mono"] for sample in samples}), len(expected))
        self.assertEqual([sample["yaw"] for sample in samples], expected)
        self.assertTrue(all(sample["aircraft_id"] == 7 for sample in samples))
        self.assertTrue(all(sample["video_rtsp_path"] == "/two_axis_pod_7" for sample in samples))

    def test_queued_samples_keep_their_own_attitude_and_frame_table(self):
        for index in range(12):
            self.bridge.frame_registry.append((index, index/30.))
            self.produce(float(index))
        self.bridge._latest_tel = {"mode": "IDLE", "ts_mono": 999., "yaw": 999.}
        self.bridge.frame_registry.clear()
        self.start_sender()
        self.wait_for_count(12)
        samples = self.telemetry()
        self.assertEqual([sample["ts_mono"] for sample in samples], list(range(12)))
        for index, sample in enumerate(samples):
            self.assertEqual(sample["yaw"], index)
            self.assertEqual(len(sample["frame_table"]), index+1)
            self.assertEqual(sample["frame_table"][-1][0], index)

    def test_pause_does_not_repeat_old_pose_and_clock_reset_resumes(self):
        self.start_sender()
        for stamp in (20., 20.01, 20.02):
            self.produce(stamp)
        self.wait_for_count(3)
        time.sleep(.26)  # 超过一次空队列维护周期，旧姿态仍不得重发。
        self.assertEqual(len(self.telemetry()), 3)
        for stamp in (0., .01):
            self.produce(stamp)
        self.wait_for_count(5)
        self.assertEqual([sample["ts_mono"] for sample in self.telemetry()],
                         [20., 20.01, 20.02, 0., .01])

    def test_repeated_stamp_and_invalid_timestamps_do_not_make_new_samples(self):
        self.produce(1., yaw=10.)
        self.produce(1., yaw=99.)
        for stamp in (None, True, False, "2", [], float("nan"), float("inf"), 10**400):
            with self.subTest(stamp=str(stamp)):
                self.produce(stamp)
        self.produce(2., yaw=20.)
        self.start_sender()
        self.wait_for_count(2)
        self.assertEqual([(s["ts_mono"], s["yaw"]) for s in self.telemetry()],
                         [(1., 10.), (2., 20.)])

    def test_mismatched_frequency_is_not_forwarded_and_warning_is_rate_limited(self):
        captured = io.StringIO()
        with contextlib.redirect_stdout(captured):
            for now in (10., 10.1, 10.9, 11.01):
                with mock.patch.object(time, "monotonic", return_value=now):
                    self.produce(now, hz=50.)
            for hz in (True, "100", [], float("nan"), float("inf"), 10**400):
                with self.subTest(hz=str(hz)), mock.patch.object(time, "monotonic", return_value=11.1):
                    self.produce(12., hz=hz)
        self.assertEqual(captured.getvalue().count("插件频率"), 2)
        self.assertEqual(len(self.bridge._telemetry_pending), 0)
        self.bridge._broadcast_telemetry()
        self.assertEqual(self.telemetry(), [])
        self.produce(13.)
        self.start_sender()
        self.wait_for_count(1)
        self.assertEqual(self.telemetry()[0]["ts_mono"], 13.)

    def test_missing_or_simulation_clock_is_not_forwarded(self):
        with contextlib.redirect_stdout(io.StringIO()):
            for index, clock in enumerate((None, "simulation", "wall", True)):
                self.produce(float(index), telemetry_clock=clock)
        self.assertEqual(len(self.bridge._telemetry_pending), 0)
        self.produce(10., telemetry_clock="steady")
        self.start_sender()
        self.wait_for_count(1)
        self.assertEqual(self.telemetry()[0]["telemetry_clock"], "steady")

    def test_full_queue_discards_oldest_and_drains_retained_samples_once(self):
        with contextlib.redirect_stdout(io.StringIO()):
            for index in range(1017):
                self.produce(float(index))
        self.assertEqual(len(self.bridge._telemetry_pending), 1000)
        self.assertEqual(self.bridge._telemetry_pending[0]["ts_mono"], 17.)
        self.assertEqual(self.bridge._telemetry_pending[-1]["ts_mono"], 1016.)
        self.start_sender()
        self.wait_for_count(1000)
        self.assertEqual([sample["ts_mono"] for sample in self.telemetry()],
                         list(range(17, 1017)))
        self.assertEqual(len(self.bridge._telemetry_pending), 0)

    def test_old_plugin_never_claims_v7_or_sends_old_attitude(self):
        for version, convention in ((6, "motion_head_joint"),
                                    (6, "motion_head_joint_right_positive"),
                                    (7, "motion_head_joint"), (True, "motion_head_joint_right_positive")):
            with self.subTest(version=version, convention=convention):
                self.produce(1., protocol_version=version, angle_convention=convention)
                self.bridge._send_hello(self.client.addr)
                hello = self.sent[-1][1]
                self.assertIsNone(hello["version"])
                self.assertIsNone(hello["angle_convention"])
                self.assertFalse(hello["plugin_compatible"])
                self.assertIsNone(self.bridge.snapshot())
                self.assertEqual(len(self.bridge._telemetry_pending), 0)

    def test_plugin_upgrade_and_downgrade_refresh_registered_clients(self):
        self.produce(1.)
        self.assertEqual(self.sent[-1][1]["version"], 7)
        self.assertEqual(len(self.bridge._telemetry_pending), 1)
        old_snapshot = self.bridge._telemetry_pending[0]
        self.produce(2., protocol_version=6, angle_convention="motion_head_joint")
        self.assertIsNone(self.sent[-1][1]["version"])
        self.assertEqual(len(self.bridge._telemetry_pending), 0)
        self.bridge._broadcast_telemetry(old_snapshot)
        self.assertEqual(self.telemetry(), [])
        self.produce(3.)
        self.assertEqual(self.sent[-1][1]["version"], 7)
        self.assertTrue(self.sent[-1][1]["plugin_compatible"])
        self.start_sender()
        self.wait_for_count(1)
        self.assertEqual(self.telemetry()[0]["ts_mono"], 3.)

    def test_udp_ping_recovers_lost_upgrade_hello_without_reusing_downgraded_pose(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        server.bind(("127.0.0.1", 0))
        server.settimeout(1.)
        self.bridge.sock = server
        self.bridge.clients.clear()
        station = CLIENT["UdpClient"](*server.getsockname(), aircraft_id=7)
        self.assertIsNone(station.bind_error)
        station.sock.settimeout(1.)
        self.addCleanup(station.sock.close)
        drop_upgrade = True

        def send(addr, payload):
            nonlocal drop_upgrade
            value = json.loads(payload) if isinstance(payload, str) else payload
            if value.get("type") == "hello" and value.get("version") == 7 and drop_upgrade:
                drop_upgrade = False  # 刻意丢失首次升级通知，模拟 UDP 单包丢失。
                return True
            server.sendto(json.dumps(value).encode(), addr)
            return True

        self.bridge._send_json = send

        def receive_station():
            data, addr = station.sock.recvfrom(65535)
            station._on_datagram(data, addr)

        def ping():
            self.assertGreater(station.send_once("ping", t=123.5), 0)
            data, addr = server.recvfrom(65535)
            self.bridge._on_datagram(data, addr)
            receive_station()  # 当前 hello
            receive_station()  # pong

        def pose(stamp, **extra):
            attitude = {axis: {"pos": 0., "vel": 0., "acc": 0.} for axis in ("yaw", "pitch")}
            self.produce(stamp, attitude_deg=attitude, **extra)

        ping()
        self.assertIsNone(station.hello["version"])
        pose(1.)  # 唯一升级 hello 丢失，v7 姿态自身不能代替 hello 授权。
        self.assertFalse(drop_upgrade)
        self.bridge._broadcast_telemetry(self.bridge._telemetry_pending.popleft())
        receive_station()
        self.assertFalse(station.public_angles_ready)
        ping()  # 已登记客户端仍能在下次保活取得当前 v7 hello。
        self.assertEqual(station.hello["version"], 7)
        self.assertTrue(station.public_angles_ready)
        self.assertGreater(station.send_manual(yaw=10.), 0)
        data, addr = server.recvfrom(65535)
        self.bridge._on_datagram(data, addr)
        published = json.loads(self.bridge._cmd_pub.publish.call_args.args[0].data)
        self.assertEqual((published["cmd"], published["yaw"]), ("manual", 10.))

        pose(2., protocol_version=6, angle_convention="motion_head_joint")
        receive_station()
        for _ in range(2):
            ping()
            self.assertIsNone(station.hello["version"])
            self.assertFalse(station.public_angles_ready)
            self.assertEqual(station.telemetry, {})
        pose(3.)
        receive_station()  # 恢复 hello 时必须等待降级后的新姿态。
        self.assertFalse(station.public_angles_ready)
        self.bridge._broadcast_telemetry(self.bridge._telemetry_pending.popleft())
        receive_station()
        self.assertTrue(station.public_angles_ready)

    def test_incompatible_plugin_rejects_motion_but_keeps_stop_available(self):
        commands = [("manual", {"yaw": 1}), ("manual", {"pitch": -1}),
                    ("home", {}), ("reset", {}), ("reset_axis", {"axis": "yaw"}),
                    ("track", {"box": [0, 0, .2, .2]}),
                    ("set_vision_mode", {"mode": "auto_lock"}),
                    ("set_vision_mode", {"mode": "manual_track"})]
        for seq, (cmd, extra) in enumerate(commands, 1):
            with self.subTest(cmd=cmd, extra=extra):
                self.bridge._handle_command(self.client, dict(cmd=cmd, seq=seq, **extra))
                self.assertFalse(self.sent[-1][1]["ok"])
                self.assertIn("插件尚未确认协议 7", self.sent[-1][1]["message"])
        self.bridge._cmd_pub.publish.assert_not_called()
        self.assertFalse(self.bridge._publish_internal("track", box=[0, 0, .2, .2]))
        self.bridge._handle_command(self.client, {"cmd": "manual", "seq": 20, "yaw": 0, "pitch": 0})
        self.bridge._handle_command(self.client, {"cmd": "stop", "seq": 21})
        published = [json.loads(call.args[0].data) for call in self.bridge._cmd_pub.publish.call_args_list]
        self.assertEqual([value["cmd"] for value in published], ["manual", "stop"])

    def test_idle_maintenance_expires_clients_without_sampling_stale_pose(self):
        self.bridge.snapshot = mock.Mock(side_effect=AssertionError("空队列不得采旧姿态"))
        self.client.last_seen = time.monotonic()-61.
        self.bridge._broadcast_telemetry()
        self.bridge.snapshot.assert_not_called()
        self.assertNotIn(self.client.addr, self.bridge.clients)
        self.assertEqual(self.sent, [])

    def test_ping_is_served_while_telemetry_sender_is_blocked(self):
        entered = threading.Event()
        release = threading.Event()
        pong = threading.Event()

        def send(addr, payload):
            value = json.loads(payload) if isinstance(payload, str) else payload
            if value.get("type") == "telemetry":
                entered.set()
                release.wait(2.)
            elif value.get("type") == "pong":
                pong.set()
            return self.record_send(addr, payload)

        self.bridge._send_json = send
        self.addCleanup(release.set)
        self.start_sender()
        self.produce(1.)
        self.assertTrue(entered.wait(1.))
        command = threading.Thread(target=self.bridge._on_datagram,
                                   args=(b'{"cmd":"ping","seq":5,"t":123.5}', self.client.addr))
        command.start()
        try:
            self.assertTrue(pong.wait(.5), "ping被遥测发送阻塞")
            self.assertFalse(release.is_set())
        finally:
            release.set()
            command.join(1.)
        self.assertFalse(command.is_alive())
        self.bridge._cmd_pub.publish.assert_not_called()
        replies = [value for _, value in self.sent if value.get("type") == "pong"]
        self.assertEqual(replies, [{"type": "pong", "seq": 5, "t": 123.5}])

    def test_close_wakes_sender_waiting_without_a_sample(self):
        waiting = threading.Event()

        class ObservedCondition(threading.Condition):
            def wait(self, timeout=None):
                waiting.set()
                return super().wait(timeout)

        self.bridge._telemetry_ready = ObservedCondition(self.bridge.lock)
        thread = self.start_sender()
        self.assertTrue(waiting.wait(1.))
        self.bridge.close()
        thread.join(.1)
        self.assertFalse(thread.is_alive(), "close没有唤醒等待中的发送线程")
        self.assertEqual(self.sent, [])


if __name__ == "__main__":
    unittest.main(verbosity=2)
