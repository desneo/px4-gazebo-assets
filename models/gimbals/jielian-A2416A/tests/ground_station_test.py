#!/usr/bin/env python3
"""A2416A地面站通信、交互和艾晟桥兼容JSON测试。"""
from __future__ import annotations

import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common import protocol as proto
from ground_station.client import TelemetryClient, TelemetrySample
from ground_station.configuration import load, with_forward_override
from ground_station.controls import box_to_source, key_movement, view_to_source
from ground_station.forward import (
    AISHENG_TELEMETRY_FIELDS,
    aisheng_telemetry_message,
)
from ground_station.main import (
    BUTTONS,
    FORWARD_START_LABEL,
    GroundStation,
    button_layout,
    parse_args,
)
from ground_station import lifecycle


class FakeSocket:
    def __init__(self, *_args):
        self.sent = []
        self.bound = None
        self.peer = None

    def bind(self, address):
        self.bound = address

    def connect(self, peer):
        self.peer = peer

    def settimeout(self, _timeout):
        pass

    def setblocking(self, _blocking):
        pass

    def send(self, raw):
        self.sent.append(bytes(raw))
        return len(raw)

    def sendto(self, raw, target):
        self.sent.append((bytes(raw), target))
        return len(raw)

    def recv(self, _size):
        raise socket.timeout

    def close(self):
        pass


class GroundStationTest(unittest.TestCase):
    def test_configuration_fixes_product_video_contract(self):
        config = load(ROOT / "ground_station" / "config.ini")
        self.assertEqual(config["network"], {"host": "127.0.0.1", "port": 8001})
        self.assertEqual(config["forward"], {"ip": "127.0.0.1", "port": 9000})
        self.assertEqual(config["video"]["width"], 1920)
        self.assertEqual((config["video"]["height"], config["video"]["fps"]), (1080, 25))

    def test_forward_cli_override_and_ui_contract(self):
        config = load(ROOT / "ground_station" / "config.ini")
        overridden = with_forward_override(config, "127.0.0.2", 9012)
        self.assertEqual(overridden["forward"], {"ip": "127.0.0.2", "port": 9012})
        self.assertEqual(config["forward"], {"ip": "127.0.0.1", "port": 9000})
        args = parse_args(["--forward-host", "127.0.0.3", "--forward-port", "9013"])
        self.assertEqual((args.forward_host, args.forward_port), ("127.0.0.3", 9013))
        self.assertEqual(FORWARD_START_LABEL, "发送艾晟报文")

        station = object.__new__(GroundStation)
        station.forward_focus = "port"
        station.forward_selected = True
        station.forward_inputs = {"ip": "127.0.0.1", "port": "9000"}
        self.assertTrue(station._handle_forward_key(ord("9")))
        self.assertEqual(station.forward_inputs["port"], "9")

    def test_visible_buttons_exclude_enhancement_and_osd(self):
        expected_actions = {
            "recognition_on", "recognition_off", "auto_lock_on", "auto_lock_off",
            "cancel", "zoom_in", "zoom_out",
        }
        self.assertEqual({action for _label, action in BUTTONS}, expected_actions)
        self.assertFalse(any("增强" in label or "OSD" in label for label, _action in BUTTONS))
        layout = button_layout(960, 540)
        self.assertEqual(len(layout), 7)
        self.assertEqual(len({rect[1] for _label, _action, rect in layout}), 1)

    def test_auto_lock_button_enables_recognition_before_auto_lock(self):
        made = []

        def factory(*args):
            sock = FakeSocket(*args)
            made.append(sock)
            return sock

        station = object.__new__(GroundStation)
        station.client = TelemetryClient(
            "127.0.0.1", 8001, socket_factory=factory, start=False)
        station.zoom = 1
        station.recognition_enabled = False
        station.auto_lock_enabled = False
        station.last_action = ""
        try:
            station._send_action("auto_lock_on")
        finally:
            station.client.close()
        commands = [raw[3] for raw in made[0].sent]
        self.assertEqual(commands, [0x3F] * 3 + [0x5F] * 3)
        self.assertTrue(station.recognition_enabled)
        self.assertTrue(station.auto_lock_enabled)
        self.assertEqual(station.last_action, "已发送：自动锁定")

    def test_discrete_commands_repeat_three_times_but_heartbeat_and_move_do_not(self):
        made = []

        def factory(*args):
            sock = FakeSocket(*args)
            made.append(sock)
            return sock

        client = TelemetryClient("127.0.0.1", 8001, socket_factory=factory, start=False)
        self.assertEqual(client.send(0xB7, p1=100, p2=200), 3)
        self.assertEqual(client.send(0x00), 1)
        self.assertEqual(client.send(0xB1, p1=100, p2=0), 1)
        self.assertEqual(len(made[0].sent), 5)
        for raw in made[0].sent:
            self.assertEqual(len(raw), 45)
        client.close()

    def test_heartbeat_is_ten_hz(self):
        made = []

        def factory(*args):
            sock = FakeSocket(*args)
            made.append(sock)
            return sock

        client = TelemetryClient("127.0.0.1", 8001, socket_factory=factory)
        time.sleep(0.36)
        client.close()
        heartbeats = [raw for raw in made[0].sent if raw[3] == 0x00]
        self.assertGreaterEqual(len(heartbeats), 3)
        self.assertLessEqual(len(heartbeats), 5)

    def test_point_and_box_coordinates_map_through_digital_zoom(self):
        self.assertEqual(view_to_source(0, 0, 960, 540, 1), (0, 0))
        self.assertEqual(view_to_source(480, 270, 960, 540, 4), (960, 540))
        box = box_to_source((0, 0), (959, 539), 960, 540, 2)
        self.assertEqual((box.x, box.y), (480, 270))
        self.assertLessEqual(box.x + box.width, 1440)
        self.assertLessEqual(box.y + box.height, 810)

    def test_crosshair_key_sends_nonzero_lease_without_immediate_zero(self):
        self.assertEqual(key_movement(ord("a")), (-100, 0))
        self.assertEqual(key_movement(ord("w")), (0, 100))
        self.assertIsNone(key_movement(ord("x")))

    def test_aisheng_json_is_exact_top_level_37_field_contract(self):
        expected = {
            "timestamp", "self_check", "work_status", "track_status",
            "pitch_angle", "yaw_angle", "gate_x", "gate_y",
            "gyro_pitch_rate", "gyro_yaw_rate", "pitch_misalignment",
            "yaw_misalignment", "tracking_confidence", "pitch_los_rate",
            "yaw_los_rate", "status", "autonomous", "target_type",
            "target_confidence", "target_lon", "target_lat", "target_alt",
            "status2", "tracking_gate_w", "tracking_gate_h", "pitch_attitude",
            "yaw_attitude", "roll_attitude", "tracking_target_type", "laser_range",
            "laser_update", "aircraft_pitch", "aircraft_roll", "aircraft_yaw",
            "ip_segment", "ip_node", "device_timestamp",
        }
        frame = proto.TelemetryFrame(
            pitch_misalignment_deg=-1.25,
            yaw_misalignment_deg=2.5,
            timestamp_ms=60,
        )
        raw = proto.encode_telemetry(frame)
        first = TelemetrySample(10, time.monotonic(), 123456, raw, proto.decode_telemetry(raw))
        second = TelemetrySample(11, time.monotonic(), 123466, raw, proto.decode_telemetry(raw))
        a, b = aisheng_telemetry_message(first), aisheng_telemetry_message(second)
        self.assertEqual(set(AISHENG_TELEMETRY_FIELDS), expected)
        self.assertEqual(tuple(a), AISHENG_TELEMETRY_FIELDS)
        self.assertEqual(len(a), 37)
        self.assertEqual(set(a), expected)
        self.assertNotIn("frame_header", a)
        self.assertNotIn("fields", a)
        self.assertNotIn("measurement_seq", a)
        self.assertEqual(a["yaw_misalignment"], 2.5)
        self.assertEqual((a["timestamp"], b["timestamp"]), (123456, 123466))
        self.assertEqual(a["device_timestamp"], b["device_timestamp"])
        json.dumps(a, allow_nan=False)

    def test_aisheng_json_rejects_invalid_wire_samples(self):
        raw = proto.encode_telemetry(proto.TelemetryFrame())
        short = TelemetrySample(1, time.monotonic(), 1000, raw[:-1], object())
        with self.assertRaises(ValueError):
            aisheng_telemetry_message(short)
        corrupt = bytearray(raw)
        corrupt[-1] ^= 0x01
        bad = TelemetrySample(2, time.monotonic(), 1010, bytes(corrupt), object())
        with self.assertRaises(ValueError):
            aisheng_telemetry_message(bad)

    def test_forwarder_sends_each_received_sample_even_when_measurement_repeats(self):
        from ground_station.forward import TelemetryForwarder
        made = []

        def factory(*args):
            sock = FakeSocket(*args)
            made.append(sock)
            return sock

        forwarder = TelemetryForwarder(socket_factory=factory)
        forwarder.start("127.0.0.1", 9000)
        raw = proto.encode_telemetry(proto.TelemetryFrame(timestamp_ms=80))
        now = time.monotonic()
        frame = proto.decode_telemetry(raw)
        forwarder.offer(TelemetrySample(1, now, 1000, raw, frame))
        forwarder.offer(TelemetrySample(2, now, 1010, raw, frame))
        deadline = time.monotonic() + 1.0
        while forwarder.snapshot()["sent"] < 2 and time.monotonic() < deadline:
            time.sleep(0.001)
        messages = [json.loads(item[0]) for item in made[0].sent]
        forwarder.close()
        self.assertEqual(len(messages), 2)
        self.assertEqual([item["timestamp"] for item in messages], [1000, 1010])
        self.assertEqual([item["device_timestamp"] for item in messages], [80, 80])
        self.assertTrue(all(set(item) == set(AISHENG_TELEMETRY_FIELDS) for item in messages))

    def test_invalid_forward_endpoint_rejected(self):
        from ground_station.forward import endpoint
        with self.assertRaises(ValueError):
            endpoint("not-an-ip", 9000)
        with self.assertRaises(ValueError):
            endpoint("127.0.0.1", 0)


class LifecycleTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name) / "jielian-A2416A"
        self.runtime = Path(self.temporary.name) / "runtime"
        (self.root / "ground_station").mkdir(parents=True)
        self.runtime.mkdir(mode=0o700)
        (self.root / "ground_station/__init__.py").write_text("", encoding="utf-8")
        (self.root / "ground_station/main.py").write_text(
            "import time\ntime.sleep(60)\n", encoding="utf-8")
        self.configs = [self.root / "one.ini", self.root / "two.ini"]
        for config in self.configs:
            config.write_text("[test]\n", encoding="utf-8")
        self.children = []
        self.patches = [
            patch.object(lifecycle, "ROOT", self.root),
            patch.object(lifecycle, "runtime_dir", return_value=self.runtime),
        ]
        for item in self.patches:
            item.start()

    def tearDown(self):
        for child in self.children:
            if child.poll() is None:
                child.terminate()
            child.wait(timeout=3)
        for item in reversed(self.patches):
            item.stop()
        self.temporary.cleanup()

    def managed_child(self, config: Path, token: str):
        environment = os.environ.copy()
        environment[lifecycle.TOKEN_ENV] = token
        child = subprocess.Popen(
            [sys.executable, "-m", lifecycle.MAIN_MODULE, "--config", str(config)],
            cwd=self.root, env=environment, start_new_session=True,
        )
        self.children.append(child)
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            try:
                ticks = lifecycle.process_start_ticks(child.pid)
                break
            except OSError:
                time.sleep(0.01)
        else:
            self.fail("测试地面站进程未启动")
        record = {
            "pid": child.pid,
            "start_ticks": ticks,
            "uid": os.getuid(),
            "executable": Path(sys.executable).resolve(),
            "module": lifecycle.MAIN_MODULE,
            "root": self.root,
            "config": config,
            "token": token,
            "started_unix_ns": time.time_ns(),
        }
        lifecycle.write_state(lifecycle.state_path(config), record)
        return child, record

    def test_selective_stop_only_signals_exact_config_instance(self):
        first, _ = self.managed_child(self.configs[0], "first-token")
        second, _ = self.managed_child(self.configs[1], "second-token")
        self.assertEqual(lifecycle.stop(self.configs[0], dry_run=False), 0)
        first.wait(timeout=3)
        self.assertIsNone(second.poll())

    def test_same_config_lock_rejects_duplicate_instance(self):
        first = lifecycle._open_lock(self.configs[0])
        self.addCleanup(first.close)
        with self.assertRaises(BlockingIOError):
            lifecycle._open_lock(self.configs[0])

    def test_run_rejects_matching_unregistered_legacy_instance(self):
        environment = os.environ.copy()
        environment.pop(lifecycle.TOKEN_ENV, None)
        child = subprocess.Popen(
            [sys.executable, "-m", lifecycle.MAIN_MODULE,
             "--config", str(self.configs[0])],
            cwd=self.root, env=environment, start_new_session=True,
        )
        self.children.append(child)
        self.assertEqual(lifecycle.run(["--config", str(self.configs[0])]), 2)
        self.assertIsNone(child.poll())

    def test_dry_run_and_all_instances_are_multi_instance_safe(self):
        first, _ = self.managed_child(self.configs[0], "first-token")
        second, _ = self.managed_child(self.configs[1], "second-token")
        self.assertEqual(lifecycle.stop(None, dry_run=True), 0)
        self.assertIsNone(first.poll())
        self.assertIsNone(second.poll())
        self.assertEqual(lifecycle.stop(None, dry_run=False), 0)
        first.wait(timeout=3)
        second.wait(timeout=3)

    def test_signal_failure_returns_nonzero_without_killing_process(self):
        child, _ = self.managed_child(self.configs[0], "first-token")
        with patch.object(lifecycle.signal, "pidfd_send_signal",
                          side_effect=PermissionError("denied")):
            self.assertEqual(lifecycle.stop(self.configs[0], dry_run=False), 2)
        self.assertIsNone(child.poll())

    def test_reused_pid_and_wrong_token_are_not_stopped(self):
        child, record = self.managed_child(self.configs[0], "real-token")
        record["start_ticks"] = int(record["start_ticks"]) + 1
        lifecycle.write_state(lifecycle.state_path(self.configs[0]), record)
        self.assertFalse(lifecycle.is_managed_process(record))
        self.assertFalse(lifecycle.signal_verified_process(record, lifecycle.signal.SIGTERM))
        self.assertIsNone(child.poll())
        record["start_ticks"] = lifecycle.process_start_ticks(child.pid)
        record["token"] = "wrong-token"
        lifecycle.write_state(lifecycle.state_path(self.configs[0]), record)
        self.assertFalse(lifecycle.is_managed_process(record))
        self.assertIsNone(child.poll())

    def test_upgrade_compatibility_stops_exact_unregistered_gcs(self):
        environment = os.environ.copy()
        environment.pop(lifecycle.TOKEN_ENV, None)
        child = subprocess.Popen(
            [sys.executable, "-m", lifecycle.MAIN_MODULE,
             "--config", str(self.configs[0])],
            cwd=self.root, env=environment, start_new_session=True,
        )
        self.children.append(child)
        self.assertEqual(lifecycle.stop(self.configs[0], dry_run=False), 0)
        child.wait(timeout=3)

    def test_upgrade_compatibility_stops_legacy_default_config(self):
        default_config = self.root / "ground_station/config.ini"
        default_config.write_text("[test]\n", encoding="utf-8")
        environment = os.environ.copy()
        environment.pop(lifecycle.TOKEN_ENV, None)
        child = subprocess.Popen(
            [sys.executable, "-m", lifecycle.MAIN_MODULE],
            cwd=self.root, env=environment, start_new_session=True,
        )
        self.children.append(child)
        self.assertEqual(lifecycle.stop(default_config, dry_run=False), 0)
        child.wait(timeout=3)

    def test_stop_rejects_relative_config_and_non_gcs_command(self):
        self.assertEqual(lifecycle.main(["stop", "--config", "relative.ini"]), 2)
        child = subprocess.Popen([sys.executable, "-c", "import time;time.sleep(60)"],
                                 cwd=self.root, start_new_session=True)
        self.children.append(child)
        fake = {
            "pid": str(child.pid),
            "start_ticks": str(lifecycle.process_start_ticks(child.pid)),
            "executable": str(Path(sys.executable).resolve()),
            "config": str(self.configs[0]),
            "token": "anything",
        }
        self.assertFalse(lifecycle.is_managed_process(fake))


if __name__ == "__main__":
    unittest.main()
