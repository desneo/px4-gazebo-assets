#!/usr/bin/env python3
"""A2416A地面站通信、交互和艾晟桥兼容JSON测试。"""
from __future__ import annotations

import json
from pathlib import Path
import socket
import sys
import time
import unittest

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
from ground_station.main import FORWARD_START_LABEL, GroundStation, parse_args


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


if __name__ == "__main__":
    unittest.main()
