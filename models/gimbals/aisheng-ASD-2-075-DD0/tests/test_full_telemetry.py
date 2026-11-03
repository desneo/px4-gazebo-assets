#!/usr/bin/env python3
"""标准遥测全部字段、四目标数组和地面站连接配置验证。"""
from __future__ import annotations

import configparser
from pathlib import Path
import socket
import struct
import tempfile
import time
import unittest
from unittest.mock import patch

from test_gcs_ui import GCS

proto = GCS.proto


class FullTelemetryTest(unittest.TestCase):
    def test_all_wire_fields_decode_and_export_with_vendor_units(self):
        # 直接按协议字节偏移写入，避免 encode/decode 同时出错而通过对称测试。
        wire = bytearray(104)
        wire[:5] = b"\x55\xaa\x68\x02\xa9"
        struct.pack_into("<hhhhhhhh", wire, 5,
                         -1234, 4567, 960, 540, -2000, 1999, -125, 250)
        wire[21] = 251
        struct.pack_into("<hhBB", wire, 22, -150, 275, 0xc4, 1)
        values = [
            (4, 80, 1191058778, 322178573, 1234),
            (2, 90, -1799999999, -899999999, 25),
            (3, 255, 1, -1, 65535),
            (5, 0, 0, 0, 0),
        ]
        for index, target in enumerate(values):
            struct.pack_into("<BBiiH", wire, 28 + 12 * index, *target)
        struct.pack_into("<BHHhhhBH BhhHBBI".replace(" ", ""), wire, 76,
                         0x1f, 501, 301, -123, 17999, -32768,
                         4, 4567, 0x3f, 120, -230, 35999, 8, 2, 4294967295)
        wire[103] = sum(wire[3:103]) & 0xff
        frame = proto.decode_telemetry(bytes(wire))
        message = proto.telemetry_message(frame, 1234567890123)
        expected = {
            "timestamp": 1234567890123,
            "self_check": 2, "work_status": 0xa9, "track_status": 1,
            "pitch_angle": -12.34, "yaw_angle": 45.67,
            "gate_x": 960, "gate_y": 540,
            "gyro_pitch_rate": -20.0, "gyro_yaw_rate": 19.99,
            "pitch_misalignment": -1.25, "yaw_misalignment": 2.5,
            "tracking_confidence": 25.1,
            "pitch_los_rate": -1.5, "yaw_los_rate": 2.75,
            "status": 0xc4, "autonomous": 1,
            "target_type": [4, 2, 3, 5],
            "target_confidence": [0.8, 0.9, 2.55, 0.0],
            "target_lon": [1191058778, -1799999999, 1, 0],
            "target_lat": [322178573, -899999999, -1, 0],
            "target_alt": [123.4, 2.5, 6553.5, 0.0],
            "status2": 0x1f, "tracking_gate_w": 501, "tracking_gate_h": 301,
            "pitch_attitude": -1.23, "yaw_attitude": 179.99,
            "roll_attitude": -327.68, "tracking_target_type": 4,
            "laser_range": 456.7, "laser_update": 0x3f,
            "aircraft_pitch": 1.2, "aircraft_roll": -2.3, "aircraft_yaw": 359.99,
            "ip_segment": 8, "ip_node": 2, "device_timestamp": 4294967295,
        }
        self.assertEqual(message, expected)
        self.assertEqual(len(message), 37)
        self.assertEqual(frame.target_type, 4)
        self.assertEqual(frame.target_confidence, 0.8)
        self.assertEqual(frame.target_lon_deg, 119.1058778)

    def test_four_targets_encode_and_keep_raw_coordinate_precision(self):
        targets = (
            proto.TelemetryTarget(4, 0.8, 1191058778, 322178573, 123.4),
            proto.TelemetryTarget(2, 0.9, -1799999999, -899999999, 2.5),
            proto.TelemetryTarget(3, 2.55, 1, -1, 6553.5),
            proto.TelemetryTarget(5, 0.0, 0, 0, 0.0),
        )
        wire = proto.encode_telemetry(proto.TelemetryFrame(targets=targets))
        self.assertEqual(proto.decode_telemetry(wire).targets, targets)
        self.assertEqual(struct.unpack_from("<ii", wire, 42), (-1799999999, -899999999))
        with self.assertRaises(ValueError):
            proto.encode_telemetry(proto.TelemetryFrame(targets=targets[:3]))

    def test_no_gateway_at_start_can_recover_with_same_socket(self):
        # 先向无人监听的端口发送心跳，触发本机 ICMP 拒绝后再启动网关。
        reserve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        reserve.bind(("127.0.0.1", 0))
        port = reserve.getsockname()[1]
        reserve.close()
        client = GCS.TelemetryClient("127.0.0.1", port)
        gateway = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            initial_peer = client.sock.getsockname()
            time.sleep(0.25)
            gateway.bind(("127.0.0.1", port))
            gateway.settimeout(2.0)
            data, peer = gateway.recvfrom(2048)
            self.assertEqual(proto.decode_control(data).command, 0)
            self.assertEqual(peer, initial_peer)
            gateway.sendto(proto.encode_telemetry(proto.TelemetryFrame(timestamp_ms=123)), peer)
            deadline = time.monotonic() + 1.0
            while client.latest is None and time.monotonic() < deadline:
                time.sleep(0.005)
            self.assertIsNotNone(client.latest)
            self.assertEqual(client.latest.timestamp_ms, 123)
        finally:
            client.close()
            gateway.close()

    def test_config_has_single_control_endpoint_and_manual_forward_defaults(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "config.ini"
            with patch("sys.argv", ["gcs", "--config", str(path)]):
                args = GCS.parse_args()
            self.assertEqual((args.host, args.control_port), ("127.0.0.1", 8001))
            self.assertEqual((args.forward_host, args.forward_port), ("127.0.0.1", 9000))
            self.assertFalse(hasattr(args, "telemetry_port"))
            self.assertFalse(hasattr(args, "json_telemetry_port"))
            with patch("sys.argv", ["gcs", "--config", str(path), "--forward-host",
                                    "192.168.1.50", "--forward-port", "9004"]):
                args = GCS.parse_args()
            GCS.save_config(args)
            saved = configparser.ConfigParser()
            saved.read(path)
            self.assertEqual(dict(saved["network"]), {"host": "127.0.0.1", "control_port": "8001"})
            self.assertEqual(dict(saved["forward"]), {"host": "192.168.1.50", "port": "9004"})
            self.assertIn("按发送按钮后才转发", path.read_text())
            with patch("sys.argv", ["gcs", "--config", str(path)]):
                restored = GCS.parse_args()
            self.assertEqual((restored.forward_host, restored.forward_port),
                             ("192.168.1.50", 9004))


if __name__ == "__main__":
    unittest.main(verbosity=2)
