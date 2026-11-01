#!/usr/bin/env python3
"""标准 45/104 字节协议单元测试。"""
from __future__ import annotations

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common import protocol as proto  # noqa: E402


class ControlTest(unittest.TestCase):
    def test_roundtrip(self):
        frame = proto.ControlFrame(
            command=proto.CMD_SPECIFIED_ANGLE,
            p1=1234, p2=-567,
            aircraft_pitch_deg=12.34, aircraft_roll_deg=-5.67,
            aircraft_heading_deg=270.0,
            aircraft_lat_deg=32.2178573, aircraft_lon_deg=119.1058778,
            aircraft_alt_m=123.456, timestamp_ms=987654,
        )
        data = proto.encode_control(frame)
        self.assertEqual(len(data), 45)
        self.assertEqual(data[:2], b"\xaa\x55")
        self.assertEqual(data[2], 0x2D)
        decoded = proto.decode_control(data)
        self.assertEqual(decoded.command, frame.command)
        self.assertEqual((decoded.p1, decoded.p2), (1234, -567))
        self.assertAlmostEqual(decoded.aircraft_pitch_deg, 12.34, places=2)
        self.assertAlmostEqual(decoded.aircraft_roll_deg, -5.67, places=2)
        self.assertAlmostEqual(decoded.aircraft_heading_deg, 270.0, places=2)
        self.assertAlmostEqual(decoded.aircraft_lat_deg, 32.2178573, places=6)
        self.assertAlmostEqual(decoded.aircraft_lon_deg, 119.1058778, places=6)
        self.assertAlmostEqual(decoded.aircraft_alt_m, 123.456, places=3)
        self.assertEqual(decoded.timestamp_ms, 987654)

    def test_bad_checksum(self):
        data = bytearray(proto.make_control(proto.CMD_FRAME_LOCK))
        data[44] ^= 1
        with self.assertRaises(ValueError):
            proto.decode_control(bytes(data))

    def test_bad_length(self):
        with self.assertRaises(ValueError):
            proto.decode_control(b"\xaa\x55")


class TelemetryTest(unittest.TestCase):
    def test_roundtrip_and_clamp(self):
        frame = proto.TelemetryFrame(
            self_test=1, work_state=0xA9,
            pitch_deg=-12.34, yaw_deg=45.67,
            gate_x=960, gate_y=540,
            gyro_pitch_dps=-20.0, gyro_yaw_dps=25.0,
            pitch_misalignment_deg=0.5, yaw_misalignment_deg=-0.25,
            tracking_confidence=25.5,
            pitch_los_dps=0.6, yaw_los_dps=-0.4,
            status_control=0xC0, autonomous=1,
            target_type=4, target_confidence=0.8,
            status_control2=0x0D, gate_w=90, gate_h=70,
            timestamp_ms=123456,
        )
        data = proto.encode_telemetry(frame)
        self.assertEqual(len(data), 104)
        self.assertEqual(data[:2], b"\x55\xaa")
        self.assertEqual(data[2], 0x68)
        decoded = proto.decode_telemetry(data)
        self.assertAlmostEqual(decoded.pitch_deg, -12.34, places=2)
        self.assertAlmostEqual(decoded.yaw_deg, 45.67, places=2)
        self.assertEqual((decoded.gate_x, decoded.gate_y), (960, 540))
        self.assertAlmostEqual(decoded.gyro_yaw_dps, 20.0, places=2)  # 协议限幅
        self.assertAlmostEqual(decoded.tracking_confidence, 25.5, places=1)
        self.assertEqual((decoded.gate_w, decoded.gate_h), (90, 70))
        self.assertEqual(decoded.timestamp_ms, 123456)

    def test_bad_checksum(self):
        data = bytearray(proto.encode_telemetry(proto.TelemetryFrame()))
        data[103] ^= 1
        with self.assertRaises(ValueError):
            proto.decode_telemetry(bytes(data))


if __name__ == "__main__":
    unittest.main()
