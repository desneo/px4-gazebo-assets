#!/usr/bin/env python3
"""A2416A 45/104字节协议契约测试。"""
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from protocol import codec as proto  # noqa: E402


class ControlCodecTest(unittest.TestCase):
    def test_roundtrip_and_wire_offsets(self):
        frame = proto.ControlFrame(
            command=proto.CMD_BOX_LOCK, p1=123, p2=-45, p3=321, p4=222,
            aircraft_pitch_deg=-12.34, aircraft_roll_deg=5.67,
            aircraft_heading_deg=270.12, status=0xA5,
            aircraft_lat_deg=32.2178573, aircraft_lon_deg=119.1058778,
            aircraft_alt_m=123.456, timestamp_ms=0x12345678,
            reserved_12=0x12, reserved_19_25=bytes(range(7)), reserved_39=0x39,
        )
        raw = proto.encode_control(frame)
        self.assertEqual(len(raw), 45)
        self.assertEqual(raw[:4], b"\xAA\x55\x2D\xB8")
        self.assertEqual(struct.unpack_from("<hhHH", raw, 4), (123, -45, 321, 222))
        self.assertEqual(raw[44], sum(raw[3:44]) & 0xFF)
        decoded = proto.decode_control(raw)
        self.assertEqual(decoded.command, frame.command)
        self.assertEqual(decoded.reserved_19_25, bytes(range(7)))
        self.assertAlmostEqual(decoded.aircraft_lat_deg, frame.aircraft_lat_deg, 7)
        self.assertAlmostEqual(decoded.aircraft_lon_deg, frame.aircraft_lon_deg, 7)
        self.assertAlmostEqual(decoded.aircraft_alt_m, frame.aircraft_alt_m, 3)
        self.assertEqual(decoded.timestamp_ms, 0x12345678)

    def test_literal_header_not_integer_endianness(self):
        self.assertEqual(proto.make_control(proto.CMD_HEARTBEAT)[:2], b"\xAA\x55")

    def test_rejects_bad_size_header_length_and_checksum(self):
        good = bytearray(proto.make_control(proto.CMD_HEARTBEAT))
        variants = [good[:-1], bytearray(good), bytearray(good), bytearray(good)]
        variants[1][0] ^= 1
        variants[2][2] = 44
        variants[3][44] ^= 1
        for raw in variants:
            with self.subTest(raw=bytes(raw).hex()):
                with self.assertRaises(ValueError):
                    proto.decode_control(bytes(raw))

    def test_encode_rejects_unsigned_parameter_overflow(self):
        with self.assertRaisesRegex(ValueError, "p3"):
            proto.make_control(proto.CMD_BOX_LOCK, p3=-1)


class TelemetryCodecTest(unittest.TestCase):
    def sample(self):
        return proto.TelemetryFrame(
            self_test=1, work_state=0x89,
            gate_x=960, gate_y=540,
            # 平台式字段必须保持零。
            pitch_deg=0, yaw_deg=0, gyro_pitch_dps=0, gyro_yaw_dps=0,
            pitch_misalignment_deg=-3.21, yaw_misalignment_deg=4.56,
            tracking_confidence=25.5,
            pitch_los_dps=0, yaw_los_dps=0, status_control=0,
            autonomous=1, status_control2=0x0D, gate_w=90, gate_h=70,
            pitch_att_deg=0, yaw_att_deg=0, roll_att_deg=0,
            tracking_target_type=4,
            aircraft_pitch_deg=1.23, aircraft_roll_deg=-2.34,
            aircraft_heading_deg=359.99, ip=(8, 2), timestamp_ms=123456,
            targets=(
                proto.TelemetryTarget(4, 1.0, 1191058778, 322178573, 123.4),
                proto.TelemetryTarget(4, 0.8, 1191050000, 322170000, 100.0),
                proto.TelemetryTarget(), proto.TelemetryTarget(),
            ), measurement_seq=17,
        )

    def test_roundtrip_all_named_fields(self):
        raw = proto.encode_telemetry(self.sample())
        self.assertEqual(len(raw), 104)
        self.assertEqual(raw[:3], b"\x55\xAA\x68")
        self.assertEqual(raw[103], sum(raw[3:103]) & 0xFF)
        decoded = proto.decode_telemetry(raw)
        self.assertEqual((decoded.pitch_deg, decoded.yaw_deg), (0, 0))
        self.assertEqual((decoded.gyro_pitch_dps, decoded.gyro_yaw_dps), (0, 0))
        self.assertEqual((decoded.pitch_los_dps, decoded.yaw_los_dps), (0, 0))
        self.assertAlmostEqual(decoded.pitch_misalignment_deg, -3.21, 2)
        self.assertAlmostEqual(decoded.yaw_misalignment_deg, 4.56, 2)
        self.assertEqual((decoded.gate_x, decoded.gate_y), (960, 540))
        self.assertEqual((decoded.gate_w, decoded.gate_h), (90, 70))
        self.assertEqual(len(decoded.targets), 4)
        self.assertEqual(decoded.targets[0].type, 4)
        self.assertEqual(decoded.targets[0].lon, 1191058778)
        self.assertEqual(decoded.ip, (8, 2))
        self.assertEqual(decoded.timestamp_ms, 123456)

    def test_misalignment_is_limited_to_twenty_degrees(self):
        frame = proto.TelemetryFrame(
            pitch_misalignment_deg=-99, yaw_misalignment_deg=99)
        decoded = proto.decode_telemetry(proto.encode_telemetry(frame))
        self.assertEqual(decoded.pitch_misalignment_deg, -20.0)
        self.assertEqual(decoded.yaw_misalignment_deg, 20.0)

    def test_full_json_is_flat_and_contains_metadata_and_four_targets(self):
        raw = proto.encode_telemetry(self.sample())
        decoded = proto.decode_telemetry(raw)
        data = proto.telemetry_message(decoded, 987654321)
        self.assertEqual(data["frame_header"], "55AA")
        self.assertEqual(data["frame_length"], 104)
        self.assertEqual(len(data["raw_frame_hex"]), 208)
        self.assertTrue(data["checksum_valid"])
        self.assertEqual(data["checksum"], raw[103])
        self.assertEqual(data["reserved_bytes"], [])
        self.assertEqual(data["timestamp"], 987654321)
        self.assertEqual(len(data["target_type"]), 4)
        expected = {
            "pitch_angle", "yaw_angle", "gyro_pitch_rate", "gyro_yaw_rate",
            "pitch_los_rate", "yaw_los_rate", "pitch_misalignment",
            "yaw_misalignment", "aircraft_pitch", "aircraft_roll", "aircraft_yaw",
            "device_timestamp", "measurement_seq",
        }
        self.assertTrue(expected.issubset(data))

    def test_rejects_wrong_target_count(self):
        with self.assertRaisesRegex(ValueError, "四个目标"):
            proto.encode_telemetry(proto.TelemetryFrame(
                targets=(proto.TelemetryTarget(),)))

    def test_rejects_bad_checksum(self):
        raw = bytearray(proto.encode_telemetry(proto.TelemetryFrame()))
        raw[103] ^= 1
        with self.assertRaisesRegex(ValueError, "校验和"):
            proto.decode_telemetry(raw)


if __name__ == "__main__":
    unittest.main(verbosity=2)
