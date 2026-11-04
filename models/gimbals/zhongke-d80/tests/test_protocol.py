"""独立中科黄金字节样本、符号/量化及非法输入边界。"""
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common import protocol as proto


def telemetry_values(**changes):
    values = dict(status1=4, status2=8, status3=0, zoom=1.0,
                  offset_x=0.0, offset_y=0.0, frame_roll=0.0,
                  frame_pitch=-12.34, frame_yaw=56.78, roll_rate=0.0,
                  pitch_rate=-20.0, yaw_rate=19.99, laser_range=0.0,
                  self_check=0x80)
    values.update(changes)
    return values


class CommandTest(unittest.TestCase):
    def test_manual_golden_frame_is_independent_of_encoder(self):
        golden = bytes.fromhex('eb 90 24 38 ff 96 00 00 00 00 00 00 00 00 00 6c')
        self.assertEqual(proto.encode_command(0x24, -200, 150), golden)
        self.assertEqual(proto.decode_command(golden),
                         dict(cmd=0x24, x=-200, y=150, p3=0, zoom_rate=0))

    def test_known_commands_use_vendor_values(self):
        self.assertEqual(proto.SUPPORTED_COMMANDS,
                         {0x00, 0x0D, 0x0E, 0x0F, 0x24, 0x26, 0x2B})
        for cmd in (0x00, 0x0D, 0x0E, 0x0F, 0x24, 0x26, 0x2B):
            self.assertEqual(proto.decode_command(proto.encode_command(cmd))['cmd'], cmd)

    def test_signed_extremes_keep_little_endian(self):
        raw = proto.encode_command(proto.CMD_POINT_TRACK, -32768, 32767)
        self.assertEqual(raw[3:7], b'\x00\x80\xff\x7f')
        self.assertEqual(proto.decode_command(raw)['x'], -32768)
        self.assertEqual(proto.decode_command(raw)['y'], 32767)

    def test_no_json_wrapper_or_other_protocol_is_accepted(self):
        for raw in (b'{"cmd":"manual"}', b'\xaa\x55' + bytes(43),
                    b'\xeb\x90' + bytes(13), b'\xeb\x90' + bytes(15)):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                proto.decode_command(raw)

    def test_bad_header_and_checksum_are_rejected(self):
        for index in (0, 1, 2, 3, 15):
            raw = bytearray(proto.encode_command(0x24, 10, -20))
            raw[index] ^= 1
            with self.subTest(index=index), self.assertRaises(ValueError):
                proto.decode_command(raw)

    def test_unknown_command_with_valid_checksum_is_rejected(self):
        raw = bytearray(16)
        raw[:3] = b'\xeb\x90\x3a'
        raw[-1] = sum(raw[:-1]) & 255
        with self.assertRaisesRegex(ValueError, '不支持'):
            proto.decode_command(raw)

    def test_unused_parameters_and_reserved_bytes_are_zero(self):
        for kwargs in ({'p3': 1}, {'zoom_rate': -1}, {'x': 1}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                proto.encode_command(proto.CMD_HOME, **kwargs)
        raw = bytearray(proto.encode_command(proto.CMD_MANUAL))
        raw[12] = 1
        raw[-1] = sum(raw[:-1]) & 255
        with self.assertRaises(ValueError):
            proto.decode_command(raw)

    def test_integer_arguments_cannot_wrap_or_accept_boolean(self):
        for kwargs in ({'x': -32769}, {'y': 32768}, {'x': 1.0}, {'x': True}, {'cmd': True}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                proto.encode_command(**{'cmd': proto.CMD_MANUAL, **kwargs})


class TelemetryTest(unittest.TestCase):
    def golden(self):
        # 直接依据厂商表2填偏移，不能调用待测编码器构建期望样本。
        wire = bytearray(32)
        wire[:6] = bytes((0xEE, 0x16, 0xA4, 0xD3, 0xBC, 0xA5))
        struct.pack_into('<hhhhh', wire, 6, -32768, 32767, -1234, 5678, -9000)
        wire[16:20] = bytes((1, 2, 3, 4))
        struct.pack_into('<hhhH', wire, 20, -2000, 1999, 0, 4567)
        wire[28:31] = bytes((0x87, 0xAB, 0xCD))
        wire[31] = sum(wire[:31]) & 255
        return bytes(wire)

    def test_all_fields_decode_from_independent_wire_sample(self):
        self.assertEqual(proto.decode_telemetry(self.golden()), {
            'status1': 0xA4, 'status2': 0xD3, 'status3': 0xA5,
            'zoom': 146.8, 'offset_x': -1638.4, 'offset_y': 1638.35,
            'frame_roll': -12.34, 'frame_pitch': 56.78, 'frame_yaw': -90.0,
            'roll_rate': -20.0, 'pitch_rate': 19.99, 'yaw_rate': 0.0,
            'laser_range': 456.7, 'self_check': 0x87,
        })

    def test_encode_offsets_reserved_bytes_and_checksum(self):
        raw = proto.encode_telemetry(telemetry_values())
        self.assertEqual(len(raw), 32)
        self.assertEqual(raw[:6], b'\xee\x16\x04\x08\x0a\x00')
        self.assertEqual(struct.unpack_from('<hh', raw, 12), (-1234, 5678))
        self.assertEqual(struct.unpack_from('<hh', raw, 22), (-2000, 1999))
        self.assertEqual(raw[16:20] + raw[29:31], bytes(6))
        self.assertEqual(raw[31], sum(raw[:31]) & 255)

    def test_zoom_high_bits_do_not_destroy_picture_bits(self):
        raw = proto.encode_telemetry(telemetry_values(zoom=409.5, status3=0xA0))
        self.assertEqual(raw[4:6], b'\xff\xaf')
        decoded = proto.decode_telemetry(raw)
        self.assertEqual(decoded['status3'], 0xAF)
        self.assertEqual(decoded['zoom'], 409.5)

    def test_nearest_rounding_is_half_away_from_zero(self):
        raw = proto.encode_telemetry(telemetry_values(
            offset_x=0.025, offset_y=-0.025, frame_roll=0.005, frame_pitch=-0.005,
            zoom=1.05, laser_range=0.05))
        self.assertEqual(struct.unpack_from('<hhhh', raw, 6), (1, -1, 1, -1))
        self.assertEqual(raw[4], 11)
        self.assertEqual(struct.unpack_from('<H', raw, 26)[0], 1)

    def test_telemetry_contains_exactly_fourteen_fields_without_metadata(self):
        decoded = proto.decode_telemetry(self.golden())
        self.assertEqual(set(decoded), set(proto.TELEMETRY_FIELDS))
        self.assertEqual(len(decoded), 14)
        for name in ('timestamp', 'rx_seq', 'aircraft_id', 'type', 'target_name'):
            self.assertNotIn(name, decoded)

    def test_bad_frames_cannot_become_measurements(self):
        for index in (0, 1, 3, 30, 31):
            raw = bytearray(self.golden())
            raw[index] ^= 1
            with self.subTest(index=index), self.assertRaises(ValueError):
                proto.decode_telemetry(raw)
        for raw in (self.golden()[:-1], self.golden() + b'\0', b'{"status1":4}'):
            with self.assertRaises(ValueError):
                proto.decode_telemetry(raw)

    def test_missing_or_extra_fields_are_rejected(self):
        missing = telemetry_values()
        missing.pop('yaw_rate')
        for values in (missing, telemetry_values(timestamp=1)):
            with self.assertRaises(ValueError):
                proto.encode_telemetry(values)

    def test_nonfinite_boolean_negative_unsigned_and_overflow_are_rejected(self):
        for field, value in (
            ('frame_yaw', float('nan')), ('pitch_rate', float('inf')),
            ('offset_x', True), ('status1', 1.0), ('status2', 256),
            ('self_check', -1), ('zoom', -0.001), ('laser_range', -0.001),
            ('zoom', 409.6), ('laser_range', 6553.6), ('frame_pitch', 327.68),
            ('yaw_rate', -327.69), ('offset_x', 1638.4),
        ):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                proto.encode_telemetry(telemetry_values(**{field: value}))

    def test_valid_mechanical_rates_are_not_silently_limited_to_twenty(self):
        values = proto.decode_telemetry(proto.encode_telemetry(
            telemetry_values(pitch_rate=30.0, yaw_rate=-40.0)))
        self.assertEqual(values['pitch_rate'], 30.0)
        self.assertEqual(values['yaw_rate'], -40.0)


if __name__ == '__main__':
    unittest.main()
