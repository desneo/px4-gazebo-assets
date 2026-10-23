"""艾晟机械轴速度使用 °/s，合法输入限幅至 ±20，非法输入仍拒绝。"""
import copy
import json
from pathlib import Path
import socket
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telemetry_forward import TelemetryForwarder, aisheng_message


def telemetry(pitch_rate=0.0, yaw_rate=0.0):
    return {
        "type": "telemetry", "protocol_version": 7,
        "angle_convention": "motion_head_joint_right_positive", "aircraft_id": 1,
        "mode": "TRACK", "tracking": {"state": "TRACK"},
        "vision": {"mode": "manual_track", "state": "MANUAL_READY"},
        "attitude_deg": {
            "pitch": {"pos": -56.37, "vel": pitch_rate, "acc": 3.0},
            "yaw": {"pos": 12.34, "vel": yaw_rate, "acc": -4.0}},
        "target": {"valid": True, "box_px": [95.0, 195.0, 50.0, 40.0],
                   "miss": {"valid": True, "offset_deg": [-0.25, 0.5]}},
        "frame_table": [[42, 12.3]], "extra": {"measurement": [1, 2]},
    }


OTHER_FIELDS = {
    "track_status": 1, "pitch_angle": -56.37, "yaw_angle": 12.34,
    "pitch_misalignment": 0.5, "yaw_misalignment": -0.25,
    "tracking_confidence": 25.5, "target_confidence": 0.80,
    "tracking_gateW": 50.0, "tracking_gateH": 40.0,
}


class RateLimitTests(unittest.TestCase):
    def test_yaw_right_positive_passes_directly_and_image_miss_stays_right_positive(self):
        for yaw in (-12.5, 0.0, 12.5):
            with self.subTest(yaw=yaw):
                source = telemetry(yaw_rate=yaw)
                source["attitude_deg"]["yaw"]["pos"] = yaw
                source["target"]["miss"]["offset_deg"] = [1.25, -.5]
                packet = aisheng_message(source)
                self.assertEqual(packet["yaw_angle"], yaw)
                self.assertEqual(packet["yaw_los_rate"], yaw)
                self.assertEqual(packet["yaw_misalignment"], 1.25)

    def test_v6_or_old_yaw_marker_is_rejected(self):
        for version, convention in ((6, "motion_head_joint"),
                                    (6, "motion_head_joint_right_positive"),
                                    (7, "motion_head_joint")):
            source = telemetry()
            source.update(protocol_version=version, angle_convention=convention)
            with self.subTest(version=version, convention=convention), self.assertRaises(ValueError):
                aisheng_message(source)

    def test_positive_and_negative_overspeed_on_both_axes(self):
        for value in (20.000001, 21, 100.5, 3.4028234663852886e38):
            for sign in (-1, 1):
                with self.subTest(value=value, sign=sign):
                    packet = aisheng_message(telemetry(sign * value, -sign * value))
                    self.assertEqual(packet["pitch_los_rate"], sign * 20.0)
                    self.assertEqual(packet["yaw_los_rate"], -sign * 20.0)

    def test_boundaries_and_in_range_values_remain_direct_degrees_per_second(self):
        for value in (-20.0, -19.25, -1.234567, 0.0, 7.75, 20.0):
            with self.subTest(value=value):
                packet = aisheng_message(telemetry(value, -value))
                self.assertEqual(packet["pitch_los_rate"], value)
                self.assertEqual(packet["yaw_los_rate"], -value)
                self.assertIs(type(packet["pitch_los_rate"]), float)
                self.assertIs(type(packet["yaw_los_rate"]), float)

    def test_missing_and_invalid_velocity_rejects_packet(self):
        for axis in ("pitch", "yaw"):
            source = telemetry()
            del source["attitude_deg"][axis]["vel"]
            with self.subTest(axis=axis, missing=True), self.assertRaises(ValueError):
                aisheng_message(source)
            for value in (None, True, "21", [], {}, float("nan"),
                          float("inf"), float("-inf")):
                source = telemetry()
                source["attitude_deg"][axis]["vel"] = value
                with self.subTest(axis=axis, value=value), self.assertRaises(ValueError):
                    aisheng_message(source)

    def test_float32_overflow_is_rejected_before_limiting(self):
        for axis in ("pitch", "yaw"):
            for value in (3.5e38, -3.5e38, 1e100, -1e100):
                source = telemetry()
                source["attitude_deg"][axis]["vel"] = value
                with self.subTest(axis=axis, value=value), self.assertRaisesRegex(
                        ValueError, "float32"):
                    aisheng_message(source)

    def test_untracked_measurements_are_also_limited(self):
        for state in ("IDLE", "TEMP_LOST", "LOST"):
            source = telemetry(-83.5, 67.25)
            source["tracking"]["state"] = state
            with self.subTest(state=state):
                packet = aisheng_message(source)
                self.assertEqual(packet["track_status"], 0)
                self.assertEqual(packet["pitch_los_rate"], -20.0)
                self.assertEqual(packet["yaw_los_rate"], 20.0)
                self.assertEqual(packet["pitch_misalignment"], 0.0)
                self.assertEqual(packet["yaw_misalignment"], 0.0)
                self.assertEqual(packet["tracking_gateW"], 0.0)
                self.assertEqual(packet["tracking_gateH"], 0.0)

    def test_other_nine_fields_and_eleven_field_format_are_unchanged(self):
        packet = aisheng_message(telemetry(-83.5, 67.25))
        self.assertEqual(set(packet), set(OTHER_FIELDS) | {"pitch_los_rate", "yaw_los_rate"})
        self.assertEqual({key: packet[key] for key in OTHER_FIELDS}, OTHER_FIELDS)

    def test_source_measurement_is_not_mutated(self):
        source = telemetry(-83.5, 67.25)
        original = copy.deepcopy(source)
        packet = aisheng_message(source)
        self.assertEqual(source, original)
        self.assertEqual(source["attitude_deg"]["pitch"]["vel"], -83.5)
        self.assertEqual(source["attitude_deg"]["yaw"]["vel"], 67.25)
        self.assertEqual(packet["pitch_los_rate"], -20.0)
        self.assertEqual(packet["yaw_los_rate"], 20.0)


class RateLimitUdpTests(unittest.TestCase):
    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.receiver.close)
        self.receiver.bind(("127.0.0.1", 0))
        self.receiver.settimeout(1.0)
        self.forwarder = TelemetryForwarder()
        self.addCleanup(self.forwarder.close)
        self.forwarder.start(*self.receiver.getsockname())

    def receive(self):
        return json.loads(self.receiver.recvfrom(8192)[0])

    def test_actual_udp_contains_limited_raw_unit_eleven_field_json(self):
        for pitch, yaw in ((45.5, -37.25), (-45.5, 37.25)):
            source = telemetry(pitch, yaw)
            original = copy.deepcopy(source)
            self.forwarder.offer(json.dumps(source).encode("utf-8"))
            packet = self.receive()
            with self.subTest(pitch=pitch, yaw=yaw):
                self.assertEqual(packet["pitch_los_rate"], 20.0 if pitch > 0 else -20.0)
                self.assertEqual(packet["yaw_los_rate"], 20.0 if yaw > 0 else -20.0)
                self.assertEqual({key: packet[key] for key in OTHER_FIELDS}, OTHER_FIELDS)
                self.assertEqual(len(packet), 11)
                self.assertEqual(source, original)
        self.assertEqual(self.forwarder.snapshot()["sent"], 2)
        self.assertEqual(self.forwarder.snapshot()["dropped"], 0)

    def test_invalid_velocity_is_not_sent_and_next_valid_packet_recovers(self):
        self.forwarder.offer(json.dumps(telemetry(1e100, 1.0)).encode("utf-8"))
        deadline = time.monotonic() + 1.0
        while self.forwarder.snapshot()["dropped"] == 0 and time.monotonic() < deadline:
            time.sleep(0.005)
        snapshot = self.forwarder.snapshot()
        self.assertEqual(snapshot["dropped"], 1)
        self.assertEqual(snapshot["sent"], 0)
        self.assertIn("float32", snapshot["error"])
        self.receiver.settimeout(0.05)
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(8192)
        self.receiver.settimeout(1.0)
        self.forwarder.offer(json.dumps(telemetry(-90.0, 90.0)).encode("utf-8"))
        packet = self.receive()
        self.assertEqual(packet["pitch_los_rate"], -20.0)
        self.assertEqual(packet["yaw_los_rate"], 20.0)
        snapshot = self.forwarder.snapshot()
        self.assertEqual(snapshot["sent"], 1)
        self.assertEqual(snapshot["dropped"], 1)
        self.assertEqual(snapshot["error"], "")


if __name__ == "__main__":
    unittest.main()
