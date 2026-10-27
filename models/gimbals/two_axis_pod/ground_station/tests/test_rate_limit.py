"""机械 gyro 与目标惯性 LOS 独立映射，均使用 °/s 并限幅 ±20。"""
import copy
import json
from pathlib import Path
import socket
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telemetry_forward import TelemetryForwarder, aisheng_message, target_los_rates, target_los_omega


def telemetry(pitch_rate=0.0, yaw_rate=0.0):
    return {
        "type": "telemetry", "protocol_version": 8,
        "angle_convention": "motion_head_joint_right_positive", "aircraft_id": 1,
        "mode": "TRACK", "tracking": {"state": "TRACK"},
        "vision": {"mode": "manual_track", "state": "MANUAL_READY"},
        "attitude_deg": {
            "pitch": {"pos": -56.37, "vel": pitch_rate, "acc": 3.0},
            "yaw": {"pos": 12.34, "vel": yaw_rate, "acc": -4.0}},
        "target": {"valid": True, "box_px": [95.0, 195.0, 50.0, 40.0],
                   "miss": {"valid": True, "offset_deg": [-0.25, 0.5]}},
        "target_los": {"valid": True, "frame": "NED", "observer_reference": "aircraft_nominal_cg",
                       "target_name": "aircraft_target_1", "pitch_rate_dps": -2.5,
                       "yaw_rate_dps": 3.25, "reason": "", "omega_valid": True,
                       "omega_ned_dps": [5.5, -32.75, 9.125], "omega_reason": "ok"},
        "frame_table": [[42, 12.3]], "extra": {"measurement": [1, 2]},
    }


OTHER_FIELDS = {
    "track_status": 1, "pitch_angle": -56.37, "yaw_angle": 12.34,
    "pitch_misalignment": 0.5, "yaw_misalignment": -0.25,
    "tracking_confidence": 25.5, "target_confidence": 0.80,
    "tracking_gate_w": 50.0, "tracking_gate_h": 40.0,
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
                self.assertEqual(packet["gyro_yaw_rate"], yaw)
                self.assertEqual(packet["yaw_misalignment"], 1.25)

    def test_v6_or_old_yaw_marker_is_rejected(self):
        for version, convention in ((6, "motion_head_joint"),
                                    (6, "motion_head_joint_right_positive"),
                                    (7, "motion_head_joint"),
                                    (7, "motion_head_joint_right_positive"),
                                    (8, "motion_head_joint")):
            source = telemetry()
            source.update(protocol_version=version, angle_convention=convention)
            with self.subTest(version=version, convention=convention), self.assertRaises(ValueError):
                aisheng_message(source)

    def test_positive_and_negative_overspeed_on_both_axes(self):
        for value in (20.000001, 21, 100.5, 3.4028234663852886e38):
            for sign in (-1, 1):
                with self.subTest(value=value, sign=sign):
                    packet = aisheng_message(telemetry(sign * value, -sign * value))
                    self.assertEqual(packet["gyro_pitch_rate"], sign * 20.0)
                    self.assertEqual(packet["gyro_yaw_rate"], -sign * 20.0)

    def test_boundaries_and_in_range_values_remain_direct_degrees_per_second(self):
        for value in (-20.0, -19.25, -1.234567, 0.0, 7.75, 20.0):
            with self.subTest(value=value):
                packet = aisheng_message(telemetry(value, -value))
                self.assertEqual(packet["gyro_pitch_rate"], value)
                self.assertEqual(packet["gyro_yaw_rate"], -value)
                self.assertIs(type(packet["gyro_pitch_rate"]), float)
                self.assertIs(type(packet["gyro_yaw_rate"]), float)

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
                self.assertEqual(packet["gyro_pitch_rate"], -20.0)
                self.assertEqual(packet["gyro_yaw_rate"], 20.0)
                self.assertEqual(packet["pitch_misalignment"], 0.0)
                self.assertEqual(packet["yaw_misalignment"], 0.0)
                self.assertEqual(packet["tracking_gate_w"], 0.0)
                self.assertEqual(packet["tracking_gate_h"], 0.0)

    def test_other_nine_fields_unchanged_and_format_has_eighteen_fields(self):
        packet = aisheng_message(telemetry(-83.5, 67.25))
        self.assertEqual(set(packet), set(OTHER_FIELDS) | {"pitch_los_rate", "yaw_los_rate", "gyro_pitch_rate", "gyro_yaw_rate",
                                                                 "los_omega_n", "los_omega_e", "los_omega_d", "gate_x", "gate_y"})
        self.assertEqual({key: packet[key] for key in OTHER_FIELDS}, OTHER_FIELDS)

    def test_source_measurement_is_not_mutated(self):
        source = telemetry(-83.5, 67.25)
        original = copy.deepcopy(source)
        packet = aisheng_message(source)
        self.assertEqual(source, original)
        self.assertEqual(source["attitude_deg"]["pitch"]["vel"], -83.5)
        self.assertEqual(source["attitude_deg"]["yaw"]["vel"], 67.25)
        self.assertEqual(packet["gyro_pitch_rate"], -20.0)
        self.assertEqual(packet["gyro_yaw_rate"], 20.0)


class TargetLosTests(unittest.TestCase):
    def test_inertial_rates_are_independent_of_joint_rates_and_limited_after_validation(self):
        for pitch, yaw in ((-20., 20.), (-3.75, 2.125), (0., 0.), (77., -88.)):
            source = telemetry(12., -13.)
            source["target_los"].update(pitch_rate_dps=pitch, yaw_rate_dps=yaw)
            original = copy.deepcopy(source)
            with self.subTest(pitch=pitch, yaw=yaw):
                packet = aisheng_message(source)
                self.assertEqual(packet["pitch_los_rate"], max(-20., min(20., pitch)))
                self.assertEqual(packet["yaw_los_rate"], max(-20., min(20., yaw)))
                self.assertEqual((packet["gyro_pitch_rate"], packet["gyro_yaw_rate"]), (12., -13.))
                self.assertEqual(source, original)

    def test_valid_zero_los_keeps_nonzero_joint_measurements(self):
        source = telemetry(15., -16.)
        source["target_los"].update(pitch_rate_dps=0., yaw_rate_dps=0.)
        self.assertEqual(target_los_rates(source), (0., 0.))
        packet = aisheng_message(source)
        self.assertEqual((packet["pitch_los_rate"], packet["yaw_los_rate"]), (0., 0.))
        self.assertEqual((packet["gyro_pitch_rate"], packet["gyro_yaw_rate"]), (15., -16.))

    def test_explicit_invalid_los_sends_zero_but_preserves_gyro_and_tracking_fields(self):
        source = telemetry(15., -16.)
        source["target_los"].update(valid=False, pitch_rate_dps=None, yaw_rate_dps=None,
                                    reason="vertical_singularity")
        self.assertIsNone(target_los_rates(source))
        packet = aisheng_message(source)
        self.assertEqual((packet["pitch_los_rate"], packet["yaw_los_rate"]), (0., 0.))
        self.assertEqual((packet["gyro_pitch_rate"], packet["gyro_yaw_rate"]), (15., -16.))
        self.assertEqual({key: packet[key] for key in OTHER_FIELDS}, OTHER_FIELDS)
        self.assertEqual(len(packet), 18)
        self.assertNotIn("los_valid", packet)

    def test_missing_los_or_bad_reference_cannot_silently_become_zero(self):
        for los in (None, {}, [], {"valid": False}, dict(telemetry()["target_los"], frame="ENU"),
                    dict(telemetry()["target_los"], observer_reference="camera_optical_center")):
            source = telemetry()
            source["target_los"] = los
            with self.subTest(los=los), self.assertRaises(ValueError):
                aisheng_message(source)
        source = telemetry()
        del source["target_los"]
        with self.assertRaises(ValueError):
            aisheng_message(source)

    def test_missing_malformed_and_nonfinite_valid_los_rejects_packet(self):
        for axis in ("pitch", "yaw"):
            for value in (None, True, "1.5", [], {}, float("nan"), float("inf"), -float("inf"), 3.5e38):
                source = telemetry()
                source["target_los"][f"{axis}_rate_dps"] = value
                with self.subTest(axis=axis, value=value), self.assertRaises(ValueError):
                    aisheng_message(source)
            source = telemetry()
            del source["target_los"][f"{axis}_rate_dps"]
            with self.assertRaises(ValueError):
                aisheng_message(source)

    def test_valid_flag_must_be_boolean_and_valid_target_must_be_named(self):
        for key, value in (("valid", 0), ("valid", 1), ("valid", None), ("valid", "false"),
                           ("target_name", ""), ("target_name", None), ("reason", None)):
            source = telemetry()
            source["target_los"][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                aisheng_message(source)

    def test_invalid_los_must_carry_explicit_null_rates(self):
        for rates in ({"pitch_rate_dps": None}, {"yaw_rate_dps": None},
                      {"pitch_rate_dps": 1., "yaw_rate_dps": None}):
            source = telemetry()
            source["target_los"] = dict(valid=False, frame="NED", observer_reference="aircraft_nominal_cg",
                                        target_name="", reason="no_target", **rates)
            with self.subTest(rates=rates), self.assertRaises(ValueError):
                aisheng_message(source)


class TargetLosOmegaTests(unittest.TestCase):
    def test_ned_vector_is_direct_and_not_limited_or_reconstructed_from_angle_rates(self):
        source = telemetry(88., -77.)
        source["target_los"].update(pitch_rate_dps=-66., yaw_rate_dps=55.,
                                    omega_ned_dps=[21.1256789, -222.5, 0.000000123])
        original = copy.deepcopy(source)
        packet = aisheng_message(source)
        self.assertEqual(target_los_omega(source), (21.1256789, -222.5, 0.000000123))
        self.assertEqual([packet[f"los_omega_{axis}"] for axis in "ned"],
                         [21.1256789, -222.5, 0.000000123])
        self.assertEqual((packet["gyro_pitch_rate"], packet["gyro_yaw_rate"]), (20., -20.))
        self.assertEqual((packet["pitch_los_rate"], packet["yaw_los_rate"]), (-20., 20.))
        self.assertEqual(source, original)

    def test_vertical_view_can_have_valid_vector_with_invalid_two_angle_rates(self):
        source = telemetry()
        source["target_los"].update(valid=False, pitch_rate_dps=None, yaw_rate_dps=None,
                                    reason="vertical_singularity", omega_valid=True,
                                    omega_ned_dps=[35., -42., 0.], omega_reason="ok")
        packet = aisheng_message(source)
        self.assertEqual((packet["pitch_los_rate"], packet["yaw_los_rate"]), (0., 0.))
        self.assertEqual([packet[f"los_omega_{axis}"] for axis in "ned"], [35., -42., 0.])

    def test_explicit_invalid_vector_sends_zero_without_changing_other_thirteen_fields(self):
        source = telemetry(8., -9.)
        before = aisheng_message(source)
        source["target_los"].update(omega_valid=False, omega_ned_dps=None,
                                    omega_reason="target_unbound")
        packet = aisheng_message(source)
        self.assertIsNone(target_los_omega(source))
        self.assertEqual([packet[f"los_omega_{axis}"] for axis in "ned"], [0., 0., 0.])
        self.assertEqual({k: v for k, v in packet.items() if not k.startswith("los_omega_")},
                         {k: v for k, v in before.items() if not k.startswith("los_omega_")})

    def test_missing_or_malformed_vector_fields_reject_packet(self):
        for key in ("omega_valid", "omega_reason", "omega_ned_dps"):
            source = telemetry()
            del source["target_los"][key]
            with self.subTest(missing=key), self.assertRaises(ValueError):
                aisheng_message(source)
        for key, value in (("omega_valid", 1), ("omega_valid", "false"),
                           ("omega_reason", None), ("omega_ned_dps", None),
                           ("omega_ned_dps", [1., 2.]), ("omega_ned_dps", [1., 2., 3., 4.]),
                           ("omega_ned_dps", "1 2 3")):
            source = telemetry()
            source["target_los"][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                aisheng_message(source)
        source = telemetry()
        source["target_los"]["omega_valid"] = False
        with self.assertRaises(ValueError):
            aisheng_message(source)

    def test_vector_components_require_finite_float32_representable_numeric_values(self):
        for value in (True, None, "2", [], float("nan"), float("inf"), -float("inf"),
                      3.5e38, -3.5e38, 10**400):
            for axis in range(3):
                source = telemetry()
                source["target_los"]["omega_ned_dps"][axis] = value
                with self.subTest(axis=axis, value=str(value)), self.assertRaises(ValueError):
                    aisheng_message(source)


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

    def test_actual_udp_contains_independent_limited_nineteen_field_json(self):
        for pitch, yaw in ((45.5, -37.25), (-45.5, 37.25)):
            source = telemetry(pitch, yaw)
            original = copy.deepcopy(source)
            self.forwarder.offer(json.dumps(source).encode("utf-8"))
            packet = self.receive()
            with self.subTest(pitch=pitch, yaw=yaw):
                self.assertEqual(packet["gyro_pitch_rate"], 20.0 if pitch > 0 else -20.0)
                self.assertEqual(packet["gyro_yaw_rate"], 20.0 if yaw > 0 else -20.0)
                self.assertEqual({key: packet[key] for key in OTHER_FIELDS}, OTHER_FIELDS)
                self.assertEqual(len(packet), 19)
                self.assertEqual(packet["pitch_los_rate"], -2.5)
                self.assertEqual(packet["yaw_los_rate"], 3.25)
                self.assertEqual(source, original)
        self.assertEqual(self.forwarder.snapshot()["sent"], 2)
        self.assertEqual(self.forwarder.snapshot()["dropped"], 0)

    def test_actual_udp_contains_unclipped_ned_vector_and_independent_invalid_zero(self):
        source = telemetry(8., -9.)
        source["target_los"].update(valid=False, pitch_rate_dps=None, yaw_rate_dps=None,
                                    reason="vertical_singularity", omega_ned_dps=[31., -42., 0.])
        self.forwarder.offer(json.dumps(source).encode())
        packet = self.receive()
        self.assertEqual(len(packet), 19)
        self.assertEqual([packet[f"los_omega_{axis}"] for axis in "ned"], [31., -42., 0.])
        self.assertEqual((packet["pitch_los_rate"], packet["yaw_los_rate"]), (0., 0.))
        source["target_los"].update(omega_valid=False, omega_ned_dps=None, omega_reason="no_target")
        self.forwarder.offer(json.dumps(source).encode())
        packet = self.receive()
        self.assertEqual([packet[f"los_omega_{axis}"] for axis in "ned"], [0., 0., 0.])
        self.assertEqual((packet["gyro_pitch_rate"], packet["gyro_yaw_rate"]), (8., -9.))

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
        self.assertEqual(packet["gyro_pitch_rate"], -20.0)
        self.assertEqual(packet["gyro_yaw_rate"], 20.0)
        snapshot = self.forwarder.snapshot()
        self.assertEqual(snapshot["sent"], 1)
        self.assertEqual(snapshot["dropped"], 1)
        self.assertEqual(snapshot["error"], "")

    def test_invalid_los_still_sends_gyro_and_next_valid_sample_resumes_actual_los(self):
        source = telemetry(5.25, -6.5)
        source["target_los"].update(valid=False, pitch_rate_dps=None, yaw_rate_dps=None,
                                    reason="observer_unavailable")
        self.forwarder.offer(json.dumps(source).encode())
        packet = self.receive()
        self.assertEqual((packet["pitch_los_rate"], packet["yaw_los_rate"]), (0., 0.))
        self.assertEqual((packet["gyro_pitch_rate"], packet["gyro_yaw_rate"]), (5.25, -6.5))
        self.forwarder.offer(json.dumps(telemetry(5.25, -6.5)).encode())
        packet = self.receive()
        self.assertEqual((packet["pitch_los_rate"], packet["yaw_los_rate"]), (-2.5, 3.25))
        self.assertEqual(self.forwarder.snapshot()["sent"], 2)
        self.assertEqual(self.forwarder.snapshot()["dropped"], 0)


if __name__ == "__main__":
    unittest.main()
