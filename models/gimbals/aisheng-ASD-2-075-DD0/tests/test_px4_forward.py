#!/usr/bin/env python3
"""19 字段 PX4 JSON 转换与 UDP 回环测试。"""
from __future__ import annotations

import copy
import json
from pathlib import Path
import socket
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gateway.px4_forward import Px4JsonSender, px4_message  # noqa: E402


def sample() -> dict:
    return {
        "type": "telemetry",
        "protocol_version": 8,
        "angle_convention": "motion_head_joint_right_positive",
        "mode": "TRACK",
        "tracking": {"state": "TRACK"},
        "target": {
            "valid": True,
            "box_px": [100.4, 200.4, 80.0, 60.0],
            "miss": {"valid": True, "offset_deg": [1.5, -2.5]},
        },
        "target_los": {
            "valid": True,
            "frame": "NED",
            "observer_reference": "aircraft_nominal_cg",
            "target_name": "target_aircraft",
            "reason": "ok",
            "pitch_rate_dps": 25.0,
            "yaw_rate_dps": -30.0,
            "omega_valid": True,
            "omega_reason": "ok",
            "omega_ned_dps": [1.25, -2.5, 3.75],
        },
        "attitude_deg": {
            "pitch": {"pos": 5.0, "vel": 61.0},
            "yaw": {"pos": -10.0, "vel": -62.0},
        },
    }


class Px4MessageTest(unittest.TestCase):
    def test_exact_fields_and_values(self):
        message = px4_message(sample(), timestamp_ms=123456789)
        self.assertEqual(set(message), {
            "timestamp", "track_status", "pitch_angle", "yaw_angle",
            "pitch_misalignment", "yaw_misalignment", "tracking_confidence",
            "target_confidence", "gyro_pitch_rate", "gyro_yaw_rate",
            "pitch_los_rate", "yaw_los_rate", "los_omega_n", "los_omega_e",
            "los_omega_d", "tracking_gate_w", "tracking_gate_h", "gate_x",
            "gate_y",
        })
        self.assertEqual(message["timestamp"], 123456789)
        self.assertEqual(message["track_status"], 1)
        self.assertEqual(message["pitch_angle"], 5.0)
        self.assertEqual(message["yaw_angle"], -10.0)
        self.assertEqual(message["pitch_misalignment"], -2.5)
        self.assertEqual(message["yaw_misalignment"], 1.5)
        self.assertEqual(message["gyro_pitch_rate"], 20.0)
        self.assertEqual(message["gyro_yaw_rate"], -20.0)
        self.assertEqual(message["pitch_los_rate"], 20.0)
        self.assertEqual(message["yaw_los_rate"], -20.0)
        self.assertEqual((message["los_omega_n"], message["los_omega_e"],
                          message["los_omega_d"]), (1.25, -2.5, 3.75))
        self.assertEqual(message["tracking_gate_w"], 80.0)
        self.assertEqual(message["tracking_gate_h"], 60.0)
        self.assertEqual((message["gate_x"], message["gate_y"]), (140, 230))

    def test_invalid_tracking_keeps_angles_and_zeroes_target_fields(self):
        data = sample()
        data["mode"] = "IDLE"
        data["tracking"] = {"state": "IDLE"}
        data["target"] = {"valid": False, "box_px": None,
                          "miss": {"valid": False, "offset_deg": None}}
        message = px4_message(data, timestamp_ms=1)
        self.assertEqual(message["track_status"], 0)
        self.assertEqual(message["pitch_angle"], 5.0)
        self.assertEqual(message["pitch_misalignment"], 0.0)
        self.assertEqual(message["tracking_gate_w"], 0.0)
        self.assertEqual((message["gate_x"], message["gate_y"]), (0, 0))

    def test_invalid_los_must_be_null(self):
        data = sample()
        data["target_los"].update(valid=False, pitch_rate_dps=None,
                                  yaw_rate_dps=None, omega_valid=False,
                                  omega_ned_dps=None)
        message = px4_message(data, timestamp_ms=1)
        self.assertEqual(message["pitch_los_rate"], 0.0)
        self.assertEqual(message["yaw_los_rate"], 0.0)
        self.assertEqual((message["los_omega_n"], message["los_omega_e"],
                          message["los_omega_d"]), (0.0, 0.0, 0.0))
        data["target_los"]["pitch_rate_dps"] = 1.0
        with self.assertRaisesRegex(ValueError, "null"):
            px4_message(data, timestamp_ms=1)

    def test_invalid_tracking_box_is_rejected(self):
        data = sample()
        data["target"]["box_px"] = [0, 0, 0, 0]
        with self.assertRaisesRegex(ValueError, "宽高"):
            px4_message(data, timestamp_ms=1)

    def test_udp_loopback_timestamp_is_send_time(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(2.0)
        sender = Px4JsonSender("127.0.0.1", receiver.getsockname()[1])
        try:
            before = time.time_ns() // 1_000_000
            sender.send(sample())
            data, _ = receiver.recvfrom(4096)
            after = time.time_ns() // 1_000_000
        finally:
            sender.close()
            receiver.close()
        message = json.loads(data)
        self.assertEqual(len(message), 19)
        self.assertLessEqual(before, message["timestamp"])
        self.assertLessEqual(message["timestamp"], after)
        self.assertEqual(message["track_status"], 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
