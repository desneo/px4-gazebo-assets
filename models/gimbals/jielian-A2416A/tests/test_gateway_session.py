#!/usr/bin/env python3
"""A2416A UDP会话、命令白名单、遥测映射和100 Hz调度测试。"""
from pathlib import Path
import socket
import sys
import threading
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gateway.service import GatewayCore, UdpGateway  # noqa: E402
from gateway.session import ControlSession  # noqa: E402
from protocol import codec as proto  # noqa: E402


class FakeClock:
    def __init__(self, value=100.0):
        self.value = value

    def __call__(self):
        return self.value


class SessionTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.session = ControlSession(clock=self.clock)
        self.first = ("127.0.0.1", 10001)
        self.second = ("127.0.0.1", 10002)

    def receive(self, cmd, peer=None, **params):
        return self.session.receive(
            proto.make_control(cmd, **params), peer or self.first)

    def test_latest_valid_source_becomes_only_telemetry_peer(self):
        self.assertTrue(self.receive(proto.CMD_HEARTBEAT).accepted)
        self.assertEqual(self.session.telemetry_peer(), self.first)
        self.clock.value += 0.01
        self.assertTrue(self.receive(proto.CMD_HEARTBEAT, self.second).accepted)
        self.assertEqual(self.session.telemetry_peer(), self.second)

    def test_invalid_or_unsupported_frame_cannot_redirect_peer(self):
        self.receive(proto.CMD_HEARTBEAT)
        bad = bytearray(proto.make_control(0x22))
        result = self.session.receive(bytes(bad), self.second)
        self.assertFalse(result.accepted)
        self.assertIn("不支持", result.error)
        self.assertEqual(self.session.telemetry_peer(), self.first)

    def test_discrete_command_deduplicates_but_renews_lease(self):
        first = self.receive(proto.CMD_GATE_GROW)
        self.clock.value += 0.10
        duplicate = self.receive(proto.CMD_GATE_GROW)
        self.assertTrue(first.accepted and not first.duplicate)
        self.assertTrue(duplicate.accepted and duplicate.duplicate)
        self.assertEqual(self.session.last_seen, self.clock.value)
        self.clock.value += 0.11
        self.assertFalse(self.receive(proto.CMD_GATE_GROW).duplicate)

    def test_heartbeat_and_crosshair_move_are_never_deduplicated(self):
        for cmd, kwargs in ((proto.CMD_HEARTBEAT, {}),
                            (proto.CMD_CROSSHAIR_MOVE, {"p1": 50, "p2": -50})):
            self.assertFalse(self.receive(cmd, **kwargs).duplicate)
            self.clock.value += 0.01
            self.assertFalse(self.receive(cmd, **kwargs).duplicate)

    def test_parameter_contracts_reject_bad_coordinates(self):
        result = self.receive(proto.CMD_POINT_LOCK, p1=1920, p2=540)
        self.assertFalse(result.accepted)
        self.assertIn("坐标", result.error)
        result = self.receive(proto.CMD_BOX_LOCK, p1=1900, p2=1000, p3=100, p4=100)
        self.assertFalse(result.accepted)
        self.assertIn("完全位于", result.error)

    def test_two_second_timeout_is_reported_once(self):
        self.receive(proto.CMD_HEARTBEAT)
        self.clock.value += 1.999
        self.assertIsNone(self.session.poll())
        self.clock.value += 0.001
        self.assertEqual(self.session.poll(), "peer_timeout")
        self.assertIsNone(self.session.poll())
        self.assertIsNone(self.session.telemetry_peer())


class GatewayCoreTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.commands = []
        self.states = []
        self.core = GatewayCore(
            clock=self.clock, command_sink=self.commands.append,
            state_sink=self.states.append)
        self.peer = ("127.0.0.1", 12345)

    def send(self, cmd, **params):
        return self.core.receive_control(proto.make_control(cmd, **params), self.peer)

    def test_white_list_commands_update_state_and_publish_deterministic_payload(self):
        self.send(proto.CMD_RECOGNITION_ON)
        self.send(proto.CMD_AUTO_LOCK_ON)
        self.send(proto.CMD_ZOOM_IN)
        self.send(proto.CMD_IMAGE_ENHANCE_ON)
        self.send(proto.CMD_STREAM_SET, p1=350)
        self.send(proto.CMD_IP_SET, p1=9, p2=12)
        self.send(proto.CMD_OSD_HALF)
        self.assertTrue(self.core.state.recognition_enabled)
        self.assertTrue(self.core.state.auto_lock_enabled)
        self.assertEqual(self.core.state.zoom, 2)
        self.assertTrue(self.core.state.enhancement_enabled)
        self.assertEqual(self.core.state.bitrate_mbps, 3.5)
        self.assertEqual(self.core.state.logical_ip, (9, 12))
        self.assertEqual(self.core.state.osd_mode, "half")
        self.assertEqual(self.commands[0]["cmd"], "recognition_on")
        self.assertEqual(self.commands[-1]["p4"], 0)

    def test_duplicate_is_not_published_twice(self):
        self.send(proto.CMD_GATE_GROW)
        self.clock.value += 0.05
        result = self.send(proto.CMD_GATE_GROW)
        self.assertTrue(result.duplicate)
        self.assertEqual([c["cmd"] for c in self.commands], ["gate_grow"])

    def test_manual_lock_or_cancel_disables_auto_reacquisition(self):
        self.send(proto.CMD_AUTO_LOCK_ON)
        self.send(proto.CMD_POINT_LOCK, p1=960, p2=540)
        self.assertFalse(self.core.state.auto_lock_enabled)
        self.clock.value += 0.21
        self.send(proto.CMD_AUTO_LOCK_ON)
        self.send(proto.CMD_CANCEL_TRACK)
        self.assertFalse(self.core.state.auto_lock_enabled)

    def test_timeout_stops_recognition_and_tracking(self):
        self.send(proto.CMD_RECOGNITION_ON)
        self.send(proto.CMD_AUTO_LOCK_ON)
        self.clock.value += 2.0
        self.assertEqual(self.core.poll(), "peer_timeout")
        self.assertFalse(self.core.state.recognition_enabled)
        self.assertFalse(self.core.state.auto_lock_enabled)
        self.assertEqual([item["cmd"] for item in self.commands[-3:]],
                         ["recognition_off", "auto_lock_off", "cancel_track"])
        self.assertTrue(all(item["reason"] == "control_timeout"
                            for item in self.commands[-3:]))

    def test_invalid_packet_arriving_at_expiry_cannot_hide_timeout(self):
        self.send(proto.CMD_RECOGNITION_ON)
        self.clock.value += 2.0
        result = self.core.receive_control(b"bad", ("127.0.0.1", 9999))
        self.assertFalse(result.accepted)
        self.assertEqual([item["cmd"] for item in self.commands[-3:]],
                         ["recognition_off", "auto_lock_off", "cancel_track"])

    def test_telemetry_uses_only_misalignment_and_zeroes_platform_fields(self):
        self.send(proto.CMD_RECOGNITION_ON,
                  aircraft_pitch_deg=1.25, aircraft_roll_deg=-2.5,
                  aircraft_heading_deg=300.0)
        self.core.update_measurement({
            "measurement_seq": 42,
            "tracking_state": "TRACKING",
            "locked_target_name": "plane_1",
            "gate_px": [900, 500, 120, 80],
            "misalignment_deg": {"pitch": 3.0, "yaw": -4.0},
            "targets": [{
                "name": "plane_1", "type": 4, "confidence": 1.0,
                "lon_deg": 119.1, "lat_deg": 32.2, "alt_m": 88.8,
            }],
        })
        frame = self.core.build_telemetry()
        self.assertEqual(frame.measurement_seq, 42)
        self.assertEqual((frame.gate_x, frame.gate_y), (960, 540))
        self.assertEqual((frame.gate_w, frame.gate_h), (120, 80))
        self.assertEqual((frame.pitch_misalignment_deg, frame.yaw_misalignment_deg),
                         (3.0, -4.0))
        self.assertEqual(frame.work_state, 0xA9)
        self.assertEqual((frame.pitch_deg, frame.yaw_deg), (0, 0))
        self.assertEqual((frame.gyro_pitch_dps, frame.gyro_yaw_dps), (0, 0))
        self.assertEqual((frame.pitch_los_dps, frame.yaw_los_dps), (0, 0))
        self.assertEqual((frame.pitch_att_deg, frame.yaw_att_deg, frame.roll_att_deg),
                         (0, 0, 0))
        self.assertEqual((frame.aircraft_pitch_deg, frame.aircraft_roll_deg,
                          frame.aircraft_heading_deg), (1.25, -2.5, 300.0))
        self.assertEqual(frame.targets[0].type, 4)

    def test_recognition_off_does_not_cancel_manual_tracking(self):
        self.send(proto.CMD_RECOGNITION_ON)
        self.send(proto.CMD_RECOGNITION_OFF)
        self.assertFalse(self.core.state.recognition_enabled)
        self.assertEqual(self.commands[-1]["cmd"], "recognition_off")
        self.core.update_measurement({
            "tracking_state": "TRACKING", "locked_target_name": "manual",
            "misalignment_deg": {"pitch": 1.0, "yaw": 2.0},
        })
        frame = self.core.build_telemetry()
        self.assertEqual(frame.work_state & 0x01, 0x01)
        self.assertEqual((frame.pitch_misalignment_deg, frame.yaw_misalignment_deg),
                         (1.0, 2.0))

    def test_fifty_hz_measurement_can_be_reused_by_two_unique_telemetry_frames(self):
        self.core.update_measurement({"measurement_seq": 7})
        first = self.core.build_telemetry()
        self.clock.value += 0.01
        second = self.core.build_telemetry()
        self.assertEqual((first.measurement_seq, second.measurement_seq), (7, 7))
        self.assertNotEqual(first.timestamp_ms, second.timestamp_ms)
        self.assertEqual(self.core.tx_sequence, 2)

    def test_target_slot_order_is_preserved_and_padded(self):
        self.core.update_measurement({
            "tracking_state": "TRACK", "locked_target_name": "locked",
            "targets": [
                {"name": "locked", "type": 4, "confidence": 1},
                {"name": "other", "type": 4, "confidence": 0.8},
            ],
        })
        frame = self.core.build_telemetry()
        self.assertEqual([target.type for target in frame.targets], [4, 4, 0, 0])

    def test_plugin_aircraft_attitude_overrides_control_fallback(self):
        self.send(proto.CMD_HEARTBEAT,
                  aircraft_pitch_deg=1, aircraft_roll_deg=2,
                  aircraft_heading_deg=3)
        self.core.update_measurement({
            "aircraft_attitude_deg": {"pitch": 4, "roll": 5, "yaw": -30},
        })
        frame = self.core.build_telemetry()
        self.assertEqual((frame.aircraft_pitch_deg, frame.aircraft_roll_deg,
                          frame.aircraft_heading_deg), (4, 5, 330))


class UdpRateTest(unittest.TestCase):
    def test_udp_reply_uses_same_socket_and_runs_near_100_hz(self):
        core = GatewayCore()
        try:
            gateway = UdpGateway(core, "127.0.0.1", 0)
        except PermissionError as exc:
            self.skipTest(f"当前沙箱禁止创建UDP socket：{exc}")
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.bind(("127.0.0.1", 0))
        client.settimeout(0.03)
        thread = threading.Thread(target=gateway.run, daemon=True)
        thread.start()
        try:
            client.sendto(proto.make_control(proto.CMD_HEARTBEAT), gateway.address)
            packets = []
            deadline = time.monotonic() + 0.12
            while time.monotonic() < deadline:
                try:
                    packet, source = client.recvfrom(2048)
                    packets.append((packet, source))
                except socket.timeout:
                    pass
            self.assertGreaterEqual(len(packets), 7)
            self.assertLessEqual(len(packets), 15)
            self.assertTrue(all(source == gateway.address for _, source in packets))
            for packet, _ in packets:
                self.assertEqual(len(packet), 104)
                proto.decode_telemetry(packet)
        finally:
            gateway.running = False
            thread.join(timeout=1.0)
            gateway.close()
            client.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
