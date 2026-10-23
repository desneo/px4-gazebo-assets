"""真实时间频率窗口、有效接收、成功发送和界面频率标签；不连接外部端口。"""
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from packet_rate import PacketRate
from telemetry_forward import TelemetryForwarder
from test_client import UdpClient
from test_joystick import GroundStation


def telemetry():
    return {"type": "telemetry", "protocol_version": 6,
            "angle_convention": "motion_head_joint", "aircraft_id": 1,
            "attitude_deg": {axis: {"pos": 0., "vel": 0., "acc": 0.}
                             for axis in ("yaw", "pitch")}}


class PacketRateTests(unittest.TestCase):
    def test_startup_warmup_then_uses_observed_real_intervals(self):
        rate = PacketRate()
        for i in range(21):
            rate.record(i / 100.)
        self.assertTrue(rate.snapshot(.2)["warming_up"])
        self.assertEqual(rate.snapshot(.2)["hz"], 0.)
        for i in range(21, 101):
            rate.record(i / 100.)
        self.assertAlmostEqual(rate.snapshot(1.)["hz"], 100.)
        self.assertFalse(rate.snapshot(1.)["warming_up"])

    def test_two_second_window_counts_actual_packets_not_target_or_lifetime_average(self):
        rate = PacketRate()
        for i in range(1, 201):
            rate.record(i / 100.)
        for i in range(1, 101):
            rate.record(2. + i / 50.)
        self.assertAlmostEqual(rate.snapshot(4.)["hz"], 50.)
        self.assertEqual(rate.snapshot(4.)["samples"], 100)

    def test_stopped_input_decays_to_zero_within_two_real_seconds(self):
        rate = PacketRate()
        for i in range(1, 401):
            rate.record(i / 100.)
        for now, expected in ((4., 100.), (4.5, 75.), (5., 50.), (6., 0.)):
            with self.subTest(now=now):
                self.assertAlmostEqual(rate.snapshot(now)["hz"], expected)

    def test_reads_do_not_drive_or_reset_statistics(self):
        rate = PacketRate()
        for i in range(1, 301):
            rate.record(i / 100.)
        expected = rate.snapshot(3.)
        for _ in range(1000):
            self.assertEqual(rate.snapshot(3.), expected)
        self.assertEqual(rate.snapshot(5.)["hz"], 0.)

    def test_stop_and_restart_reset_and_disallow_records_while_inactive(self):
        rate = PacketRate()
        rate.record(1.)
        rate.record(1.5)
        rate.reset(active=False)
        rate.record(2.)
        self.assertEqual(rate.snapshot(2.)["samples"], 0)
        self.assertEqual(rate.snapshot(2.)["hz"], 0.)
        rate.reset()
        rate.record(3.)
        self.assertTrue(rate.snapshot(3.1)["warming_up"])
        rate.record(3.25)
        self.assertEqual(rate.snapshot(3.25)["hz"], 4.)

    def test_thread_safe_packet_count(self):
        rate = PacketRate(window_s=10)
        threads = [threading.Thread(target=lambda: [rate.record() for _ in range(100)])
                   for _ in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(rate.snapshot()["samples"], 400)

    def test_invalid_window_is_rejected(self):
        for window, warmup in ((0, .25), (2, 0), (2, 3), (float("inf"), .25)):
            with self.subTest(window=window, warmup=warmup), self.assertRaises(ValueError):
                PacketRate(window, warmup)


class ReceiveRateTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        clock = mock.patch("packet_rate.time.monotonic", side_effect=lambda: self.now)
        clock.start()
        self.addCleanup(clock.stop)
        self.client = UdpClient.__new__(UdpClient)
        self.client.lock = threading.RLock()
        self.client._receive_rate = PacketRate()
        self.client._stop = threading.Event()
        self.client.aircraft_id = 1
        self.client.target = ("127.0.0.1", 8766)
        self.client.clock_domain = None
        self.client.client_timeout = 5.
        self.client.last_rx_mono = self.client.last_telemetry_mono = 0.
        self.client.messages = self.client.pongs = self.client.pings_sent = 0
        self.client.telemetry, self.client.frame_t0, self.client._pending = {}, {}, {}
        self.client.results, self.client.errors = [], []
        self.client.hello = self.client.last_result = None
        self.client.on_log = lambda _: None

    def receive(self, value, addr=None):
        self.client._on_datagram(json.dumps(value).encode(), addr or self.client.target)

    def hello(self, hz=100):
        self.receive(dict(type="hello", version=6, angle_convention="motion_head_joint",
                          aircraft_id=1, telemetry_hz=hz))

    def test_counts_valid_received_samples_and_exposes_target(self):
        self.hello()
        for i in range(1, 301):
            self.now = 10. + i / 100.
            self.receive(telemetry())
        stats = self.client.stats()
        self.assertEqual(stats["telemetry_target_hz"], 100.)
        self.assertAlmostEqual(stats["receive_hz"], 100.)
        self.assertEqual(stats["rate_window_s"], 2.)
        self.assertFalse(stats["receive_rate_warming_up"])
        self.now += 2.
        self.assertEqual(self.client.stats()["receive_hz"], 0.)

    def test_only_valid_identity_confirmed_telemetry_contributes(self):
        self.receive(telemetry())  # 未确认 hello，不能作为有效姿态样本。
        self.hello()
        self.receive(dict(type="pong"))
        self.receive(telemetry(), ("127.0.0.1", 8767))
        self.receive(dict(telemetry(), protocol_version=5))
        self.receive(dict(type="telemetry"))
        self.receive(dict(telemetry(), attitude_deg=[]))
        self.assertEqual(self.client._receive_rate.snapshot()["samples"], 0)
        self.receive(telemetry())
        self.assertEqual(self.client._receive_rate.snapshot()["samples"], 1)
        self.assertTrue(self.client.stats()["receive_rate_warming_up"])

    def test_stop_immediately_zeroes_receive_rate(self):
        self.hello()
        self.receive(telemetry())
        self.now += .5
        self.receive(telemetry())
        self.assertGreater(self.client.stats()["receive_hz"], 0.)
        self.client.stop()
        self.assertEqual(self.client.stats()["receive_hz"], 0.)
        self.receive(telemetry())
        self.assertEqual(self.client.stats()["receive_hz"], 0.)

    def test_invalid_target_hz_is_displayed_as_unknown(self):
        for value in (None, True, "100", 0, -1, float("nan"), float("inf"), 10**400):
            with self.subTest(value=str(value)):
                self.hello(value)
                self.assertIsNone(self.client.stats()["telemetry_target_hz"])


class SendRateTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.
        clock = mock.patch("packet_rate.time.monotonic", side_effect=lambda: self.now)
        clock.start()
        self.addCleanup(clock.stop)
        class FakeSocket:
            fail = False
            def setblocking(self, _):
                pass
            def sendto(self, data, target):
                if self.fail:
                    raise OSError("synthetic send failure")
                return len(data)
            def close(self):
                pass
        self.socket_patch = mock.patch("telemetry_forward.socket.socket", side_effect=lambda *_: FakeSocket())
        self.socket_patch.start()
        self.addCleanup(self.socket_patch.stop)
        self.forwarder = TelemetryForwarder()
        self.addCleanup(self.forwarder.close)

    def wait_for(self, predicate):
        deadline = time.perf_counter() + 1
        while time.perf_counter() < deadline:
            if predicate():
                return
            time.sleep(.001)
        self.fail("发送线程未在一秒内完成合成样本")

    def test_only_successful_sendto_counts_and_stop_restart_clears(self):
        self.assertEqual(self.forwarder.snapshot()["send_hz"], 0.)
        self.forwarder.start("127.0.0.1", 9000)
        raw = json.dumps(telemetry()).encode()
        for i in range(3):
            self.now = 10. + i * .25
            self.forwarder.offer(raw)
            self.wait_for(lambda: self.forwarder.snapshot()["sent"] == i + 1)
        self.assertAlmostEqual(self.forwarder.snapshot()["send_hz"], 4.)
        self.forwarder.offer(b"invalid json")
        self.wait_for(lambda: self.forwarder.snapshot()["dropped"] == 1)
        self.forwarder._socket.fail = True
        self.forwarder.offer(raw)
        self.wait_for(lambda: self.forwarder.snapshot()["dropped"] == 2)
        self.assertAlmostEqual(self.forwarder.snapshot()["send_hz"], 4.)
        self.assertEqual(self.forwarder._send_rate.snapshot()["samples"], 3)
        self.now += 2.
        self.assertEqual(self.forwarder.snapshot()["send_hz"], 0.)
        self.forwarder.stop()
        self.assertEqual(self.forwarder.snapshot()["send_hz"], 0.)
        self.forwarder.start("127.0.0.1", 9001)
        self.forwarder.offer(raw)
        self.wait_for(lambda: self.forwarder.snapshot()["sent"] == 1)
        self.assertEqual(self.forwarder._send_rate.snapshot()["samples"], 1)
        self.assertTrue(self.forwarder.snapshot()["send_rate_warming_up"])


class RateLabelsTests(unittest.TestCase):
    def test_target_receive_and_send_are_distinct_and_disabled_is_zero(self):
        labels = GroundStation._rate_panel_lines(
            dict(telemetry_target_hz=100., receive_hz=97.5),
            dict(active=False, send_hz=99.), 960)
        self.assertIn("目标 100.0 Hz", labels[0])
        self.assertIn("实际接收 97.5 Hz", labels[0])
        self.assertIn("艾晟发送 0.0 Hz（未发送）", labels[0])

    def test_compact_layout_labels_unknown_and_warmup(self):
        labels = GroundStation._rate_panel_lines(
            dict(telemetry_target_hz=None, receive_rate_warming_up=True),
            dict(active=True, send_rate_warming_up=True), 480)
        self.assertIn("目标 未知", labels[0])
        self.assertIn("实际接收 预热中", labels[0])
        self.assertIn("艾晟发送 预热中", labels[1])


if __name__ == "__main__":
    unittest.main()
