"""实际 UDP 接收验证中科转发的字段、采样和发送生命周期。"""
import json
import socket
import struct
import sys
import time
import threading
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from common.runtime_types import ReceivedTelemetry
from ground_station.telemetry_forward import TelemetryForwarder, save_destination

FIELDS = dict(status1=4, status2=8, status3=0, zoom=1.0, offset_x=-1.0,
              offset_y=1.5, frame_roll=0.0, frame_pitch=-12.34,
              frame_yaw=43.21, roll_rate=0.0, pitch_rate=21.5,
              yaw_rate=-45.67, laser_range=0.0, self_check=128)


def golden_telemetry():
    packet = bytearray(32)
    packet[:6] = bytes((0xEE, 0x16, 4, 8, 10, 0))
    for offset, raw in ((6, -20), (8, 30), (12, -1234), (14, 4321),
                        (22, 2150), (24, -4567)):
        struct.pack_into('<h', packet, offset, raw)
    packet[28] = 128
    packet[31] = sum(packet[:31]) & 255
    return bytes(packet)


class ForwardTest(unittest.TestCase):
    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(('127.0.0.1', 0))
        self.receiver.settimeout(.15)
        self.forward = TelemetryForwarder()
        self.sequence = 0

    def tearDown(self):
        self.forward.close()
        self.receiver.close()

    def sample(self, *, age=0, fields=None):
        self.sequence += 1
        return ReceivedTelemetry(rx_seq=self.sequence,
                                 received_monotonic=time.monotonic() - age,
                                 fields=dict(FIELDS if fields is None else fields),
                                 source=('127.0.0.1', 8870), raw=golden_telemetry())

    def receive(self):
        return json.loads(self.receiver.recvfrom(4096)[0])

    def assert_silent(self):
        with self.assertRaises(socket.timeout):
            self.receiver.recvfrom(4096)

    def test_default_off_and_start_never_replays_off_period(self):
        self.forward.offer(self.sample())
        self.forward.start(*self.receiver.getsockname())
        self.assert_silent()
        self.forward.offer(self.sample())
        self.assertEqual(self.receive()['yaw_rate'], -45.67)

    def test_exact_fields_timestamp_and_identical_content_samples(self):
        self.forward.start(*self.receiver.getsockname())
        before = time.time_ns() // 1_000_000
        self.forward.offer(self.sample())
        self.forward.offer(self.sample())
        first, second = self.receive(), self.receive()
        after = time.time_ns() // 1_000_000
        self.assertEqual(set(first), set(FIELDS) | {'timestamp'})
        self.assertEqual(first['pitch_rate'], 21.5)
        self.assertEqual(first['yaw_rate'], -45.67)
        self.assertIs(type(first['timestamp']), int)
        self.assertTrue(before <= first['timestamp'] <= after)
        self.assertEqual({k: v for k, v in first.items() if k != 'timestamp'}, FIELDS)
        self.assertEqual({k: v for k, v in second.items() if k != 'timestamp'}, FIELDS)

    def test_same_receive_sequence_at_most_once_and_stale_samples_drop(self):
        self.forward.start(*self.receiver.getsockname())
        one = self.sample()
        self.forward.offer(one)
        self.receive()
        self.forward.offer(one)
        self.forward.offer(self.sample(age=1.1))
        self.assert_silent()

    def test_stop_then_start_discards_old_session_and_zeroes_rate(self):
        self.forward.start(*self.receiver.getsockname())
        self.forward.stop()
        self.forward.offer(self.sample())
        self.assertEqual(self.forward.snapshot()['send_hz'], 0)
        self.forward.start(*self.receiver.getsockname())
        self.assert_silent()
        self.forward.offer(self.sample())
        self.receive()

    def test_invalid_numeric_fields_never_send(self):
        self.forward.start(*self.receiver.getsockname())
        for invalid in (float('nan'), float('inf'), True):
            bad = dict(FIELDS, yaw_rate=invalid)
            self.forward.offer(self.sample(fields=bad))
        self.assert_silent()
        self.assertGreaterEqual(self.forward.snapshot()['dropped'], 3)

    def test_diagnostic_or_other_values_cannot_replace_raw_measurement(self):
        self.forward.start(*self.receiver.getsockname())
        self.forward.offer(self.sample(fields=dict(FIELDS, yaw_rate=99.0)))
        self.assert_silent()
        self.assertEqual(self.forward.snapshot()['dropped'], 1)
        self.forward.offer(self.sample())
        self.assertEqual(self.receive()['yaw_rate'], -45.67)

    def test_save_address_preserves_comments_and_no_enable_flag(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.ini'
            path.write_text('# 中文配置\n[forward]\n# 接收主机\nip = 127.0.0.1\nport = 9000\n[video]\ndecoder = nvdec\n', encoding='utf-8')
            save_destination(path, '127.0.0.2', 9500)
            text = path.read_text(encoding='utf-8')
            self.assertIn('# 中文配置', text)
            self.assertIn('# 接收主机', text)
            self.assertIn('ip = 127.0.0.2', text)
            self.assertIn('port = 9500', text)
            self.assertIn('decoder = nvdec', text)
            self.assertNotIn('enabled', text)

    def test_bounded_queue_drops_old_samples_and_queued_samples_expire(self):
        class GatedForwarder(TelemetryForwarder):
            def __init__(self, **kwargs):
                self.gate=threading.Event()
                super().__init__(**kwargs)
            def _run(self):
                self.gate.wait(2)
                super()._run()
        self.forward.close()
        self.forward=GatedForwarder()
        self.forward.start(*self.receiver.getsockname())
        for _ in range(40):
            self.forward.offer(self.sample())
        self.assertEqual(self.forward.snapshot()['queued'],16)
        self.assertEqual(self.forward.snapshot()['dropped'],24)
        self.forward.gate.set()
        for _ in range(16):
            self.receive()
        self.assert_silent()
        self.forward.close()
        self.forward=GatedForwarder(max_age_s=.02)
        self.forward.start(*self.receiver.getsockname())
        self.forward.offer(self.sample())
        time.sleep(.03)
        self.forward.gate.set()
        self.assert_silent()

    def test_actual_binary_receive_callback_drives_forwarding(self):
        from ground_station.binary_client import BinaryClient
        server=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        server.bind(('127.0.0.1',0))
        server.settimeout(2)
        client=BinaryClient(*server.getsockname(),on_telemetry=self.forward.offer)
        self.forward.start(*self.receiver.getsockname())
        try:
            client.start()
            _,peer=server.recvfrom(128)
            server.sendto(golden_telemetry(),peer)
            self.assertEqual(self.receive()['yaw_rate'],-45.67)
            self.assertEqual(client.snapshot().raw,golden_telemetry())
        finally:
            client.stop()
            server.close()


if __name__ == '__main__':
    unittest.main()
