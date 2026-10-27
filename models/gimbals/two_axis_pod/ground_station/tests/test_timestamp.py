"""实际UDP发送使用发送时Unix整数毫秒，测量字段不受时间戳影响。"""
import json
from pathlib import Path
import socket
import sys
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telemetry_forward import TelemetryForwarder, aisheng_message
from test_forward import telemetry


class TimestampTests(unittest.TestCase):
    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(('127.0.0.1', 0))
        self.receiver.settimeout(1)
        self.addCleanup(self.receiver.close)
        self.forwarder = TelemetryForwarder()
        self.addCleanup(self.forwarder.close)
        self.forwarder.start(*self.receiver.getsockname())

    def receive(self, source):
        self.forwarder.offer(json.dumps(source).encode())
        return json.loads(self.receiver.recv(8192))

    def test_actual_udp_timestamp_is_current_unix_ms_and_preserves_measurements(self):
        source = telemetry()
        source['timestamp'] = 123
        before = time.time_ns() // 1_000_000
        packet = self.receive(source)
        after = time.time_ns() // 1_000_000
        self.assertEqual(len(packet), 19)
        timestamp = packet.pop('timestamp')
        self.assertIs(type(timestamp), int)
        self.assertLessEqual(before, timestamp)
        self.assertLessEqual(timestamp, after)
        self.assertEqual(packet, aisheng_message(source))
        self.assertEqual(source['timestamp'], 123)

    def test_timestamp_is_generated_after_conversion_for_each_packet(self):
        clock = {'ns': 0}
        moments = iter((1_792_999_999_123_999_999, 1_792_999_999_456_000_001))

        def convert(source):
            payload = aisheng_message(source)
            clock['ns'] = next(moments)
            return payload

        with patch('telemetry_forward.aisheng_message', side_effect=convert), \
             patch('telemetry_forward.time.time_ns', side_effect=lambda: clock['ns']):
            first = self.receive(telemetry())
            second = self.receive(telemetry())
        self.assertEqual(first['timestamp'], 1_792_999_999_123)
        self.assertEqual(second['timestamp'], 1_792_999_999_456)
        self.assertEqual({k: v for k, v in first.items() if k != 'timestamp'},
                         {k: v for k, v in second.items() if k != 'timestamp'})


if __name__ == '__main__':
    unittest.main()
