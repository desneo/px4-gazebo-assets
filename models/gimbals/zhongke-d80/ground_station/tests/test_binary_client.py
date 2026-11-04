"""使用真实 UDP 和独立黄金帧检查中科客户端接收边界。"""
import socket
import struct
import sys
import threading
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from ground_station.binary_client import BinaryClient


def golden_telemetry():
    frame = bytearray(32)
    frame[:6] = bytes((0xEE, 0x16, 4, 8, 10, 0xA0))
    for offset, raw in ((6, -20), (8, 30), (10, 0), (12, -1234),
                        (14, 4321), (20, 0), (22, 2150), (24, -4567)):
        struct.pack_into('<h', frame, offset, raw)
    frame[28] = 128
    frame[31] = sum(frame[:31]) & 255
    return bytes(frame)


def wait_until(predicate, timeout=2):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(.005)
    return False


class BinaryClientTest(unittest.TestCase):
    def setUp(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.bind(('127.0.0.1', 0))
        self.server.settimeout(2)
        self.received = []
        self.client = BinaryClient(*self.server.getsockname(), on_telemetry=self.received.append)
        self.client.start()
        frame, self.peer = self.server.recvfrom(128)
        self.assertEqual(len(frame), 16)
        self.assertEqual(frame[:3], bytes((0xEB, 0x90, 0)))

    def tearDown(self):
        self.client.stop()
        self.server.close()

    def test_identical_udp_datagrams_are_distinct_samples_without_rate_clipping(self):
        packet = golden_telemetry()
        self.server.sendto(packet, self.peer)
        self.server.sendto(packet, self.peer)
        self.assertTrue(wait_until(lambda: len(self.received) == 2))
        self.assertNotEqual(self.received[0].rx_seq, self.received[1].rx_seq)
        self.assertEqual(self.received[0].fields['yaw_rate'], -45.67)
        self.assertEqual(self.received[0].fields['pitch_rate'], 21.5)
        self.assertEqual(self.client.stats()['received'], 2)

    def test_corrupt_short_and_wrong_source_do_not_refresh_valid_telemetry(self):
        packet = golden_telemetry()
        other = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            other.sendto(packet, self.peer)
            self.server.sendto(packet[:-1], self.peer)
            self.server.sendto(packet[:-1] + bytes((packet[-1] ^ 1,)), self.peer)
            self.assertTrue(wait_until(lambda: self.client.stats()['rejected'] >= 3))
            self.assertEqual(self.received, [])
            self.assertFalse(self.client.stats()['connected'])
        finally:
            other.close()

    def test_point_control_sent_once_as_binary_and_shutdown_closes_socket(self):
        self.client.send(0x0D, x=-960, y=540)
        points = []
        end = time.monotonic() + .3
        self.server.settimeout(.05)
        while time.monotonic() < end:
            try:
                frame, peer = self.server.recvfrom(128)
            except socket.timeout:
                continue
            self.assertEqual(peer, self.peer)
            self.assertEqual(len(frame), 16)
            self.assertEqual(frame[:2], bytes((0xEB, 0x90)))
            if frame[2] == 0x0D:
                points.append(frame)
        self.assertEqual(len(points), 1)
        self.assertEqual(struct.unpack_from('<hh', points[0], 3), (-960, 540))
        self.client.stop()
        self.assertFalse(self.client.is_alive())


if __name__ == '__main__':
    unittest.main()
