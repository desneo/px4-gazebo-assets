#!/usr/bin/env python3
"""同一UDP端口回传、合法报文更新对端与心跳超时回归。"""
from pathlib import Path
import socket
import sys
import threading
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common import protocol as proto
from gateway.main import Gateway


class UdpSessionTest(unittest.TestCase):
    def setUp(self):
        self.gateway = Gateway.__new__(Gateway)
        g = self.gateway
        g.lock = threading.Lock()
        g.args = SimpleNamespace(verbose=False)
        g.control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        g.control_sock.bind(('127.0.0.1', 0))
        g.control_sock.settimeout(0.2)
        self.addCleanup(g.control_sock.close)
        g.telemetry_peer = None
        g.peer_last_mono = 0.0
        g.last_discrete = None
        g.last_discrete_time = 0.0
        g.last_control = None
        g.last_control_mono = 0.0
        g.rx_control = g.tx_telemetry = g.tx_errors = 0
        self.clients = []

    def client(self):
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.bind(('127.0.0.1', 0))
        client.settimeout(0.05)
        self.addCleanup(client.close)
        return client

    def receive_control(self, client, payload=None):
        client.sendto(payload or proto.make_control(0x00),
                      self.gateway.control_sock.getsockname())
        packet, peer = self.gateway.control_sock.recvfrom(2048)
        self.gateway._on_control(packet, peer)

    def test_reply_uses_control_port_and_client_source_port(self):
        client = self.client()
        self.receive_control(client)
        self.gateway._send_telemetry(proto.TelemetryFrame(yaw_deg=12.34))
        data, source = client.recvfrom(2048)
        self.assertEqual(source, self.gateway.control_sock.getsockname())
        self.assertEqual(proto.decode_telemetry(data).yaw_deg, 12.34)
        self.assertEqual(self.gateway.telemetry_peer, client.getsockname())

    def test_repeated_heartbeat_from_new_client_updates_reply_peer(self):
        first, second = self.client(), self.client()
        self.receive_control(first)
        self.receive_control(second)
        self.gateway._send_telemetry(proto.TelemetryFrame())
        second.recvfrom(2048)
        with self.assertRaises(socket.timeout):
            first.recvfrom(2048)
        self.assertEqual(self.gateway.telemetry_peer, second.getsockname())

    def test_invalid_frame_does_not_redirect_reply(self):
        first, invalid = self.client(), self.client()
        self.receive_control(first)
        self.receive_control(invalid, b'not-a-control-frame')
        self.gateway._send_telemetry(proto.TelemetryFrame())
        first.recvfrom(2048)
        with self.assertRaises(socket.timeout):
            invalid.recvfrom(2048)

    def test_no_peer_or_expired_peer_does_not_send(self):
        self.gateway._send_telemetry(proto.TelemetryFrame())
        self.assertEqual(self.gateway.tx_telemetry, 0)
        client = self.client()
        self.receive_control(client)
        self.gateway.peer_last_mono = time.monotonic() - 2.1
        self.gateway._send_telemetry(proto.TelemetryFrame())
        with self.assertRaises(socket.timeout):
            client.recvfrom(2048)
        self.assertEqual(self.gateway.tx_telemetry, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
