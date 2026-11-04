"""真实UDP socket核验同端点32字节回传及单客户端边界，无Gazebo依赖。"""
from pathlib import Path
import json
import socket
import sys
import threading
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from common.config import load_config
from common import protocol as proto
from controller.service.gimbal_bridge import BridgeCore, Gateway


class UdpWireTest(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.commands = []
        self.config = load_config()
        self.gateway = Gateway.__new__(Gateway)
        self.gateway.core = BridgeCore(self.config, self.commands.append, clock=lambda: self.now)
        self.gateway.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gateway.sock.bind(('127.0.0.1', 0))
        self.gateway.sock.settimeout(.2)
        self.gateway.closing = threading.Event()
        self.gateway.tx_count = 0
        self.addCleanup(self.gateway.sock.close)
        self.first = self.client()
        self.second = self.client()

    def client(self):
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.bind(('127.0.0.1', 0))
        client.connect(self.gateway.sock.getsockname())
        client.settimeout(.03)
        self.addCleanup(client.close)
        return client

    def receive_control(self, client, raw=None):
        client.send(raw or proto.encode_command(proto.CMD_NOP))
        packet, peer = self.gateway.sock.recvfrom(2048)
        accepted = self.gateway.core.receive(packet, peer)
        return packet, accepted

    def source(self, stamp=1.0):
        value = dict(config_hash=self.config['config_hash'], configured=True,
            mechanical_valid=True, motor_enabled=True, camera_ready=True, ts_mono=stamp,
            joints_deg={'yaw': {'pos': 12.34, 'vel': 0}, 'pitch': {'pos': -23.45, 'vel': 0}},
            mode='IDLE', tracking={'state': 'IDLE', 'fault': None},
            target={'valid': False, 'miss': {'valid': False}})
        self.gateway._on_telemetry(SimpleNamespace(data=json.dumps(value)))

    def test_control_sixteen_bytes_no_ack_and_telemetry_thirty_two_same_server_endpoint(self):
        raw, accepted = self.receive_control(self.first)
        self.assertTrue(accepted)
        self.assertEqual(len(raw), 16)
        with self.assertRaises(socket.timeout):
            self.first.recv(2048)
        self.source()
        wire, origin = self.first.recvfrom(2048)
        self.assertEqual(len(wire), 32)
        self.assertEqual(origin, self.gateway.sock.getsockname())
        self.assertEqual(proto.decode_telemetry(wire)['frame_yaw'], 12.34)

    def test_other_client_and_invalid_data_do_not_redirect_or_emit_json(self):
        self.receive_control(self.first)
        self.assertFalse(self.receive_control(self.second, proto.encode_command(proto.CMD_MANUAL, 100))[1])
        self.assertFalse(self.receive_control(self.first, b'{"cmd":"ping"}')[1])
        self.source()
        self.assertEqual(len(self.first.recv(2048)), 32)
        with self.assertRaises(socket.timeout):
            self.second.recv(2048)
        self.assertFalse(self.commands)

    def test_expired_peer_receives_nothing_and_new_client_can_take_lease(self):
        self.receive_control(self.first)
        self.now = 2.0
        self.source()
        with self.assertRaises(socket.timeout):
            self.first.recv(2048)
        self.assertEqual(self.commands[-1]['cmd'], 'stop')
        self.receive_control(self.second)
        self.source(2.0)
        self.assertEqual(len(self.second.recv(2048)), 32)


if __name__ == '__main__':
    unittest.main()
