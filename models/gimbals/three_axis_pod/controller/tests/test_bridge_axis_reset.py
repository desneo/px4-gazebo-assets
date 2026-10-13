#!/usr/bin/env python3
"""单轴复位 UDP 校验、回执关联和去重；不依赖 GUI 或 Gazebo。"""
import ast
import json
import math
from pathlib import Path
import threading
import time
import types
import unittest
import uuid

SOURCE = Path(__file__).resolve().parents[1] / 'service/gimbal_bridge.py'
tree = ast.parse(SOURCE.read_text())
names = {'_as_int', '_Client', 'GimbalBridge'}
namespace = dict(time=time, threading=threading, json=json, math=math, uuid=uuid,
    SEQ_WINDOW=1024, MAX_PENDING=4096, RESULT_TIMEOUT=30., SESSION_RETENTION=60.,
    MAX_DATAGRAM=8192, StringMsg=type('StringMsg', (), {}))
selected = [node for node in tree.body if
    (isinstance(node, (ast.ClassDef, ast.FunctionDef)) and node.name in names) or
    (isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'PASSTHROUGH_CMDS'
                                         for t in node.targets))]
exec(compile(ast.Module(body=selected, type_ignores=[]), str(SOURCE), 'exec'), namespace)


class AxisResetProtocolTests(unittest.TestCase):
    def setUp(self):
        self.bridge = namespace['GimbalBridge'].__new__(namespace['GimbalBridge'])
        self.bridge.clients_lock = threading.RLock()
        self.bridge.args = types.SimpleNamespace(client_timeout=5., telemetry_hz=50, mount='/gimbal')
        self.bridge.clock_domain = 'test'
        self.bridge.retired_clients = {}
        self.bridge.pending_requests = {}
        self.client = namespace['_Client'](('127.0.0.1', 10001))
        self.bridge.clients = {self.client.addr: self.client}
        self.sent, self.published = [], []
        self.bridge._send_json = lambda addr, payload: self.sent.append((addr,
            json.loads(payload) if isinstance(payload, str) else payload))
        self.bridge._cmd_pub = types.SimpleNamespace(publish=lambda msg:
            self.published.append(json.loads(msg.data)))
        self.bridge.snapshot = lambda: None

    def command(self, axis='yaw', seq=1, client=None):
        self.bridge._handle_command(client or self.client,
            dict(cmd='reset_axis', axis=axis, seq=seq))

    def result(self, index=0):
        request = self.published[index]
        self.bridge._forward_command_result(dict(type='command_result', cmd='reset_axis',
            request_id=request['request_id'], ok=True, message=''))

    def test_all_axes_publish_reliable_correlated_requests(self):
        for index, axis in enumerate(['yaw', 'pitch', 'roll']):
            self.command(axis, index+1)
            self.assertEqual(self.published[-1]['axis'], axis)
            self.assertIn(self.published[-1]['request_id'], self.bridge.pending_requests)
            self.result(index)
            self.assertEqual(self.sent[-1][1]['seq'], index+1)
            self.assertTrue(self.sent[-1][1]['ok'])

    def test_invalid_axes_are_rejected_without_publishing(self):
        for seq, axis in enumerate([None, '', 'YAW', 'x', [], {}, True, 1], 1):
            self.command(axis, seq)
            self.assertEqual(self.sent[-1][1]['seq'], seq)
            self.assertFalse(self.sent[-1][1]['ok'])
            self.assertEqual(self.sent[-1][1]['message'], 'axis 必须为 yaw、pitch 或 roll')
        self.assertEqual(self.published, [])
        self.bridge._handle_command(self.client, dict(cmd='reset_axis', seq=20))
        self.assertFalse(self.sent[-1][1]['ok'])

    def test_duplicate_replays_without_second_execution(self):
        self.command()
        self.command()
        self.assertEqual(len(self.published), 1)
        self.result()
        response = self.sent[-1]
        self.command()
        self.assertEqual(self.sent[-1], response)
        self.assertEqual(len(self.published), 1)

    def test_changed_axis_same_seq_and_old_command_are_rejected(self):
        self.command('yaw', 10)
        self.command('pitch', 10)
        self.assertFalse(self.sent[-1][1]['ok'])
        self.command('roll', 9)
        self.assertFalse(self.sent[-1][1]['ok'])
        self.assertEqual(len(self.published), 1)

    def test_results_only_reach_originating_client(self):
        other = namespace['_Client'](('127.0.0.1', 10002))
        self.bridge.clients[other.addr] = other
        self.command('yaw', 1)
        self.command('pitch', 1, other)
        self.result(1)
        self.assertEqual(self.sent[-1][0], other.addr)
        self.result(0)
        self.assertEqual(self.sent[-1][0], self.client.addr)

    def test_request_timeout_is_cached_and_correlated(self):
        self.command()
        request_id = self.published[0]['request_id']
        cl, seq, cmd, started = self.bridge.pending_requests[request_id]
        self.bridge.pending_requests[request_id] = cl, seq, cmd, started-31
        self.bridge._expire_requests(time.monotonic())
        self.assertFalse(self.sent[-1][1]['ok'])
        self.assertEqual(self.sent[-1][1]['seq'], 1)
        self.command()
        self.assertEqual(len(self.published), 1)

    def test_publish_failure_returns_correlated_failure(self):
        self.bridge._cmd_pub.publish = lambda msg: False
        self.command()
        self.assertFalse(self.sent[-1][1]['ok'])
        self.assertEqual(self.sent[-1][1]['seq'], 1)
        self.assertEqual(self.bridge.pending_requests, {})

    def test_singular_yaw_roll_stops_but_pitch_can_leave(self):
        self.bridge.snapshot = lambda: {'attitude_deg': {'singular': True}}
        self.bridge._handle_command(self.client, dict(cmd='manual', seq=1, yaw=10))
        self.assertEqual({key: self.published[-1][key] for key in ('cmd', 'yaw', 'pitch', 'roll')},
                         {'cmd': 'manual', 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0})
        self.assertFalse(self.sent[-1][1]['ok'])
        self.bridge._handle_command(self.client, dict(cmd='manual', seq=2, pitch=-10))
        self.assertEqual(self.published[-1]['pitch'], -10)


if __name__ == '__main__':
    unittest.main()
