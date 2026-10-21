#!/usr/bin/env python3
"""自检使用实际客户端的协议校验；隔离网络和后台线程。"""
import ast
import contextlib
import io
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

from test_client import UdpClient


SOURCE = Path(__file__).resolve().parents[1] / 'ground_station.py'


class SelftestProtocolTests(unittest.TestCase):
    def run_selftest(self, version=6, convention='motion_head_joint', stale=False):
        class SnapshotClient:
            public_angles_ready = UdpClient.public_angles_ready
            _valid_attitude = staticmethod(UdpClient._valid_attitude)

            def __init__(self):
                self.bind_error = None
                self.identity_ready = True
                self.lock = threading.RLock()
                self.client_timeout = 5.
                self.last_telemetry_mono = time.monotonic() - (10 if stale else 0)
                self.hello = {'version': version, 'angle_convention': convention,
                              'aircraft_id': 1}
                self.telemetry = {
                    'type': 'telemetry', 'protocol_version': version,
                    'angle_convention': convention, 'frame_table': [[1, 0]],
                    'attitude_deg': {axis: {'pos': 0., 'vel': 0., 'acc': 0.}
                                     for axis in ('yaw', 'pitch')}}

            def start(self):
                pass

            def stop(self):
                pass

            def snapshot(self):
                return self.telemetry

            def stats(self):
                return {'identity_ready': True, 'identity_error': None,
                        'messages': 1, 'pongs': 1, 'pings_sent': 1,
                        'connected': True, 'pending': 0, 'results': [], 'errors': []}

        client = SnapshotClient()
        namespace = {
            'time': time, 'UdpClient': lambda *args, **kwargs: client,
            'ManualController': lambda _: SimpleNamespace(start=lambda: None, stop=lambda: None),
            'udp_endpoint': lambda _: '127.0.0.1:8766',
        }
        tree = ast.parse(SOURCE.read_text(encoding='utf-8'))
        definition = next(node for node in tree.body
                          if isinstance(node, ast.FunctionDef) and node.name == 'run_selftest')
        exec(compile(ast.Module(body=[definition], type_ignores=[]), str(SOURCE), 'exec'), namespace)
        args = SimpleNamespace(selftest=.001, send_home=False, send_manual=False,
                               test_timeout=None)
        with contextlib.redirect_stdout(io.StringIO()) as output:
            result = namespace['run_selftest']({}, args)
        return result, output.getvalue()

    def test_old_or_mismatched_protocol_fails_without_motion_requests(self):
        for version, convention in ((5, 'upright_joint'), (6, 'upright_joint'),
                                    (5, 'motion_head_joint')):
            with self.subTest(version=version, convention=convention):
                result, output = self.run_selftest(version, convention)
                self.assertEqual(result, 9)
                self.assertIn('必须同时符合协议 6 / motion_head_joint', output)

    def test_stale_mechanical_telemetry_fails_even_with_fresh_identity(self):
        self.assertEqual(self.run_selftest(stale=True)[0], 9)

    def test_matching_protocol_and_fresh_mechanical_telemetry_pass(self):
        self.assertEqual(self.run_selftest()[0], 0)


if __name__ == '__main__':
    unittest.main()
