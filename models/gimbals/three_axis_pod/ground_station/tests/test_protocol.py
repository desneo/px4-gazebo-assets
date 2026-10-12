#!/usr/bin/env python3
"""UDP 协议逻辑自验证；仅需 Python 标准库，不启动 GUI、视频或 Gazebo。

通过 AST 加载业务类，绕过模块顶层的 cv2、gi 和 gz 依赖。
"""
import ast
import contextlib
import io
import json
import math
import uuid
from pathlib import Path
import threading
import time
import types
import unittest


ROOT = Path(__file__).resolve().parents[5]


def load_definitions(path, names, namespace):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    definitions = [node for node in tree.body
                   if isinstance(node, (ast.ClassDef, ast.FunctionDef))
                   and node.name in names]
    if {node.name for node in definitions} != set(names):
        raise AssertionError(f"缺少待验证定义: {path}")
    exec(compile(ast.Module(body=definitions, type_ignores=[]), str(path), "exec"),
         namespace)
    return namespace


BRIDGE = load_definitions(
    ROOT / "models/gimbals/three_axis_pod/controller/service/gimbal_bridge.py",
    {"_as_int", "_Client", "GimbalBridge"},
    {"time": time, "threading": threading, "json": json, "math": math, "uuid": uuid,
     "SEQ_WINDOW": 1024, "MAX_PENDING": 4096, "RESULT_TIMEOUT": 30.,
     "SESSION_RETENTION": 60., "MAX_DATAGRAM": 8192,
     "StringMsg": type("StringMsg", (), {}),
     "PASSTHROUGH_CMDS": {"track", "stop", "home", "reset", "manual", "reset_fault"}})
CLIENT = load_definitions(
    ROOT / "models/gimbals/three_axis_pod/ground_station/ground_station.py", {"UdpClient"},
    {"threading": threading, "time": time, "json": json, "MAX_DATAGRAM": 8192})


class BridgeProtocolTests(unittest.TestCase):
    def setUp(self):
        self.bridge = BRIDGE["GimbalBridge"].__new__(BRIDGE["GimbalBridge"])
        self.bridge.clients_lock = threading.RLock()
        self.bridge.args = types.SimpleNamespace(client_timeout=5., telemetry_hz=50,
                                                  mount="/gimbal")
        self.bridge.clock_domain = "test-boot"
        self.bridge.retired_clients = {}
        self.bridge.pending_requests = {}
        self.client = BRIDGE["_Client"](("127.0.0.1", 10001))
        self.bridge.clients = {self.client.addr: self.client}
        self.sent = []
        self.published = []
        self.bridge._send_json = lambda addr, payload: self.sent.append(
            (addr, json.loads(payload) if isinstance(payload, str) else payload))
        self.bridge._cmd_pub = types.SimpleNamespace(
            publish=lambda msg: self.published.append(json.loads(msg.data)))
        self.bridge.snapshot = lambda: None

    def command(self, **payload):
        self.bridge._handle_command(self.client, payload)

    def result(self, index=-1, **extra):
        request = self.published[index]
        self.bridge._forward_command_result(dict(type="command_result", cmd=request["cmd"],
                                                 request_id=request["request_id"],
                                                 ok=True, **extra))

    def test_ping_echoes_seq_and_timestamp_without_publish(self):
        self.command(cmd="ping", seq=1, t=123.5)
        self.assertEqual(self.sent[-1][1], {"type": "pong", "seq": 1, "t": 123.5})
        self.assertEqual(self.published, [])

    def test_manual_preserves_three_axis_rates(self):
        self.command(cmd="manual", seq=1, yaw=20, pitch=-10, roll=5)
        actual = dict(self.published[0])
        self.assertIsInstance(actual.pop("request_id"), str)
        self.assertEqual(actual, {"cmd": "manual", "yaw": 20., "pitch": -10., "roll": 5.})
        self.assertEqual(self.sent, [])

    def test_duplicate_replays_result_without_second_execution(self):
        self.command(cmd="home", seq=1)
        self.command(cmd="home", seq=1)
        self.assertEqual(len(self.published), 1)
        self.result()
        first_result = self.sent[-1]
        self.command(cmd="home", seq=1)
        self.assertEqual(len(self.published), 1)
        self.assertEqual(self.sent[-1], first_result)

    def test_missing_track_box_returns_correlated_failure(self):
        self.command(cmd="track", seq=10)
        self.assertFalse(self.sent[-1][1]["ok"])
        self.assertEqual(self.sent[-1][1]["seq"], 10)

    def test_same_command_results_keep_request_seq_in_any_order(self):
        self.command(cmd="home", seq=11)
        self.command(cmd="home", seq=12)
        self.assertNotEqual(self.published[0]["request_id"], self.published[1]["request_id"])
        self.result(1)
        self.result(0)
        self.assertEqual([payload["seq"] for _, payload in self.sent], [12, 11])

    def test_multiple_clients_receive_only_their_own_result(self):
        other = BRIDGE["_Client"](("127.0.0.1", 10002))
        self.bridge.clients[other.addr] = other
        self.command(cmd="home", seq=1)
        self.bridge._handle_command(other, {"cmd": "home", "seq": 1})
        self.result(1)
        self.assertEqual(self.sent[-1][0], other.addr)
        self.assertEqual(len(self.sent), 1)
        self.result(0)
        self.assertEqual(self.sent[-1][0], self.client.addr)

    def test_reordered_control_is_explicitly_rejected(self):
        self.command(cmd="stop", seq=14)
        self.command(cmd="reset", seq=13)
        self.assertEqual(len(self.published), 1)
        self.assertEqual(self.sent[-1][1]["seq"], 13)
        self.assertFalse(self.sent[-1][1]["ok"])

    def test_reordered_manual_cannot_restore_motion_after_zero(self):
        self.command(cmd="manual", seq=14, yaw=0)
        self.command(cmd="manual", seq=13, yaw=20)
        self.assertEqual(len(self.published), 1)
        self.assertFalse(self.sent[-1][1]["ok"])

    def test_invalid_values_return_correlated_failure(self):
        cases = [{"cmd": "manual", "yaw": float("nan")},
                 {"cmd": "manual", "yaw": float("inf")},
                 {"cmd": "manual", "roll": "bad"},
                 {"cmd": "manual", "pitch": True},
                 {"cmd": "track", "box": [1, 2, -3, 4]},
                 {"cmd": "track", "box": [1, 2, None, 4]},
                 {"cmd": []}]
        for seq, payload in enumerate(cases, 1):
            with self.subTest(payload=payload):
                self.command(seq=seq, **payload)
                self.assertFalse(self.sent[-1][1]["ok"])
                self.assertEqual(self.sent[-1][1]["seq"], seq)
        self.assertEqual(self.published, [])

    def test_invalid_json_top_level_is_rejected(self):
        for raw in (b"[]", b"null", b"1", b"garbage"):
            self.bridge._on_datagram(raw, self.client.addr)
            self.assertEqual(self.sent[-1][1]["type"], "error")
        self.assertEqual(self.published, [])

    def test_same_seq_different_command_cannot_reuse_cached_success(self):
        self.command(cmd="home", seq=1)
        self.result()
        self.command(cmd="stop", seq=1)
        self.assertFalse(self.sent[-1][1]["ok"])
        self.assertEqual(len(self.published), 1)

    def test_cache_is_bounded_and_old_requests_explicitly_fail(self):
        for seq in range(1, 1030):
            self.command(cmd="home", seq=seq)
            self.result()
        self.assertEqual(len(self.client.seen), 1024)
        self.assertEqual(len(self.client.results), 1024)
        self.command(cmd="home", seq=1)
        self.assertFalse(self.sent[-1][1]["ok"])
        self.assertEqual(len(self.published), 1029)

    def test_no_seq_still_receives_only_its_result(self):
        self.command(cmd="home")
        self.result()
        self.assertIsNone(self.sent[-1][1]["seq"])
        self.assertEqual(len(self.sent), 1)

    def test_unmatched_and_wrong_command_results_are_ignored(self):
        self.command(cmd="home", seq=1)
        self.bridge._forward_command_result({"cmd": "home", "ok": True})
        self.bridge._forward_command_result({"cmd": "home", "ok": True,
                                             "request_id": "unknown"})
        self.bridge._forward_command_result({"cmd": "stop", "ok": True,
                                             "request_id": self.published[0]["request_id"]})
        self.assertEqual(self.sent, [])
        self.result()
        self.assertEqual(len(self.sent), 1)

    def test_expired_client_reregistration_does_not_reexecute_or_receive_late_result(self):
        self.command(cmd="home", seq=1)
        self.client.last_seen -= 6
        with contextlib.redirect_stdout(io.StringIO()):
            self.bridge._on_datagram(b'{"cmd":"home","seq":1}', self.client.addr)
        self.assertEqual(len(self.published), 1)
        self.assertEqual(self.sent[-1][1]["seq"], 1)
        self.assertFalse(self.sent[-1][1]["ok"])
        count = len(self.sent)
        self.result()
        self.assertEqual(len(self.sent), count)

    def test_plugin_timeout_is_correlated_and_does_not_reexecute(self):
        self.command(cmd="home", seq=1)
        request_id = self.published[0]["request_id"]
        cl, seq, cmd, started = self.bridge.pending_requests[request_id]
        self.bridge.pending_requests[request_id] = (cl, seq, cmd, started - 31)
        self.bridge._expire_requests(time.monotonic())
        self.assertFalse(self.sent[-1][1]["ok"])
        self.assertEqual(self.sent[-1][1]["seq"], 1)
        self.command(cmd="home", seq=1)
        self.assertEqual(len(self.published), 1)

    def test_publish_failure_returns_correlated_failure(self):
        self.bridge._cmd_pub.publish = lambda _msg: False
        self.command(cmd="stop", seq=1)
        self.assertFalse(self.sent[-1][1]["ok"])
        self.assertEqual(self.sent[-1][1]["seq"], 1)
        self.assertEqual(self.bridge.pending_requests, {})

    def test_unseen_reordered_control_after_status_is_executed(self):
        self.command(cmd="status", seq=14)
        self.command(cmd="home", seq=13)
        self.assertEqual(self.published[-1]["cmd"], "home")
        self.result()
        self.assertEqual(self.sent[-1][1]["seq"], 13)
        self.assertTrue(self.sent[-1][1]["ok"])

    def test_result_arrival_after_client_expiry_retains_unknown_result(self):
        self.command(cmd="home", seq=1)
        self.client.last_seen -= 6
        self.result()
        self.assertEqual(self.sent, [])
        self.assertNotIn(self.client.addr, self.bridge.clients)
        self.assertFalse(json.loads(self.client.results[1][0])["ok"])
        self.assertEqual(self.bridge.pending_requests, {})

    def test_pending_queue_limit_returns_failure_without_publish(self):
        self.bridge.pending_requests = {str(index): (self.client, None, "home", time.monotonic())
                                        for index in range(4096)}
        self.command(cmd="stop", seq=1)
        self.assertEqual(self.published, [])
        self.assertFalse(self.sent[-1][1]["ok"])
        self.assertEqual(self.sent[-1][1]["seq"], 1)

    def test_concurrent_clients_and_callbacks_keep_exact_association(self):
        clients = [BRIDGE["_Client"](("127.0.0.1", 11000 + index)) for index in range(16)]
        self.bridge.clients.update((cl.addr, cl) for cl in clients)
        threads = [threading.Thread(target=self.bridge._handle_command,
                                     args=(cl, {"cmd": "home", "seq": index + 1}))
                   for index, cl in enumerate(clients)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        associations = {request_id: (cl.addr, seq)
                        for request_id, (cl, seq, _cmd, _started)
                        in self.bridge.pending_requests.items()}
        threads = [threading.Thread(target=self.result, args=(index,))
                   for index in reversed(range(len(clients)))]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(len(self.sent), len(clients))
        self.assertEqual({(addr, data["seq"]) for addr, data in self.sent},
                         set(associations.values()))
        self.assertEqual(self.bridge.pending_requests, {})

    def test_continuous_manual_and_reset_fault_do_not_wait_for_results(self):
        for seq in range(1, 2001):
            self.command(cmd="manual", seq=seq, yaw=20)
        self.command(cmd="reset_fault", seq=2001)
        self.assertEqual(len(self.published), 2001)
        self.assertEqual(self.bridge.pending_requests, {})
        self.assertEqual(self.sent, [])

    def test_invalid_datagrams_do_not_register_or_refresh_client(self):
        last_seen = self.client.last_seen
        for raw in (b"garbage", b"[]", b'{"cmd":[]}', b'{"cmd":"unknown"}'):
            self.bridge._on_datagram(raw, self.client.addr)
            self.bridge._on_datagram(raw, ("127.0.0.1", 12345))
        self.assertEqual(self.client.last_seen, last_seen)
        self.assertEqual(list(self.bridge.clients), [self.client.addr])


class ClientProtocolTests(unittest.TestCase):
    def setUp(self):
        self.client = CLIENT["UdpClient"].__new__(CLIENT["UdpClient"])
        self.client.lock = threading.RLock()
        self.client.last_result = None
        self.client.results = []
        self.client._pending = {}
        self.client.on_log = lambda message: None
        self.client.last_rx_mono = 0.
        self.client.telemetry = {}
        self.client.frame_t0 = {}
        self.client.messages = 0

    def test_telemetry_populates_frame_timestamp_lookup(self):
        self.client._on_datagram(json.dumps({"type": "telemetry", "mode": "IDLE",
                                           "frame_table": [[42, 123.5]]}).encode())
        self.assertEqual(self.client.lookup(42), 123.5)
        self.assertEqual(self.client.messages, 1)
        self.assertEqual(self.client.snapshot()["mode"], "IDLE")

    def test_ack_removes_completed_request_from_pending(self):
        self.client._pending[1] = {"result": None}
        self.client._on_datagram(b'{"type":"command_result","cmd":"home",'
                                 b'"seq":1,"ok":true}')
        self.assertEqual(self.client.find_result(cmd="home", seq=1)["ok"], True)
        self.assertNotIn(1, self.client._pending)

    def test_invalid_json_does_not_refresh_server_liveness(self):
        self.client._on_datagram(b"garbage")
        self.assertEqual(self.client.last_rx_mono, 0.)


if __name__ == "__main__":
    unittest.main(verbosity=2)
