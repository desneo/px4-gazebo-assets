#!/usr/bin/env python3
"""自动广角阈值、同一真值目标恢复与网关镜头同步回归。"""
from pathlib import Path
import json
import os
import shlex
import subprocess
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common import protocol as proto
from gateway.main import Gateway
from gz.msgs10.stringmsg_pb2 import StringMsg


class LensSwitchTest(unittest.TestCase):
    def test_controller_threshold_and_same_truth_resume(self):
        shared = Path(os.environ.get("AISHENG_SHARED_CONTROLLER", str(
            ROOT.parent / "three_axis_pod" / "controller")))
        sources = [ROOT / "tests/lens_switch_test.cc",
                   ROOT / "controller/src/control.cc",
                   ROOT / "controller/src/kinematics.cc",
                   ROOT / "controller/src/truth_tracking.cc",
                   shared / "src/kinematics.cc", shared / "src/tracker.cc"]
        flags = shlex.split(subprocess.check_output(
            ["pkg-config", "--cflags", "--libs", "opencv4"], text=True))
        with tempfile.TemporaryDirectory(prefix="aisheng-lens-") as work:
            binary = str(Path(work) / "lens_switch_test")
            subprocess.run(["c++", "-std=c++17", "-O1", "-I" + str(ROOT / "controller/include"),
                            "-I" + str(shared / "include"), *map(str, sources),
                            *flags, "-o", binary], check=True)
            subprocess.run([binary], check=True)

    def gateway(self):
        gateway = Gateway.__new__(Gateway)
        gateway.lock = threading.Lock()
        gateway.args = SimpleNamespace(verbose=False)
        gateway.active_lens = "tele"
        gateway.pending_lens = None
        gateway.zoom = 4
        gateway.rx_telemetry = 0
        gateway.auto_lock = True
        gateway.tracking_command = True
        gateway._build_telemetry = lambda data: data
        gateway._send_telemetry = lambda frame: None
        return gateway

    def test_automatic_plugin_lens_change_preserves_lock_intent(self):
        gateway = self.gateway()
        gateway._on_telemetry(StringMsg(data=json.dumps({"camera": {"active": "wide"}})))
        self.assertEqual(gateway.active_lens, "wide")
        self.assertEqual(gateway.zoom, 1)
        self.assertTrue(gateway.auto_lock)
        self.assertTrue(gateway.tracking_command)

    def test_pending_lens_blocks_old_camera_tracking_request(self):
        gateway = self.gateway()
        gateway.active_lens = "wide"
        gateway.latest = {"camera": {"active": "wide"}}
        gateway.pending_lens = "tele"
        gateway.tracking_command = False
        published = []
        gateway._publish = published.append
        gateway._start_track({"center_px": [960, 540]}, auto=True)
        self.assertEqual(published, [])
        self.assertFalse(gateway.tracking_command)
        self.assertTrue(gateway.auto_lock)

    def test_manual_lens_status_waits_for_actual_camera_ack(self):
        gateway = self.gateway()
        gateway.active_lens = "wide"
        gateway.published = []
        gateway._publish = gateway.published.append
        gateway._handle(proto.ControlFrame(command=proto.CMD_DETECTOR_SWITCH, p1=2))
        self.assertEqual(gateway.active_lens, "wide")
        self.assertEqual(gateway.pending_lens, "tele")
        self.assertEqual(gateway.published[-1], {"cmd": "set_camera", "sensor": "tele"})
        gateway._on_telemetry(StringMsg(data=json.dumps({"camera": {"active": "wide"}})))
        self.assertEqual(gateway.active_lens, "wide")
        gateway._on_telemetry(StringMsg(data=json.dumps({"camera": {"active": "tele"}})))
        self.assertEqual(gateway.active_lens, "tele")
        self.assertIsNone(gateway.pending_lens)

    def test_manual_lens_request_ignores_old_telemetry_until_ack(self):
        gateway = self.gateway()
        gateway.pending_lens = "tele"
        gateway._on_telemetry(StringMsg(data=json.dumps({"camera": {"active": "wide"}})))
        self.assertEqual(gateway.active_lens, "tele")
        gateway._on_telemetry(StringMsg(data=json.dumps({"camera": {"active": "tele"}})))
        self.assertIsNone(gateway.pending_lens)
        gateway._on_telemetry(StringMsg(data=json.dumps({"camera": {"active": "wide"}})))
        self.assertEqual(gateway.active_lens, "wide")


if __name__ == "__main__":
    unittest.main()
