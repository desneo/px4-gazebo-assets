"""Station 只管理本机、同仓库且身份匹配的网关，并防止 PID 复用误停。"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import station_lifecycle as lifecycle


class LifecycleTests(unittest.TestCase):
    def test_match_requires_same_repository_port_and_identity(self):
        config = {"udp_port": 8770, "aircraft_id": 1}
        command = ["python3", str(lifecycle.BRIDGE), "--udp-port=8770", "--aircraft-id", "1"]
        self.assertTrue(lifecycle.matches_bridge(command, config))
        self.assertFalse(lifecycle.matches_bridge(command, dict(config, aircraft_id=2)))
        self.assertFalse(lifecycle.matches_bridge(command, dict(config, udp_port=8771)))
        command[1] = "/another-checkout/gimbal_bridge.py"
        self.assertFalse(lifecycle.matches_bridge(command, config))

    def test_process_snapshot_identifies_current_process(self):
        record = lifecycle.process_info(os.getpid())
        self.assertEqual(record["pid"], os.getpid())
        self.assertTrue(lifecycle.same_process(record))

    def test_reused_pid_is_never_signalled(self):
        record = {"pid": 42, "starttime": "10", "argv": ["python3"]}
        with patch.object(lifecycle, "process_info", return_value=dict(record, starttime="11")):
            with patch.object(lifecycle.os, "kill") as kill:
                lifecycle.stop_process(record)
                kill.assert_not_called()

    def test_remote_gateway_is_not_initialized_or_stopped(self):
        with patch.object(lifecycle, "local_host", return_value=False):
            with patch.object(lifecycle.subprocess, "run") as run:
                with self.assertRaisesRegex(RuntimeError, "只管理本机"):
                    lifecycle.ensure_bridge({"gimbal_host": "192.0.2.1"}, Path("session.json"))
                run.assert_not_called()

    def test_option_handles_equals_and_last_override(self):
        self.assertEqual(lifecycle.option(["--udp-port=8766", "--udp-port", "8770"],
                                         "--udp-port", "0"), "8770")

    def test_background_process_ignoring_sigint_stops_with_sigterm(self):
        child = subprocess.Popen(
            [sys.executable, "-u", "-c",
             "import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); "
             "print('ready',flush=True); time.sleep(30)"],
            stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), "ready")
            lifecycle.stop_process(lifecycle.process_info(child.pid), timeout=4)
            self.assertEqual(child.wait(timeout=1), -signal.SIGTERM)
        finally:
            if child.poll() is None:
                child.terminate()
                child.wait(timeout=2)
            child.stdout.close()
