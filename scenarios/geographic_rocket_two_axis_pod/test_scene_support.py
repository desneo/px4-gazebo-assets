"""配置、坐标转换、连续日志与安全进程清理的离线检查。"""

import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

from flight_patrol import PatrolTruth, SampleSink, load_config
from process_state import process_info, read_live
from prepare_world import prepare

HERE = Path(__file__).resolve().parent


class SceneSupportTest(unittest.TestCase):
    def test_partial_config_and_unknown_field(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'config.json'
            path.write_text(json.dumps({'altitude_m': 300, 'speed_m_s': 12}))
            config = load_config(path)
            self.assertEqual(config['altitude_m'], 300)
            self.assertEqual(config['leg_length_m'], 300)
            path.write_text(json.dumps({'altitdue_m': 80}))
            with self.assertRaisesRegex(ValueError, '未知场景配置字段'):
                load_config(path)

    def test_world_truth_enu_to_ned(self):
        truth = PatrolTruth.__new__(PatrolTruth)
        truth.origin = (10, 20, 30)
        self.assertEqual(truth.to_ned((1, 15, 25, 110), 7), [5, 5, -73])

    def test_configurable_target_is_saved_with_marker_and_close_camera(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            model = directory / 'model.sdf'
            model.write_text('<sdf version="1.9"><model name="rocket_two_axis_pod">'
                             '<static>false</static><link name="base_link"/></model></sdf>')
            output = prepare(directory / 'world.sdf', model, view='close',
                             target_position=(200., 150., 150.), target_size=6.)
            world = ET.parse(output).find('world')
            target = world.find("model[@name='AERIAL_TRACKING_TARGET']")
            self.assertEqual(target.findtext('pose'), '200.0 150.0 150.0 0 0 0')
            self.assertFalse(target.findall('.//collision'))
            self.assertFalse(world.find("model[@name='ROCKET_TAKEOFF_LOCATION_MARKER']").findall('.//collision'))
            self.assertEqual(world.findtext("gui/plugin[@filename='MinimalScene']/camera_clip/near"), '0.005')

    def test_continuous_sink_does_not_retain_samples(self):
        stream = io.StringIO()
        sink = SampleSink(stream, 'px4')
        for index in range(10000):
            sink.append({'sim_time_s': index})
        self.assertEqual(sink.count, 10000)
        self.assertEqual(len(sink), 0)
        self.assertEqual(len(stream.getvalue().splitlines()), 10000)

    def test_pid_start_time_mismatch_is_not_owned(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'launcher.pid'
            stamp = process_info(os.getpid())['start_time']
            state = {'pid': os.getpid(), 'start_time': stamp, 'group': False}
            path.write_text(json.dumps(state))
            self.assertIsNotNone(read_live(path))
            state['start_time'] = str(int(stamp) + 1)
            path.write_text(json.dumps(state))
            self.assertIsNone(read_live(path))

    def test_group_cleanup_after_leader_exit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'service.pid'
            leader = subprocess.Popen(
                [sys.executable, '-c',
                 'import subprocess,time; '
                 'subprocess.Popen(["sleep","20"]); time.sleep(.5)'],
                start_new_session=True)
            try:
                subprocess.run([sys.executable, str(HERE / 'process_state.py'),
                                'record', str(path), str(leader.pid)], check=True)
                leader.wait(timeout=3)
                self.assertIsNotNone(read_live(path), '组长退出后应仍识别其session中的子进程')
                subprocess.run([sys.executable, str(HERE / 'process_state.py'),
                                'signal', str(path), 'TERM'], check=True)
            finally:
                try:
                    os.killpg(leader.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                leader.wait(timeout=3)


if __name__ == '__main__':
    unittest.main()
