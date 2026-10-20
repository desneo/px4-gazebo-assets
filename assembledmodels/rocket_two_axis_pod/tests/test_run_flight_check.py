#!/usr/bin/env python3
"""以进程替身验证完整飞行入口，不启动 Gazebo、PX4 或网络连接。"""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class FlightLauncherTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='rocket-pod-flight-launcher-')
        self.addCleanup(temporary.cleanup)
        self.work = Path(temporary.name)
        self.repo = self.work / 'repo'
        self.model = self.repo / 'assembledmodels/rocket_two_axis_pod'
        self.script = self.model / 'tests/run_flight_check.sh'
        self.script.parent.mkdir(parents=True)
        shutil.copy2(ROOT / 'tests/run_flight_check.sh', self.script)
        self.events = self.work / 'events.jsonl'
        self.bin = self.work / 'bin'
        self.bin.mkdir()
        self.tmp = self.work / 'tmp'
        self.tmp.mkdir()
        self.plugin = self.repo / 'models/gimbals/two_axis_pod/controller/build/lib/libgz-two-axis-pod-controller.so'
        self.plugin.parent.mkdir(parents=True)
        self.plugin.touch()
        self.px4_root = self.work / 'px4'
        px4 = self.px4_root / 'build/px4_sitl_default/bin/px4'
        (self.px4_root / 'build/px4_sitl_default/etc/init.d-posix/airframes').mkdir(parents=True)
        px4.parent.mkdir(parents=True)
        common = '''import json, os, signal, sys, time
from pathlib import Path
def event(kind, **values):
    with open(os.environ['STUB_EVENTS'], 'a') as stream:
        stream.write(json.dumps(dict(kind=kind, pid=os.getpid(), args=sys.argv[1:], **values)) + '\\n')
def idle(kind):
    event(kind, partition=os.environ.get('GZ_PARTITION'), plugins=os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH'),
          model=os.environ.get('PX4_GZ_MODEL_NAME'), world=os.environ.get('PX4_GZ_WORLD'))
    def stop(signum, frame):
        event(kind + '_terminated')
        sys.exit(0)
    signal.signal(signal.SIGTERM, stop)
    while True:
        time.sleep(.02)
'''
        self.executable(px4, common + "idle('px4')\n")
        self.executable(self.bin / 'gz', common + '''
if sys.argv[1] == 'sim':
    idle('gazebo')
elif sys.argv[1] == 'topic':
    event('imu')
    print('orientation { w: 1 }')
    sys.exit(int(os.environ.get('STUB_IMU_CODE', '0')))
''')
        runner = self.bin / 'flight-python'
        self.executable(runner, '''import os, sys
args = sys.argv[1:]
if args[:1] == ['-c']:
    sys.exit(0)  # 可选仿真依赖边界替身。
if args[:1] == ['-']:
    source = sys.stdin.read()
    if 'socket.SOCK_DGRAM' in source:
        sys.exit(0)  # 不创建真实套接字。
    sys.argv = args
    exec(compile(source, '<launcher-stdin>', 'exec'))
else:
    os.execv(sys.executable, [sys.executable, *args])
''')
        generator = self.model / 'tools/generate_dynamic_model.py'
        generator.parent.mkdir()
        generator.write_text(common + '''
event('generate')
output = Path(sys.argv[sys.argv.index('--output-dir') + 1])
(output / 'tests').mkdir(parents=True)
(output / 'px4').mkdir()
(output / 'tests/world.sdf').write_text('<sdf/>')
(output / 'px4/4023_gz_rocket_two_axis_pod').write_text('param set-default SYS_AUTOSTART 4023\\n')
config = dict(model=dict(name='rocket_two_axis_pod'),
              simulation=dict(world_name='rocket_two_axis_pod_test', imu_wait_timeout_s=3),
              px4=dict(airframe_id=4023), flight_test=dict(hover_seconds=30, cruise_seconds=30,
              cruise_speed_km_h=200, sprint_seconds=10, sprint_speed_km_h=300))
(output / 'parameters.json').write_text(json.dumps(config))
''')
        flight = self.repo / 'models/vehicles/rocket_racing_quad/tests/flight_check.py'
        flight.parent.mkdir(parents=True)
        flight.write_text(common + '''
for attempt in range(100):
    if '"kind": "px4"' in Path(os.environ['STUB_EVENTS']).read_text():
        break
    time.sleep(.01)
config = Path(sys.argv[sys.argv.index('--config') + 1])
event('flight', config=json.loads(config.read_text()), partition=os.environ.get('GZ_PARTITION'))
result = Path(sys.argv[sys.argv.index('--log') + 1])
result.write_text(json.dumps(dict(stub=True, model_parameters=json.loads(config.read_text()))))
sys.exit(int(os.environ.get('STUB_FLIGHT_CODE', '0')))
''')
        self.env = dict(os.environ, PX4_ROOT=str(self.px4_root), FLIGHT_PYTHON=str(runner),
                        STUB_EVENTS=str(self.events), TMPDIR=str(self.tmp),
                        PATH=f'{self.bin}:{os.environ["PATH"]}')
        self.env.pop('PX4_INSTANCE', None)

    @staticmethod
    def executable(path, source):
        path.write_text(f'#!{sys.executable}\n' + source)
        path.chmod(0o755)

    def run_launcher(self, mode='headless', code=0, instance=0):
        result = self.work / 'result with spaces.json'
        env = dict(self.env, STUB_FLIGHT_CODE=str(code), PX4_INSTANCE=str(instance))
        completed = subprocess.run(['bash', str(self.script), mode, str(result)], env=env,
                                   text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20)
        runs = list(self.tmp.glob('rocket-two-axis-pod-flight.*'))
        events = [json.loads(line) for line in self.events.read_text().splitlines()] if self.events.exists() else []
        return completed, result, runs, events

    def test_headless_uses_snapshot_thresholds_and_cleans_up(self):
        completed, result, runs, events = self.run_launcher()
        self.assertEqual(completed.returncode, 0, completed.stdout)
        self.assertTrue(result.is_file())
        self.assertEqual(len(runs), 1)
        run = runs[0]
        self.assertEqual((run / 'exit_code.txt').read_text().strip(), '0')
        execution = json.loads((run / 'execution.json').read_text())
        self.assertEqual(execution['px4_instance'], 0)
        self.assertEqual(execution['mavlink_endpoint'], 'udpin:0.0.0.0:14540')
        names = [event['kind'] for event in events]
        self.assertLess(names.index('imu'), names.index('px4'))
        self.assertIn('gazebo_terminated', names)
        self.assertIn('px4_terminated', names)
        gazebo, = [event for event in events if event['kind'] == 'gazebo']
        self.assertEqual(gazebo['args'][:4], ['sim', '-s', '-r', '--headless-rendering'])
        flight, = [event for event in events if event['kind'] == 'flight']
        self.assertIn('--ground-truth', flight['args'])
        self.assertEqual(flight['args'][flight['args'].index('--endpoint') + 1], 'udpin:0.0.0.0:14540')
        self.assertEqual(flight['config']['flight_test'], dict(hover_seconds=30, cruise_seconds=30,
            cruise_speed_km_h=200, sprint_seconds=10, sprint_speed_km_h=300))
        self.assertEqual(flight['partition'], gazebo['partition'])
        for option in ('--hover-seconds', '--cruise-seconds', '--sprint-seconds', '--cruise-speed', '--sprint-speed'):
            self.assertNotIn(option, flight['args'])
        self.assertTrue((run / 'etc/init.d-posix/airframes/4023_gz_rocket_two_axis_pod').is_file())
        self.assertTrue((run / 'rootfs').is_dir())

    def test_sitl_instance_and_original_failure_code(self):
        completed, _, runs, events = self.run_launcher(mode='sitl', code=47, instance=1)
        self.assertEqual(completed.returncode, 47, completed.stdout)
        self.assertEqual((runs[0] / 'exit_code.txt').read_text().strip(), '47')
        gazebo, = [event for event in events if event['kind'] == 'gazebo']
        self.assertEqual(gazebo['args'][:2], ['sim', '-r'])
        self.assertNotIn('--headless-rendering', gazebo['args'])
        px4, = [event for event in events if event['kind'] == 'px4']
        self.assertEqual(px4['args'][:2], ['-i', '1'])
        self.assertEqual(px4['world'], 'rocket_two_axis_pod_test')
        flight, = [event for event in events if event['kind'] == 'flight']
        self.assertEqual(flight['args'][flight['args'].index('--endpoint') + 1], 'udpin:0.0.0.0:14541')
        execution = json.loads((runs[0] / 'execution.json').read_text())
        self.assertEqual(execution['px4_instance'], 1)

    def test_missing_plugin_fails_before_starting_services(self):
        self.plugin.unlink()
        completed, _, runs, events = self.run_launcher()
        self.assertEqual(completed.returncode, 1)
        self.assertIn('缺少两轴吊舱控制插件', completed.stdout)
        self.assertFalse(runs)
        self.assertFalse(events)

    def test_imu_failure_prevents_px4_and_preserves_code(self):
        self.env['STUB_IMU_CODE'] = '23'
        completed, _, runs, events = self.run_launcher()
        self.assertEqual(completed.returncode, 23, completed.stdout)
        names = [event['kind'] for event in events]
        self.assertNotIn('px4', names)
        self.assertNotIn('flight', names)
        self.assertIn('gazebo_terminated', names)
        self.assertEqual((runs[0] / 'exit_code.txt').read_text().strip(), '23')

    def test_invalid_instance_rejected_before_start(self):
        completed, _, runs, events = self.run_launcher(instance=9)
        self.assertEqual(completed.returncode, 2)
        self.assertIn('PX4_INSTANCE', completed.stdout)
        self.assertFalse(runs)
        self.assertFalse(events)


if __name__ == '__main__':
    unittest.main(verbosity=2)
