"""验证手工配置通过真实启动入口保留，且验收快照反映实际运行文件。"""
from pathlib import Path
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import prepare_runtime


class NativeRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='rocket-native-test-')
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.source = self.work / 'source'
        for path in prepare_runtime.asset_paths(ROOT) + [ROOT / 'parameters.json',
                ROOT / 'tools/prepare_runtime.py', ROOT / 'tests/run.sh']:
            target = self.source / path.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
        # 不提供生成器；启动必须读取手调文件而非从 JSON 重建。
        model_path = self.source / 'model.sdf'
        model = ET.parse(model_path)
        base = model.find("model/link[@name='base_link']/inertial")
        base.find('mass').text = '8.2'
        pose = base.find('pose')
        if pose is None:
            pose = ET.SubElement(base, 'pose')
        pose.text = '.01 -.02 -.083 .1 .2 .3'
        base.find('inertia/ixx').text = '.231'
        model.write(model_path)
        self.airframe = next((self.source / 'px4').glob('*_gz_rocket_racing_quad'))
        text = self.airframe.read_text()
        text = text.replace('MPC_XY_VEL_MAX 95', 'MPC_XY_VEL_MAX 47')
        self.airframe.write_text(text)
        world_path = self.source / 'tests/world.sdf'
        world = ET.parse(world_path)
        world.find('world/physics/max_step_size').text = '.002'
        world.write(world_path)
        config = json.loads((self.source / 'parameters.json').read_text())
        config['mass']['body_kg'] = 999
        config['geometry']['body_length_m'] = -1  # 此处只作为生成器输入，不参与启动。
        config['px4']['parameters']['MPC_XY_VEL_MAX'] = 12
        config['flight_test']['hover_seconds'] = 7
        config['simulation']['imu_wait_timeout_s'] = 3
        self.config = self.work / 'custom.json'
        self.config.write_text(json.dumps(config))

    def assert_runtime(self, output):
        for path in prepare_runtime.asset_paths(self.source):
            self.assertEqual(path.read_bytes(), (output / path.relative_to(self.source)).read_bytes())
        snapshot = json.loads((output / 'parameters.json').read_text())
        self.assertAlmostEqual(snapshot['model']['total_mass_kg'], 8.46)
        base = snapshot['model']['links']['base_link']
        self.assertEqual(base['inertial_pose'], '.01 -.02 -.083 .1 .2 .3')
        self.assertEqual(base['inertia']['ixx'], .231)
        self.assertEqual(snapshot['px4']['parameters']['MPC_XY_VEL_MAX'], 47)
        self.assertEqual(snapshot['flight_test']['hover_seconds'], 7)
        self.assertEqual(snapshot['simulation']['imu_wait_timeout_s'], 3)
        self.assertEqual(snapshot['simulation']['physics']['max_step_size'], '.002')
        self.assertNotIn('geometry', snapshot, '不能将未使用的 JSON 物理参数冒充实际配置')
        self.assertEqual(snapshot['runtime']['source_sha256'],
                         prepare_runtime.source_fingerprints(self.source))

    def test_view_and_sitl_launcher_use_native_files(self):
        bindir = self.work / 'bin'
        bindir.mkdir()
        fake_gz = bindir / 'gz'
        fake_gz.write_text('''#!/bin/sh
case "$1" in
  sim)
    printf '%s\\n' "$3" > "$CAPTURE_WORLD"
    if [ "$2" = -s ]; then while :; do sleep .1; done; fi
    ;;
  topic) echo 'imu sample' ;;
esac
''')
        fake_gz.chmod(0o755)
        px4_root = self.work / 'PX4'
        px4_bin = px4_root / 'build/px4_sitl_default/bin/px4'
        px4_bin.parent.mkdir(parents=True)
        (px4_bin.parent.parent / 'etc/init.d-posix/airframes').mkdir(parents=True)
        px4_bin.write_text('''#!/bin/sh
set -eu
cmp "$2/init.d-posix/airframes/$EXPECTED_AIRFRAME" "$EXPECTED_SOURCE_AIRFRAME"
touch "$CAPTURE_PX4"
''')
        px4_bin.chmod(0o755)
        before = prepare_runtime.source_fingerprints(self.source)
        for mode in ('view', 'headless', 'sitl'):
            with self.subTest(mode=mode):
                capture_world = self.work / f'{mode}-world'
                capture_px4 = self.work / f'{mode}-px4'
                env = dict(os.environ, PATH=str(bindir) + os.pathsep + os.environ['PATH'],
                           TMPDIR=str(self.work), PX4_ROOT=str(px4_root),
                           CAPTURE_WORLD=str(capture_world), CAPTURE_PX4=str(capture_px4),
                           EXPECTED_AIRFRAME=self.airframe.name,
                           EXPECTED_SOURCE_AIRFRAME=str(self.airframe))
                result = subprocess.run(['sh', str(self.source / 'tests/run.sh'), mode, str(self.config)],
                                        env=env, capture_output=True, text=True, timeout=10)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                output = Path(next(line.removeprefix('运行目录：') for line in result.stdout.splitlines()
                                   if line.startswith('运行目录：'))) / 'models/rocket_racing_quad'
                self.assert_runtime(output)
                if mode != 'view':
                    self.assertTrue(capture_px4.is_file(), '必须将同一份原生机体文件安装给 PX4')
                self.assertEqual(before, prepare_runtime.source_fingerprints(self.source))

    def test_refuses_to_overwrite_source_or_mix_old_runtime(self):
        before = prepare_runtime.source_fingerprints(self.source)
        for output in (self.source, self.source / 'runtime', self.work):
            with self.subTest(output=output), self.assertRaises(ValueError):
                prepare_runtime.prepare(output, source=self.source)
        self.assertEqual(before, prepare_runtime.source_fingerprints(self.source))

    def test_ambiguous_airframe_is_rejected_before_copying(self):
        shutil.copy2(self.airframe, self.airframe.with_name('4999_gz_rocket_racing_quad'))
        output = self.work / 'output'
        with self.assertRaisesRegex(ValueError, '唯一'):
            prepare_runtime.prepare(output, source=self.source)
        self.assertFalse(output.exists())

    def test_generator_requires_separate_export_directory(self):
        before = prepare_runtime.source_fingerprints(ROOT)
        for arguments in ([], ['--output-dir', str(ROOT)]):
            result = subprocess.run([sys.executable, str(ROOT / 'tools/generate_model.py'), *arguments],
                                    capture_output=True, text=True, timeout=10)
            self.assertNotEqual(result.returncode, 0)
        self.assertEqual(before, prepare_runtime.source_fingerprints(ROOT))


if __name__ == '__main__':
    unittest.main()
