"""场景运行快照必须记录实际世界，不能继续标记基础模型自测世界。"""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'models/vehicles/rocket_racing_quad/tools'))
import prepare_runtime

SPEC = importlib.util.spec_from_file_location('speed_world_snapshot', Path(__file__).with_name('prepare_world.py'))
prepare_world = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(prepare_world)


class WorldSnapshotTests(unittest.TestCase):
    def test_active_world_replaces_component_test_world_in_snapshot(self):
        with tempfile.TemporaryDirectory(prefix='rocket-speed-world-') as directory:
            run = Path(directory)
            model_dir = run / 'models/rocket_racing_quad'
            initial = prepare_runtime.prepare(model_dir)
            before = (model_dir / 'model.sdf').read_bytes()
            self.assertEqual(initial['simulation']['world_name'], 'rocket_racing_quad_test')
            (run / 'config.json').write_text(json.dumps({'imu_wait_timeout_s': 123}))
            original_config = (model_dir / 'parameters.json').read_bytes()
            prepare_world.prepare(run / 'world.sdf', model_dir / 'model.sdf', 'standalone')
            self.assertEqual((model_dir / 'parameters.json').read_bytes(), original_config,
                             '单独导出世界时默认不修改输入模型的配置')
            prepare_world.prepare(run / 'world.sdf', model_dir / 'model.sdf', 'standalone', update_snapshot=True)
            actual = json.loads((model_dir / 'parameters.json').read_text())
            self.assertEqual(actual['simulation']['world_name'], 'rocket_speed_comparison')
            self.assertEqual(float(actual['simulation']['gravity'].split()[2]), -9.80665)
            self.assertEqual(float(actual['simulation']['spawn_pose'].split()[2]), .24)
            self.assertEqual(actual['simulation']['imu_wait_timeout_s'], 123)
            self.assertEqual(actual['model'], initial['model'])
            self.assertEqual(actual['px4'], initial['px4'])
            self.assertEqual((model_dir / 'model.sdf').read_bytes(), before)
            self.assertEqual(actual['runtime']['world_sha256'], prepare_world.sha256(run / 'world.sdf'))
            manifest = json.loads((run / 'asset_manifest.json').read_text())
            self.assertEqual(manifest['generated_sha256']['parameters.json'],
                             prepare_world.sha256(model_dir / 'parameters.json'))


if __name__ == '__main__':
    unittest.main()
