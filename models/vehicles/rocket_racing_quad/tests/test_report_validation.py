"""飞行报告快照及来源匹配检查；不生成验收证据或导入绘图库。"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / 'tools/report_validation.py'
SPEC = importlib.util.spec_from_file_location('rocket_report_validation', MODULE_PATH)
report_validation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(report_validation)


def report_for(snapshot):
    payload = (json.dumps(snapshot, indent=2, ensure_ascii=False) + '\n').encode('utf-8')
    return {'model_parameters': snapshot, 'config_sha256': hashlib.sha256(payload).hexdigest()}


def validate_report_sources(report, current_sources):
    sources = report_validation.report_source_fingerprints(report)
    return report_validation.validate_source_fingerprints(sources, current_sources)


class ReportValidationTests(unittest.TestCase):
    def setUp(self):
        self.sources = {
            'model.sdf': 'a' * 64,
            'px4/4022_gz_rocket_racing_quad': 'b' * 64,
            'tests/flight_check.py': 'c' * 64,
        }
        self.snapshot = {
            'model': {'name': 'rocket_racing_quad', 'description': '原生模型快照'},
            'flight_test': {'hover_seconds': 30},
            'simulation': {'imu_wait_timeout_s': 90},
            'px4': {'airframe_id': 4022},
            'runtime': {'source_sha256': copy.deepcopy(self.sources)},
        }

    def test_matching_snapshot_and_native_sources_are_accepted(self):
        actual = validate_report_sources(report_for(self.snapshot), self.sources)
        self.assertEqual(actual, self.sources)
        self.assertIsNot(actual, self.snapshot['runtime']['source_sha256'])

    def test_modified_snapshot_cannot_reuse_its_original_hash(self):
        report = report_for(self.snapshot)
        report['model_parameters']['flight_test']['hover_seconds'] = 60
        with self.assertRaisesRegex(ValueError, '参数快照不一致'):
            validate_report_sources(report, self.sources)

    def test_wrong_or_missing_config_hash_is_rejected(self):
        for config_hash in (None, '0' * 64):
            with self.subTest(config_hash=config_hash):
                report = report_for(self.snapshot)
                report['config_sha256'] = config_hash
                with self.assertRaisesRegex(ValueError, '参数快照不一致'):
                    validate_report_sources(report, self.sources)

    def test_legacy_reports_without_native_sources_are_rejected(self):
        for runtime in (None, {}, {'source_sha256': {}}, {'source_sha256': []}):
            with self.subTest(runtime=runtime):
                snapshot = copy.deepcopy(self.snapshot)
                if runtime is None:
                    del snapshot['runtime']
                else:
                    snapshot['runtime'] = runtime
                with self.assertRaisesRegex(ValueError, '缺少原生资产来源指纹'):
                    validate_report_sources(report_for(snapshot), self.sources)

    def test_each_asset_or_script_change_invalidates_the_report(self):
        for name in self.sources:
            with self.subTest(source=name):
                current = dict(self.sources)
                current[name] = 'd' * 64
                with self.assertRaisesRegex(ValueError, '原生资产来源与当前'):
                    validate_report_sources(report_for(self.snapshot), current)

    def test_missing_or_extra_sources_are_rejected(self):
        for name in ('model.sdf', 'tools/prepare_runtime.py'):
            with self.subTest(source=name):
                current = dict(self.sources)
                if name in current:
                    del current[name]
                else:
                    current[name] = 'd' * 64
                with self.assertRaisesRegex(ValueError, '原生资产来源与当前'):
                    validate_report_sources(report_for(self.snapshot), current)

    def test_missing_snapshot_or_current_sources_are_rejected(self):
        with self.assertRaisesRegex(ValueError, '缺少模型参数快照'):
            validate_report_sources({}, self.sources)
        with self.assertRaisesRegex(ValueError, '原生资产来源与当前'):
            validate_report_sources(report_for(self.snapshot), {})


if __name__ == '__main__':
    unittest.main()
