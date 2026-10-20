"""飞行报告快照及来源匹配检查；不生成验收证据或导入绘图库。"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType
import unittest
from unittest.mock import patch


MODULE_PATH = Path(__file__).resolve().parents[1] / 'tools/report_validation.py'
SPEC = importlib.util.spec_from_file_location('rocket_report_validation', MODULE_PATH)
report_validation = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(report_validation)


def report_for(snapshot):
    payload = (json.dumps(snapshot, indent=2, ensure_ascii=False) + '\n').encode('utf-8')
    return {'model_parameters': snapshot, 'config_sha256': hashlib.sha256(payload).hexdigest(),
            'measurement_source': 'gazebo_pose',
            'measurement_target': dict(world_name=snapshot['simulation']['world_name'],
                                       model_name=snapshot['model']['name']),
            'measurement_reference': dict(kind='nominal_cg', body_offset_flu_m=[0., 0., .03],
                                          reference_pose='zero_joint_positions')}


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
            'model': {'name': 'rocket_racing_quad', 'description': '单测参数快照',
                      'links': {'base_link': dict(mass_kg=4.5, pose='0 0 0 0 0 0',
                          inertial_pose='0 0 .0331777777777778 0 0 0',
                          inertia=dict(ixx=.139854907060, iyy=.139243371000, izz=.190493151860,
                                       ixy=0., ixz=0., iyz=0.))}},
            'flight_test': {'hover_seconds': 30},
            'simulation': {'imu_wait_timeout_s': 90, 'world_name': 'rocket_racing_quad_test'},
            'px4': {'airframe_id': 4022},
            'runtime': {'source_sha256': copy.deepcopy(self.sources)},
        }
        for index, (sx, sy) in enumerate(((1, -1), (-1, 1), (1, 1), (-1, -1))):
            for name, mass, z, tensor in (
                    ('nacelle', .04, 0., dict(ixx=.000043, iyy=.000043, izz=.000032)),
                    ('rotor', .025, -.065, dict(ixx=.00010090833333333334,
                                               iyy=.00010090833333333334, izz=.0002016125))):
                self.snapshot['model']['links'][f'{name}_{index}'] = dict(
                    mass_kg=mass, pose=f'{sx * .2086} {sy * .2086} {z} 0 0 0',
                    inertial_pose='0 0 0 0 0 0', inertia=dict(tensor, ixy=0., ixz=0., iyz=0.))
        self.snapshot['model']['total_mass_kg'] = 4.76

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

    def test_nominal_cg_metadata_matches_snapshot_and_is_preserved(self):
        report = report_for(self.snapshot)
        metadata = report_validation.report_measurement_metadata(report)
        self.assertEqual(metadata, {key: report[key] for key in ('measurement_target', 'measurement_reference')})
        metadata['measurement_reference']['body_offset_flu_m'][2] = 99
        metadata['measurement_target']['model_name'] = 'different'
        self.assertEqual(report['measurement_reference']['body_offset_flu_m'][2], .03)
        self.assertEqual(report['measurement_target']['model_name'], 'rocket_racing_quad')

    def test_metadata_validation_accepts_failed_report_without_claiming_pass(self):
        report = report_for(self.snapshot)
        report['passed'] = False
        report['landing_confirmed'] = False
        self.assertEqual(validate_report_sources(report, self.sources), self.sources)
        metadata = report_validation.report_measurement_metadata(report)
        self.assertEqual(metadata['measurement_reference']['kind'], 'nominal_cg')
        self.assertNotIn('passed', metadata)
        self.assertIs(report['passed'], False)

    def test_gazebo_report_missing_or_wrong_reference_is_rejected(self):
        candidates = [None, {}, dict(kind='model_origin', body_offset_flu_m=[0., 0., 0.],
                                     reference_pose='zero_joint_positions'),
                      dict(kind='nominal_cg', body_offset_flu_m=[0., 0., .03], reference_pose='instantaneous'),
                      dict(kind='nominal_cg', body_offset_flu_m=[0., 0., .03 + 2e-12],
                           reference_pose='zero_joint_positions')]
        for reference in candidates:
            with self.subTest(reference=reference):
                report = report_for(self.snapshot)
                if reference is None:
                    report.pop('measurement_reference')
                else:
                    report['measurement_reference'] = reference
                with self.assertRaises(ValueError):
                    report_validation.report_measurement_metadata(report)

    def test_nominal_cg_offset_tolerance_is_absolute_one_picometer(self):
        report = report_for(self.snapshot)
        report['measurement_reference']['body_offset_flu_m'][2] += 5e-13
        report_validation.report_measurement_metadata(report)

    def test_bad_reference_vector_or_incomplete_snapshot_is_rejected(self):
        for offset in ([0., 0.], [0., 0., float('nan')], [0., False, .03], [0., 0., float('inf')]):
            with self.subTest(offset=offset):
                report = report_for(self.snapshot)
                report['measurement_reference']['body_offset_flu_m'] = offset
                with self.assertRaises(ValueError):
                    report_validation.report_measurement_metadata(report)
        report = report_for(self.snapshot)
        del report['model_parameters']['model']['links']
        with self.assertRaises(ValueError):
            report_validation.report_measurement_metadata(report)

    def test_mismatched_or_missing_measurement_target_is_rejected(self):
        for target in (None, {}, dict(world_name='other', model_name='rocket_racing_quad'),
                       dict(world_name='rocket_racing_quad_test', model_name='other')):
            with self.subTest(target=target):
                report = report_for(self.snapshot)
                report['measurement_target'] = target
                with self.assertRaisesRegex(ValueError, '测量目标与运行快照'):
                    report_validation.report_measurement_metadata(report)

    def test_px4_report_keeps_optional_reference_without_requiring_nominal_cg(self):
        report = report_for(self.snapshot)
        report['measurement_source'] = 'px4_local_position'
        report.pop('measurement_reference')
        self.assertEqual(report_validation.report_measurement_metadata(report),
                         dict(measurement_target=report['measurement_target']))
        report['measurement_reference'] = dict(kind='estimator_body_reference')
        self.assertEqual(report_validation.report_measurement_metadata(report)['measurement_reference'],
                         report['measurement_reference'])

    def test_invalid_measurement_source_is_rejected(self):
        report = report_for(self.snapshot)
        report['measurement_source'] = 'unknown'
        with self.assertRaisesRegex(ValueError, '测量来源'):
            report_validation.report_measurement_metadata(report)

    def test_main_rejects_missing_reference_before_creating_archive(self):
        report = report_for(self.snapshot)
        report.pop('measurement_reference')
        runtime = ModuleType('prepare_runtime')
        runtime.source_fingerprints = lambda _root: dict(self.sources)
        with tempfile.TemporaryDirectory(prefix='rocket-report-metadata-') as temporary:
            root = Path(temporary)
            input_path = root / 'input-fixture.json'
            input_path.write_text(json.dumps(report))
            with patch.object(report_validation, 'ROOT', root), patch.object(sys, 'argv',
                    ['report_validation.py', str(input_path)]), patch.dict(sys.modules, {'prepare_runtime': runtime}):
                with self.assertRaisesRegex(ValueError, '缺少测量参考点'):
                    report_validation.main()
            self.assertEqual(list(root.iterdir()), [input_path])


if __name__ == '__main__':
    unittest.main()
