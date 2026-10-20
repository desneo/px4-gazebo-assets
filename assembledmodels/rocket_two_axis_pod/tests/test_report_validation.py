"""组合完整飞行归档门槛回归；合成样例仅写临时测试目录。"""
from pathlib import Path
import copy
import gzip
import hashlib
import importlib.util
import json
import tempfile
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
SPEC = importlib.util.spec_from_file_location('pod_flight_report_under_test', ROOT / 'tools/report_validation.py')
archive = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(archive)


def config_hash(report):
    raw = (json.dumps(report['model_parameters'], indent=2, ensure_ascii=False) + '\n').encode()
    report['config_sha256'] = hashlib.sha256(raw).hexdigest()


def fixture():
    defaults = json.loads((archive.dynamic.ROCKET / 'parameters.json').read_text())['flight_test']
    sources = {'assembledmodels/rocket_two_axis_pod/tools/generate_dynamic_model.py': 'a' * 64,
               'models/vehicles/rocket_racing_quad/model.sdf': 'b' * 64,
               'models/vehicles/rocket_racing_quad/tests/flight_check.py': 'c' * 64}
    settings = copy.deepcopy(defaults)
    snapshot = dict(model=dict(name='rocket_two_axis_pod', total_mass_kg=7.94),
                    simulation=dict(world_name='rocket_two_axis_pod_test'),
                    flight_test=copy.deepcopy(defaults), runtime=dict(source_sha256=copy.deepcopy(sources)))
    results = {'hover': dict(passed=True, max_drift_m=.1, max_altitude_error_m=.2)}
    truth = []
    clock = 0
    for phase in ('hover', 'cruise', 'sprint'):
        seconds = settings[phase + '_seconds']
        speed = 0 if phase == 'hover' else (settings[phase + '_speed_km_h'] + 1) / 3.6
        for index in range(int(seconds) + 1):
            truth.append(dict(phase=phase, sim_time_s=clock + index,
                              position_ned_m=[0., 0., -30.], velocity_ned_m_s=[speed, 0., 0.]))
        clock += int(seconds) + 2
        if phase != 'hover':
            results[phase] = dict(passed=True, required_speed_km_h=settings[phase + '_speed_km_h'],
                                  required_continuous_s=seconds, longest_continuous_s=seconds,
                                  peak_speed_km_h=settings[phase + '_speed_km_h'] + 1,
                                  max_altitude_error_m=.2)
    truth.append(dict(phase='landing', sim_time_s=clock, position_ned_m=[0., 0., 0.], velocity_ned_m_s=[0., 0., 0.]))
    samples = [dict(row, command_z_m=-30., command_vx_m_s=row['velocity_ned_m_s'][0]) for row in truth]
    report = dict(model_parameters=snapshot, settings=settings, passed=True, landing_confirmed=True,
                  measurement_source='gazebo_pose',
                  measurement_target=dict(world_name='rocket_two_axis_pod_test', model_name='rocket_two_axis_pod'),
                  results=results, samples=samples, ground_truth_samples=copy.deepcopy(truth),
                  started_at_unix_s=100., finished_at_unix_s=200.)
    config_hash(report)
    return report, sources, defaults


class FlightArchiveTests(unittest.TestCase):
    def setUp(self):
        self.report, self.sources, self.defaults = fixture()

    def validate(self, report=None, sources=None):
        return archive.validate_report(self.report if report is None else report,
                                       self.sources if sources is None else sources, self.defaults)

    def test_complete_ground_truth_report_is_accepted(self):
        sources, windows = self.validate()
        self.assertEqual(sources, self.sources)
        self.assertEqual(windows, {'cruise': 30., 'sprint': 10.})
        self.report['settings']['endpoint'] = 'udpin:0.0.0.0:14541'
        self.validate()

    def test_config_hash_and_native_fingerprints_must_match(self):
        for change in ('snapshot', 'source', 'legacy'):
            with self.subTest(change=change):
                report = copy.deepcopy(self.report)
                if change == 'snapshot':
                    report['model_parameters']['model']['total_mass_kg'] = 1
                elif change == 'source':
                    report['model_parameters']['runtime']['source_sha256'].pop(next(iter(self.sources)))
                    config_hash(report)
                else:
                    report['model_parameters'].pop('runtime')
                    config_hash(report)
                with self.assertRaises(ValueError):
                    self.validate(report)

    def test_success_flags_must_be_boolean_true_and_phases_complete(self):
        for key in ('passed', 'landing_confirmed'):
            for value in (False, 1, 'true', None):
                with self.subTest(key=key, value=value):
                    report = copy.deepcopy(self.report)
                    report[key] = value
                    with self.assertRaises(ValueError):
                        self.validate(report)
        for phase in ('hover', 'cruise', 'sprint'):
            with self.subTest(missing_phase=phase):
                report = copy.deepcopy(self.report)
                del report['results'][phase]
                with self.assertRaises(ValueError):
                    self.validate(report)

    def test_target_and_measurement_source_cannot_be_substituted(self):
        for key, value in (('measurement_source', 'px4_local_position'),
                           ('measurement_target', dict(world_name='rocket_racing_quad_test', model_name='rocket_racing_quad'))):
            with self.subTest(key=key):
                report = copy.deepcopy(self.report)
                report[key] = value
                with self.assertRaisesRegex(ValueError, 'Gazebo 真值'):
                    self.validate(report)

    def test_shortened_test_or_relaxed_tolerance_is_not_full_validation(self):
        for key, value in (('hover_seconds', 3), ('cruise_seconds', 3), ('sprint_seconds', 1),
                           ('cruise_speed_km_h', 100), ('sprint_speed_km_h', 200),
                           ('altitude_tolerance_m', 20), ('hover_drift_limit_m', 10)):
            with self.subTest(key=key):
                report = copy.deepcopy(self.report)
                report['settings'][key] = value
                with self.assertRaisesRegex(ValueError, '完整验收门槛'):
                    self.validate(report)

    def test_forged_phase_pass_cannot_hide_raw_speed_or_height_failure(self):
        for reason in ('speed', 'height', 'gap'):
            with self.subTest(reason=reason):
                report = copy.deepcopy(self.report)
                rows = [row for row in report['ground_truth_samples'] if row['phase'] == 'cruise']
                if reason == 'speed':
                    rows[15]['velocity_ned_m_s'][0] = 0.
                elif reason == 'height':
                    rows[15]['position_ned_m'][2] = -25.
                else:
                    report['ground_truth_samples'] = [row for row in report['ground_truth_samples']
                        if row['phase'] != 'cruise' or not 10 <= row['sim_time_s'] - rows[0]['sim_time_s'] <= 14]
                with self.assertRaisesRegex(ValueError, '连续速度和高度'):
                    self.validate(report)

    def test_hover_limit_violation_missing_samples_and_recovery_error_are_rejected(self):
        for reason in ('hover', 'missing_samples', 'recovery'):
            with self.subTest(reason=reason):
                report = copy.deepcopy(self.report)
                if reason == 'hover':
                    report['results']['hover']['max_drift_m'] = self.defaults['hover_drift_limit_m'] + .01
                elif reason == 'missing_samples':
                    report['ground_truth_samples'] = []
                else:
                    report['recovery_error'] = 'landing timeout'
                with self.assertRaises(ValueError):
                    self.validate(report)

    def test_rejected_report_does_not_touch_existing_evidence(self):
        report = copy.deepcopy(self.report)
        report['landing_confirmed'] = False
        with tempfile.TemporaryDirectory(prefix='pod-report-reject-') as directory:
            output = Path(directory)
            existing = output / 'docs/validation.json'
            existing.parent.mkdir()
            existing.write_text('existing evidence')
            with self.assertRaises(ValueError):
                archive.archive_report(json.dumps(report).encode(), output_root=output,
                                       current_sources=self.sources, defaults=self.defaults,
                                       renderer=lambda _: self.fail('不能渲染无效报告'))
            self.assertEqual(existing.read_text(), 'existing evidence')
            self.assertFalse((output / 'tests/results/flight_result.json.gz').exists())

    def test_archive_preserves_raw_bytes_and_summary_provenance_in_temporary_fixture(self):
        raw = (json.dumps(self.report, ensure_ascii=False) + '\n').encode()
        with tempfile.TemporaryDirectory(prefix='pod-report-fixture-') as directory:
            output = Path(directory)
            summary = archive.archive_report(raw, output_root=output, current_sources=self.sources,
                defaults=self.defaults, renderer=lambda _: b'test-only-renderer')
            self.assertEqual(gzip.decompress((output / 'tests/results/flight_result.json.gz').read_bytes()), raw)
            self.assertEqual(summary['asset_sha256'], self.sources)
            self.assertEqual(summary['raw_report_sha256'], hashlib.sha256(raw).hexdigest())
            self.assertEqual(summary['verified_continuous_s'], {'cruise': 30., 'sprint': 10.})
            self.assertEqual(json.loads((output / 'docs/validation.json').read_text()), summary)
            self.assertEqual((output / 'docs/flight_validation.png').read_bytes(), b'test-only-renderer')

    def test_native_source_coverage_includes_runner_and_shared_checkers(self):
        sources = archive.dynamic.source_fingerprints()
        for name in ('assembledmodels/rocket_two_axis_pod/tests/run_flight_check.sh',
                     'assembledmodels/rocket_two_axis_pod/tools/generate_dynamic_model.py',
                     'models/vehicles/rocket_racing_quad/tests/flight_check.py',
                     'models/vehicles/rocket_racing_quad/tests/gazebo_truth.py'):
            self.assertIn(name, sources)
        self.assertFalse(any('/docs/' in name or '/results/' in name or name.endswith('report_validation.py')
                             for name in sources))


if __name__ == '__main__':
    unittest.main()
