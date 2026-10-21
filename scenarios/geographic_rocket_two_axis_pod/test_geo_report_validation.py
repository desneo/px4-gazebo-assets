"""正式归档拒绝过期快照、错配证据和不满足门槛的原始数据。"""
import copy
import importlib.util
import json
import math
from pathlib import Path
import shutil
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import report_validation as report

HERE = Path(__file__).resolve().parent


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2)+'\n')


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(''.join(json.dumps(row)+'\n' for row in rows))


class ReportAuthenticityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.template = tempfile.TemporaryDirectory(prefix='geo-archive-template-')
        cls.check = report.assembly_tools()
        cls.check.dynamic.generate(Path(cls.template.name) / 'rocket_two_axis_pod')
        spec = importlib.util.spec_from_file_location('geo_archive_test_tracking', HERE / 'check_tracking.py')
        cls.tracker = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.tracker)

    @classmethod
    def tearDownClass(cls):
        cls.template.cleanup()

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='geo-archive-test-')
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.run = self.directory / 'run'
        self.model = self.run / 'models/rocket_two_axis_pod'
        shutil.copytree(Path(self.template.name) / 'rocket_two_axis_pod', self.model)
        self.settings = report.read_json(HERE / 'config.json')
        write_json(self.run / 'config.json', self.settings)
        self.rows, legs = [], []
        for index in (1, 2):
            phase = f'leg_{index}_to_{"B" if index == 1 else "A"}'
            positions = (0., 150., 300.) if index == 1 else (300., 150., 0.)
            velocity = 15. if index == 1 else -15.
            for sample, north in enumerate(positions):
                stamp = 10.+(index-1)*3+sample
                self.rows.append(dict(kind='gazebo', phase=phase, sim_time_s=stamp,
                    position_ned_m=[north, 0., -80.], velocity_ned_m_s=[velocity, 0., 0.]))
            self.rows.append(dict(kind='px4', phase=phase, sim_time_s=10.+(index-1)*3,
                command_z_m=-80., position_ned_m=[positions[0], 0., -80.], velocity_ned_m_s=[velocity, 0., 0.]))
            target = 300. if index == 1 else 0.
            leg = dict(samples=3, max_altitude_error_m=0., max_cross_track_error_m=0.,
                cruise_samples=1, cruise_speed_sum_m_s=15., cruise_samples_within_tolerance=1,
                phase=phase, target_north_from_start_m=target, truth_endpoint_ned_m=[target, 0., -80.],
                truth_endpoint_error_m=0., cruise_mean_speed_m_s=15.,
                cruise_speed_within_tolerance_fraction=1., passed=True)
            legs.append(leg)
            self.rows.append(dict(kind='leg_result', **leg))
        self.flight = dict(passed=True, landing_confirmed=True, settings=self.settings,
            measurement_source='gazebo_pose', started_at_unix_s=100., finished_at_unix_s=500.,
            model_parameters=report.read_json(self.model / 'parameters.json'),
            model_config_sha256=report.sha(self.model / 'parameters.json'),
            cycles_requested=1, completed_legs=2, last_leg=legs[-1], samples_count=2,
            ground_truth_samples_count=6, status_text_count=0)
        self.library_name = 'models/gimbals/two_axis_pod/controller/build/lib/libgz-two-axis-pod-controller.so'
        library_hash = report.sha(report.ROOT / self.library_name)
        self.execution = dict(captured_at_unix_s=99., controller_library_sha256=library_hash,
            source_sha256={name: report.sha(report.ROOT / name) for name in (
                self.library_name, 'models/gimbals/two_axis_pod/model.sdf',
                'scenarios/geographic_rocket_two_axis_pod/flight_patrol.py')})
        write_json(self.run / 'execution_fingerprints.json', self.execution)
        self.acceptance = dict(max_truth_error=3., max_center_error=3., min_track_ratio=.95,
            min_north_span=180., min_direction_distance=50., min_attitude_excursion=5.,
            min_sim_duration=40., timebase='simulation', duration=120.)
        self.samples = []
        for index in range(1201):
            t, phase = index/10, (index/10) % 40
            north = phase*15 if phase <= 20 else (40-phase)*15
            half = math.radians(10 if phase <= 20 else -10)/2
            self.samples.append(dict(kind='sample', wall=110.+t, stamp=10.+t,
                position_enu=(0., north, 80.), quaternion_world=(math.cos(half), math.sin(half), 0., 0.),
                reachable=True, truth_singular=False, track=True, target_ray_error_deg=.1,
                optical_axis_error_deg=.1, center_error_deg=.1, camera_rotation_disagreement_deg=.01,
                telemetry_image_age_sim_s=.02, target_bbox_iou=.8, singular=False, limits={},
                constraint='NONE', state='TRACK', tracking_valid_frames=index, tracking_invalid_frames=0))
        self.tracking = self.tracker.summarize(self.samples, SimpleNamespace(**self.acceptance))
        self.tracking.update(acceptance_settings=self.acceptance,
            source_sha256={'scenarios/geographic_rocket_two_axis_pod/check_tracking.py': report.sha(HERE / 'check_tracking.py')},
            target_enu=[200., 150., 280.], target_size_m=8.)
        self.tracking_rows = self.samples+[dict(kind='event', wall=231., event='result', **self.tracking)]
        self.binding_dir = self.run / 'rtsp_binding'
        payload = dict(aircraft_id=23, video_rtsp_path='/two_axis_pod_23', clock_domain='test-clock',
                       version=6, protocol_version=6, angle_convention='motion_head_joint')
        self.binding_rows = [dict(kind='hello', wall=120., payload=payload)]
        for index in range(20):
            self.binding_rows.extend([dict(kind='video', wall=121.+index, seq=index),
                dict(kind='telemetry', wall=121.+index, payload=dict(payload, frame_table=[[index, 0.]]))])
        self.binding = dict(result='passed', source_fingerprints_current=True,
            aircraft_id=23, expected_path='/two_axis_pod_23', rtsp_url='rtsp://127.0.0.1:8555/two_axis_pod_23',
            source_sha256={'scenarios/geographic_rocket_two_axis_pod/check_rtsp_binding.py': report.sha(HERE / 'check_rtsp_binding.py')},
            frames=20, hello_count=1, telemetry_count=20, decoded_frames=20, unique_video_sequences=20,
            decoded_ratio=1., matched_frame_table_sequences=20, frame_table_match_ratio=1., clock_domains=['test-clock'])
        from prepare_world import prepare
        prepare(self.run / 'world.sdf', self.model / 'model.sdf')
        self.binding_dir.mkdir(parents=True)
        (self.binding_dir / 'rtsp_snapshot.png').write_bytes(b'rtsp-image')
        (self.run / 'tracking').mkdir()
        (self.run / 'tracking/frame_000001.png').write_bytes(b'tracking-image')
        self.output, self.evidence = self.directory / 'docs', self.directory / 'evidence'
        self.output.mkdir(); self.evidence.mkdir()
        (self.output / 'validation.json').write_bytes(b'existing-summary')
        (self.evidence / 'existing.gz').write_bytes(b'existing-evidence')
        self.renderer = Mock(return_value=b'actual-curve')
        self.save()

    def save(self):
        write_json(self.run / 'flight_result.json', self.flight)
        write_rows(self.run / 'flight_result.jsonl', self.rows)
        write_json(self.run / 'tracking/summary.json', self.tracking)
        write_rows(self.run / 'tracking/tracking.jsonl', self.tracking_rows)
        write_json(self.binding_dir / 'binding_summary.json', self.binding)
        write_rows(self.binding_dir / 'binding_samples.jsonl', self.binding_rows)

    def generate(self, binding=True):
        return report.generate(self.run, self.output, self.binding_dir if binding else None,
                               evidence_dir=self.evidence, renderer=self.renderer)

    def assertRejected(self, pattern):
        self.save()
        before = {str(p): p.read_bytes() for base in (self.output, self.evidence) for p in base.iterdir()}
        with self.assertRaisesRegex(ValueError, pattern):
            self.generate()
        after = {str(p): p.read_bytes() for base in (self.output, self.evidence) for p in base.iterdir()}
        self.assertEqual(before, after, '拒绝报告时不得覆盖入库证据')

    def test_consistent_raw_evidence_is_archived_with_model_world_and_execution(self):
        result = self.generate()
        self.assertTrue(result['passed'])
        self.assertEqual(result['controller_library_sha256'], self.execution['controller_library_sha256'])
        for name in ('combined_flight.json.gz', 'tracking_summary.json.gz', 'execution_fingerprints.json.gz',
                     'world.sdf.gz', 'assembled_model.sdf.gz'):
            self.assertTrue((self.evidence / name).is_file())
            self.assertEqual(result['evidence_sha256']['tests/results/'+name], report.sha(self.evidence / name))
        self.assertEqual((self.output / 'flight_tracking.png').read_bytes(), b'actual-curve')
        self.assertEqual(self.renderer.call_count, 1)

    def test_flight_snapshot_content_cannot_be_changed_under_old_hash(self):
        self.flight['model_parameters']['model']['name'] = 'another-aircraft'
        self.assertRejected('快照')

    def test_land_mode_transition_after_leg_result_is_not_rescored_as_cruise(self):
        self.rows.append(dict(kind='gazebo', phase='leg_2_to_A', sim_time_s=16.,
            position_ned_m=[0., 0., -70.], velocity_ned_m_s=[0., 0., 2.]))
        self.flight['ground_truth_samples_count'] += 1
        self.save()
        self.assertTrue(self.generate()['passed'])
        scored_truth = self.renderer.call_args.args[2]
        self.assertEqual(len(scored_truth), 6)
        self.assertTrue(all(row['sim_time_s'] < 16. for row in scored_truth))
        self.assertTrue(all(row['position_ned_m'][2] == -80. for row in scored_truth))

    def test_ground_reference_samples_are_retained_without_scoring_as_flight_leg(self):
        self.rows.insert(0, dict(kind='gazebo', phase='ground_reference', sim_time_s=5.,
            position_ned_m=[0., 0., 0.], velocity_ned_m_s=[0., 0., 0.]))
        self.flight['ground_truth_samples_count'] += 1
        self.save()
        self.assertTrue(self.generate()['passed'])

    def test_generated_model_tampering_is_rejected(self):
        path = self.model / 'model.sdf'
        text = path.read_text().replace('<mass>4.5</mass>', '<mass>4.6</mass>')
        self.assertIn('<mass>4.6</mass>', text)
        path.write_text(text)
        self.assertRejected('动态模型')

    def test_truth_altitude_failure_cannot_be_overridden_by_passed_summary(self):
        self.rows[1]['position_ned_m'][2] = -84.
        self.assertRejected('原始真值')

    def test_changed_execution_source_and_library_are_rejected(self):
        self.execution['source_sha256'][self.library_name] = 'outdated'
        write_json(self.run / 'execution_fingerprints.json', self.execution)
        self.assertRejected('启动记录源文件变化')

    def test_tracking_pass_flag_cannot_hide_wrong_target_samples(self):
        self.samples[100]['target_ray_error_deg'] = 20.
        self.assertRejected('原始图像独立评分')

    def test_unrelated_run_binding_wall_time_is_rejected(self):
        self.binding_rows[0]['wall'] = 50.
        self.assertRejected('本次飞行时间窗口')

    def test_wrong_rtsp_aircraft_identity_is_rejected(self):
        self.binding_rows[2]['payload']['aircraft_id'] = 24
        self.assertRejected('ID')

    def test_claimed_frame_match_ratio_must_match_original_video(self):
        self.binding_rows[1]['seq'] = 900
        self.assertRejected('RTSP原始记录与摘要不一致')

    def test_missing_binding_cannot_be_archived_as_full_chain_pass(self):
        with self.assertRaisesRegex(ValueError, 'binding-dir'):
            self.generate(binding=False)

    def test_render_failure_does_not_replace_prior_passed_evidence(self):
        self.renderer.side_effect = RuntimeError('renderer failure')
        with self.assertRaisesRegex(RuntimeError, 'renderer failure'):
            self.generate()
        self.assertEqual((self.output / 'validation.json').read_bytes(), b'existing-summary')
        self.assertEqual(list(self.evidence.iterdir()), [self.evidence / 'existing.gz'])


if __name__ == '__main__':
    unittest.main()
