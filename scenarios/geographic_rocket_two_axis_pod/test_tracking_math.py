#!/usr/bin/env python3
"""不启动仿真的运动学、时序与验收反例检查。"""

import math
import threading
from types import SimpleNamespace
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import check_tracking as check
from target_fixture import TARGET_POSITION, add_target, target_model


def pose(stamp=0., position=(0., 0., 0.), quaternion=(1., 0., 0., 0.)):
    return dict(stamp=stamp, position=position, quaternion=quaternion)


class TrackingMathTest(unittest.TestCase):
    def assertVectorClose(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=7)

    def test_positive_pitch_points_toward_negative_mount_x(self):
        center = (0.015, 0., check.MOUNT_Z + .056)
        for direction in ((-10., 4., 20.), (-10., -4., 20.)):
            target = check.add(center, direction)
            solution = check.reachable_angles(pose(), target)
            self.assertTrue(solution['reachable'])
            self.assertGreater(solution['pitch'], 0)
            ray = check.mv(check.joints_rotation(solution['yaw'], solution['pitch']), (0., 0., 1.))
            self.assertVectorClose(ray, tuple(v/check.length(direction) for v in direction))

    def test_right_target_requires_positive_yaw_and_left_target_negative(self):
        center = (0.015, 0., check.MOUNT_Z + .056)
        for direction_y, expected_sign in ((-20., 1.), (20., -1.)):
            target = check.add(center, (0., direction_y, 100.))
            solution = check.reachable_angles(pose(), target)
            self.assertTrue(solution['reachable'])
            self.assertGreater(expected_sign * solution['yaw'], 0.)
            telemetry = dict(joints_deg=dict(yaw=dict(pos=solution['yaw']), pitch=dict(pos=0.)))
            origin, optical = check.camera_pose(pose(), telemetry)
            ray = check.mv(check.joints_rotation(solution['yaw'], 0.), (0., 0., 1.))
            self.assertVectorClose(ray, check.mv(optical, (1., 0., 0.)))
            self.assertVectorClose(check.project(target, origin, optical, 1920, 1080, 60), (960., 540.))

    def test_reachable_grid_and_limits(self):
        center = (0.015, 0., check.MOUNT_Z + .056)
        for yaw in (-60., -43., 0., 35., 60.):
            for pitch in (-80., -30., 0., 20., 30.):
                ray = check.mv(check.joints_rotation(yaw, pitch), (0., 0., 1.))
                solution = check.reachable_angles(pose(), check.add(center, tuple(100*v for v in ray)))
                self.assertTrue(solution['reachable'], (yaw, pitch, solution))
                computed = check.mv(check.joints_rotation(solution['yaw'], solution['pitch']), (0., 0., 1.))
                self.assertVectorClose(ray, computed)
        for ray in ((-2., 0., 1.), (1., 0., -1.), (0., 2., 1.)):
            self.assertFalse(check.reachable_angles(pose(), check.add(center, ray))['reachable'])
        self.assertFalse(check.reachable_angles(pose(), check.add(center, (0., 0., 1.)))['singular'])
        self.assertTrue(check.reachable_angles(pose(), check.add(center, (1., 0., 0.)))['singular'])

    def test_fixture_position_reachable_during_both_flight_directions(self):
        for north in (0., 150., 300.):
            for tilt in (-20., 0., 20.):
                half = math.radians(tilt)/2
                aircraft = pose(position=(0., north, 80.), quaternion=(math.cos(half), math.sin(half), 0., 0.))
                result = check.reachable_angles(aircraft, TARGET_POSITION)
                self.assertTrue(result['reachable'], (north, tilt, result))
                self.assertFalse(result['singular'])

    def test_optical_sensor_axis_and_camera_offset(self):
        telemetry = dict(joints_deg=dict(yaw=dict(pos=0.), pitch=dict(pos=0.)))
        origin, optical = check.camera_pose(pose(), telemetry)
        self.assertVectorClose(origin, (.015, 0., check.MOUNT_Z + .0797))
        self.assertVectorClose(check.mv(optical, (1., 0., 0.)), (0., 0., 1.))
        self.assertVectorClose(check.mv(optical, (0., -1., 0.)), (0., -1., 0.))
        self.assertVectorClose(check.mv(optical, (0., 0., 1.)), (-1., 0., 0.))
        point = check.add(origin, (0., 0., 100.))
        self.assertVectorClose(check.project(point, origin, optical, 1920, 1080, 60), (960., 540.))
        telemetry['joints_deg']['pitch']['pos'] = -80.
        _, optical = check.camera_pose(pose(), telemetry)
        self.assertVectorClose(check.mv(optical, (1., 0., 0.)), (math.sin(math.radians(80.)), 0., math.cos(math.radians(80.))))

    def test_outer_joint_preserves_sphere_center_and_moves_zero_los(self):
        center = (.015, 0., check.MOUNT_Z + .056)
        for yaw in (-60., -20., 0., 45., 60.):
            telemetry = dict(joints_deg=dict(yaw=dict(pos=yaw), pitch=dict(pos=0.)))
            origin, optical = check.camera_pose(pose(), telemetry)
            los = check.mv(optical, (1., 0., 0.))
            self.assertVectorClose(check.sub(origin, tuple(.0237*v for v in los)), center)
            self.assertVectorClose(los, (0., -math.sin(math.radians(yaw)), math.cos(math.radians(yaw))))

    def test_low_target_and_outer_axis_limit_are_reported_unreachable(self):
        aircraft = pose(position=(0., 0., 80.))
        result = check.reachable_angles(aircraft, (200., 150., 150.))
        self.assertFalse(result['reachable'])
        self.assertLess(result['yaw'], -60.)

    def test_pose_and_joint_interpolation_use_frame_time(self):
        half = math.pi/4
        rows = [pose(0., (0., 0., 0.)), pose(.02, (2., 4., 6.), (math.cos(half), 0., 0., math.sin(half)))]
        result = check.interpolate_pose(rows, .01)
        self.assertVectorClose(result['position'], (1., 2., 3.))
        self.assertVectorClose(check.mv(check.rotation(result['quaternion']), (1., 0., 0.)),
                               (math.sqrt(.5), math.sqrt(.5), 0.))
        telemetry = [dict(ts_mono=t, joints_deg=dict(yaw=dict(pos=y), pitch=dict(pos=p)))
                     for t, y, p in ((0., 0., 20.), (.02, 20., 40.))]
        joints = check.interpolate_joints(telemetry, .01)
        self.assertAlmostEqual(joints['joints_deg']['yaw']['pos'], 10.)
        self.assertAlmostEqual(joints['joints_deg']['pitch']['pos'], 30.)
        self.assertIsNone(check.interpolate_pose(rows, 1.))
        self.assertIsNone(check.interpolate_joints(telemetry, 1.))

    def test_opposite_quaternion_sign_preserves_rotation(self):
        q = (.5, .5, .5, .5)
        self.assertVectorClose(check.quaternion_interpolate(q, tuple(-v for v in q), .5), q)

    def test_stable_cruise_excludes_climb_hover_and_short_history(self):
        cruise = [pose(i/10, (0., i*1.5, 80.)) for i in range(33)]
        self.assertTrue(check.stable_cruise(cruise, 80., 15.))
        self.assertFalse(check.stable_cruise(cruise[-10:], 80., 15.))
        climb = [pose(i/10, (0., i*1.5, 74.+i*.2)) for i in range(33)]
        self.assertFalse(check.stable_cruise(climb, 80., 15.))
        hover = [pose(i/10, (0., 0., 80.)) for i in range(33)]
        self.assertFalse(check.stable_cruise(hover, 80., 15.))

    def test_target_fixture_is_static_visible_and_collision_free(self):
        model = target_model()
        self.assertEqual(model.findtext('static'), 'true')
        self.assertEqual(model.findtext('pose'), '200.0 150.0 280.0 0 0 0')
        self.assertEqual(len(model.findall('.//visual')), 151)
        self.assertFalse(model.findall('.//collision'))
        world = ET.Element('world')
        add_target(world)
        with self.assertRaises(ValueError):
            add_target(world)


class InitializerProtocolTest(unittest.TestCase):
    def session(self, receipt):
        session = check.Session.__new__(check.Session)
        session.send = mock.Mock(return_value=41)
        session.command_results = {40: dict(ok=True)}
        def poll():
            if receipt is not None:
                session.command_results[41] = receipt
        session.poll = mock.Mock(side_effect=poll)
        return session

    def test_manual_track_mode_waits_for_corresponding_success(self):
        session = self.session(dict(ok=True))
        session.enter_manual_tracking()
        session.send.assert_called_once_with('set_vision_mode', mode='manual_track')
        session.poll.assert_called_once_with()

    def test_mode_rejection_prevents_selection(self):
        session = self.session(dict(ok=False, message='不可用'))
        with self.assertRaisesRegex(AssertionError, '模式启用失败'):
            session.enter_manual_tracking()

    def test_unrelated_receipt_does_not_enable_selection(self):
        session = self.session(None)
        with mock.patch.object(check.time, 'monotonic', side_effect=(0., 0., 6.)):
            with self.assertRaisesRegex(AssertionError, '缺少对应成功回执'):
                session.enter_manual_tracking()

    def test_initializer_enables_manual_mode_before_single_selection(self):
        session = check.Session.__new__(check.Session)
        session.lock = threading.RLock()
        center = (.015, 0., check.MOUNT_Z + .056)
        session.args = SimpleNamespace(target=check.add(center, (0., 0., 100.)), target_size=8.,
                                       hfov=60., altitude=80., cruise_speed=15.)
        session.latest = dict(ts_mono=10., frame=dict(seq=314),
                              joints_deg=dict(yaw=dict(pos=0.), pitch=dict(pos=0.)),
                              tracking=dict(state='IDLE'), target=dict(valid=False))
        session.poses = [pose()]
        session.command_results = {}
        session.poll = mock.Mock()
        session.record = mock.Mock()
        session.matched = mock.Mock(return_value=(session.poses[0], dict(width=1920, height=1080, stamp=10.),
                                                  session.latest))
        commands = []
        def send(cmd, **kwargs):
            commands.append((cmd, kwargs))
            seq = len(commands)
            if cmd in ('set_vision_mode', 'track'):
                session.command_results[seq] = dict(ok=True)
            if cmd == 'track':
                session.latest.update(ts_mono=11., tracking=dict(state='TRACK'), target=dict(valid=True))
            return seq
        session.send = send
        def delayed_snapshot(*args):
            # 编码期间实时画面前进；选框必须已经发送，并绑定编码前的匹配帧。
            self.assertEqual(commands[-1][0], 'track')
            self.assertEqual(commands[-1][1]['frame_seq'], 314)
            session.latest = dict(session.latest, ts_mono=12., frame=dict(seq=999))
        session.save_snapshot = mock.Mock(side_effect=delayed_snapshot)
        with mock.patch.object(check, 'stable_cruise', return_value=True), \
                mock.patch.object(check, 'reachable_angles', return_value=dict(
                    reachable=True, singular=False, yaw=0., pitch=0.)):
            session.initialize()
        self.assertEqual([cmd for cmd, _ in commands], ['stop', 'stop', 'set_vision_mode', 'track'])
        self.assertEqual(commands[2][1], dict(mode='manual_track'))
        self.assertEqual(commands[3][1]['frame_seq'], 314)
        box = commands[3][1]['box']
        self.assertTrue(all(0 < value < 1 for value in box))
        session.save_snapshot.assert_called_once()


class AcceptanceCounterexampleTest(unittest.TestCase):
    def setUp(self):
        self.args = SimpleNamespace(max_truth_error=3., max_center_error=3., min_track_ratio=.95,
            min_north_span=180., min_direction_distance=50., min_attitude_excursion=5.,
            min_sim_duration=40., timebase='simulation', duration=120.)
        self.samples = []
        for i in range(1201):
            t = i/10
            phase = t % 40
            y = phase*15 if phase <= 20 else (40-phase)*15
            half_tilt = math.radians(10 if phase <= 20 else -10)/2
            self.samples.append(dict(stamp=t, position_enu=(0., y, 80.),
                quaternion_world=(math.cos(half_tilt), math.sin(half_tilt), 0., 0.),
                reachable=True, truth_singular=False, track=True, target_ray_error_deg=.1,
                optical_axis_error_deg=.1, center_error_deg=.1, camera_rotation_disagreement_deg=.01,
                telemetry_image_age_sim_s=.02, target_bbox_iou=.8, singular=False, limits={},
                constraint='NONE', state='TRACK', tracking_valid_frames=i, tracking_invalid_frames=0))

    def test_real_motion_and_independently_confirmed_track_pass(self):
        report = check.summarize(self.samples, self.args)
        self.assertEqual(report['result'], 'passed', report)

    def test_execution_metadata_freezes_actual_thresholds_and_sources(self):
        metadata = check.execution_metadata(self.args)
        self.assertEqual(metadata['acceptance_settings']['max_truth_error'], 3.)
        self.assertEqual(metadata['acceptance_settings']['duration'], 120.)
        self.args.max_truth_error = 99.
        self.assertEqual(metadata['acceptance_settings']['max_truth_error'], 3.)
        self.assertIn('scenarios/geographic_rocket_two_axis_pod/check_tracking.py', metadata['source_sha256'])
        self.assertTrue(all(len(digest) == 64 for digest in metadata['source_sha256'].values()))

    def test_csrt_track_false_positive_fails_truth_scoring(self):
        for row in self.samples:
            row.update(target_ray_error_deg=12., target_bbox_iou=0.)
        report = check.summarize(self.samples, self.args)
        self.assertEqual(report['csrt_track_ratio'], 1.)
        self.assertEqual(report['independently_confirmed_ratio'], 0.)
        self.assertEqual(report['result'], 'failed')

    def test_static_aircraft_cannot_pass_dynamic_flight(self):
        for row in self.samples:
            row['position_enu'] = (0., 0., 80.)
        self.assertEqual(check.summarize(self.samples, self.args)['result'], 'failed')

    def test_wallclock_elapsed_cannot_substitute_simulation_coverage(self):
        self.args.timebase = 'wall'
        for row in self.samples:
            row['stamp'] /= 20
        self.assertEqual(check.summarize(self.samples, self.args)['result'], 'failed')

    def test_unannounced_limits_fail(self):
        for row in self.samples[:10]:
            row['limits'] = dict(yaw=True)
        report = check.summarize(self.samples, self.args)
        self.assertEqual(report['unannounced_constraint_frames'], 10)
        self.assertEqual(report['result'], 'failed')


if __name__ == '__main__':
    unittest.main()
