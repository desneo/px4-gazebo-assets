"""Gazebo真值坐标转换回归，不需要运行Gazebo。"""
from collections import deque
import copy
import math
import threading
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from gazebo_truth import GazeboTruth, nominal_cg_reference, validate_measurement_reference, world_reference_point


class TruthTests(unittest.TestCase):
    def setUp(self):
        self.truth = GazeboTruth.__new__(GazeboTruth)
        self.truth.lock = threading.Lock()
        self.truth.rows = deque()
        self.truth.latest = (0, 10, 20, .225)
        self.truth.previous = None
        self.truth.origin = None
        self.truth.last_wall = 0
        self.truth.model_name = 'rocket_racing_quad'
        self.truth.body_offset_flu_m = [0., 0., 0.]
        self.truth.error = None
        self.assertTrue(self.truth.calibrate())

    def message(self, stamp, xyz, name='rocket_racing_quad', quaternion=(1., 0., 0., 0.)):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nsec=0)),
            pose=[SimpleNamespace(name=name, position=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
                                  orientation=SimpleNamespace(**dict(zip(('w','x','y','z'), quaternion))))])

    def test_pitch_rotation_moves_reference_and_produces_lever_arm_velocity(self):
        self.truth.body_offset_flu_m = [0., 0., .03]
        self.truth.latest = None
        self.truth.origin = None
        self.truth.receive(self.message(0, (10., 20., .225)))
        self.assertTrue(self.truth.calibrate())
        self.truth.receive(self.message(1, (10., 20., .225),
            quaternion=(math.sqrt(.5), 0., math.sqrt(.5), 0.)))
        stamp, position, velocity = self.truth.drain(0)[0]
        self.assertEqual(stamp, 1)
        for actual, expected in zip(position, [0., .03, .03]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(velocity, [0., .03, .03]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.truth.position(0), position)

    def test_yaw_rotation_of_general_offset_and_quaternion_sign(self):
        offset = [.1, .2, .3]
        message = self.message(0, (2., 3., 4.), quaternion=(math.sqrt(.5), 0., 0., math.sqrt(.5)))
        expected = [1.8, 3.1, 4.3]
        for sign in (1, -1):
            q = message.pose[0].orientation
            q.w *= sign
            q.z *= sign
            for actual, want in zip(world_reference_point(message.pose[0], offset), expected):
                self.assertAlmostEqual(actual, want)

    def test_invalid_orientation_and_backwards_time_fail_in_main_thread(self):
        for quaternion in ((0., 0., 0., 0.), (2., 0., 0., 0.), (float('nan'), 0., 0., 0.)):
            with self.subTest(quaternion=quaternion):
                self.truth.body_offset_flu_m = [0., 0., .03]
                self.truth.receive(self.message(1, (10., 20., .225), quaternion=quaternion))
                for method in (self.truth.calibrate, lambda:self.truth.drain(0), lambda:self.truth.position(0)):
                    with self.assertRaises(RuntimeError):
                        method()
                self.truth.error = None
        self.truth.receive(self.message(-1, (10., 20., .225)))
        with self.assertRaisesRegex(RuntimeError, '时钟倒退'):
            self.truth.drain(0)

    def test_duplicate_frame_does_not_refresh_truth_freshness(self):
        self.truth.receive(self.message(1, (10., 20., .225)))
        last_wall = self.truth.last_wall
        self.truth.receive(self.message(1, (100., 200., .225)))
        self.assertEqual(self.truth.last_wall, last_wall)

    def test_world_enu_to_local_ned_and_velocity(self):
        self.truth.receive(self.message(1, (12, 23, 30.225)))
        samples = self.truth.drain(-.01)
        self.assertEqual(len(samples), 1)
        stamp, position, velocity = samples[0]
        self.assertEqual(stamp, 1)
        for actual, expected in zip(position, [3, 2, -30.01]):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(velocity, [3, 2, -30]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(self.truth.drain(-.01), [])
        self.assertEqual(self.truth.position(-.01), position)

    def test_duplicate_and_other_model_pose_do_not_extend_measurement(self):
        self.truth.receive(self.message(1, (12, 23, .225), 'base_link'))
        self.assertEqual(self.truth.drain(0), [])
        self.truth.receive(self.message(1, (12, 23, .225)))
        self.truth.receive(self.message(1, (1000, 2000, .225)))
        samples = self.truth.drain(0)
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0][1], [3, 2, 0])

    def test_nondefault_world_and_model_bind_subscription_and_filter(self):
        transport, messages = ModuleType('gz.transport13'), ModuleType('gz.msgs10.pose_v_pb2')
        node = Mock()
        transport.Node = Mock(return_value=node)
        messages.Pose_V = object()
        with patch.dict('sys.modules', {'gz.transport13': transport, 'gz.msgs10.pose_v_pb2': messages}):
            truth = GazeboTruth('rocket_two_axis_pod_test', 'rocket_two_axis_pod')
        node.subscribe.assert_called_once_with(messages.Pose_V,
            '/world/rocket_two_axis_pod_test/dynamic_pose/info', truth.receive)
        truth.receive(self.message(1, (1, 2, 3)))
        self.assertIsNone(truth.latest, '其他实体不能被当作组合体真值')
        truth.receive(self.message(1, (4, 5, 6), 'rocket_two_axis_pod'))
        self.assertEqual(truth.latest, (1, 4, 5, 6))
        truth.close()
        node.unsubscribe.assert_called_once_with('/world/rocket_two_axis_pod_test/dynamic_pose/info')


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.snapshot = {'model': {'links': {
            'a': {'mass_kg': 1., 'pose': f'1 0 0 {math.pi/2} 0 0', 'inertial_pose': '0 .1 .2 0 0 0'},
            'b': {'mass_kg': 2., 'pose': '2 0 0 0 0 0', 'inertial_pose': '0 0 .03 0 0 0'},
        }}}

    def test_mass_center_rotates_local_inertial_offset_before_weighting(self):
        result = nominal_cg_reference(self.snapshot)
        self.assertEqual(result['kind'], 'nominal_cg')
        self.assertEqual(result['reference_pose'], 'zero_joint_positions')
        for actual, expected in zip(result['body_offset_flu_m'], [5/3, -.2/3, .16/3]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(validate_measurement_reference(self.snapshot, result), result)

    def test_missing_and_invalid_mass_or_pose_are_rejected(self):
        for field, value in (('mass_kg', 0), ('mass_kg', True), ('mass_kg', float('inf')),
                              ('pose', None), ('pose', '0 0 0'), ('inertial_pose', '0 0 nan 0 0 0')):
            with self.subTest(field=field, value=value):
                snapshot = copy.deepcopy(self.snapshot)
                snapshot['model']['links']['a'][field] = value
                with self.assertRaises(ValueError):
                    nominal_cg_reference(snapshot)
        with self.assertRaises(ValueError):
            nominal_cg_reference({'model': {}})


if __name__ == '__main__':
    unittest.main()
