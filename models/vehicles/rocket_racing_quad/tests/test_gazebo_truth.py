"""Gazebo真值坐标转换回归，不需要运行Gazebo。"""
from collections import deque
import threading
from types import SimpleNamespace
import unittest
from gazebo_truth import GazeboTruth


class TruthTests(unittest.TestCase):
    def setUp(self):
        self.truth = GazeboTruth.__new__(GazeboTruth)
        self.truth.lock = threading.Lock()
        self.truth.rows = deque()
        self.truth.latest = (0, 10, 20, .225)
        self.truth.previous = None
        self.truth.origin = None
        self.truth.last_wall = 0
        self.assertTrue(self.truth.calibrate())

    def message(self, stamp, xyz, name='rocket_racing_quad'):
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp, nsec=0)),
            pose=[SimpleNamespace(name=name, position=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]))])

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


if __name__ == '__main__':
    unittest.main()
