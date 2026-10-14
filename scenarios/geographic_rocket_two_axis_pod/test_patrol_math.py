"""往返调度的停止距离、超调和掉头条件检查。"""

import math
import unittest

from patrol_math import at_endpoint, desired_velocity, slew_velocity


class PatrolTrajectoryTest(unittest.TestCase):
    def test_braking_distance_and_signed_overshoot(self):
        self.assertEqual(desired_velocity(200, 15, 2, .8), 15)
        self.assertAlmostEqual(desired_velocity(40, 15, 2, .8), math.sqrt(160))
        self.assertLess(desired_velocity(-5, 15, 2, .8), 0)
        self.assertEqual(desired_velocity(0, 15, 2, .8), 0)

    def test_crossing_endpoint_at_speed_does_not_turn(self):
        self.assertFalse(at_endpoint(.1, 15, 2, .8))
        self.assertFalse(at_endpoint(3, 0, 2, .8))
        self.assertTrue(at_endpoint(-1, -.4, 2, .8))

    def test_command_reversal_obeys_acceleration_bound(self):
        self.assertAlmostEqual(slew_velocity(15, -15, 2, .05), 14.9)
        self.assertEqual(slew_velocity(1, -1, 2, 0), 1)

    def test_feedback_reaches_both_endpoints_after_moderate_overshoot(self):
        position, velocity, command = 0., 0., 0.
        for target in (300., 0.):
            for _ in range(6000):
                error = target - position
                command = slew_velocity(command, desired_velocity(error, 15, 2, .8), 2, .05)
                # 低通响应引入真实飞控常见的执行滞后，检验超调后的收敛。
                velocity += (command - velocity) * .05 / .5
                position += velocity * .05
                if at_endpoint(target - position, velocity, 2, .8):
                    break
            else:
                self.fail('位置反馈未在期限内到达端点')
            self.assertLessEqual(abs(target - position), 2)


if __name__ == '__main__':
    unittest.main()
