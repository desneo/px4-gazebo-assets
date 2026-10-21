"""地面高度参考初始化：使用真实等待循环，隔离飞控与Gazebo连接。"""
from collections import deque
import contextlib
import io
import threading
import unittest
from unittest import mock

from flight_patrol import Patrol, PatrolTruth, mavutil


class GroundReferenceTest(unittest.TestCase):
    def make_patrol(self, motion=None, *, armed=False, on_ground=True, stale_landed=False):
        patrol = Patrol.__new__(Patrol)
        patrol.cfg = {'telemetry_timeout_s': 5., 'phase_wall_timeout_s': 25.}
        patrol.report = {}
        patrol.sim_time = 0.
        patrol.position = [0., 0., -.04]
        patrol.velocity = [0., 0., 0.]
        patrol.home_z = patrol.z_command = -.04
        patrol._armed = armed
        patrol.armed = lambda: patrol._armed
        patrol.landed = (mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND if on_ground
                         else mavutil.mavlink.MAV_LANDED_STATE_IN_AIR)
        patrol.last_landed_wall = 0.
        patrol.truth = PatrolTruth.__new__(PatrolTruth)
        patrol.truth.lock = threading.Lock()
        patrol.truth.rows = deque([(0., 10., 20., .21)])
        patrol.truth.latest = patrol.truth.previous = (0., 10., 20., .21)
        patrol.truth.origin = (0., 0., 0.)
        patrol.latest_samples = []
        clock = [0.]

        def service():
            clock[0] += .25
            patrol.sim_time += .25
            z, vz = motion(patrol.sim_time) if motion else (-.04, 0.)
            patrol.position = [.02, -.01, z]
            patrol.velocity = [0., 0., vz]
            patrol.latest_samples = [(patrol.sim_time, patrol.position, patrol.velocity)]
            patrol.last_landed_wall = clock[0] - (10. if stale_landed else 0.)
            patrol.truth.latest = (patrol.sim_time, 10., 20., .21)
            return True

        patrol.service = service
        return patrol, clock

    def initialize(self, patrol, clock):
        with (mock.patch('flight_patrol.time.monotonic', side_effect=lambda: clock[0]),
              contextlib.redirect_stdout(io.StringIO())):
            patrol.initialize_ground_reference()

    def test_baro_reset_after_five_seconds_does_not_preserve_old_home(self):
        # 约5秒后的气压/GNSS校准，使本地z重置−3.9969m，实体仍停在地面。
        patrol, clock = self.make_patrol(lambda t: (-.04-(3.9969 if t >= 5.25 else 0.), 0.))
        self.initialize(patrol, clock)
        self.assertGreaterEqual(patrol.sim_time, 10.25)
        self.assertAlmostEqual(patrol.home_z, -4.0369)
        self.assertEqual(patrol.z_command, patrol.home_z)
        self.assertEqual(patrol.home_north, .02)
        self.assertEqual(patrol.truth.origin, (10., 20., .21))
        self.assertEqual(len(patrol.truth.rows), 0)
        self.assertEqual(patrol.truth.previous[0], patrol.sim_time)
        summary = patrol.report['ground_reference']
        self.assertTrue(summary['passed'])
        self.assertEqual(patrol.phase, 'ground_reference')
        self.assertLessEqual(summary['z_range_m'], .25)

    def test_stable_ground_still_waits_at_least_ten_simulation_seconds(self):
        patrol, clock = self.make_patrol()
        self.initialize(patrol, clock)
        self.assertEqual(patrol.sim_time, 10.)
        self.assertEqual(patrol.home_z, -.04)
        self.assertTrue(patrol.report['ground_reference']['passed'])

    def test_unstable_height_times_out_without_recapturing_reference(self):
        patrol, clock = self.make_patrol(lambda t: (int(t) % 2, 0.))
        with self.assertRaisesRegex(RuntimeError, 'ground_reference.*安全时限'):
            self.initialize(patrol, clock)
        self.assertFalse(patrol.report['ground_reference']['passed'])
        self.assertEqual(patrol.home_z, -.04)
        self.assertEqual(patrol.truth.origin, (0., 0., 0.))

    def test_vertical_motion_cannot_be_declared_stable(self):
        patrol, clock = self.make_patrol(lambda t: (-.04, .2))
        with self.assertRaisesRegex(RuntimeError, '安全时限'):
            self.initialize(patrol, clock)
        self.assertFalse(patrol.report['ground_reference']['passed'])

    def test_batched_telemetry_cannot_hide_an_intermediate_height_jump(self):
        patrol, clock = self.make_patrol()
        service = patrol.service

        def service_with_batch():
            value = service()
            if patrol.sim_time == 8.:
                patrol.latest_samples.insert(0, (7.875, [0., 0., .96], [0., 0., 0.]))
            return value

        patrol.service = service_with_batch
        self.initialize(patrol, clock)
        self.assertGreaterEqual(patrol.sim_time, 13.)

    def test_already_armed_is_rejected_before_waiting(self):
        patrol, clock = self.make_patrol(armed=True)
        with self.assertRaisesRegex(RuntimeError, '已解锁'):
            self.initialize(patrol, clock)
        self.assertEqual(patrol.sim_time, 0.)
        self.assertEqual(patrol.truth.origin, (0., 0., 0.))

    def test_arming_during_wait_is_rejected(self):
        patrol, clock = self.make_patrol()
        service = patrol.service

        def arm_during_service():
            value = service()
            patrol._armed = patrol.sim_time >= 2.
            return value

        patrol.service = arm_during_service
        with self.assertRaisesRegex(RuntimeError, '初始化期间飞控已解锁'):
            self.initialize(patrol, clock)
        self.assertFalse(patrol.report['ground_reference']['passed'])

    def test_not_on_ground_or_stale_landed_state_cannot_authorize_takeoff(self):
        for options in ({'on_ground': False}, {'stale_landed': True}):
            with self.subTest(**options):
                patrol, clock = self.make_patrol(**options)
                with self.assertRaisesRegex(RuntimeError, '安全时限'):
                    self.initialize(patrol, clock)
                self.assertFalse(patrol.report['ground_reference']['passed'])

    def test_brief_on_ground_departure_restarts_the_full_stability_window(self):
        patrol, clock = self.make_patrol()
        service = patrol.service

        def service_with_air_state():
            value = service()
            patrol.landed = (mavutil.mavlink.MAV_LANDED_STATE_IN_AIR if patrol.sim_time == 8.
                             else mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND)
            return value

        patrol.service = service_with_air_state
        self.initialize(patrol, clock)
        self.assertEqual(patrol.sim_time, 13.25)


if __name__ == '__main__':
    unittest.main()
