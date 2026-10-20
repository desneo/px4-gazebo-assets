"""飞行验收的离线回归测试，不连接 MAVLink 或启动仿真。"""
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch


def load_flight_check():
    # 用协议边界替身隔离可选 pymavlink 依赖及网络连接。
    constants = SimpleNamespace(
        MAV_TYPE_GCS=6, MAV_AUTOPILOT_INVALID=8, MAV_AUTOPILOT_PX4=12,
        MAV_COMP_ID_AUTOPILOT1=1, MAV_FRAME_LOCAL_NED=1,
        MAV_MODE_FLAG_SAFETY_ARMED=128, MAV_CMD_SET_MESSAGE_INTERVAL=511,
        MAV_LANDED_STATE_ON_GROUND=1,
    )
    mavutil = SimpleNamespace(mavlink=constants, mavlink_connection=Mock(),
        interpret_px4_mode=Mock(return_value="OFFBOARD"))
    package = ModuleType("pymavlink")
    package.mavutil = mavutil
    spec = importlib.util.spec_from_file_location("rocket_flight_check", Path(__file__).with_name("flight_check.py"))
    module = importlib.util.module_from_spec(spec)
    with patch.dict("sys.modules", {"pymavlink": package}):
        spec.loader.exec_module(module)
    return module


flight = load_flight_check()


def message(kind, system=1, component=1, **values):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: system,
        get_srcComponent=lambda: component, **values)


def heartbeat(**values):
    return message("HEARTBEAT", autopilot=12, base_mode=values.pop("base_mode", 0),
        custom_mode=0, **values)


def position(stamp=1000, **values):
    fields = dict(time_boot_ms=stamp, x=0., y=0., z=-30., vx=0., vy=0., vz=0.)
    fields.update(values)
    return message("LOCAL_POSITION_NED", **fields)


class FlightCheckTests(unittest.TestCase):
    def setUp(self):
        self.master = Mock(target_system=1, target_component=0)
        self.master.recv_match.return_value = None
        flight.mavutil.mavlink_connection.return_value = self.master
        settings = flight.load_settings(flight.DEFAULT_CONFIG)
        self.report = {"samples": [], "status_text": [], "results": {}}
        self.check = flight.FlightCheck(settings, self.report)
        self.check.flight_z = -30.

    def test_truth_target_uses_snapshot_and_explicit_overrides(self):
        snapshot = {'model': {'name': 'rocket_two_axis_pod'},
                    'simulation': {'world_name': 'rocket_two_axis_pod_test'}}
        target = flight.measurement_target(snapshot)
        self.assertEqual(target, {'world_name': 'rocket_two_axis_pod_test', 'model_name': 'rocket_two_axis_pod'})
        self.assertEqual(flight.measurement_target(snapshot, 'other_world', 'other_entity'),
                         {'world_name': 'other_world', 'model_name': 'other_entity'})
        truth_module = ModuleType('gazebo_truth')
        truth_module.GazeboTruth = Mock()
        with patch.dict('sys.modules', {'gazebo_truth': truth_module}):
            flight.FlightCheck(self.check.cfg, self.report, ground_truth=True, **target)
        truth_module.GazeboTruth.assert_called_once_with(**target)

    def service_messages(self, *messages):
        self.master.recv_match.side_effect = [*messages, None]
        with patch.object(flight.time, "monotonic", return_value=100.), patch.object(flight.time, "sleep"):
            return self.check.service(check_health=False)

    def test_px4_heartbeat_selects_component_before_position_filter(self):
        self.assertTrue(self.service_messages(heartbeat(), position()))
        self.assertEqual(self.master.target_component, 1)
        self.assertEqual(self.check.position, [0., 0., -30.])
        self.assertEqual(len(self.report["samples"]), 1)

    def test_telemetry_from_other_system_or_component_is_ignored(self):
        self.service_messages(heartbeat(), position())
        self.assertFalse(self.service_messages(
            position(2000, system=2, x=99.), position(2000, component=2, x=99.),
            heartbeat(system=2, base_mode=128), heartbeat(component=2, base_mode=128)))
        self.assertEqual(self.check.position, [0., 0., -30.])
        self.assertEqual(self.check.sim_time, 1.)
        self.assertFalse(self.check.armed())

    def test_repeated_positions_do_not_extend_time_and_rollback_fails(self):
        self.service_messages(heartbeat(), position())
        self.assertFalse(self.service_messages(position()))
        self.assertEqual(len(self.report["samples"]), 1)
        with self.assertRaisesRegex(RuntimeError, "时钟重置"):
            self.service_messages(position(999))

    def test_already_armed_flight_is_rejected_without_landing_command(self):
        self.master.recv_match.side_effect = [heartbeat(base_mode=128), position(), None]
        with patch.object(flight.time, "monotonic", return_value=100.), patch.object(flight.time, "sleep"):
            with self.assertRaisesRegex(RuntimeError, "拒绝接管"):
                self.check.connect()
            self.check.emergency_land()
        self.master.set_mode_px4.assert_not_called()
        self.assertFalse(self.check.armed_by_script)

    def test_command_confirmation_continues_servicing_between_retries(self):
        clock = [100.]
        def service():
            clock[0] += .25
        self.check.service = Mock(side_effect=service)
        action = Mock()
        with patch.object(flight.time, "monotonic", side_effect=lambda: clock[0]):
            self.check.command_until(action, lambda: self.check.service.call_count >= 6, 3)
        self.assertEqual(self.check.service.call_count, 6)
        self.assertEqual(action.call_count, 2)

    def speed_samples(self, samples):
        self.check.sim_time = 0.
        self.check.ramp = Mock()
        def phase_loop(name, callback, timeout=None):
            for stamp, kmh, z in samples:
                self.check.sim_time = stamp
                self.check.velocity = [kmh / 3.6, 0., 0.]
                self.check.position = [0., 0., z]
                if callback():
                    return
            raise RuntimeError("测试样本用尽")
        self.check.phase_loop = phase_loop

    def test_speed_requires_full_threshold_and_continuous_valid_altitude(self):
        self.check.cfg["telemetry_timeout_s"] = 2.
        self.speed_samples([(1, 295, -30), (2, 300, -30), (3, 300, -30),
            (4, 300, -40), (5, 300, -30), (6, 300, -30), (7, 300, -30)])
        self.check.speed_test("sprint", 300, 2)
        self.assertEqual(self.check.sim_time, 7)
        self.assertEqual(self.report["results"]["sprint"]["longest_continuous_s"], 2)
        self.assertTrue(self.report["results"]["sprint"]["passed"])

    def test_ninety_eight_percent_speed_never_passes(self):
        self.check.cfg["telemetry_timeout_s"] = 2.
        self.speed_samples([(stamp, 295, -30) for stamp in range(1, 12)])
        with self.assertRaises(RuntimeError):
            self.check.speed_test("sprint", 300, 2)
        self.assertFalse(self.report["results"]["sprint"]["passed"])
        self.assertEqual(self.report["results"]["sprint"]["longest_continuous_s"], 0)

    def test_batch_intermediate_speed_or_altitude_violation_resets_hold(self):
        for bad_speed, bad_z in ((295, -30), (300, -40)):
            with self.subTest(bad_speed=bad_speed, bad_z=bad_z):
                self.check.cfg["telemetry_timeout_s"] = 2.
                self.check.sim_time = 0.
                self.check.ramp = Mock()
                batches = [[(1, 300, -30)],
                    [(2, bad_speed, bad_z), (3, 300, -30)],
                    [(4, 300, -30)], [(5, 300, -30)]]
                decisions = []
                def phase_loop(name, callback, timeout=None):
                    for batch in batches:
                        self.check.latest_samples = [(stamp, [0., 0., z],
                            [kmh / 3.6, 0., 0.]) for stamp, kmh, z in batch]
                        stamp, pos, vel = self.check.latest_samples[-1]
                        self.check.sim_time = stamp
                        self.check.position = pos
                        self.check.velocity = vel
                        done = callback()
                        decisions.append(done)
                        if done:
                            return
                    raise RuntimeError("测试样本用尽")
                self.check.phase_loop = phase_loop
                self.check.speed_test("sprint", 300, 2)
                self.assertEqual(decisions, [False, False, False, True])
                self.assertEqual(self.check.sim_time, 5)
                self.assertEqual(self.report["results"]["sprint"]["longest_continuous_s"], 2)
                if bad_z == -40:
                    self.assertEqual(self.report["results"]["sprint"]["max_altitude_error_m"], 10)


class ContinuousHoldTests(unittest.TestCase):
    def test_invalid_sample_resets_continuity_and_preserves_longest(self):
        hold = flight.ContinuousHold(2)
        self.assertEqual(hold.update(0, True), 0)
        self.assertEqual(hold.update(1, True), 1)
        self.assertEqual(hold.update(2, False), 0)
        self.assertEqual(hold.update(3, True), 0)
        self.assertEqual(hold.update(4, True), 1)
        self.assertEqual(hold.longest, 1)

    def test_missing_sample_gap_restarts_continuity(self):
        hold = flight.ContinuousHold(1)
        hold.update(0, True)
        self.assertEqual(hold.update(1, True), 1)
        self.assertEqual(hold.update(2.1, True), 0)
        self.assertAlmostEqual(hold.update(2.6, True), .5)
        self.assertEqual(hold.longest, 1)

    def test_duplicate_or_backwards_clock_is_rejected(self):
        for stamp in (2, 1):
            with self.subTest(stamp=stamp):
                hold = flight.ContinuousHold(2)
                hold.update(2, True)
                with self.assertRaises(RuntimeError):
                    hold.update(stamp, True)


if __name__ == "__main__":
    unittest.main()
