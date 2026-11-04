"""网关真实测量来源、受限能力状态及人工接管回归。"""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from common.config import load_config
from common import protocol as proto
from controller.service.gimbal_bridge import BridgeCore


class BridgeTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.commands = []
        self.diagnostics = []
        self.now = 0.0
        self.core = BridgeCore(self.config, self.commands.append, self.diagnostics.append,
                               clock=lambda: self.now)
        self.peer = ('127.0.0.1', 40101)
        self.sample = dict(config_hash=self.config['config_hash'], configured=True, motor_enabled=True,
            mechanical_valid=True, camera_ready=True, ts_mono=1.0,
            mode='IDLE', joints_deg={'yaw': {'pos': 12.34, 'vel': 30.0},
                                     'pitch': {'pos': -56.78, 'vel': -25.0}},
            tracking={'state': 'IDLE', 'fault': None},
            target={'valid': False, 'entity_name': None, 'miss': {'valid': False}},
            truth_candidates=[], limits={'yaw': False, 'pitch': True}, singular=False)

    def command(self, cmd, x=0, y=0):
        return self.core.receive(proto.encode_command(cmd, x, y), self.peer)

    def source(self, **changes):
        value = deepcopy(self.sample)
        value.update(changes)
        return self.core.on_telemetry(value)

    def candidate(self, name='target_a', x=960, distance=100):
        return dict(name=name, entity_id=10 if name == 'target_a' else 11,
                    frame_seq=30, center_px=[x, 540], range_m=distance,
                    depth_m=50, valid=True)

    def test_actual_joint_measurements_not_command_or_los_are_encoded(self):
        self.command(proto.CMD_MANUAL, 200)
        fields = proto.decode_telemetry(self.source(target_los={'yaw_rate_dps': 9999}))
        self.assertEqual((fields['frame_yaw'], fields['frame_pitch']), (12.34, -56.78))
        self.assertEqual((fields['yaw_rate'], fields['pitch_rate']), (30.0, -25.0))
        self.assertEqual((fields['frame_roll'], fields['roll_rate'], fields['laser_range']), (0, 0, 0))
        self.assertEqual(fields['status2'], 8)
        self.assertEqual(fields['self_check'], 0x80)

    def test_invalid_or_wrong_configuration_does_not_generate_zero_measurements(self):
        for changes in ({'config_hash': 'wrong'}, {'mechanical_valid': False},
                        {'joints_deg': {}}, {'configured': False}):
            with self.subTest(changes=changes):
                self.assertIsNone(self.source(**changes))

    def test_duplicate_source_is_not_replayed_but_identical_new_step_is_sent(self):
        self.assertIsNotNone(self.source())
        self.assertIsNone(self.source())
        self.assertIsNotNone(self.source(ts_mono=1.01))

    def test_self_check_tracks_initialization_and_real_errors(self):
        initial = proto.decode_telemetry(self.source(camera_ready=False))
        self.assertEqual(initial['self_check'], 1)
        ready = proto.decode_telemetry(self.source(ts_mono=2))
        self.assertEqual(ready['self_check'], 0x80)
        failed = proto.decode_telemetry(self.source(ts_mono=3, camera_ready=False,
                          tracking={'state': 'FAULT', 'fault': 'encoder_fault'}))
        self.assertEqual(failed['self_check'], 0x83)
        self.assertEqual(failed['status2'] & 8, 0)

    def test_out_of_range_angle_does_not_establish_or_renew_lease(self):
        self.assertFalse(self.command(proto.CMD_FRAME_ANGLE, 9001))
        self.assertIsNone(self.core.session.peer)
        self.assertFalse(self.commands)

    def test_manual_command_is_actually_limited_before_publishing(self):
        self.command(proto.CMD_MANUAL, 500, -500)
        self.assertEqual(self.commands[-1]['cmd'], 'manual')
        self.assertEqual((self.commands[-1]['yaw'], self.commands[-1]['pitch']), (20, -20))

    def test_old_manual_watchdog_cannot_stop_new_home(self):
        self.command(proto.CMD_MANUAL, 200)
        self.now = .1
        self.command(proto.CMD_HOME)
        count = len(self.commands)
        self.now = .7
        self.core.poll()
        self.assertEqual(len(self.commands), count)
        self.now = 2.1
        self.core.poll()
        self.assertEqual(self.commands[-1]['cmd'], 'stop')

    def test_point_uses_center_origin_and_keeps_entity_frame_identity(self):
        self.source(truth_candidates=[self.candidate(x=1200)])
        self.command(proto.CMD_POINT_TRACK, 240, 0)
        binding = self.commands[-1]
        self.assertEqual(binding['cmd'], 'track_entity')
        self.assertEqual((binding['entity_name'], binding['entity_id'], binding['frame_seq']),
                         ('target_a', 10, 30))

    def test_auto_waits_then_acquires_and_does_not_switch_healthy_identity(self):
        self.source()
        self.command(proto.CMD_AUTO_LOCK)
        self.assertEqual(self.core.tracking_state, 'AUTO_WAIT')
        self.now = .3
        self.source(ts_mono=2, truth_candidates=[self.candidate(x=1000)])
        self.assertEqual(self.core.bound_name, 'target_a')
        count = len(self.commands)
        self.now = .6
        valid_target = {'valid': True, 'entity_name': 'target_a', 'entity_id': 10,
                        'miss': {'valid': True, 'offset_deg': [0.2, 0.3]},
                        'box_px': [990, 530, 20, 20]}
        fields = proto.decode_telemetry(self.source(ts_mono=3, mode='TRACK',
            tracking={'state': 'TRACK', 'fault': None}, target=valid_target,
            truth_candidates=[self.candidate(x=1000, distance=301), self.candidate('target_b')]))
        self.assertEqual(len(self.commands), count)
        self.assertEqual(self.core.bound_name, 'target_a')
        self.assertTrue(fields['status1'] & 4)
        self.assertEqual((fields['offset_x'], fields['offset_y']), (.2, -.3))
        self.assertEqual(self.diagnostics[-1]['target_box_px'], [990, 530, 20, 20])

    def test_lost_target_clears_offsets_and_reacquisition_applies_discovery_range(self):
        self.source(truth_candidates=[self.candidate()])
        self.command(proto.CMD_AUTO_LOCK)
        self.now = .3
        fields = proto.decode_telemetry(self.source(ts_mono=2,
                            truth_candidates=[self.candidate(distance=301)]))
        self.assertEqual((fields['status1'] & 4, fields['offset_x'], fields['offset_y']), (0, 0, 0))
        self.assertIsNone(self.core.bound_name)

    def test_manual_takeover_ignores_late_old_binding_result(self):
        self.source(truth_candidates=[self.candidate()])
        self.command(proto.CMD_AUTO_LOCK)
        old_request = self.commands[-1]['request_id']
        self.command(proto.CMD_MANUAL, 100)
        self.core.on_result(dict(request_id=old_request, ok=True, cmd='track_entity'))
        self.assertEqual(self.core.selection_mode, 'OFF')
        self.assertIsNone(self.core.bound_name)
        self.assertFalse(self.core.target_valid)

    def test_repeated_auto_command_keeps_current_binding(self):
        self.source(truth_candidates=[self.candidate()])
        self.command(proto.CMD_AUTO_LOCK)
        count, request = len(self.commands), self.core.pending_request
        self.now = .1
        self.command(proto.CMD_AUTO_LOCK)
        self.assertEqual(len(self.commands), count)
        self.assertEqual(self.core.pending_request, request)
        self.assertEqual(self.core.bound_id, 10)

    def test_replaced_entity_with_same_name_is_not_valid_binding(self):
        self.source(truth_candidates=[self.candidate()])
        self.command(proto.CMD_POINT_TRACK)
        fields = proto.decode_telemetry(self.source(ts_mono=2, mode='TRACK',
            tracking={'state': 'TRACK'}, target={'valid': True, 'entity_name': 'target_a',
            'entity_id': 999, 'miss': {'valid': True, 'offset_deg': [1, 2]}}))
        self.assertFalse(fields['status1'] & 4)
        self.assertEqual((fields['offset_x'], fields['offset_y']), (0, 0))


if __name__ == '__main__':
    unittest.main()
