"""最终框的原始视频中心、half-up整数、有效性与UI/实际UDP一致性。"""
import copy
import json
from pathlib import Path
import socket
import sys
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from telemetry_forward import TelemetryForwarder, aisheng_message, tracking_gate_center
from test_forward import telemetry
from test_joystick import GroundStation


# 当前既有16字段值，新增框中心不改变这些字段的数值、类型或有效性条件。
EXISTING_FIELDS = {
    'track_status': 1, 'pitch_angle': -56.37, 'yaw_angle': 12.34,
    'pitch_misalignment': .5, 'yaw_misalignment': -.25,
    'tracking_confidence': 25.5, 'target_confidence': .8,
    'gyro_pitch_rate': -1.23, 'gyro_yaw_rate': 2.34,
    'pitch_los_rate': 4.5, 'yaw_los_rate': -6.25,
    'los_omega_n': 7.125, 'los_omega_e': -28.5, 'los_omega_d': 32.75,
    'tracking_gate_w': 50., 'tracking_gate_h': 40.,
}


def old_fields(packet):
    return {key: value for key, value in packet.items() if key not in ('gate_x', 'gate_y')}


class GateCenterTests(unittest.TestCase):
    def test_raw_video_pixel_center_is_half_up_integer_on_both_axes(self):
        for box, expected in (([95., 195., 50., 40.], (120, 215)),
                              ([0., 0., 1., 1.], (1, 1)),
                              ([2., 4., 1., 1.], (3, 5)),
                              ([.125, .375, .75, .25], (1, 1)),
                              ([1910.25, 1070.25, 9., 9.], (1915, 1075))):
            source = telemetry()
            source['target']['box_px'] = box
            original = copy.deepcopy(source)
            with self.subTest(box=box):
                self.assertEqual(tracking_gate_center(source), expected)
                packet = json.loads(json.dumps(aisheng_message(source)))
                self.assertEqual((packet['gate_x'], packet['gate_y']), expected)
                self.assertIs(type(packet['gate_x']), int)
                self.assertIs(type(packet['gate_y']), int)
                self.assertEqual(source, original)

    def test_center_uses_unrounded_coordinates_and_width_but_width_output_stays_scaled(self):
        source = telemetry()
        source['target']['box_px'] = [100.4999997, 200.4999997, 10.0000002, 20.0000002]
        packet = aisheng_message(source)
        self.assertEqual((packet['gate_x'], packet['gate_y']), (105, 210))
        self.assertEqual(packet['tracking_gate_w'], 10.)
        self.assertEqual(packet['tracking_gate_h'], 20.)
        source['target']['box_px'] = [100.5, 200.5, 10., 20.]
        packet = aisheng_message(source)
        self.assertEqual((packet['gate_x'], packet['gate_y']), (106, 211))

    def test_final_clipped_box_center_does_not_use_miss_projection_or_los_entity_position(self):
        source = telemetry()
        source['target']['box_px'] = [0., 0., 31., 19.]
        source['target']['miss']['center_px'] = [-5.25, -7.75]
        source['target']['center_px'] = [999., 888.]
        source['target_los']['relative_position_ned_m'] = [321., -654., 987.]
        packet = aisheng_message(source)
        self.assertEqual((packet['gate_x'], packet['gate_y']), (16, 10))
        self.assertEqual(GroundStation._gate_center_line(source, True), '跟踪框中心 px: X 16  Y 10')

    def test_all_existing_sixteen_fields_remain_identical_with_integer_center_added(self):
        packet = aisheng_message(telemetry())
        self.assertEqual(len(packet), 18)
        self.assertEqual(old_fields(packet), EXISTING_FIELDS)
        self.assertEqual({key: type(value) for key, value in old_fields(packet).items()},
                         {key: type(value) for key, value in EXISTING_FIELDS.items()})

    def test_invalid_tracking_uses_integer_zero_and_preserves_prior_invalid_field_values(self):
        changes = ((('mode',), 'IDLE'), (('tracking', 'state'), 'TEMP_LOST'),
                   (('tracking', 'state'), 'LOST'), (('tracking', 'state'), 'INIT'),
                   (('target', 'valid'), False), (('target', 'miss', 'valid'), False))
        expected = dict(EXISTING_FIELDS, track_status=0, pitch_misalignment=0., yaw_misalignment=0.,
                        tracking_confidence=0., tracking_gate_w=0., tracking_gate_h=0.)
        for path, value in changes:
            source = telemetry()
            node = source
            for key in path[:-1]:
                node = node[key]
            node[path[-1]] = value
            source['target']['box_px'] = '过期的错误框无需解析'
            with self.subTest(path=path, value=value):
                packet = aisheng_message(source)
                self.assertEqual(old_fields(packet), expected)
                self.assertEqual((packet['gate_x'], packet['gate_y']), (0, 0))
                self.assertIs(type(packet['gate_x']), int)
                self.assertIs(type(packet['gate_y']), int)
                self.assertEqual(GroundStation._gate_center_line(source, True), '跟踪框中心 px: X 0  Y 0')

    def test_auto_search_detect_off_or_vision_error_cannot_reuse_old_center(self):
        for vision in ({'mode': 'off'}, {'mode': 'detect'},
                       {'mode': 'auto_lock', 'state': 'AUTO_SEARCH'},
                       {'mode': 'auto_lock', 'state': 'AUTO_LOCKED', 'locked_track_id': None},
                       {'mode': 'auto_lock', 'state': 'AUTO_LOCKED', 'locked_track_id': True},
                       {'mode': 'manual_track', 'error': '视觉错误'}):
            source = telemetry()
            source['vision'] = vision
            with self.subTest(vision=vision):
                packet = aisheng_message(source)
                self.assertEqual((packet['gate_x'], packet['gate_y']), (0, 0))
                self.assertEqual(packet['track_status'], 0)
                self.assertEqual(GroundStation._gate_center_line(source, True), '跟踪框中心 px: X 0  Y 0')

    def test_confirmed_auto_lock_has_same_center_and_ui_as_manual_track(self):
        source = telemetry()
        expected = tracking_gate_center(source)
        source['vision'] = {'mode': 'auto_lock', 'state': 'AUTO_LOCKED', 'locked_track_id': 7}
        packet = aisheng_message(source)
        self.assertEqual((packet['gate_x'], packet['gate_y']), expected)
        self.assertEqual(GroundStation._gate_center_line(source, True), '跟踪框中心 px: X 120  Y 215')

    def test_nonfinite_boolean_negative_or_missing_box_origin_rejects_packet(self):
        for axis in (0, 1):
            for value in (True, False, None, '2', [], -0.00001, float('nan'), float('inf'), -float('inf'), 10**400):
                source = telemetry()
                source['target']['box_px'][axis] = value
                with self.subTest(axis=axis, value=str(value)), self.assertRaises(ValueError):
                    aisheng_message(source)
                self.assertIn('X --  Y --', GroundStation._gate_center_line(source, True))
        for box in (None, [], [1., 2., 3.]):
            source = telemetry()
            source['target']['box_px'] = box
            with self.subTest(box=box), self.assertRaises(ValueError):
                tracking_gate_center(source)

    def test_stale_ui_hides_previous_center_and_resumes_without_cached_value(self):
        source = telemetry()
        self.assertIn('X 120  Y 215', GroundStation._gate_center_line(source, True))
        self.assertIsNone(tracking_gate_center(source, fresh=False))
        self.assertEqual(GroundStation._gate_center_line(source, False), '跟踪框中心 px: X --  Y --')
        source['target']['box_px'] = [10., 20., 9., 9.]
        self.assertIn('X 15  Y 25', GroundStation._gate_center_line(source, True))


class GateCenterUdpTests(unittest.TestCase):
    def setUp(self):
        self.receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.receiver.bind(('127.0.0.1', 0))
        self.receiver.settimeout(1.)
        self.addCleanup(self.receiver.close)
        self.forward = TelemetryForwarder()
        self.addCleanup(self.forward.close)
        self.forward.start(*self.receiver.getsockname())

    def test_actual_udp_uses_nineteen_fields_and_ui_matches_valid_then_search_zero(self):
        source = telemetry()
        source['target']['box_px'] = [2., 4., 1., 1.]
        self.forward.offer(json.dumps(source).encode())
        packet = json.loads(self.receiver.recv(8192))
        self.assertEqual(len(packet), 19)
        self.assertEqual((packet['gate_x'], packet['gate_y']), (3, 5))
        self.assertIs(type(packet['gate_x']), int)
        self.assertIn('X 3  Y 5', GroundStation._gate_center_line(source, True))
        source['vision'] = {'mode': 'auto_lock', 'state': 'AUTO_SEARCH'}
        self.forward.offer(json.dumps(source).encode())
        packet = json.loads(self.receiver.recv(8192))
        self.assertEqual((packet['gate_x'], packet['gate_y']), (0, 0))
        self.assertIn('X 0  Y 0', GroundStation._gate_center_line(source, True))
        self.assertEqual(self.forward.snapshot()['sent'], 2)

    def test_bad_origin_is_not_sent_and_next_valid_measurement_recovers(self):
        source = telemetry()
        source['target']['box_px'][0] = -1.
        self.forward.offer(json.dumps(source).encode())
        deadline = time.monotonic() + 1.
        while self.forward.snapshot()['dropped'] == 0 and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertEqual(self.forward.snapshot()['dropped'], 1)
        self.assertEqual(self.forward.snapshot()['sent'], 0)
        self.forward.offer(json.dumps(telemetry()).encode())
        packet = json.loads(self.receiver.recv(8192))
        self.assertEqual((packet['gate_x'], packet['gate_y']), (120, 215))
        self.assertEqual(self.forward.snapshot()['error'], '')


if __name__ == '__main__':
    unittest.main()
