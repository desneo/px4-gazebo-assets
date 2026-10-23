"""组合机 ULog 与实际 camera.quat_world 稳定性门禁回归。"""
from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'pod_stability_under_test', ROOT / 'tools/analyze_flight_stability.py')
stability = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(stability)


class Dataset:
    def __init__(self, data):
        self.data = data


class FakeULog:
    def __init__(self, datasets):
        self.datasets = datasets

    def get_dataset(self, name):
        if name not in self.datasets:
            raise KeyError(name)
        return Dataset(self.datasets[name])


def settings():
    return json.loads((ROOT / 'parameters.json').read_text())['stability_validation']


def quaternion(roll, pitch):
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    return cr*cp, sr*cp, cr*sp, -sr*sp


def fixture(bad=False):
    rate_t = np.arange(0., 30.001, .004)
    amplitude = math.radians(15 if bad else .1)
    roll_rate = amplitude * np.sin(2*math.pi*16*rate_t)
    pitch_rate = (1.1*amplitude) * np.sin(2*math.pi*16*rate_t + .3)
    setpoint_t = np.arange(0., 30.001, .02)
    attitude_t = np.arange(0., 30.001, .01)
    attitude_amplitude = math.radians(.7 if bad else .01)
    q = quaternion(attitude_amplitude*np.sin(2*math.pi*16*attitude_t),
                   attitude_amplitude*np.sin(2*math.pi*16*attitude_t+.2))
    allocator_t = np.arange(0., 30.001, .01)
    achieved = np.ones(len(allocator_t), dtype=bool)
    unallocated = np.zeros(len(allocator_t))
    if bad:
        achieved[700:1000] = False
        unallocated[700:1000] = .05
    motor_t = np.arange(0., 30.001, .01)
    motor = np.full(len(motor_t), .2)
    if bad:
        motor[700:1000] = 0.
    timestamp = lambda values: np.rint(values*1e6).astype(np.uint64)
    datasets = {
        'vehicle_angular_velocity': dict(timestamp=timestamp(rate_t),
            **{'xyz[0]': roll_rate, 'xyz[1]': pitch_rate}),
        'vehicle_rates_setpoint': dict(timestamp=timestamp(setpoint_t),
            roll=np.zeros(len(setpoint_t)), pitch=np.zeros(len(setpoint_t))),
        'vehicle_attitude': dict(timestamp=timestamp(attitude_t),
            **{f'q[{index}]': values for index, values in enumerate(q)}),
        'control_allocator_status': dict(timestamp=timestamp(allocator_t),
            torque_setpoint_achieved=achieved,
            **{'unallocated_torque[0]': unallocated, 'unallocated_torque[1]': unallocated}),
        'actuator_motors': dict(timestamp=timestamp(motor_t),
            **{f'control[{index}]': motor.copy() for index in range(4)}),
    }
    estimated = [dict(phase='hover', sim_time_s=float(value)) for value in np.arange(0, 30.1, .1)]
    truth = [dict(phase='hover', sim_time_s=float(100+value)) for value in np.arange(0, 30.1, .1)]
    return FakeULog(datasets), dict(samples=estimated, ground_truth_samples=truth)


class StabilityTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='flight-stability-test-')
        self.addCleanup(temporary.cleanup)
        self.work = Path(temporary.name)
        self.model = self.work / 'model.sdf'
        self.model.write_text('''<sdf version="1.9"><model name="test"><link name="camera_link">
          <sensor name="camera" type="camera"><camera><horizontal_fov>1.0471975512</horizontal_fov>
          <image><width>1920</width><height>1080</height></image></camera></sensor>
          </link></model></sdf>''')

    def camera(self, amplitude_deg=.01, mode='IDLE'):
        path = self.work / 'camera.jsonl'
        rows = []
        for stamp in np.arange(100., 130.001, .02):
            angle = math.radians(amplitude_deg)*math.sin(2*math.pi*10*(stamp-100))
            rows.append(dict(ts_mono=float(stamp), mode=mode,
                             camera=dict(quat_world=[math.cos(angle/2), 0., math.sin(angle/2), 0.])))
        path.write_text('\n'.join(json.dumps(row) for row in rows) + '\n')
        return path

    def test_stable_hover_and_actual_camera_los_pass(self):
        ulog, report = fixture()
        result = stability.analyze(ulog, report, dict(stability_validation=settings()),
                                   self.camera(), self.model)
        self.assertTrue(result['passed'])
        self.assertTrue(result['camera_los']['available'])
        self.assertTrue(result['camera_los']['passed'])
        self.assertEqual(result['hover']['ulog_time_s'], [5., 30.])
        self.assertEqual(result['hover']['gazebo_time_s'], [105., 130.])
        for axis in stability.AXES:
            self.assertTrue(result['axes'][axis]['passed'])
            self.assertAlmostEqual(result['axes'][axis]['spectral_band']['peak_hz'], 16., delta=.3)

    def test_rate_allocator_and_motor_limit_cycle_fails(self):
        ulog, report = fixture(bad=True)
        result = stability.analyze(ulog, report, dict(stability_validation=settings()),
                                   self.camera(), self.model)
        self.assertFalse(result['passed'])
        self.assertFalse(result['axes']['pitch']['passed'])
        self.assertFalse(result['allocator']['passed'])
        self.assertFalse(result['motors']['passed'])
        self.assertGreater(result['allocator']['longest_unachieved_s'], 1.)

    def test_missing_camera_is_explicitly_unavailable_not_fabricated(self):
        ulog, report = fixture()
        result = stability.analyze(ulog, report, dict(stability_validation=settings()),
                                   self.work/'missing.jsonl', self.model)
        self.assertFalse(result['passed'])
        self.assertFalse(result['camera_los']['available'])
        self.assertFalse(result['camera_los']['passed'])
        self.assertTrue(result['camera_los']['required'])
        self.assertIn('未收到', result['camera_los']['reason'])

    def test_available_camera_failure_is_a_gate(self):
        ulog, report = fixture()
        result = stability.analyze(ulog, report, dict(stability_validation=settings()),
                                   self.camera(amplitude_deg=1.), self.model)
        self.assertFalse(result['passed'])
        self.assertTrue(result['camera_los']['available'])
        self.assertFalse(result['camera_los']['passed'])

    def test_non_idle_camera_window_is_not_misclassified_as_jitter(self):
        ulog, report = fixture()
        result = stability.analyze(ulog, report, dict(stability_validation=settings()),
                                   self.camera(mode='TRACK'), self.model)
        self.assertFalse(result['passed'])
        self.assertIn('不是IDLE', result['camera_los']['reason'])

    def test_missing_required_ulog_topic_is_rejected(self):
        ulog, report = fixture()
        del ulog.datasets['control_allocator_status']
        with self.assertRaisesRegex(ValueError, 'control_allocator_status'):
            stability.analyze(ulog, report, dict(stability_validation=settings()))


if __name__ == '__main__':
    unittest.main(verbosity=2)
