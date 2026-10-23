"""核对原生模型的机尾基准、整机重心与飞控/传感器参考点。"""
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from prepare_runtime import parameter_values


def xyz(text):
    return np.fromstring(text or '0 0 0 0 0 0', sep=' ')[:3]


class NativeLayoutTests(unittest.TestCase):
    def setUp(self):
        self.model = ET.parse(ROOT / 'model.sdf').getroot().find('model')
        self.base = self.model.find("link[@name='base_link']")
        links = self.model.findall('link')
        self.mass = sum(float(link.findtext('inertial/mass')) for link in links)
        self.center = sum(float(link.findtext('inertial/mass')) *
                          (xyz(link.findtext('pose')) + xyz(link.findtext('inertial/pose')))
                          for link in links) / self.mass
        self.parameters = parameter_values(next((ROOT / 'px4').glob('*_gz_rocket_racing_quad')).read_text())
        self.flu_to_frd = np.array([1., -1., -1.])

    def test_center_of_mass_is_three_centimeters_forward_of_arm_plane(self):
        np.testing.assert_allclose(self.center, [0, 0, .03], rtol=0, atol=1e-12)
        for index in range(4):
            arm = self.model.find(f"link[@name='nacelle_{index}']")
            rotor = self.model.find(f"link[@name='rotor_{index}']")
            arm_position, rotor_position = xyz(arm.findtext('pose')), xyz(rotor.findtext('pose'))
            self.assertAlmostEqual(arm_position[2], 0)
            np.testing.assert_allclose(rotor_position - arm_position, [0, 0, -.065], atol=1e-12)

    def test_control_allocation_levers_are_relative_to_vehicle_center(self):
        for index in range(4):
            rotor = xyz(self.model.findtext(f"link[@name='rotor_{index}']/pose"))
            actual = [self.parameters[f'CA_ROTOR{index}_P{axis}'] for axis in 'XYZ']
            np.testing.assert_allclose(actual, (rotor - self.center) * self.flu_to_frd, atol=1e-12)

    def test_sensor_levers_use_same_vehicle_center(self):
        for sensor_type, parameter in (('imu', 'IMU'), ('navsat', 'GPS')):
            sensor = self.base.find(f"sensor[@type='{sensor_type}']")
            position = xyz(sensor.findtext('pose'))
            np.testing.assert_allclose(position, [0, 0, .015], atol=1e-12)
            actual = [self.parameters[f'EKF2_{parameter}_POS_{axis}'] for axis in 'XYZ']
            np.testing.assert_allclose(actual, (position - self.center) * self.flu_to_frd, atol=1e-12)
        aerodynamic = self.model.find("plugin[@name='gz::sim::systems::AdvancedLiftDrag']")
        np.testing.assert_allclose(xyz(aerodynamic.findtext('cp')), [0, 0, .015], atol=1e-12)

    def test_tail_clearance_and_configured_hover_mapping(self):
        vertices = [list(map(float, line.split()[1:]))
                    for line in (ROOT / 'meshes/fuselage.stl').read_text().splitlines()
                    if line.strip().startswith('vertex ')]
        lower, upper = np.asarray(vertices)[:, 2].min(), np.asarray(vertices)[:, 2].max()
        self.assertAlmostEqual(lower, -.21)
        self.assertAlmostEqual(upper - lower, .59)
        world = ET.parse(ROOT / 'tests/world.sdf').getroot().find('world')
        spawn = xyz(world.findtext("include[uri='model://rocket_racing_quad']/pose"))
        self.assertAlmostEqual(spawn[2] + lower, .005)
        gravity = np.linalg.norm(xyz(world.findtext('gravity')))
        configured = self.parameters['MPC_THR_HOVER']
        thrust = 0.
        for plugin in self.model.findall("plugin[@name='gz::sim::systems::MulticopterMotorModel']"):
            channel = int(plugin.findtext('motorNumber')) + 1
            idle = self.parameters[f'SIM_GZ_EC_MIN{channel}']
            maximum = self.parameters[f'SIM_GZ_EC_MAX{channel}']
            self.assertEqual(maximum, float(plugin.findtext('maxRotVelocity')))
            self.assertEqual(maximum, 800)
            speed = idle + (maximum - idle) * np.sqrt(configured)
            thrust += float(plugin.findtext('motorConstant')) * speed**2
        self.assertAlmostEqual(thrust, self.mass * gravity, places=10)

    def test_high_acceleration_profile_is_explicit(self):
        expected = {
            'MPC_ACC_HOR': 15,
            'MPC_ACC_HOR_MAX': 15,
            'MPC_ACC_UP_MAX': 12,
            'MPC_ACC_DOWN_MAX': 6,
            'MPC_JERK_AUTO': 30,
            'MPC_JERK_MAX': 40,
        }
        self.assertEqual({key: self.parameters[key] for key in expected}, expected)


if __name__ == '__main__':
    unittest.main()
