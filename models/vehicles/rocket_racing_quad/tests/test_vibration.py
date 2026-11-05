"""Runtime profile validation and isolation checks (no Gazebo process required)."""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('configure_vibration', ROOT / 'tools/configure_vibration.py')
vibration = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vibration)


class VibrationRuntimeTests(unittest.TestCase):
    def test_real_model_patch_keeps_dynamics_and_other_sensors(self):
        original_bytes = (ROOT / 'model.sdf').read_bytes()
        model = ET.fromstring(original_bytes).find('model')
        original = copy.deepcopy(model)
        summary = vibration.configure_model(model, vibration.DEFAULT_PROFILE, 'unit_world')
        self.assertEqual(summary['imu_rate_hz'], 1000)
        imu = model.find("link[@name='base_link']/sensor[@name='imu_sensor']")
        original_imu = original.find("link[@name='base_link']/sensor[@name='imu_sensor']")
        self.assertEqual(imu.findtext('topic'), summary['raw_imu_topic'])
        self.assertNotEqual(summary['raw_imu_topic'], summary['output_imu_topic'])
        self.assertEqual(ET.tostring(imu.find('imu')), ET.tostring(original_imu.find('imu')))
        # Removing only the opt-in plugin and explicit sensor overrides restores XML.
        model.remove(model.find(f"plugin[@name='{vibration.PLUGIN_NAME}']"))
        for tag in ['topic', 'update_rate']:
            new, old = imu.find(tag), original_imu.find(tag)
            if old is None:
                imu.remove(new)
            else:
                new.text = old.text
        self.assertEqual(ET.tostring(model), ET.tostring(original))
        self.assertEqual(original_bytes, (ROOT / 'model.sdf').read_bytes())

    def test_multiaircraft_topics_and_duplicate_rejection(self):
        model = ET.parse(ROOT / 'model.sdf').getroot().find('model')
        model.set('name', 'rocket_two_axis_pod_7')
        summary = vibration.configure_model(model, vibration.DEFAULT_PROFILE, 'geographic')
        self.assertIn('/model/rocket_two_axis_pod_7/', summary['output_imu_topic'])
        self.assertEqual(len(model.findall(f"plugin[@name='{vibration.PLUGIN_NAME}']")), 1)
        with self.assertRaisesRegex(ValueError, 'already'):
            vibration.configure_model(model, vibration.DEFAULT_PROFILE, 'geographic')

    def test_world_patch_preserves_environment_and_smaller_step(self):
        world = ET.fromstring('<world name="w"><physics name="p" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics><gravity>0 0 -9.81</gravity><include><uri>model://unchanged</uri></include></world>')
        vibration.configure_world(world)
        self.assertEqual(float(world.findtext('physics/max_step_size')), .001)
        self.assertEqual(float(world.findtext('physics/real_time_update_rate')), 1000)
        self.assertEqual(world.findtext('include/uri'), 'model://unchanged')
        self.assertEqual(world.findtext('gravity'), '0 0 -9.81')
        world.find('physics/max_step_size').text = '.0005'
        vibration.configure_world(world)
        self.assertEqual(float(world.findtext('physics/max_step_size')), .0005)

    def test_profile_invalid_physics_and_aliasing_rejected_before_mutation(self):
        profile = vibration.load_profile(vibration.DEFAULT_PROFILE)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'invalid.json'
            for mutate in [lambda p: p.update(imu_rate_hz=500),
                           lambda p: p.update(rotor_reference_rad_s=0),
                           lambda p: p['gyro_components'][0].update(amplitude_rad_s=[float('nan'), 0, 0])]:
                p = copy.deepcopy(profile)
                mutate(p)
                path.write_text(json.dumps(p))
                model = ET.parse(ROOT / 'model.sdf').getroot().find('model')
                before = ET.tostring(model)
                with self.assertRaises(ValueError):
                    vibration.configure_model(model, path, 'unit_world')
                self.assertEqual(before, ET.tostring(model))

    def test_filter_defaults_are_whitelisted_and_runtime_only(self):
        profile = vibration.load_profile(vibration.DEFAULT_PROFILE)
        with tempfile.TemporaryDirectory() as directory:
            profile_path = Path(directory) / 'profile.json'
            path = Path(directory) / '4022_runtime'
            original = '#!/bin/sh\nparam set-default IMU_GYRO_CUTOFF 40\nparam set-default MC_ROLL_P 4\n'
            path.write_text(original)
            profile['px4_parameters'] = {'IMU_GYRO_CUTOFF': 25, 'IMU_DGYRO_CUTOFF': 20}
            profile_path.write_text(json.dumps(profile))
            vibration.configure_airframe(path, profile_path)
            self.assertTrue(path.read_text().startswith(original))
            self.assertIn('param set-default IMU_GYRO_CUTOFF 25', path.read_text())
            self.assertEqual(path.read_text().count('MC_ROLL_P'), 1)
            with self.assertRaisesRegex(ValueError, 'already'):
                vibration.configure_airframe(path, profile_path)
            with self.assertRaisesRegex(ValueError, 'runtime copy'):
                vibration.configure_airframe(ROOT / 'px4/4022_gz_rocket_racing_quad', profile_path)
            profile['px4_parameters']['MC_ROLL_P'] = 6.5
            profile_path.write_text(json.dumps(profile))
            with self.assertRaisesRegex(ValueError, 'only accepts'):
                vibration.load_profile(profile_path)


if __name__ == '__main__':
    unittest.main()
