#!/usr/bin/env python3
"""火箭机 + 艾晟导引头动态装配生成检查。"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('seeker_dynamic', ROOT / 'tools/generate_dynamic_model.py')
dynamic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dynamic)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class DynamicModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.before = (sha(dynamic.ROCKET / 'model.sdf'), sha(dynamic.POD / 'model.sdf'))
        cls.temp = tempfile.TemporaryDirectory(prefix='rocket-aisheng-dynamic-')
        cls.output = Path(cls.temp.name) / dynamic.NAME
        cls.manifest = dynamic.generate(cls.output)

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_source_models_unchanged(self):
        self.assertEqual(self.before, (sha(dynamic.ROCKET / 'model.sdf'), sha(dynamic.POD / 'model.sdf')))

    def test_manifest_and_mass(self):
        manifest = self.manifest
        self.assertEqual(manifest['model_name'], 'rocket_aisheng_seeker')
        self.assertEqual(manifest['airframe_id'], 4024)
        self.assertEqual(manifest['topic_prefix'], '/rocket_aisheng_seeker')
        self.assertEqual(manifest['image_topic'], '/rocket_aisheng_seeker/image')
        self.assertEqual(manifest['image_topic_tele'], '/rocket_aisheng_seeker/image_tele')
        self.assertAlmostEqual(manifest['total_mass_kg'], 5.0, places=9)
        self.assertAlmostEqual(manifest['mount_z_m'], 0.289, places=9)
        self.assertEqual(manifest['px4_reference']['reference_pose'], 'zero_joint_positions')
        self.assertEqual(manifest['los_observer_reference']['kind'], 'nominal_cg')

    def test_model_structure_and_interfaces(self):
        model = ET.parse(self.output / 'model.sdf').getroot().find('model')
        self.assertEqual(model.get('name'), 'rocket_aisheng_seeker')
        self.assertEqual(model.get('canonical_link'), 'base_link')
        self.assertEqual(model.findtext('static'), 'false')
        links = {link.get('name') for link in model.findall('link')}
        self.assertTrue({'base_link', 'mount_link', 'yaw_arm_link', 'camera_link'} <= links)
        joints = {joint.get('name'): joint for joint in model.findall('joint')}
        self.assertTrue({'yaw_joint', 'pitch_joint', 'pod_mount_joint'} <= set(joints))
        self.assertEqual(joints['pod_mount_joint'].get('type'), 'fixed')
        self.assertEqual(joints['pod_mount_joint'].findtext('parent'), 'base_link')
        self.assertEqual(joints['pod_mount_joint'].findtext('child'), 'mount_link')
        plugin = model.find("plugin[@name='two_axis::GimbalController']")
        self.assertIsNotNone(plugin)
        self.assertEqual(plugin.get('filename'), 'gz-aisheng-seeker-controller')
        self.assertEqual(plugin.findtext('mount_dynamic'), 'true')
        self.assertEqual(plugin.findtext('command_topic'), '/rocket_aisheng_seeker/command')
        self.assertEqual(plugin.findtext('image_topic'), '/rocket_aisheng_seeker/image')
        self.assertEqual(plugin.findtext('image_topic_tele'), '/rocket_aisheng_seeker/image_tele')
        self.assertEqual(plugin.findtext('camera_sensor'), 'camera')
        self.assertEqual(plugin.findtext('camera_sensor_tele'), 'camera_tele')
        observer = [float(v) for v in plugin.findtext('los_observer_cg_model').split()]
        for actual, expected in zip(observer, self.manifest['px4_reference']['assembly_nominal_cg_flu_m']):
            self.assertAlmostEqual(actual, expected, places=12)
        cameras = {s.get('name'): s.findtext('topic') for s in model.findall('.//sensor') if s.get('type') == 'camera'}
        self.assertEqual(cameras, {'camera': '/rocket_aisheng_seeker/image',
                                   'camera_tele': '/rocket_aisheng_seeker/image_tele'})

    def test_scenario_topic_prefix(self):
        with tempfile.TemporaryDirectory(prefix='rocket-aisheng-prefix-') as temp:
            output = Path(temp) / dynamic.NAME
            manifest = dynamic.generate(output, topic_prefix='/uav_3/pod')
            self.assertEqual(manifest['topic_prefix'], '/uav_3/pod')
            self.assertEqual(manifest['image_topic'], '/uav_3/pod/image')
            self.assertEqual(manifest['image_topic_tele'], '/uav_3/pod/image_tele')
            model = ET.parse(output / 'model.sdf').getroot().find('model')
            plugin = model.find("plugin[@name='two_axis::GimbalController']")
            self.assertEqual(plugin.findtext('command_topic'), '/uav_3/pod/command')
            self.assertEqual(plugin.findtext('telemetry_topic'), '/uav_3/pod/telemetry')
            self.assertEqual(plugin.findtext('image_topic'), '/uav_3/pod/image')
            self.assertEqual(plugin.findtext('image_topic_tele'), '/uav_3/pod/image_tele')
            cameras = {s.get('name'): s.findtext('topic')
                       for s in model.findall('.//sensor') if s.get('type') == 'camera'}
            self.assertEqual(cameras, {'camera': '/uav_3/pod/image',
                                       'camera_tele': '/uav_3/pod/image_tele'})

    def test_airframe_and_meshes(self):
        airframe = self.output / 'px4' / '4024_gz_rocket_aisheng_seeker'
        self.assertTrue(airframe.is_file())
        text = airframe.read_text()
        self.assertIn('gz_rocket_aisheng_seeker', text)
        for key in ('MPC_THR_HOVER', 'CA_ROTOR0_PX', 'EKF2_IMU_POS_X',
                    'EKF2_GPS_POS_Z', 'EKF2_HGT_REF'):
            self.assertIn(f'param set-default {key} ', text)
        model = ET.parse(self.output / 'model.sdf').getroot().find('model')
        for uri in model.findall('.//mesh/uri'):
            self.assertTrue(uri.text.startswith('model://rocket_aisheng_seeker/'))
            relative = uri.text.removeprefix('model://rocket_aisheng_seeker/')
            self.assertTrue((self.output / relative).is_file(), relative)


if __name__ == '__main__':
    unittest.main(verbosity=2)
