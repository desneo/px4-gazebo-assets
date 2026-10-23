#!/usr/bin/env python3
"""验证原生几何继承、顶部替换接口、网格、静态资产与两轴机械角。"""
from collections import Counter
import importlib.util
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('appearance_generator', ROOT / 'tools/generate_model.py')
appearance = importlib.util.module_from_spec(spec)
spec.loader.exec_module(appearance)


class AppearanceTests(unittest.TestCase):
    def test_native_extent_and_mount(self):
        source = appearance.source_geometry()
        model = ET.parse(appearance.ROCKET / 'model.sdf').getroot().find('model')
        collision = model.find("link[@name='base_link']/collision[@name='body_collision']")
        length = float(collision.findtext('geometry/cylinder/length'))
        center = appearance.transform(collision.findtext('pose'))[2, 3]
        self.assertAlmostEqual(source['min_z_m'], center - length / 2)
        self.assertAlmostEqual(source['max_z_m'], center + length / 2)
        self.assertAlmostEqual(source['length_m'], length)
        assembly, _, pod = appearance.configuration()
        mount = center + length / 2 - pod['overall_height_m']
        np.testing.assert_allclose(appearance.dimensions(),
                                   [length, mount, mount - assembly['shoulder_length_m']], atol=1e-12)

    def test_cut_preserves_native_lower_triangles_and_is_closed(self):
        source, pose, scale = appearance.source_mesh('fuselage_visual')
        triangles = appearance.cut_fuselage() * scale
        triangles = triangles @ pose[:3, :3].T + pose[:3, 3]
        shoulder, mount = appearance.dimensions()[2], appearance.dimensions()[1]
        original = source[np.all(source[:, :, 2] < shoulder, axis=1)]
        retained = triangles[np.all(triangles[:, :, 2] < shoulder, axis=1)]
        np.testing.assert_allclose(retained, original, atol=1e-12)
        self.assertAlmostEqual(triangles[:, :, 2].min(), source[:, :, 2].min())
        self.assertAlmostEqual(triangles[:, :, 2].max(), mount)
        rim = triangles.reshape(-1, 3)
        rim = rim[np.isclose(rim[:, 2], mount) & (np.linalg.norm(rim[:, :2], axis=1) > 1e-10)]
        np.testing.assert_allclose(np.linalg.norm(rim[:, :2], axis=1), .042, atol=1e-12)
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        self.assertTrue(np.all(np.linalg.norm(normals, axis=1) > 1e-14))
        edges = Counter()
        for triangle in np.round(triangles, 9):
            for index in range(3):
                edge = tuple(sorted((tuple(triangle[index]), tuple(triangle[(index + 1) % 3]))))
                edges[edge] += 1
        self.assertEqual(set(edges.values()), {2})
        signed_volume = np.einsum('ij,ij->i', triangles[:, 0],
                                 np.cross(triangles[:, 1], triangles[:, 2])).sum() / 6
        self.assertGreater(signed_volume, 0)

    def test_other_rocket_visuals_keep_source_geometry_and_poses(self):
        output = appearance.build_model().find('model/link')
        source = ET.parse(appearance.ROCKET / 'model.sdf').getroot().find('model')
        count = 0
        for link in source.findall('link'):
            for visual in link.findall('visual'):
                result = output.find(f"visual[@name='rocket_{link.get('name')}_{visual.get('name')}']")
                self.assertIsNotNone(result)
                expected_pose = appearance.transform(link.findtext('pose')) @ appearance.transform(visual.findtext('pose'))
                np.testing.assert_allclose(appearance.transform(result.findtext('pose')), expected_pose, atol=1e-10)
                if visual.get('name') != 'fuselage_visual':
                    expected_geometry, actual_geometry = visual.find('geometry'), result.find('geometry')
                    self.assertEqual(ET.tostring(expected_geometry).strip(), ET.tostring(actual_geometry).strip())
                count += 1
        self.assertEqual(len([v for v in output.findall('visual') if v.get('name').startswith('rocket_')]), count)
        pod = ET.parse(appearance.POD / 'model.sdf').getroot().find('model')
        self.assertEqual(len(output.findall('visual')), count + len(pod.findall('link/visual')))
        self.assertEqual(len(output.findall('collision')), 0)

    def test_preview_location_and_navigation(self):
        for view in ('overall', 'close'):
            world = appearance.build_world(view).find('world')
            self.assertEqual(float(world.findtext('gui/plugin/camera_clip/near')), .005)
            marker = world.find("model[@name='ROCKET_POD_LOCATION_MARKER']")
            self.assertEqual(len(marker.findall('.//collision')), 0)
            marker_z = appearance.transform(marker.findtext('link/visual/pose'))[2, 3]
            model_z = appearance.transform(world.findtext('include/pose'))[2, 3]
            geometry = appearance.source_geometry()
            self.assertAlmostEqual(model_z + geometry['min_z_m'], .015)
            self.assertAlmostEqual(model_z + geometry['max_z_m'], geometry['length_m'] + .015)
            self.assertAlmostEqual(marker_z, model_z + appearance.dimensions()[1] + .036)
        self.assertEqual(appearance.configuration()[0]['view']['mouse_sensitivity'], .1)

    def test_actual_camera_visual_rotation(self):
        source = ET.parse(appearance.POD / 'model.sdf').find("model/link[@name='camera_link']/visual")
        name = source.get('name')
        source_rotation = appearance.transform(source.findtext('pose'))[:3, :3]
        for yaw, pitch in ((0, 0), (0, -80), (45, -45), (0, 30), (-60, -80), (60, -80)):
            with self.subTest(yaw=yaw, pitch=pitch):
                model = appearance.build_model(yaw, pitch)
                camera = model.find(f"model/link/visual[@name='pod_camera_link_{name}']")
                actual = appearance.transform(camera.findtext('pose'))[:3, :3] @ source_rotation.T @ np.array([0., 0., 1.])
                y, p = math.radians(yaw), math.radians(pitch)
                np.testing.assert_allclose(actual, (-math.sin(p),
                    -math.sin(y)*math.cos(p), math.cos(y)*math.cos(p)), atol=1e-10)

    def test_pitch_limits_and_zero_determinism(self):
        for pitch in (-80.01, 30.01, float('inf'), float('nan')):
            with self.subTest(pitch=pitch), self.assertRaises(ValueError):
                appearance.build_model(pitch_deg=pitch)
        expected = ET.parse(appearance.ROOT / 'model.sdf')
        def canonical(element):
            return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(),
                    tuple(canonical(child) for child in element))
        self.assertEqual(canonical(appearance.build_model()), canonical(expected.getroot()))

    def test_outer_axis_uses_sdf_pivot_and_keeps_camera_center_fixed(self):
        chain = appearance.kinematics.PodKinematics()
        center = np.array([.015, 0., .056])
        for yaw in (-60., 0., 60.):
            for pitch in (-80., 0., 30.):
                poses = chain.link_poses(yaw, pitch)
                np.testing.assert_allclose(poses['camera_link'][:3, 3], center, atol=1e-12)
                np.testing.assert_allclose(poses['mount_link'], np.eye(4), atol=1e-12)
                y, p = map(math.radians, (yaw, pitch))
                expected = np.array([-math.sin(p), -math.sin(y)*math.cos(p), math.cos(y)*math.cos(p)])
                sensor = chain.camera_pose(yaw, pitch)
                np.testing.assert_allclose(sensor[:3, 0], expected, atol=1e-10)
                np.testing.assert_allclose(sensor[:3, 3], center + .0237*expected, atol=1e-12)

    def test_positive_yaw_turns_toward_image_right_and_inverse_preserves_sign(self):
        chain = appearance.kinematics.PodKinematics()
        center = np.array([.015, 0., .056])
        for yaw in (-40., 0., 40.):
            for pitch in (-70., 0., 20.):
                with self.subTest(yaw=yaw, pitch=pitch):
                    previous = chain.camera_pose(yaw, pitch)
                    following = chain.camera_pose(yaw + 1., pitch)
                    image_right = -previous[:3, 1]
                    self.assertGreater(np.dot(following[:3, 0] - previous[:3, 0], image_right), 0.)
                    target = center + 100. * previous[:3, 0]
                    solution = chain.inverse(target)
                    self.assertTrue(solution['reachable'])
                    self.assertAlmostEqual(solution['yaw'], yaw, places=9)
                    self.assertAlmostEqual(solution['pitch'], pitch, places=9)

    def test_joint_pose_is_read_in_child_frame(self):
        source = ET.parse(appearance.POD / 'model.sdf')
        joint = source.find("model/joint[@name='yaw_joint']")
        # 枢轴沿 Z 平移一米后，外轴转60°应绕新枢轴改变球心，而非套用固定Rx。
        joint.find('pose').text = '0 0 1 0 0 0'
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.sdf'
            source.write(path)
            chain = appearance.kinematics.PodKinematics(path)
            actual = chain.link_poses(60., 0.)['camera_link'][:3, 3]
        np.testing.assert_allclose(actual, [.015, math.sqrt(3)/2, .556], atol=1e-12)

    def test_regeneration_matches_checked_assets(self):
        with tempfile.TemporaryDirectory(prefix='rocket-pod-appearance-check-') as temporary:
            subprocess.run([sys.executable, str(ROOT / 'tools/generate_model.py'), '--output-dir', temporary],
                           check=True, stdout=subprocess.PIPE, text=True)
            for relative in ('model.sdf', 'model.config', 'meshes/fuselage_cut.stl',
                             'tests/world.sdf', 'tests/closeup.sdf'):
                self.assertEqual((Path(temporary) / relative).read_bytes(), (ROOT / relative).read_bytes(), relative)


if __name__ == '__main__':
    unittest.main(verbosity=2)
