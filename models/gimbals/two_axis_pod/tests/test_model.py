#!/usr/bin/env python3
"""核验导入CAD资产、实际SDF运动链、光学外参、视场与质量惯性。"""
import hashlib
import copy
import importlib.util
import json
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)


def rpy(pose):
    return (rotation([0, 0, 1], pose[5]) @ rotation([0, 1, 0], pose[4]) @
            rotation([1, 0, 0], pose[3]))


class ModelTest(unittest.TestCase):
    def setUp(self):
        self.model = ET.parse(ROOT/'model.sdf').getroot().find('model')
        self.parameters = json.loads((ROOT/'parameters.json').read_text())
        self.properties = json.loads((ROOT/'cad/properties.json').read_text())

    def axis(self, name):
        joint = self.model.find(f"joint[@name='{name}_joint']")
        return np.fromstring(joint.findtext('axis/xyz'), sep=' ')

    def chain(self, yaw, pitch):
        return rotation(self.axis('yaw'), math.radians(yaw)) @ rotation(self.axis('pitch'), math.radians(pitch))

    def optical(self):
        pose = np.fromstring(self.model.findtext("link[@name='camera_link']/sensor/pose"), sep=' ')
        return rpy(pose)

    def mesh(self, part):
        mesh = ET.parse(ROOT/self.properties['parts'][part]['mesh'])
        vertices = np.fromstring(mesh.findtext(f".//c:source[@id='{part}-positions']/c:float_array", namespaces=NS), sep=' ').reshape(-1, 3)
        indices = np.fromstring(mesh.findtext('.//c:triangles/c:p', namespaces=NS), sep=' ', dtype=int).reshape(-1, 6)[:, ::2]
        return vertices, indices

    def posed_triangles(self, part, yaw, pitch):
        data = self.properties['parts'][part]
        points, faces = self.mesh(part)
        origin = np.array(self.properties['groups'][data['link']]['origin_m'])
        points = points + origin
        if data['link'] == 'camera_link':
            pivot = np.array(self.parameters['camera_center_m'])
            points = (points-pivot) @ rotation(self.axis('pitch'), math.radians(pitch)).T + pivot
        if data['link'] != 'mount_link':
            pivot = np.array(self.parameters['kinematics']['yaw_origin_m'])
            points = (points-pivot) @ rotation(self.axis('yaw'), math.radians(yaw)).T + pivot
        return points[faces]

    def test_chain_and_limits(self):
        self.assertEqual(len(self.model.findall('joint')), 2)
        for name, parent, child, axis, limits in [
            ('yaw', 'mount_link', 'yaw_arm_link', [-1, 0, 0], (-60, 60)),
            ('pitch', 'yaw_arm_link', 'camera_link', [0, -1, 0], (-80, 30))]:
            joint = self.model.find(f"joint[@name='{name}_joint']")
            self.assertEqual(joint.findtext('parent'), parent)
            self.assertEqual(joint.findtext('child'), child)
            np.testing.assert_array_equal(self.axis(name), axis)
            for key, limit in zip(('lower', 'upper'), limits):
                self.assertAlmostEqual(math.degrees(float(joint.findtext('axis/limit/'+key))), limit)
        center = np.array(self.parameters['camera_center_m'])
        origin = np.array(self.parameters['kinematics']['yaw_origin_m'])
        for angle in (-60, 0, 60):
            np.testing.assert_allclose(rotation(self.axis('yaw'), math.radians(angle))@(center-origin)+origin, center, atol=1e-12)

    def test_actual_optical_pose_and_extremes(self):
        optical = self.optical()
        np.testing.assert_allclose(optical[:, 0], [0, 0, 1], atol=1e-11)
        np.testing.assert_allclose(optical[:, 1], [0, 1, 0], atol=1e-11)
        np.testing.assert_allclose(optical[:, 2], [-1, 0, 0], atol=1e-11)
        for a, p, expected in [
            (0, 0, (0, 0, 1)), (0, -80, (.984807753, 0, .173648178)),
            (-60, 0, (0, -.866025404, .5)), (60, 0, (0, .866025404, .5)),
            (0, 30, (-.5, 0, .866025404)), (60, -80, (.984807753, .150383733, .086824089))]:
            np.testing.assert_allclose((self.chain(a, p)@optical)[:, 0], expected, atol=1e-8)

    def test_cad_source_articulation_and_asset_hashes(self):
        sidecar = json.loads((ROOT/'cad/motion_head.step.json').read_text())
        self.assertEqual(sidecar['documentHash'], hashlib.sha256((ROOT/'cad/motion_head.step').read_bytes()).hexdigest())
        conversion = np.diag([-1, -1, 1])
        for mate, axis in zip(sidecar['kinematics']['mates'], ('yaw', 'pitch')):
            np.testing.assert_array_equal(conversion@mate['axis']['dir'], self.axis(axis))
        for relative, expected in self.properties['sha256'].items():
            self.assertEqual(hashlib.sha256((ROOT/relative).read_bytes()).hexdigest(), expected, relative)
        self.assertEqual(len(self.properties['parts']), 9)
        self.assertEqual([len(link.findall('visual')) for link in self.model.findall('link')], [3, 3, 3])
        mount = self.model.find("link[@name='mount_link']")
        self.assertIsNotNone(mount.find("visual[@name='fixed_cowling_visual']"))
        self.assertIsNotNone(mount.find("visual[@name='azimuth_bearing_housing_visual']"))

    def test_mesh_units_envelope_and_closed_solid_volumes(self):
        bounds = []
        for part, data in self.properties['parts'].items():
            points, indices = self.mesh(part)
            triangles = points[indices]
            volume = np.einsum('ij,ij->i', triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])).sum()/6
            self.assertGreater(volume, 0, part)
            self.assertAlmostEqual(volume*1e9/data['volume_mm3'], 1, delta=.03, msg=part)
            world = points + np.array(self.properties['groups'][data['link']]['origin_m'])
            expected = np.array(data['bounds_mount_m'])
            np.testing.assert_allclose([world.min(axis=0), world.max(axis=0)], expected, atol=.00007, err_msg=part)
            bounds.append(world)
        world = np.vstack(bounds)
        np.testing.assert_allclose(world.max(axis=0)-world.min(axis=0), [.084, .084, .091], atol=.00007)
        self.assertAlmostEqual(world[:, 2].min(), 0, places=8)

    def test_mass_and_inertia_feasibility(self):
        mass = 0
        for link in self.model.findall('link'):
            inertial = link.find('inertial')
            mass += float(inertial.findtext('mass'))
            data = {k: float(inertial.findtext('inertia/'+k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')}
            tensor = np.array([[data['ixx'], data['ixy'], data['ixz']],
                               [data['ixy'], data['iyy'], data['iyz']],
                               [data['ixz'], data['iyz'], data['izz']]])
            eigen = np.linalg.eigvalsh(tensor)
            self.assertGreater(eigen.min(), 0)
            self.assertLessEqual(eigen[-1], eigen[0]+eigen[1]+1e-12)
            center = np.fromstring(inertial.findtext('pose'), sep=' ')[:3]
            self.assertTrue(np.isfinite(center).all())
        self.assertAlmostEqual(mass, .24, places=10)

    def test_lens_sensor_and_real_self_occlusion(self):
        sensor = np.fromstring(self.model.findtext("link[@name='camera_link']/sensor/pose"), sep=' ')
        center = np.array(self.parameters['camera_center_m'])
        window = self.properties['parts']['optical_window']['bounds_mount_m']
        bezel = self.properties['parts']['lens_bezel']['bounds_mount_m']
        self.assertAlmostEqual((window[0][0]+window[1][0])/2, center[0])
        self.assertGreater(center[2]+sensor[2], bezel[1][2])
        near = float(self.model.findtext("link[@name='camera_link']/sensor/camera/clip/near"))
        self.assertEqual(near, .005)
        for yaw in (-60, 60):
            optical = self.chain(yaw, 30)@self.optical()
            origin = center + self.chain(yaw, 30)@sensor[:3]
            # 60°水平视场、16:9：图像上缘中心的真实几何射线。
            direction = optical@np.array([1, 0, math.tan(math.pi/6)*9/16])
            direction /= np.linalg.norm(direction)
            triangles = self.posed_triangles('fixed_cowling', yaw, 30)
            edge1, edge2 = triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]
            h = np.cross(direction, edge2); determinant = np.einsum('ij,ij->i', edge1, h)
            usable = abs(determinant)>1e-12
            inv = np.zeros_like(determinant); inv[usable] = 1/determinant[usable]
            delta = origin-triangles[:, 0]
            u = inv*np.einsum('ij,ij->i', delta, h)
            q = np.cross(delta, edge1)
            v = inv*(q@direction)
            t = inv*np.einsum('ij,ij->i', edge2, q)
            hits = t[usable & (u>=0) & (v>=0) & (u+v<=1) & (t>0)]
            self.assertGreater(len(hits), 0, '极限姿态的罩壳遮挡应保留')
            self.assertGreater(hits.min(), near)
            self.assertLess(hits.min(), .04)

    def test_generation_and_demo(self):
        paths = [ROOT/'model.sdf', ROOT/'tests/world.sdf']
        before = [p.read_bytes() for p in paths]
        spec = importlib.util.spec_from_file_location('generate', ROOT/'tools/generate_model.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        module.generate()
        self.assertEqual(before, [p.read_bytes() for p in paths])
        demo = ET.parse(paths[1]).getroot().find("world/model[@name='two_axis_pod']")
        self.assertEqual(demo.findtext("joint[@name='world_mount']/parent"), 'world')
        self.assertEqual(demo.findtext("joint[@name='world_mount']/child"), 'mount_link')

    def test_reject_unsupported_optical_basis_and_mechanical_axis(self):
        spec = importlib.util.spec_from_file_location('generate', ROOT/'tools/generate_model.py')
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        config = copy.deepcopy(self.parameters)
        config['camera']['sensor_rpy_rad'][0] = math.pi
        with self.assertRaisesRegex(ValueError, '光学基底'):
            module.validate_configuration(config, self.properties)
        config = copy.deepcopy(self.parameters)
        config['kinematics']['yaw_axis'] = [0, 0, 1]
        with self.assertRaisesRegex(ValueError, '机械轴'):
            module.validate_configuration(config, self.properties)


if __name__ == '__main__':
    unittest.main()
