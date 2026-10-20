#!/usr/bin/env python3
"""验证原生几何继承、顶部替换接口、封闭网格与入库静态资产一致性。"""
from collections import Counter
import importlib.util
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
        self.assertAlmostEqual(source['min_z_m'], -.225)
        self.assertAlmostEqual(source['max_z_m'], .365)
        self.assertAlmostEqual(source['length_m'], .59)
        np.testing.assert_allclose(appearance.dimensions(), [.59, .274, .262], atol=1e-12)

    def test_cut_preserves_native_lower_triangles_and_is_closed(self):
        source, pose, scale = appearance.source_mesh('fuselage_visual')
        triangles = appearance.cut_fuselage() * scale
        triangles = triangles @ pose[:3, :3].T + pose[:3, 3]
        shoulder, mount = appearance.dimensions()[2], appearance.dimensions()[1]
        original = source[np.all(source[:, :, 2] < shoulder, axis=1)]
        retained = triangles[np.all(triangles[:, :, 2] < shoulder, axis=1)]
        np.testing.assert_allclose(retained, original, atol=1e-12)
        self.assertAlmostEqual(triangles[:, :, 2].min(), -.225)
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
        self.assertEqual(len(output.findall('collision')), 0)

    def test_preview_location_and_navigation(self):
        for view in ('overall', 'close'):
            world = appearance.build_world(view).find('world')
            self.assertEqual(float(world.findtext('gui/plugin/camera_clip/near')), .005)
            marker = world.find("model[@name='ROCKET_POD_LOCATION_MARKER']")
            self.assertEqual(len(marker.findall('.//collision')), 0)
            marker_z = appearance.transform(marker.findtext('link/visual/pose'))[2, 3]
            self.assertAlmostEqual(marker_z, .55)
            model_z = appearance.transform(world.findtext('include/pose'))[2, 3]
            self.assertAlmostEqual(model_z + appearance.dimensions()[1], .514)
        self.assertEqual(appearance.configuration()[0]['view']['mouse_sensitivity'], .1)

    def test_regeneration_matches_checked_assets(self):
        with tempfile.TemporaryDirectory(prefix='rocket-pod-appearance-check-') as temporary:
            subprocess.run([sys.executable, str(ROOT / 'tools/generate_model.py'), '--output-dir', temporary],
                           check=True, stdout=subprocess.PIPE, text=True)
            for relative in ('model.sdf', 'model.config', 'meshes/fuselage_cut.stl',
                             'tests/world.sdf', 'tests/closeup.sdf'):
                self.assertEqual((Path(temporary) / relative).read_bytes(), (ROOT / relative).read_bytes(), relative)


if __name__ == '__main__':
    unittest.main(verbosity=2)
