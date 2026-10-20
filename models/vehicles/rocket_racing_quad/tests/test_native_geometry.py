"""检查当前原生几何的平底尾端、四片主翼、小尾翼与桨盘净空。"""
from collections import Counter
import math
from pathlib import Path
import unittest

import numpy as np

MESH = Path(__file__).resolve().parents[1] / 'meshes'
MOTOR_XY = .2086
MOTOR_RADIUS = math.hypot(MOTOR_XY, MOTOR_XY)


def triangles(name):
    return np.asarray([list(map(float, line.split()[1:]))
                       for line in (MESH / name).read_text().splitlines()
                       if line.strip().startswith('vertex ')]).reshape(-1, 3, 3)


def components(mesh):
    """按共享顶点找独立闭合部件，不依赖三角面编号。"""
    rounded = np.round(mesh, 8)
    vertices = {}
    for index, triangle in enumerate(rounded):
        for point in triangle:
            vertices.setdefault(tuple(point), []).append(index)
    pending = set(range(len(mesh)))
    result = []
    while pending:
        queue, piece = [pending.pop()], []
        while queue:
            index = queue.pop()
            piece.append(index)
            adjacent = {item for point in rounded[index] for item in vertices[tuple(point)]}
            fresh = adjacent & pending
            pending -= fresh
            queue.extend(fresh)
        result.append(mesh[piece])
    return result


def radial_profile(mesh):
    points = mesh.reshape(-1, 3)
    heights = np.unique(points[:, 2])
    radii = [np.linalg.norm(points[points[:, 2] == height, :2], axis=1).max() for height in heights]
    return heights, np.asarray(radii)


def plane_segments(mesh, height):
    for triangle in mesh:
        hits = []
        for start, end in zip(triangle, np.roll(triangle, -1, axis=0)):
            if abs(start[2] - height) < 1e-10:
                hits.append(start[:2])
            if (start[2] - height) * (end[2] - height) < 0:
                hits.append((start + (end - start) * ((height - start[2]) / (end[2] - start[2])))[:2])
        if len(hits) >= 2:
            yield np.asarray(hits[0]), np.asarray(hits[1])


class NativeGeometryTests(unittest.TestCase):
    def test_closed_non_degenerate_positive_volume(self):
        for name, expected_parts in (('fuselage.stl', 1), ('tail.stl', 4), ('tail_fins.stl', 4)):
            with self.subTest(mesh=name):
                mesh = triangles(name)
                self.assertTrue(np.isfinite(mesh).all())
                normals = np.cross(mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0])
                self.assertGreater(np.linalg.norm(normals, axis=1).min(), 1e-12)
                edges = Counter()
                for triangle in np.round(mesh, 8):
                    for index in range(3):
                        edges[tuple(sorted((tuple(triangle[index]), tuple(triangle[(index + 1) % 3]))))] += 1
                self.assertEqual(set(edges.values()), {2})
                parts = components(mesh)
                self.assertEqual(len(parts), expected_parts)
                for piece in parts:
                    volume = np.einsum('ij,ij->i', piece[:, 0], np.cross(piece[:, 1], piece[:, 2])).sum() / 6
                    self.assertGreater(volume, 0)

    def test_body_extent_flat_tail_and_smooth_transition(self):
        mesh = triangles('fuselage.stl')
        z, radius = radial_profile(mesh)
        np.testing.assert_allclose([z.min(), z.max(), z.max() - z.min()], [-.21, .38, .59], atol=1e-9)
        self.assertAlmostEqual(radius.max(), .045, places=7)
        self.assertAlmostEqual(radius[0], .034, places=7)
        tail = mesh[np.all(np.isclose(mesh[:, :, 2], -.21), axis=1)]
        self.assertGreater(len(tail), 0)
        normals = np.cross(tail[:, 1] - tail[:, 0], tail[:, 2] - tail[:, 0])
        self.assertTrue(np.all(normals[:, 2] < 0))
        expected_area = .5 * 64 * .034**2 * math.sin(2 * math.pi / 64)
        self.assertAlmostEqual(np.linalg.norm(normals, axis=1).sum() / 2, expected_area, places=9)
        selected = z <= -.156 + 1e-10
        t = (z[selected] + .21) / .054
        expected = .034 + .011 * (3 * t**2 - 2 * t**3)
        np.testing.assert_allclose(radius[selected], expected, atol=2e-8)

    def test_main_wings_connect_body_and_nacelles(self):
        body_z, body_r = radial_profile(triangles('fuselage.stl'))
        nacelle_z, nacelle_r = radial_profile(triangles('nacelle.stl'))
        headings = []
        for piece in components(triangles('tail.stl')):
            points = np.unique(piece.reshape(-1, 3), axis=0)
            heading = math.atan2(points[:, 1].mean(), points[:, 0].mean())
            headings.append(round(math.degrees(heading)) % 360)
            radial = points[:, 0] * math.cos(heading) + points[:, 1] * math.sin(heading)
            tangent = -points[:, 0] * math.sin(heading) + points[:, 1] * math.cos(heading)
            np.testing.assert_allclose(abs(tangent), .002, atol=2e-8)
            root = points[radial < .011]
            self.assertTrue(np.all(np.linalg.norm(root[:, :2], axis=1) < np.interp(root[:, 2], body_z, body_r)))
            tip = points[np.isclose(radial, MOTOR_RADIUS, atol=1e-7)]
            self.assertEqual(len(tip), 4)
            center_xy = MOTOR_RADIUS * np.array([math.cos(heading), math.sin(heading)])
            local_radius = np.linalg.norm(tip[:, :2] - center_xy, axis=1)
            self.assertTrue(np.all(local_radius < np.interp(tip[:, 2], nacelle_z, nacelle_r)))
            np.testing.assert_allclose([tip[:, 2].min(), tip[:, 2].max()], [-.02, .02], atol=1e-9)
        self.assertEqual(sorted(headings), [45, 135, 225, 315])

    def test_small_tail_fins_are_offset_and_join_body(self):
        body_z, body_r = radial_profile(triangles('fuselage.stl'))
        headings = []
        for piece in components(triangles('tail_fins.stl')):
            points = np.unique(piece.reshape(-1, 3), axis=0)
            heading = math.atan2(points[:, 1].mean(), points[:, 0].mean())
            headings.append(round(math.degrees(heading)) % 360)
            radial = points[:, 0] * math.cos(heading) + points[:, 1] * math.sin(heading)
            tangent = -points[:, 0] * math.sin(heading) + points[:, 1] * math.cos(heading)
            np.testing.assert_allclose(abs(tangent), .002, atol=1e-9)
            self.assertAlmostEqual(radial.max(), .100)
            np.testing.assert_allclose([points[:, 2].min(), points[:, 2].max()], [-.21, -.105])
            for height in (-.21, -.105):
                root = points[np.isclose(points[:, 2], height)]
                self.assertTrue(np.all(np.linalg.norm(root[:, :2], axis=1) < np.interp(root[:, 2], body_z, body_r)))
        self.assertEqual(sorted(headings), [0, 90, 180, 270])

    def test_wings_stay_clear_of_rotor_disc_and_hub(self):
        centers = [np.array([sx * MOTOR_XY, sy * MOTOR_XY]) for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
        # 同时检查桨叶厚度带与整个桨毂高度带；每个截线均避开127mm桨盘包络。
        levels = [-.065 + offset for offset in (-.014, -.007, -.0035, 0., .0035, .007, .014)]
        for name in ('tail.stl', 'tail_fins.stl'):
            mesh = triangles(name)
            for height in levels:
                for start, end in plane_segments(mesh, height):
                    direction = end - start
                    denominator = float(direction @ direction)
                    for center in centers:
                        t = np.clip((center - start) @ direction / denominator, 0, 1) if denominator > 1e-18 else 0
                        self.assertGreaterEqual(np.linalg.norm(start + t * direction - center) - .127, .005)


if __name__ == '__main__':
    unittest.main(verbosity=2)
