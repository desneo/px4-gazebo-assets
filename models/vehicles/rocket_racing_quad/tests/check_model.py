#!/usr/bin/env python3
"""离线检查生成工具及原生资产的运行复制；不启动仿真。

运行：python3 tests/check_model.py。输出使用临时目录，失败时返回非零退出码。
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import generate_model as generator
import generate_meshes


def vertices(path):
    points = [list(map(float, line.split()[1:])) for line in path.read_text().splitlines()
              if line.strip().startswith('vertex ')]
    return np.asarray(points).reshape((-1, 3, 3))


def px4_parameters(text):
    return {parts[2]: float(parts[3]) for line in text.splitlines()
            if len(parts := line.split()) == 4 and parts[:2] == ['param', 'set-default']}


class ModelChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = json.loads((ROOT / 'parameters.json').read_text())
        cls.workspace = tempfile.TemporaryDirectory(prefix='rocket-static-')
        cls.addClassCleanup(cls.workspace.cleanup)
        cls.output = Path(cls.workspace.name) / 'generated'
        cls.generate(cls.config, cls.output)
        cls.model = ET.parse(cls.output / 'model.sdf').getroot().find('model')

    @classmethod
    def generate(cls, config, output):
        config_path = Path(cls.workspace.name) / (output.name + '.json')
        config_path.write_text(json.dumps(config))
        subprocess.run([sys.executable, str(ROOT / 'tools/generate_model.py'), '--config',
                        str(config_path), '--output-dir', str(output)], check=True,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def test_explicit_generator_is_reproducible(self):
        repeated = Path(self.workspace.name) / 'repeated'
        self.generate(self.config, repeated)
        assets = ['model.sdf', 'tests/world.sdf', 'model.config',
                  f"px4/{self.config['px4']['airframe_id']}_gz_rocket_racing_quad"]
        assets += ['meshes/' + name for name in generate_meshes.build_meshes(self.config)]
        for asset in assets:
            with self.subTest(asset=asset):
                expected = (self.output / asset).read_bytes()
                self.assertEqual(expected, (repeated / asset).read_bytes(), '生成结果不可复现')

    def test_runtime_preserves_native_assets(self):
        import prepare_runtime
        output = Path(self.workspace.name) / 'native'
        before = prepare_runtime.source_fingerprints(ROOT)
        snapshot = prepare_runtime.prepare(output)
        for path in prepare_runtime.asset_paths(ROOT):
            self.assertEqual(path.read_bytes(), (output / path.relative_to(ROOT)).read_bytes())
        self.assertEqual(snapshot['runtime']['source_sha256'], before)
        self.assertEqual(prepare_runtime.source_fingerprints(ROOT), before)

    def test_mesh_quality_and_clearance(self):
        g = self.config['geometry']
        meshes = {}
        for path in (self.output / 'meshes').glob('*.stl'):
            mesh = vertices(path)
            meshes[path.stem] = mesh.reshape((-1, 3))
            self.assertTrue(np.isfinite(mesh).all(), path.name)
            areas = np.linalg.norm(np.cross(mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0]), axis=1)
            self.assertGreater(areas.min(), 1e-12, path.name + ' 存在退化三角形')
            volume = np.einsum('ij,ij->i', mesh[:, 0], np.cross(mesh[:, 1], mesh[:, 2])).sum() / 6
            self.assertGreater(volume, 0, path.name + ' 有向体积非正')
        body = meshes['fuselage']
        np.testing.assert_allclose([body[:, 2].min(), body[:, 2].max()],
                                   [-g['body_length_m'] / 2, g['body_length_m'] / 2], atol=1e-7)
        rotor = meshes['propeller_10in']
        radius = np.linalg.norm(rotor[:, :2], axis=1).max()
        self.assertAlmostEqual(radius, g['propeller_diameter_m'] / 2, places=6)
        self.assertGreaterEqual(g['diagonal_wheelbase_m'] / math.sqrt(2) - 2 * radius, .005)
        self.assertGreaterEqual(g['diagonal_wheelbase_m'] / 2 - radius - g['body_radius_m'], .005)
        # 对翼面与桨叶高度带求截线，检查截线到每个桨轴的最近距离。
        tail = vertices(self.output / 'meshes/tail.stl')
        offset = g['diagonal_wheelbase_m'] / (2 * math.sqrt(2))
        centers = [np.array([sx*offset, sy*offset]) for sx, sy in ((1,-1),(-1,1),(1,1),(-1,-1))]
        for height in (g['rotor_z_m'] - .0035, g['rotor_z_m'], g['rotor_z_m'] + .0035):
            for triangle in tail:
                hits = []
                for a, b in zip(triangle, np.roll(triangle, -1, axis=0)):
                    if (a[2] - height) * (b[2] - height) <= 0 and abs(b[2] - a[2]) > 1e-12:
                        hits.append((a + (b-a) * ((height-a[2])/(b[2]-a[2])))[:2])
                if len(hits) < 2:
                    continue
                a, b = hits[:2]
                direction = b-a
                for center in centers:
                    denom = float(np.dot(direction, direction))
                    fraction = np.clip(np.dot(center-a, direction)/denom, 0, 1) if denom > 1e-16 else 0
                    distance = np.linalg.norm(a+fraction*direction-center)
                    self.assertGreaterEqual(distance-radius, .005, '翼面进入桨叶包络')

    def test_links_joints_mass_and_ground(self):
        g = self.config['geometry']
        links = {link.attrib['name']: link for link in self.model.findall('link')}
        joints = {joint.findtext('child'): joint
                  for joint in self.model.findall('joint')}
        expected_mass = self.config['mass']['body_kg'] + 4 * (
            self.config['mass']['nacelle_kg'] + self.config['mass']['propeller_kg'])
        self.assertAlmostEqual(sum(float(link.findtext('inertial/mass')) for link in links.values()), expected_mass)
        a = g['diagonal_wheelbase_m'] / (2 * math.sqrt(2))
        airframe = next((self.output / 'px4').glob('*')).read_text()
        params = px4_parameters(airframe)
        for i, (sx, sy) in enumerate(((1, -1), (-1, 1), (1, 1), (-1, -1))):
            for prefix, z, kind in [('rotor', g['rotor_z_m'], 'revolute'),
                                    ('nacelle', g['nacelle_z_m'], 'fixed')]:
                name = f'{prefix}_{i}'
                np.testing.assert_allclose(list(map(float, links[name].findtext('pose').split())),
                                           [sx * a, sy * a, z, 0, 0, 0], atol=1e-9)
                self.assertEqual(joints[name].attrib['type'], kind)
                self.assertEqual(joints[name].findtext('parent'), 'base_link')
            self.assertEqual(joints[f'rotor_{i}'].findtext('axis/xyz'), '0 0 1')
            self.assertAlmostEqual(params[f'CA_ROTOR{i}_PX'], sx * a, places=8)
            self.assertAlmostEqual(params[f'CA_ROTOR{i}_PY'], -sy * a, places=8)
            self.assertAlmostEqual(params[f'CA_ROTOR{i}_PZ'], -g['rotor_z_m'], places=8)
            shaft = links[f'nacelle_{i}'].find("visual[@name='motor_shaft_%d_visual']" % i)
            self.assertIsNotNone(shaft, '动力舱需通过电机轴连接桨毂')
            shaft_z = float(shaft.findtext('pose').split()[2]) + g['nacelle_z_m']
            shaft_length = float(shaft.findtext('geometry/cylinder/length'))
            self.assertAlmostEqual(shaft_z - shaft_length / 2, g['rotor_z_m'] + .014)
            self.assertAlmostEqual(shaft_z + shaft_length / 2, g['nacelle_z_m'] - g['nacelle_length_m'] / 2)
        world = ET.parse(self.output / 'tests/world.sdf').getroot().find('world')
        spawn_z = float(world.findtext('include/pose').split()[2])
        ground = world.find("model[@name='ground_plane']/link")
        for kind in ('collision', 'visual'):
            shape = 'box' if kind == 'collision' else 'plane'
            size = list(map(float, ground.findtext(f'{kind}/geometry/{shape}/size').split()))
            self.assertEqual(size[:2], [self.config['simulation']['ground_size_m']] * 2)
            if kind == 'collision':
                self.assertAlmostEqual(float(ground.findtext('collision/pose').split()[2]) + size[2]/2, 0)
        self.assertAlmostEqual(spawn_z - g['body_length_m'] / 2,
                               self.config['simulation']['spawn_clearance_m'])
        # 所有碰撞形状均为圆柱；变换后垂直包络不得穿过地面。
        for link in links.values():
            link_z = float((link.findtext('pose') or '0 0 0 0 0 0').split()[2])
            for collision in link.findall('collision'):
                cylinder = collision.find('geometry/cylinder')
                if cylinder is None:
                    continue
                position = list(map(float, (collision.findtext('pose') or '0 0 0 0 0 0').split()))
                radius, length = float(cylinder.findtext('radius')), float(cylinder.findtext('length'))
                axis_z = math.cos(position[3]) * math.cos(position[4])
                half_height = abs(axis_z) * length / 2 + math.sqrt(max(0, 1 - axis_z**2)) * radius
                self.assertGreaterEqual(spawn_z + link_z + position[2] - half_height, -1e-9)

    def test_parameter_changes_reach_outputs(self):
        config = copy.deepcopy(self.config)
        config['geometry']['body_length_m'] += .05
        config['geometry']['diagonal_wheelbase_m'] += .02
        config['mass']['body_kg'] += .2
        config['motor']['thrust_constant'] *= 1.1
        config['simulation']['step_size_s'] /= 2
        config['simulation']['spawn_clearance_m'] += .01
        config['simulation']['ground_size_m'] *= .5
        config['px4']['parameters']['MPC_XY_VEL_MAX'] += 1
        output = Path(self.workspace.name) / 'changed'
        self.generate(config, output)
        model = ET.parse(output / 'model.sdf').getroot().find('model')
        self.assertAlmostEqual(float(model.findtext("link[@name='base_link']/inertial/mass")), config['mass']['body_kg'])
        self.assertAlmostEqual(float(model.findtext("plugin[@name='gz::sim::systems::MulticopterMotorModel']/motorConstant")), config['motor']['thrust_constant'])
        self.assertNotEqual((output / 'meshes/fuselage.stl').read_bytes(), (self.output / 'meshes/fuselage.stl').read_bytes())
        world = ET.parse(output / 'tests/world.sdf').getroot().find('world')
        self.assertAlmostEqual(float(world.findtext('physics/max_step_size')), config['simulation']['step_size_s'])
        for kind in ('collision', 'visual'):
            shape = 'box' if kind == 'collision' else 'plane'
            size = world.findtext(f"model[@name='ground_plane']/link/{kind}/geometry/{shape}/size")
            self.assertEqual(list(map(float, size.split()))[:2], [config['simulation']['ground_size_m']] * 2)
        self.assertAlmostEqual(float(world.findtext('include/pose').split()[2]), config['geometry']['body_length_m'] / 2 + config['simulation']['spawn_clearance_m'])
        params = px4_parameters(next((output / 'px4').glob('*')).read_text())
        self.assertAlmostEqual(params['CA_ROTOR0_PX'], config['geometry']['diagonal_wheelbase_m'] / (2 * math.sqrt(2)), places=8)
        self.assertEqual(params['MPC_XY_VEL_MAX'], config['px4']['parameters']['MPC_XY_VEL_MAX'])

    def test_stationary_sensors_have_configured_noise(self):
        sensors = self.config['sensors']
        base = self.model.find("link[@name='base_link']")
        pressure_noise = base.find(
            "sensor[@name='air_pressure_sensor']/air_pressure/pressure/noise")
        self.assertIsNotNone(pressure_noise)
        self.assertEqual(pressure_noise.get('type'), 'gaussian')
        self.assertEqual(float(pressure_noise.findtext('mean')), 0)
        self.assertEqual(float(pressure_noise.findtext('stddev')),
                         sensors['air_pressure_noise_stddev_pa'])

        magnetometer = base.find("sensor[@name='magnetometer_sensor']/magnetometer")
        self.assertIsNotNone(magnetometer)
        for axis in ('x', 'y', 'z'):
            noise = magnetometer.find(f'{axis}/noise')
            self.assertIsNotNone(noise, f'磁力计{axis}轴缺少噪声配置')
            self.assertEqual(noise.get('type'), 'gaussian')
            self.assertEqual(float(noise.findtext('mean')), 0)
            self.assertEqual(float(noise.findtext('stddev')),
                             sensors['magnetometer_noise_stddev_t'])

    def test_armed_idle_and_hover_mapping(self):
        params = px4_parameters(next((self.output / 'px4').glob('*')).read_text())
        motor = self.config['motor']
        self.assertGreater(motor['idle_velocity_rad_s'], 0)
        # PX4 将推力命令开方，再将输出映射到解锁 MIN/MAX；检查物理悬停推力。
        self.assertEqual(params['THR_MDL_FAC'], 1)
        for channel in range(1, 5):
            idle, maximum = params[f'SIM_GZ_EC_MIN{channel}'], params[f'SIM_GZ_EC_MAX{channel}']
            self.assertGreater(int(idle * 60 / (2 * math.pi)), 0, 'ESC 转速量化后仍需非零')
            self.assertEqual(params[f'SIM_GZ_EC_DIS{channel}'], 0, '未解锁时必须停转')
            velocity = idle + (maximum-idle) * math.sqrt(params['MPC_THR_HOVER'])
            total_mass = self.config['mass']['body_kg'] + 4 * (
                self.config['mass']['nacelle_kg'] + self.config['mass']['propeller_kg'])
            self.assertAlmostEqual(4 * motor['thrust_constant'] * velocity**2,
                                   total_mass * 9.8, places=7)

    def test_bad_configuration_is_rejected(self):
        cases = [('simulation', key, bad) for key in self.config['simulation']
                 for bad in (float('nan'), float('inf'), True)]
        cases += [('simulation', 'imu_wait_timeout_s', 0), ('simulation', 'spawn_clearance_m', -.001),
                  ('simulation', 'step_size_s', .02), ('simulation', 'ground_size_m', 0),
                  ('simulation', 'latitude_deg', 91),
                  ('geometry', 'propeller_diameter_m', .3), ('geometry', 'rotor_z_m', -.03),
                  ('geometry', 'nacelle_z_m', -.145), ('mass', 'body_kg', True),
                  ('motor', 'idle_velocity_rad_s', self.config['motor']['max_velocity_rad_s'])]
        for section, key, bad in cases:
            with self.subTest(section=section, key=key, value=bad):
                config = copy.deepcopy(self.config)
                config[section][key] = bad
                with self.assertRaises(ValueError):
                    generator.validate(config)
        hover = self.config['px4']['parameters']['MPC_THR_HOVER']
        for key, value in [('MPC_THR_MIN', hover), ('MPC_THR_HOVER', 0),
                           ('MPC_THR_MAX', hover), ('THR_MDL_FAC', -.1), ('THR_MDL_FAC', 1.1)]:
            with self.subTest(px4_parameter=key, value=value):
                config = copy.deepcopy(self.config)
                config['px4']['parameters'][key] = value
                with self.assertRaises(ValueError):
                    generator.validate(config)
        for factor in (0, 1):
            config = copy.deepcopy(self.config)
            config['px4']['parameters']['THR_MDL_FAC'] = factor
            generator.validate(config)
        # 地面接触与零离地间隙均为合法输入，不强制悬空。
        config = copy.deepcopy(self.config)
        config['simulation']['spawn_clearance_m'] = 0
        config['motor']['idle_velocity_rad_s'] = 0
        generator.validate(config)


if __name__ == '__main__':
    unittest.main(verbosity=2)
