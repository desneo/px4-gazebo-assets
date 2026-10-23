"""原生动态装配的来源与拒绝覆盖回归；不启动 Gazebo 或 PX4。"""
from pathlib import Path
import copy
import hashlib
import importlib.util
import json
import math
import shutil
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('rocket_dynamic_under_test', ROOT / 'tools/generate_dynamic_model.py')
dynamic = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dynamic)


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(),
            tuple(canonical(child) for child in element))


def independent_mass_center(model):
    """用显式 Rz Ry Rx 计算 link 到模型坐标变换，不调用生成器的变换/重心实现。"""
    mass, moment = 0., [0., 0., 0.]
    for link in model.findall('link'):
        link_mass = float(link.findtext('inertial/mass'))
        x, y, z, roll, pitch, yaw = map(float, link.findtext('pose', '0 0 0 0 0 0').split())
        ix, iy, iz = map(float, link.findtext('inertial/pose', '0 0 0 0 0 0').split()[:3])
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                                  math.cos(yaw), math.sin(yaw))
        # 局部惯性中心依次绕 X、Y、Z 旋转，再加 link 平移。
        iy, iz = cr * iy - sr * iz, sr * iy + cr * iz
        ix, iz = cp * ix + sp * iz, -sp * ix + cp * iz
        ix, iy = cy * ix - sy * iy, sy * ix + cy * iy
        for axis, position in enumerate((x + ix, y + iy, z + iz)):
            moment[axis] += link_mass * position
        mass += link_mass
    return mass, [value / mass for value in moment]


class DynamicNativeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='rocket-dynamic-test-')
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.source = self.work / 'rocket_racing_quad'
        for path in dynamic.native.asset_paths(dynamic.ROCKET) + [dynamic.ROCKET / 'parameters.json']:
            target = self.source / path.relative_to(dynamic.ROCKET)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(path, target)
        self.airframe = next((self.source / 'px4').glob('*_gz_rocket_racing_quad'))
        self.original_model = ET.parse(self.source / 'model.sdf').getroot().find('model')
        self.pod_model = ET.parse(dynamic.POD / 'model.sdf').getroot().find('model')
        self.original_px4 = {parts[2]: float(parts[3]) for line in self.airframe.read_text().splitlines()
                             if len(parts := line.split()) == 4 and parts[:2] == ['param', 'set-default']}
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(dynamic, 'ROCKET', self.source).start()
        mock.patch.object(dynamic.appearance, 'ROCKET', self.source).start()
        mock.patch.object(dynamic, 'source_fingerprints', side_effect=self.fingerprints).start()

    def fingerprints(self):
        return {str(path.relative_to(self.source)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted(self.source.rglob('*')) if path.is_file()}

    def expected_total_mass(self):
        source = ET.parse(self.source / 'model.sdf').getroot().find('model')
        return sum(float(link.findtext('inertial/mass'))
                   for model in (source, self.pod_model) for link in model.findall('link'))

    def assert_reference_inheritance(self, output, manifest):
        source = ET.parse(self.source / 'model.sdf').getroot().find('model')
        actual = ET.parse(output / 'model.sdf').getroot().find('model')
        source_mass, source_cg = independent_mass_center(source)
        pod_mass, pod_cg = independent_mass_center(self.pod_model)
        expected_cg = [(source_mass * source_cg[axis] + pod_mass *
                        (pod_cg[axis] + (manifest['mount_z_m'] if axis == 2 else 0.))) /
                       (source_mass + pod_mass) for axis in range(3)]
        actual_mass, actual_cg = independent_mass_center(actual)
        self.assertAlmostEqual(actual_mass, source_mass + pod_mass, places=12)
        for actual_value, expected_value in zip(actual_cg, expected_cg):
            self.assertAlmostEqual(actual_value, expected_value, places=12)
        source_parameters = dynamic.native.parameter_values(self.airframe.read_text())
        actual_parameters = dynamic.native.parameter_values((output / 'px4' /
            f"{manifest['airframe_id']}_gz_rocket_two_axis_pod").read_text())
        snapshot = json.loads((output / 'parameters.json').read_text())
        self.assertEqual(actual_parameters, snapshot['px4']['parameters'])
        reference = manifest['px4_reference']
        self.assertEqual(reference, snapshot['assembly']['px4_reference'])
        self.assertEqual(reference['reference_pose'], 'zero_joint_positions')
        prefixes = [f'CA_ROTOR{index}_P' for index in range(4)] + ['EKF2_IMU_POS_', 'EKF2_GPS_POS_']
        signs = (1, -1, -1)
        reference_keys = {prefix + axis for prefix in prefixes for axis in 'XYZ'}
        self.assertEqual(set(reference['source_parameters']), reference_keys)
        self.assertEqual(set(reference['parameters']), reference_keys)
        for axis, sign in enumerate(signs):
            shift = sign * (source_cg[axis] - expected_cg[axis])
            self.assertAlmostEqual(reference['base_cg_flu_m'][axis], source_cg[axis], places=12)
            self.assertAlmostEqual(reference['assembly_nominal_cg_flu_m'][axis], expected_cg[axis], places=12)
            self.assertAlmostEqual(reference['offset_translation_frd_m'][axis], shift, places=12)
            for prefix in prefixes:
                key = prefix + 'XYZ'[axis]
                self.assertEqual(reference['source_parameters'][key], source_parameters[key])
                self.assertAlmostEqual(actual_parameters[key], source_parameters[key] + shift, places=12)
                self.assertAlmostEqual(reference['parameters'][key], actual_parameters[key], places=12)
                # 从任意 CG 参考恢复后的物理点必须相同。
                self.assertAlmostEqual(actual_parameters[key] + sign * actual_cg[axis],
                                       source_parameters[key] + sign * source_cg[axis], places=12)
        self.assertEqual(set(actual_parameters), set(source_parameters))
        for key, value in source_parameters.items():
            if key not in reference_keys and key != 'MPC_THR_HOVER':
                self.assertEqual(actual_parameters[key], value, key)

    def test_old_json_physics_cannot_replace_native_model_or_px4(self):
        path = self.source / 'parameters.json'
        defaults = json.loads(path.read_text())
        defaults['mass']['body_kg'] = 999
        defaults['geometry']['body_length_m'] = -3
        defaults['motor']['thrust_constant'] = -1
        defaults['px4']['parameters']['MPC_XY_VEL_MAX'] = 12
        path.write_text(json.dumps(defaults))
        before = self.fingerprints()
        output = self.work / 'output'
        manifest = dynamic.generate(output)
        snapshot = json.loads((output / 'parameters.json').read_text())
        self.assertAlmostEqual(manifest['total_mass_kg'], self.expected_total_mass())
        source_inertial = self.original_model.find("link[@name='base_link']/inertial")
        actual_inertial = ET.parse(output / 'model.sdf').find("model/link[@name='base_link']/inertial")
        self.assertEqual(canonical(actual_inertial), canonical(source_inertial))
        self.assertEqual(snapshot['model']['links']['base_link']['mass_kg'], float(source_inertial.findtext('mass')))
        motor = self.original_model.find("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
        self.assertEqual(snapshot['model']['motors'][0]['motorConstant'], float(motor.findtext('motorConstant')))
        self.assertEqual(snapshot['px4']['parameters']['MPC_XY_VEL_MAX'], self.original_px4['MPC_XY_VEL_MAX'])
        self.assertNotIn('geometry', snapshot)
        self.assertEqual(snapshot['flight_test'], defaults['flight_test'])
        self.assertEqual(before, self.fingerprints())

    def test_native_custom_inertial_and_extra_px4_parameters_are_preserved(self):
        path = self.source / 'model.sdf'
        model = ET.parse(path)
        inertial = model.find("model/link[@name='base_link']/inertial")
        inertial.find('mass').text = '8.0'
        inertial.find('pose').text = '.01 -.02 -.083 .1 .2 .3'
        inertial.find('inertia/ixx').text = '.231'
        inertial.find('inertia/ixy').text = '.002'
        model.write(path)
        self.airframe.write_text(self.airframe.read_text() + '\n# custom native tuning\nparam set-default MC_ROLLRATE_P .22222\n')
        before = self.fingerprints()
        output = self.work / 'output'
        manifest = dynamic.generate(output)
        actual = ET.parse(output / 'model.sdf').find("model/link[@name='base_link']/inertial")
        self.assertEqual(canonical(actual), canonical(inertial))
        self.assertAlmostEqual(manifest['total_mass_kg'], self.expected_total_mass())
        text = (output / 'px4/4023_gz_rocket_two_axis_pod').read_text()
        original = dynamic.native.parameter_values(self.airframe.read_text())
        generated = dynamic.native.parameter_values(text)
        self.assertEqual(set(generated), set(original))
        self.assert_reference_inheritance(output, manifest)
        self.assertIn('# custom native tuning\nparam set-default MC_ROLLRATE_P .22222\n', text)
        self.assertEqual(before, self.fingerprints())

    def test_nominal_cg_reference_preserves_rotor_and_sensor_physical_points(self):
        before = self.fingerprints()
        output = self.work / 'output'
        manifest = dynamic.generate(output)
        self.assert_reference_inheritance(output, manifest)
        self.assertEqual(before, self.fingerprints())

    def test_two_axis_pitch_direction_and_limits_are_inherited(self):
        output = self.work / 'output'
        manifest = dynamic.generate(output)
        model = ET.parse(output / 'model.sdf').getroot().find('model')
        for name, axis, limits in (('yaw_joint', '-1 0 0', (-60., 60.)),
                                   ('pitch_joint', '0 -1 0', (-80., 30.))):
            joint = model.find(f"joint[@name='{name}']")
            self.assertEqual(canonical(joint), canonical(self.pod_model.find(f"joint[@name='{name}']")))
            self.assertEqual(joint.findtext('axis/xyz'), axis)
            for key, degrees in zip(('lower', 'upper'), limits):
                self.assertAlmostEqual(float(joint.findtext('axis/limit/' + key)), math.radians(degrees))
        # 关节零位重心仍由原生惯性派生，轴正向变化不得改变 PX4 参考规则。
        self.assert_reference_inheritance(output, manifest)

    def test_rotated_native_link_and_custom_inertial_pose_use_full_transform(self):
        path = self.source / 'model.sdf'
        document = ET.parse(path)
        link = document.find("model/link[@name='nacelle_0']")
        self.assertIsNotNone(link)
        dynamic.set_text(link, 'pose', '.15 -.14 .025 .35 -.17 .4')
        dynamic.set_text(link.find('inertial'), 'pose', '.012 -.018 .027 .4 .1 -.2')
        document.write(path)
        before = self.fingerprints()
        output = self.work / 'output'
        manifest = dynamic.generate(output)
        self.assert_reference_inheritance(output, manifest)
        actual = ET.parse(output / 'model.sdf').find("model/link[@name='nacelle_0']")
        # 整个原生惯性定义与非零姿态保持不变；这里只对 PX4 参考原点做平移。
        self.assertEqual(canonical(actual.find('inertial')), canonical(link.find('inertial')))
        self.assertEqual(actual.findtext('pose'), link.findtext('pose'))
        naive_moment = [0., 0., 0.]
        total = 0.
        for native_link in document.findall('model/link'):
            mass = float(native_link.findtext('inertial/mass'))
            pose = list(map(float, native_link.findtext('pose', '0 0 0 0 0 0').split()))
            inertial = list(map(float, native_link.findtext('inertial/pose', '0 0 0 0 0 0').split()))
            for axis in range(3):
                naive_moment[axis] += mass * (pose[axis] + inertial[axis])
            total += mass
        actual_cg = manifest['px4_reference']['base_cg_flu_m']
        self.assertGreater(max(abs(actual_cg[axis] - naive_moment[axis] / total) for axis in range(3)), 1e-5)
        self.assertEqual(before, self.fingerprints())

    def test_missing_or_duplicate_reference_parameter_is_rejected_before_write(self):
        original = self.airframe.read_text()
        line = next(line for line in original.splitlines() if line.startswith('param set-default EKF2_IMU_POS_X '))
        for text in (original.replace(line + '\n', ''), original + '\n' + line + '\n'):
            with self.subTest(text=text):
                self.airframe.write_text(text)
                output = self.work / 'invalid-reference'
                with self.assertRaisesRegex(ValueError, 'EKF2_IMU_POS_X'):
                    dynamic.generate(output)
                self.assertFalse(output.exists())

    def test_interface_override_and_component_world_snapshot_match_output(self):
        output = self.work / 'output'
        manifest = dynamic.generate(output, airframe_id=4999, topic_prefix='/test/pod_23')
        model = ET.parse(output / 'model.sdf').getroot().find('model')
        plugin = model.find("plugin[@name='two_axis::GimbalController']")
        for key in ('command', 'telemetry', 'command_result'):
            self.assertEqual(plugin.findtext(key + '_topic'), '/test/pod_23/' + key)
        self.assertEqual(plugin.findtext('mount_dynamic'), 'true')
        self.assertEqual(float(plugin.findtext('max_rate_dps')), 20)
        for axis in ('yaw', 'pitch'):
            velocity = model.findtext(f"joint[@name='{axis}_joint']/axis/limit/velocity")
            self.assertAlmostEqual(math.degrees(float(velocity)), 20, places=9)
        self.assertEqual(model.findtext("link[@name='camera_link']/sensor[@name='camera']/topic"), '/test/pod_23/image')
        self.assertEqual(manifest['airframe_id'], 4999)
        self.assertTrue((output / 'px4/4999_gz_rocket_two_axis_pod').is_file())
        snapshot = json.loads((output / 'parameters.json').read_text())
        world = ET.parse(output / 'tests/world.sdf').getroot().find('world')
        self.assertEqual(snapshot['simulation']['world_name'], world.get('name'))
        self.assertEqual(snapshot['simulation']['gravity'], world.findtext('gravity'))
        self.assertEqual(snapshot['simulation']['spawn_pose'], world.findtext("include[uri='model://rocket_two_axis_pod']/pose"))
        self.assertEqual(snapshot['runtime']['source_sha256'], self.fingerprints())
        self.assertEqual(len(snapshot['model']['links']), 12)
        self.assertEqual(len(model.findall('joint')), 11)

    def test_mass_override_is_rejected_before_writing(self):
        body_mass = float(self.original_model.findtext("link[@name='base_link']/inertial/mass"))
        for value in (body_mass / 2, body_mass * 2, -1, 0, float('nan'), float('inf'), True):
            with self.subTest(value=value):
                output = self.work / 'invalid'
                with self.assertRaisesRegex(ValueError, '不能单独覆盖'):
                    dynamic.generate(output, body_mass_kg=value)
                self.assertFalse(output.exists())

    def test_configured_mass_override_is_also_rejected(self):
        assembly, geometry, pod = dynamic.appearance.configuration()
        assembly = copy.deepcopy(assembly)
        body_mass = float(self.original_model.findtext("link[@name='base_link']/inertial/mass"))
        assembly.setdefault('dynamic', {})['body_mass_kg'] = body_mass / 2
        with mock.patch.object(dynamic.appearance, 'configuration', return_value=(assembly, geometry, pod)):
            with self.assertRaisesRegex(ValueError, '不能单独覆盖'):
                dynamic.settings()

    def test_hover_uses_each_native_channel_limit(self):
        config = dynamic.settings()
        model = dynamic.build_model(config).find('model')
        parameters = dict(config['px4_parameters'])
        parameters['SIM_GZ_EC_MIN1'] = 45
        parameters['SIM_GZ_EC_MAX2'] = 1500
        result = dynamic.hover_configuration(model, parameters, 9.80665)
        thrust = 0.
        for motor in model.findall(f"plugin[@name='{dynamic.MOTOR}']"):
            channel = int(motor.findtext('motorNumber')) + 1
            idle = parameters[f'SIM_GZ_EC_MIN{channel}']
            maximum = parameters[f'SIM_GZ_EC_MAX{channel}']
            velocity = idle + (maximum - idle) * math.sqrt(result['px4_hover_thrust'])
            thrust += float(motor.findtext('motorConstant')) * velocity**2
        self.assertAlmostEqual(thrust, self.expected_total_mass() * 9.80665, places=10)


if __name__ == '__main__':
    unittest.main()
