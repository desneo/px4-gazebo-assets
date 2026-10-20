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
        self.addCleanup(mock.patch.stopall)
        mock.patch.object(dynamic, 'ROCKET', self.source).start()
        mock.patch.object(dynamic.appearance, 'ROCKET', self.source).start()
        mock.patch.object(dynamic, 'source_fingerprints', side_effect=self.fingerprints).start()

    def fingerprints(self):
        return {str(path.relative_to(self.source)): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted(self.source.rglob('*')) if path.is_file()}

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
        self.assertAlmostEqual(manifest['total_mass_kg'], 7.94)
        self.assertEqual(snapshot['model']['links']['base_link']['mass_kg'], 7.44)
        self.assertEqual(snapshot['model']['motors'][0]['motorConstant'], 8.9119e-5)
        self.assertEqual(snapshot['px4']['parameters']['MPC_XY_VEL_MAX'], 95)
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
        self.assertAlmostEqual(manifest['total_mass_kg'], 8.5)
        text = (output / 'px4/4023_gz_rocket_two_axis_pod').read_text()
        original = dynamic.native.parameter_values(self.airframe.read_text())
        generated = dynamic.native.parameter_values(text)
        self.assertEqual(set(generated), set(original))
        for key, value in original.items():
            if key != 'MPC_THR_HOVER':
                self.assertEqual(generated[key], value, key)
        self.assertIn('# custom native tuning\nparam set-default MC_ROLLRATE_P .22222\n', text)
        self.assertEqual(before, self.fingerprints())

    def test_interface_override_and_component_world_snapshot_match_output(self):
        output = self.work / 'output'
        manifest = dynamic.generate(output, airframe_id=4999, topic_prefix='/test/pod_23')
        model = ET.parse(output / 'model.sdf').getroot().find('model')
        plugin = model.find("plugin[@name='two_axis::GimbalController']")
        for key in ('command', 'telemetry', 'command_result'):
            self.assertEqual(plugin.findtext(key + '_topic'), '/test/pod_23/' + key)
        self.assertEqual(plugin.findtext('mount_dynamic'), 'true')
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
        for value in (.9, 1000, -1, 0, float('nan'), float('inf'), True):
            with self.subTest(value=value):
                output = self.work / 'invalid'
                with self.assertRaisesRegex(ValueError, '不能单独覆盖'):
                    dynamic.generate(output, body_mass_kg=value)
                self.assertFalse(output.exists())

    def test_configured_mass_override_is_also_rejected(self):
        assembly, geometry, pod = dynamic.appearance.configuration()
        assembly = copy.deepcopy(assembly)
        assembly.setdefault('dynamic', {})['body_mass_kg'] = .9
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
        self.assertAlmostEqual(thrust, 7.94 * 9.80665, places=10)


if __name__ == '__main__':
    unittest.main()
