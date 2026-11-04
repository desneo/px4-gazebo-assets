"""中科火箭装配的源隔离、动力继承、顶部水平安装和逐机INI契约。"""
from pathlib import Path
import hashlib
import importlib.util
import json
import math
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('rocket_zhongke_dynamic_test', ROOT / 'tools/generate_dynamic_model.py')
dynamic = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(dynamic)
appearance = dynamic.appearance


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(),
            tuple(canonical(child) for child in element))


def independent_cg(model):
    total, moment = 0., np.zeros(3)
    for link in model.findall('link'):
        mass = float(link.findtext('inertial/mass'))
        x, y, z, roll, pitch, yaw = map(float, link.findtext('pose', '0 0 0 0 0 0').split())
        ix, iy, iz = map(float, link.findtext('inertial/pose', '0 0 0 0 0 0').split()[:3])
        iy, iz = math.cos(roll)*iy-math.sin(roll)*iz, math.sin(roll)*iy+math.cos(roll)*iz
        ix, iz = math.cos(pitch)*ix+math.sin(pitch)*iz, -math.sin(pitch)*ix+math.cos(pitch)*iz
        ix, iy = math.cos(yaw)*ix-math.sin(yaw)*iy, math.sin(yaw)*ix+math.cos(yaw)*iy
        moment += mass * np.array([x+ix, y+iy, z+iz])
        total += mass
    return total, moment / total


class AssemblyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = dynamic.settings()
        cls.model = dynamic.build_model(cls.config).find('model')
        cls.rocket = cls.config['source_model'].find('model')
        cls.pod = cls.config['pod_model']

    def test_source_baseline_unchanged(self):
        baseline = json.loads((ROOT / 'tests/source_baseline.json').read_text())['source_sha256']
        changed = [relative for relative, digest in baseline.items()
                   if hashlib.sha256((dynamic.REPO / relative).read_bytes()).hexdigest() != digest]
        self.assertEqual(changed, [])

    def test_physics_and_source_inertials_preserved(self):
        self.assertEqual(len(self.model.findall('link')), 12)
        self.assertEqual(len(self.model.findall('joint')), 11)
        mass, _ = independent_cg(self.model)
        self.assertAlmostEqual(mass, 4.955, places=10)
        for source in (self.rocket, self.pod):
            for link in source.findall('link'):
                result = self.model.find(f"link[@name='{link.get('name')}']")
                self.assertEqual(canonical(result.find('inertial')), canonical(link.find('inertial')))
        for joint in self.pod.findall('joint'):
            self.assertEqual(canonical(self.model.find(f"joint[@name='{joint.get('name')}']")), canonical(joint))
        for name in (dynamic.MOTOR, dynamic.AERO):
            self.assertEqual([canonical(p) for p in self.model.findall(f"plugin[@name='{name}']")],
                             [canonical(p) for p in self.rocket.findall(f"plugin[@name='{name}']")])

    def test_mount_has_no_standalone_y90_and_one_fixed_joint(self):
        self.assertIsNone(self.model.find('pose'))
        self.assertAlmostEqual(self.config['mount_z_m'], .289)
        for link in self.pod.findall('link'):
            result = self.model.find(f"link[@name='{link.get('name')}']")
            expected = appearance.transform(link.findtext('pose'))
            expected[2, 3] += .289
            np.testing.assert_allclose(appearance.transform(result.findtext('pose')), expected, atol=1e-12)
        mount = self.model.find("joint[@name='pod_mount_joint']")
        self.assertEqual(mount.get('type'), 'fixed')
        self.assertEqual((mount.findtext('parent'), mount.findtext('child')), ('base_link', 'mount_link'))
        chain = appearance.kinematics.PodKinematics(model=self.pod)
        np.testing.assert_allclose(chain.camera_pose()[:3, 0], [0, 0, 1], atol=1e-12)
        np.testing.assert_allclose(chain.camera_pose(pitch_deg=-80)[:3, 0],
                                   [math.sin(math.radians(80)), 0, math.cos(math.radians(80))], atol=1e-12)
        for yaw, pitch in ((-90,-90),(90,-90),(0,30),(90,30)):
            self.assertTrue(np.isfinite(chain.camera_pose(yaw,pitch)).all())
        source = ET.parse(dynamic.POD / 'model.sdf').getroot().find('model')
        np.testing.assert_allclose(appearance.transform(source.findtext('pose'))[:3, :3] @ [0,0,1], [1,0,0], atol=1e-12)

    def test_default_topics_and_camera_and_dynamic_mount(self):
        plugin = self.model.find("plugin[@name='zhongke_d80::GimbalController']")
        self.assertEqual(plugin.get('filename'), 'gz-zhongke-d80-controller')
        self.assertEqual(plugin.findtext('mount_dynamic'), 'true')
        self.assertEqual(plugin.findtext('model_name'), self.config['component_config']['model']['name'])
        self.assertEqual(plugin.findtext('config_hash'), self.config['component_config']['config_hash'])
        prefix = '/rocket_zhongke_d80'
        for topic in ('command','telemetry','command_result'):
            self.assertEqual(plugin.findtext(topic+'_topic'), prefix+'/'+topic)
        self.assertEqual(plugin.findtext('image_topic'), prefix+'/image')
        cameras = self.model.findall("link/sensor[@type='camera']")
        self.assertEqual(len(cameras), 1)
        self.assertEqual(cameras[0].findtext('topic'), prefix+'/image')
        self.assertEqual(cameras[0].findtext('camera/camera_info_topic'), prefix+'/camera_info')
        d = self.config['component_config']['derived']
        for key in ('fx','fy','cx','cy'):
            self.assertAlmostEqual(float(cameras[0].findtext('camera/lens/intrinsics/'+key)),d[key],places=9)

    def test_cg_reference_and_hover_are_rederived(self):
        _, source_cg = independent_cg(self.rocket)
        mass, cg = independent_cg(self.model)
        reference = dynamic.px4_reference_configuration(self.rocket, self.model, self.config['px4_parameters'])
        np.testing.assert_allclose(reference['assembly_nominal_cg_flu_m'],cg,atol=1e-14)
        np.testing.assert_allclose(reference['offset_translation_frd_m'],(source_cg-cg)*[1,-1,-1],atol=1e-14)
        self.assertEqual(len(reference['parameters']),18)
        offset=list(map(float,self.model.findtext("plugin[@name='zhongke_d80::GimbalController']/los_observer_cg_model").split()))
        np.testing.assert_allclose(offset,cg,atol=1e-14)
        hover=dynamic.hover_configuration(self.model,self.config['px4_parameters'],9.8)
        u=math.sqrt(hover['px4_hover_thrust'])
        thrust=0.
        for motor in self.model.findall(f"plugin[@name='{dynamic.MOTOR}']"):
            channel=int(motor.findtext('motorNumber'))+1
            lo=self.config['px4_parameters'][f'SIM_GZ_EC_MIN{channel}']
            hi=self.config['px4_parameters'][f'SIM_GZ_EC_MAX{channel}']
            thrust+=float(motor.findtext('motorConstant'))*(lo+(hi-lo)*u)**2
        self.assertAlmostEqual(thrust,mass*9.8,places=10)

    def test_per_aircraft_config_full_generation_and_asset_closure(self):
        with tempfile.TemporaryDirectory(prefix='rocket-zhongke-contract-') as temporary:
            temporary=Path(temporary)
            text=(dynamic.POD/'parameters.ini').read_text()
            text=text.replace('name = zhongke_d80','name = rocket_zhongke_d80_7')
            text=text.replace('aircraft_id = 1','aircraft_id = 7')
            text=text.replace('mass_kg = 0.195','mass_kg = 0.24')
            text=text.replace('target_a, target_b, target_c','rocket_zhongke_d80_1, xiaomotuo, xiaobianfu')
            config=temporary/'vehicle.ini';config.write_text(text)
            loaded=appearance.component.load_config(config)
            self.assertEqual(loaded['model']['name'],'rocket_zhongke_d80_7')
            output=temporary/dynamic.NAME
            before=dynamic.source_fingerprints(config)
            manifest=dynamic.generate(output,config_path=config,topic_prefix='/uav_7/pod')
            self.assertEqual(before,dynamic.source_fingerprints(config))
            self.assertEqual(manifest['airframe_id'],4025)
            self.assertEqual(manifest['config_hash'],loaded['config_hash'])
            self.assertEqual(manifest['component_model_name'],'rocket_zhongke_d80_7')
            self.assertAlmostEqual(manifest['total_mass_kg'],5.,places=10)
            model=ET.parse(output/'model.sdf').getroot().find('model')
            self.assertEqual(model.get('name'),dynamic.NAME)
            plugin=model.find("plugin[@name='zhongke_d80::GimbalController']")
            self.assertEqual(plugin.findtext('config_hash'),loaded['config_hash'])
            self.assertEqual([e.text for e in plugin.findall('sim_truth_target_name')],loaded['tracking']['target_names'])
            self.assertEqual(plugin.findtext('image_topic'),'/uav_7/pod/image')
            for uri in model.findall('.//mesh/uri'):
                self.assertTrue(uri.text.startswith('model://rocket_zhongke_d80/meshes/'))
                self.assertTrue((output/uri.text.split('/',3)[3]).is_file(),uri.text)
            airframe=output/'px4/4025_gz_rocket_zhongke_d80'
            self.assertTrue(airframe.is_file())
            snapshot=json.loads((output/'parameters.json').read_text())
            self.assertEqual(dynamic.native.parameter_values(airframe.read_text()),snapshot['px4']['parameters'])

    def test_bad_overrides_and_source_output_are_rejected(self):
        for prefix in ('relative','/uav_1/pod/','/uav 1/pod'):
            with self.subTest(prefix=prefix),self.assertRaises(ValueError):dynamic.settings(topic_prefix=prefix)
        with self.assertRaises(ValueError):dynamic.settings(body_mass_kg=4.)
        for path in (ROOT, dynamic.POD, dynamic.REPO/'assembledmodels/rocket_two_axis_pod/runtime/x'):
            with self.subTest(path=path),self.assertRaises(ValueError):dynamic.generate(path)

    def test_static_asset_and_navigation(self):
        model=ET.parse(ROOT/'model.sdf').getroot().find('model')
        self.assertEqual(model.findtext('static'),'true')
        self.assertEqual(len(model.findall('link/visual')),
                         len(self.rocket.findall('link/visual'))+len(self.pod.findall('link/visual')))
        self.assertFalse(model.findall('.//collision'))
        self.assertFalse(model.findall('joint'))
        for uri in model.findall('.//mesh/uri'):
            self.assertTrue((ROOT/uri.text.split('/',3)[3]).is_file())
        for view in ('overall','close'):
            world=appearance.build_world(view).find('world')
            marker=world.find("model[@name='ROCKET_ZHONGKE_D80_LOCATION_MARKER']")
            self.assertIsNotNone(marker)
            self.assertFalse(marker.findall('.//collision'))
            self.assertEqual(float(world.findtext('gui/plugin/camera_clip/near')),.005)


if __name__=='__main__':unittest.main()
