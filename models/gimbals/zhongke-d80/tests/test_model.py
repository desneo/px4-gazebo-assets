"""独立核对复制几何、195g惯性、前视安装和配置生成的真实SDF。"""
from pathlib import Path
import configparser
import hashlib
import importlib.util
import json
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from common.config import load_config, to_model_parameters


def rotation_y(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,0,s],[0,1,0],[-s,0,c]])


class ModelTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('zhongke_generator',ROOT/'tools/generate_model.py')
        cls.generator=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.generator)
        cls.temp=tempfile.TemporaryDirectory()
        cls.destination=Path(cls.temp.name)
        cls.generator.generate(output_dir=cls.destination)
        cls.model=ET.parse(cls.destination/'model.sdf').getroot().find('model')
        cls.world=ET.parse(cls.destination/'tests/world.sdf').getroot().find('world')
        cls.c=load_config();cls.params=to_model_parameters(cls.c)
        cls.properties=json.loads((ROOT/'cad/properties.json').read_text())

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_asset_hashes_and_single_camera(self):
        for name,digest in self.properties['sha256'].items():
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),digest,name)
        self.assertEqual(len(self.properties['parts']),9)
        self.assertEqual(len(self.model.findall('.//sensor')),1)
        self.assertEqual([len(link.findall('visual')) for link in self.model.findall('link')],[3,3,3])
        for uri in self.model.findall('.//mesh/uri'):
            self.assertTrue(uri.text.startswith('model://zhongke-d80/'))

    def test_mass_inertia_volume_and_centers(self):
        total=0
        for link in self.model.findall('link'):
            group=self.properties['groups'][link.get('name')]
            inertial=link.find('inertial');mass=float(inertial.findtext('mass'));total+=mass
            self.assertAlmostEqual(mass,.195*group['volume_mm3']/self.properties['volume_mm3'],places=12)
            values={k:float(inertial.findtext('inertia/'+k)) for k in ('ixx','iyy','izz','ixy','ixz','iyz')}
            tensor=np.array([[values['ixx'],values['ixy'],values['ixz']],
                             [values['ixy'],values['iyy'],values['iyz']],
                             [values['ixz'],values['iyz'],values['izz']]])
            np.testing.assert_allclose(tensor,mass*np.array(group['unit_mass_inertia_m2']),rtol=1e-12,atol=1e-15)
            eigen=np.linalg.eigvalsh(tensor)
            self.assertGreater(eigen.min(),0)
            self.assertLessEqual(eigen[-1],eigen[0]+eigen[1]+1e-12)
            np.testing.assert_allclose(np.fromstring(inertial.findtext('pose'),sep=' ')[:3],group['center_local_m'],atol=1e-13)
        self.assertAlmostEqual(total,.195,places=12)

    def test_axes_actual_limits_and_velocity(self):
        self.assertEqual(len(self.model.findall('joint')),2)
        for axis,parent,child,direction,limits in [('yaw','mount_link','yaw_arm_link',[1,0,0],[-90,90]),
                                                  ('pitch','yaw_arm_link','camera_link',[0,-1,0],[-90,30])]:
            joint=self.model.find(f"joint[@name='{axis}_joint']")
            self.assertEqual(joint.findtext('parent'),parent);self.assertEqual(joint.findtext('child'),child)
            np.testing.assert_allclose(np.fromstring(joint.findtext('axis/xyz'),sep=' '),direction)
            for key,expected in zip(('lower','upper'),limits):
                self.assertAlmostEqual(math.degrees(float(joint.findtext('axis/limit/'+key))),expected,places=10)
            self.assertAlmostEqual(math.degrees(float(joint.findtext('axis/limit/velocity'))),20,places=10)
        plugin=self.model.find('plugin')
        self.assertEqual(plugin.get('name'),'zhongke_d80::GimbalController')
        self.assertEqual(float(plugin.findtext('max_rate_dps')),20)
        self.assertEqual(plugin.findtext('config_hash'),self.c['config_hash'])

    def test_one_mount_rotation_with_unchanged_sensor_basis(self):
        pose=np.fromstring(self.model.findtext('pose'),sep=' ')
        self.assertAlmostEqual(pose[4],math.pi/2,places=13)
        fixture=self.world.find(f"model[@name='{self.c['model']['name']}']")
        world_pose=np.fromstring(fixture.findtext('pose'),sep=' ')
        np.testing.assert_allclose(world_pose[3:],pose[3:],atol=1e-14)
        sensor=np.fromstring(self.model.findtext('.//sensor/pose'),sep=' ')
        self.assertAlmostEqual(sensor[4],-math.pi/2,places=13)
        np.testing.assert_allclose(rotation_y(pose[4])@rotation_y(sensor[4])@np.array([1,0,0]),[1,0,0],atol=1e-12)
        for link in self.model.findall('link'):
            np.testing.assert_allclose(np.fromstring(link.findtext('pose'),sep=' ')[:3],self.properties['groups'][link.get('name')]['origin_m'],atol=1e-13)

    def test_intrinsics_are_shared_with_derived_optics(self):
        camera=self.model.find('.//sensor/camera')
        for key in ('fx','fy','cx','cy'):
            self.assertAlmostEqual(float(camera.findtext('lens/intrinsics/'+key)),self.c['derived'][key],places=8)
        self.assertNotEqual(float(camera.findtext('lens/intrinsics/fx')),float(camera.findtext('lens/intrinsics/fy')))
        self.assertEqual(int(camera.findtext('image/width')),1920)
        self.assertEqual(int(camera.findtext('image/height')),1080)
        self.assertEqual(camera.findtext('camera_info_topic'),'/zhongke_d80/camera_info')

    def test_marker_has_no_collision_and_demo_has_registered_targets(self):
        marker=self.world.find(f"model[@name='{self.c['demo']['marker_name']}']")
        self.assertIsNotNone(marker)
        self.assertEqual(marker.findall('.//collision'),[])
        for name in self.c['tracking']['target_names']:
            self.assertIsNotNone(self.world.find(f"model[@name='{name}']"))

    def test_generation_is_deterministic(self):
        paths=[self.destination/'model.sdf',self.destination/'tests/world.sdf']
        before=[p.read_bytes() for p in paths]
        self.generator.generate(output_dir=self.destination)
        self.assertEqual(before,[p.read_bytes() for p in paths])

    def generated_variant(self, changes):
        config=configparser.ConfigParser()
        config.read(ROOT/'parameters.ini')
        for (section,key),value in changes.items():
            config[section][key]=str(value)
        path=self.destination/'variant.ini'
        with path.open('w') as stream:
            config.write(stream)
        self.generator.generate(path,self.destination/'variant')
        return ET.parse(self.destination/'variant/model.sdf').getroot().find('model')

    def test_reconfigured_motion_mass_resolution_and_topics(self):
        for rate in (10,20,30):
            model=self.generated_variant({('mechanics','max_rate_dps'):rate,
                ('model','mass_kg'):.25,('model','name'):'custom_gimbal',
                ('mechanics','yaw_min_deg'):-70,('mechanics','yaw_max_deg'):80,
                ('optics','width_px'):1280,('optics','height_px'):720})
            self.assertAlmostEqual(sum(float(x.findtext('inertial/mass')) for x in model.findall('link')),.25,places=12)
            self.assertEqual(model.get('name'),'custom_gimbal')
            for joint in model.findall('joint'):
                self.assertAlmostEqual(math.degrees(float(joint.findtext('axis/limit/velocity'))),rate,places=10)
            joint=model.find("joint[@name='yaw_joint']")
            self.assertAlmostEqual(math.degrees(float(joint.findtext('axis/limit/lower'))),-70,places=10)
            self.assertAlmostEqual(math.degrees(float(joint.findtext('axis/limit/upper'))),80,places=10)
            self.assertEqual(model.findtext('.//sensor/topic'),'/custom_gimbal/camera')
            camera=model.find('.//sensor/camera')
            self.assertEqual(camera.findtext('camera_info_topic'),'/custom_gimbal/camera_info')
            self.assertEqual(int(camera.findtext('image/width')),1280)
            self.assertEqual(int(camera.findtext('image/height')),720)
            self.assertAlmostEqual(float(camera.findtext('lens/intrinsics/fx')),15628.71306360365*2/3,places=7)
            self.assertAlmostEqual(float(camera.findtext('lens/intrinsics/fy')),15659.476350139897*2/3,places=7)
            plugin=model.find('plugin')
            for key,suffix in [('command_topic','command'),('telemetry_topic','telemetry'),
                               ('command_result_topic','command_result'),('image_topic','camera')]:
                self.assertEqual(plugin.findtext(key),'/custom_gimbal/'+suffix)

    def test_hundred_mm_intrinsics_drive_both_actual_fields_of_view(self):
        model=self.generated_variant({('optics','focal_length_mm'):100})
        camera=model.find('.//sensor/camera')
        fx=float(camera.findtext('lens/intrinsics/fx'))
        fy=float(camera.findtext('lens/intrinsics/fy'))
        self.assertAlmostEqual(math.degrees(2*math.atan(1920/(2*fx))),3.5183103922,places=8)
        self.assertAlmostEqual(math.degrees(2*math.atan(1080/(2*fy))),1.9755868472,places=8)
        self.assertNotEqual(fx,fy)


if __name__=='__main__':
    unittest.main()
