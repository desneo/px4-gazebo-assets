#!/usr/bin/env python3
"""校验实际SDF轴链、光学姿态、机械限位和可重复生成。"""
import importlib.util
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def matmul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rot(axis, angle):
    c,s=math.cos(angle),math.sin(angle)
    if axis=='x': return [[1,0,0],[0,c,-s],[0,s,c]]
    if axis=='y': return [[c,0,s],[0,1,0],[-s,0,c]]
    return [[c,-s,0],[s,c,0],[0,0,1]]


class ModelTest(unittest.TestCase):
    def setUp(self):
        self.model=ET.parse(ROOT/'model.sdf').getroot().find('model')

    def test_chain_and_limits(self):
        joints=self.model.findall('joint')
        self.assertEqual(len(joints),2)
        for name, parent, child, axis, limits in [
            ('yaw_joint','mount_link','yaw_arm_link','0 0 1',(-90,90)),
            ('pitch_joint','yaw_arm_link','camera_link','0 1 0',(-45,90))]:
            j=self.model.find(f"joint[@name='{name}']")
            self.assertEqual(j.findtext('parent'),parent)
            self.assertEqual(j.findtext('child'),child)
            self.assertEqual(j.findtext('axis/xyz'),axis)
            for key, limit in zip(('lower','upper'),limits):
                self.assertAlmostEqual(math.degrees(float(j.findtext('axis/limit/'+key))),limit)
        self.assertIsNone(self.model.find("joint[@name='roll_joint']"))

    def test_actual_optical_pose_and_extremes(self):
        pose=list(map(float,self.model.findtext("link[@name='camera_link']/sensor/pose").split()))
        roll,pitch,yaw=pose[3:]
        optical=matmul(rot('z',yaw),matmul(rot('y',pitch),rot('x',roll)))
        # 从实际SDF光学姿态计算，而非只验证公式本身。
        for a,p,expected in [(0,0,(0,0,1)),(0,90,(1,0,0)),(90,0,(0,0,1)),(-90,0,(0,0,1)),(0,-45,(-2**-.5,0,2**-.5)),(90,90,(0,1,0))]:
            r=matmul(rot('z',math.radians(a)),matmul(rot('y',math.radians(p)),optical))
            for i in range(3): self.assertAlmostEqual(r[i][0],expected[i],places=8)

    def test_generation_and_demo(self):
        paths=[ROOT/'model.sdf',ROOT/'tests/world.sdf',
               ROOT/'meshes/carrier_ring.dae',ROOT/'meshes/rear_shell.dae']
        before=[p.read_bytes() for p in paths]
        spec=importlib.util.spec_from_file_location('generate',ROOT/'tools/generate_model.py')
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.generate()
        self.assertEqual(before,[p.read_bytes() for p in paths])
        demo=ET.parse(paths[1]).getroot().find("world/model[@name='two_axis_pod']")
        self.assertEqual(demo.findtext("joint[@name='world_mount']/parent"),'world')
        self.assertEqual(demo.findtext("joint[@name='world_mount']/child"),'mount_link')

    def test_engineering_envelope(self):
        parameters=__import__('json').loads((ROOT/'parameters.json').read_text())
        self.assertAlmostEqual(parameters['housing_diameter_m'],.084)
        self.assertAlmostEqual(parameters['overall_height_m'],.091)
        self.assertAlmostEqual(parameters['body_height_m'],.083)
        self.assertAlmostEqual(parameters['flange_thickness_m'],.008)
        self.assertAlmostEqual(parameters['overall_height_m']-parameters['flange_thickness_m'],
                               parameters['body_height_m'])
        self.assertAlmostEqual(parameters['camera_diameter_m'],.048)
        self.assertEqual(parameters['camera_center_m'],[.015,0,.056])
        self.assertEqual(parameters['mounting_holes']['count'],6)
        self.assertAlmostEqual(parameters['mounting_holes']['thread_diameter_m'],.003)
        camera=self.model.find("link[@name='camera_link']")
        sphere=float(camera.findtext("visual[@name='camera_shell_visual']/geometry/sphere/radius"))
        self.assertAlmostEqual(2*sphere,.048)
        flange=self.model.find("link[@name='mount_link']/visual[@name='mounting_flange_visual']")
        self.assertAlmostEqual(2*float(flange.findtext('geometry/cylinder/radius')),.084)
        self.assertAlmostEqual(float(flange.findtext('geometry/cylinder/length')),.008)
        flange_pose=list(map(float,flange.findtext('pose').split()))
        # SDF cylinder 的本地轴为 Z：RPY 为零确保底面水平，下表面落在安装面。
        self.assertAlmostEqual(flange_pose[2],.004)
        self.assertEqual(flange_pose[3:],[0,0,0])
        self.assertIsNone(self.model.find("link/visual[@name='horizontal_base_visual']"))
        arm=self.model.find("link[@name='yaw_arm_link']")
        self.assertIsNotNone(arm.find("visual[@name='rear_shell_visual']"))
        self.assertIsNotNone(arm.find("visual[@name='carrier_ring_visual']"))
        ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
        mesh=ET.parse(ROOT/'meshes/rear_shell.dae')
        values=list(map(float,mesh.findtext(".//c:source[@id='rear_shell-positions']/c:float_array",namespaces=ns).split()))
        self.assertAlmostEqual(max(values[2::3]),.091)
        self.assertLessEqual(max(abs(v) for v in values[0::3]),.042)
        self.assertLessEqual(max(abs(v) for v in values[1::3]),.042)


if __name__=='__main__': unittest.main()
