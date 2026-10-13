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
            ('yaw_joint','mount_link','yaw_arm_link','1 0 0',(-90,90)),
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
        for a,p,expected in [(0,0,(0,0,1)),(0,90,(1,0,0)),(90,0,(0,-1,0)),(-90,0,(0,1,0)),(0,-45,(-2**-.5,0,2**-.5)),(90,90,(1,0,0))]:
            r=matmul(rot('x',math.radians(a)),matmul(rot('y',math.radians(p)),optical))
            for i in range(3): self.assertAlmostEqual(r[i][0],expected[i],places=8)

    def test_generation_and_demo(self):
        paths=[ROOT/'model.sdf',ROOT/'tests/world.sdf']
        before=[p.read_bytes() for p in paths]
        spec=importlib.util.spec_from_file_location('generate',ROOT/'tools/generate_model.py')
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.generate()
        self.assertEqual(before,[p.read_bytes() for p in paths])
        demo=ET.parse(paths[1]).getroot().find("world/model[@name='two_axis_pod']")
        self.assertEqual(demo.findtext("joint[@name='world_mount']/parent"),'world')
        self.assertEqual(demo.findtext("joint[@name='world_mount']/child"),'mount_link')


if __name__=='__main__': unittest.main()
