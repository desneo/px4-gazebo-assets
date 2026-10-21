#!/usr/bin/env python3
"""靶标初始化复用场景源姿态，避免改变飞机检测外观。"""
import math
import re
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import check_target as check
from prepare_world import target_model


class TargetInitializationTest(unittest.TestCase):
    def test_source_pose_and_prepare_reset_are_consistent(self):
        pose=tuple(map(float,target_model().findtext('pose').split()))
        self.assertEqual(check.INITIAL_POSE,pose)
        self.assertEqual(check.INITIAL,pose[:3])
        self.assertAlmostEqual(pose[5],math.pi)
        session=check.Session.__new__(check.Session)
        zero=dict(mode='IDLE',joints_deg={a:dict(pos=0.,vel=0.) for a in ('yaw','pitch')})
        session.collect=Mock(return_value=[zero])
        session.latest=dict(joints_deg=dict(yaw=dict(pos=0.),pitch=dict(pos=-math.degrees(
            math.atan2(check.INITIAL[0]-.015,check.INITIAL[2]-50.056)))))
        session.drive=Mock();session.send=Mock();session.record=Mock()
        with patch.object(check,'set_pose') as set_pose:
            session.prepare()
        self.assertEqual(set_pose.call_args_list[0].args,('occlusion_panel',check.HIDDEN_PANEL))
        self.assertEqual(set_pose.call_args_list[1].args,('tracking_target',pose[:3],pose[3:]))

    def test_pose_request_restores_yaw_and_default_stays_identity(self):
        success=SimpleNamespace(stdout='data: true',stderr='')
        with patch.object(check.subprocess,'run',return_value=success) as run:
            check.set_pose('tracking_target',check.INITIAL,check.INITIAL_POSE[3:])
            request=run.call_args.args[0][-1]
            self.assertRegex(request,r'name: "tracking_target"')
            quaternion=request.split('orientation:')[1]
            values=dict((axis,float(value)) for axis,value in re.findall(r'([wxyz]): ([^, }]+)',quaternion))
            self.assertAlmostEqual(values['w'],0.,places=10)
            self.assertAlmostEqual(values['z'],1.)
            check.set_pose('occlusion_panel',check.HIDDEN_PANEL)
            request=run.call_args.args[0][-1]
            self.assertIn('orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}',request)


if __name__=='__main__':
    unittest.main()
