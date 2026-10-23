"""从基础吊舱 SDF 读取的运动链，供装配预览和场景真值验收使用。"""
from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

POD = Path(__file__).resolve().parents[3] / 'models/gimbals/two_axis_pod'


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def transform(pose):
    values = [float(v) for v in (pose or '0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('SDF pose 必须包含六个有限值')
    matrix = np.eye(4)
    matrix[:3, :3] = rotation(values[3:])
    matrix[:3, 3] = values[:3]
    return matrix


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('关节轴必须为非零有限向量')
    x, y, z = axis / norm
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross @ cross)


class PodKinematics:
    """以模型 home 位姿及 SDF joint 枢轴/转轴计算活动 link 位姿。"""
    def __init__(self, model_path=POD / 'model.sdf'):
        model = ET.parse(model_path).getroot().find('model')
        self.home = {}
        for link in model.findall('link'):
            pose = link.find('pose')
            if pose is not None and pose.get('relative_to', '__model__') != '__model__':
                raise ValueError('吊舱 link home pose 必须相对模型坐标系')
            self.home[link.get('name')] = transform(link.findtext('pose'))
        self.joints, self.child_joints = {}, {}
        for joint in model.findall('joint'):
            name, child, parent = joint.get('name'), joint.findtext('child'), joint.findtext('parent')
            if child not in self.home or (parent not in self.home and parent != 'world'):
                raise ValueError(f'关节 {name} 的父子 link 不在吊舱中')
            pose = joint.find('pose')
            frame = pose.get('relative_to', child) if pose is not None else child
            reference = np.eye(4) if frame == '__model__' else self.home[frame]
            pivot = reference @ transform(joint.findtext('pose'))
            xyz = joint.find('axis/xyz')
            axis = None
            limits = None
            if joint.get('type') == 'revolute':
                axis_frame = xyz.get('expressed_in')
                axis_reference = (pivot if not axis_frame else
                                  np.eye(4) if axis_frame == '__model__' else self.home[axis_frame])
                axis = axis_reference[:3, :3] @ np.fromstring(xyz.text, sep=' ')
                limits = tuple(math.degrees(float(joint.findtext('axis/limit/' + bound)))
                               for bound in ('lower', 'upper'))
            elif joint.get('type') != 'fixed':
                raise ValueError(f'不支持吊舱关节类型 {joint.get("type")}')
            if child in self.child_joints:
                raise ValueError('吊舱运动链不支持一个 child 的多个父关节')
            self.joints[name] = dict(parent=parent, child=child, pivot=pivot, axis=axis, limits=limits)
            self.child_joints[child] = name
        camera = model.find("link[@name='camera_link']/sensor[@name='camera']")
        if camera is None:
            raise ValueError('基础模型没有 camera_link/camera')
        sensor_pose = camera.find('pose')
        if sensor_pose is not None and sensor_pose.get('relative_to', 'camera_link') != 'camera_link':
            raise ValueError('相机 sensor pose 必须相对 camera_link')
        self.sensor = transform(camera.findtext('pose'))

    def link_poses(self, yaw_deg=0., pitch_deg=0., *, validate_limits=True):
        angles = dict(yaw_joint=yaw_deg, pitch_joint=pitch_deg)
        for name, value in angles.items():
            limits = self.joints[name]['limits']
            if not math.isfinite(value) or (validate_limits and not limits[0]-1e-8 <= value <= limits[1]+1e-8):
                raise ValueError(f'{name} 超出机械限位')
        result, visiting = {}, set()

        def resolve(link):
            if link in result:
                return result[link]
            if link in visiting:
                raise ValueError('吊舱关节包含环路')
            visiting.add(link)
            if link not in self.child_joints:
                value = self.home[link].copy()
            else:
                name = self.child_joints[link]
                joint = self.joints[name]
                parent = joint['parent']
                parent_motion = (np.eye(4) if parent == 'world' else
                                 resolve(parent) @ np.linalg.inv(self.home[parent]))
                motion = np.eye(4)
                if joint['axis'] is not None:
                    motion[:3, :3] = axis_rotation(joint['axis'], math.radians(angles.get(name, 0.)))
                    pivot = joint['pivot'][:3, 3]
                    motion[:3, 3] = pivot - motion[:3, :3] @ pivot
                value = parent_motion @ motion @ self.home[link]
            visiting.remove(link)
            result[link] = value
            return value

        for link in self.home:
            resolve(link)
        return result

    def camera_pose(self, yaw_deg=0., pitch_deg=0., mount_z=0.):
        pose = self.link_poses(yaw_deg, pitch_deg, validate_limits=False)['camera_link'] @ self.sensor
        pose[2, 3] += mount_z
        return pose

    def inverse(self, target_mount, mount_z=0.):
        """新 CAD 机构解析逆解；视线原点沿光轴的偏移不改变对准角。"""
        # 球心位于外轴线上，因此外轴转动不会使球心公转。
        center = self.home['camera_link'][:3, 3] + np.array([0., 0., mount_z])
        vector = np.asarray(target_mount, dtype=float) - center
        distance = np.linalg.norm(vector)
        if not np.isfinite(vector).all() or distance < 1e-9:
            return dict(reachable=False, yaw=0., pitch=0., singular=True)
        unit = vector / distance
        yaw = math.degrees(math.atan2(-unit[1], unit[2]))
        pitch = -math.degrees(math.atan2(unit[0], math.hypot(unit[1], unit[2])))
        limits = [self.joints[axis + '_joint']['limits'] for axis in ('yaw', 'pitch')]
        reachable = all(lo-1e-8 <= angle <= hi+1e-8 for angle, (lo, hi) in zip((yaw, pitch), limits))
        singular = math.hypot(unit[1], unit[2]) < math.sin(math.radians(3.))
        if reachable:
            actual = self.camera_pose(yaw, pitch, mount_z)
            ray = np.asarray(target_mount) - actual[:3, 3]
            if np.dot(actual[:3, 0], ray) <= 1e-9:
                return dict(reachable=False, yaw=yaw, pitch=pitch, singular=singular)
            # 校核所读取的 SDF 确实满足该机构几何，防止基础接口改变后静默误算。
            if np.linalg.norm(np.cross(actual[:3, 0], ray / np.linalg.norm(ray))) > 1e-7:
                raise ValueError('基础 SDF 运动链与 motion_head_joint_right_positive 解析逆解不一致')
        return dict(reachable=reachable, yaw=yaw, pitch=pitch, singular=singular)
