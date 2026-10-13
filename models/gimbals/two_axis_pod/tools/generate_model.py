#!/usr/bin/env python3
"""由参数生成侧装两轴吊舱；坐标 X 朝板外、Y 朝左、Z 朝天。"""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def elem(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e


def geometry(parent, shape, values):
    g = elem(elem(parent, 'geometry'), shape)
    for k, v in values.items():
        elem(g, k, v)


def part(link, name, shape, values, pose='0 0 0 0 0 0', color='.09 .09 .10 1', collision=True):
    for tag in ('visual', 'collision') if collision else ('visual',):
        p = elem(link, tag, name=name + '_' + tag)
        elem(p, 'pose', pose)
        geometry(p, shape, values)
        if tag == 'visual':
            mat = elem(p, 'material')
            elem(mat, 'ambient', color)
            elem(mat, 'diffuse', color)
            elem(mat, 'specular', '.35 .35 .35 1')


def link(model, name, mass, xyz, inertia):
    l = elem(model, 'link', name=name)
    elem(l, 'pose', ' '.join(map(str, (*xyz, 0, 0, 0))))
    i = elem(l, 'inertial')
    elem(i, 'mass', mass)
    tensor = elem(i, 'inertia')
    for key, value in zip(('ixx', 'iyy', 'izz'), inertia):
        elem(tensor, key, value)
    for key in ('ixy', 'ixz', 'iyz'):
        elem(tensor, key, 0)
    return l


def joint(model, name, parent, child, axis, limits, rate):
    j = elem(model, 'joint', name=name, type='revolute')
    elem(j, 'parent', parent)
    elem(j, 'child', child)
    a = elem(j, 'axis')
    elem(a, 'xyz', axis)
    lim = elem(a, 'limit')
    elem(lim, 'lower', math.radians(limits[0]))
    elem(lim, 'upper', math.radians(limits[1]))
    elem(lim, 'effort', 5)
    elem(lim, 'velocity', math.radians(rate))
    elem(elem(a, 'dynamics'), 'damping', .1)


def generate():
    c = json.loads((ROOT / 'parameters.json').read_text())
    sdf = ET.Element('sdf', version='1.9')
    m = elem(sdf, 'model', name='two_axis_pod')
    elem(m, 'self_collide', 'false')
    sx, sy, sz = c['plate_size_m']
    mount = link(m, 'mount_link', .1, (0, 0, 0), (.1*(sy*sy+sz*sz)/12, .1*(sx*sx+sz*sz)/12, .1*(sx*sx+sy*sy)/12))
    elem(mount.find('inertial'), 'pose', '0 0 -0.035 0 0 0')
    part(mount, 'side_plate', 'box', {'size': f'{sx} {sy} {sz}'}, pose='0 0 -0.035 0 0 0')
    for z in (.04, -.11):
        part(mount, 'bolt_' + str(z), 'cylinder', {'radius': .004, 'length': .002}, f'{sx/2+.001} 0 {z} 0 {math.pi/2} 0', '.55 .55 .57 1', False)
    arm = link(m, 'yaw_arm_link', .09, (0, 0, 0), (.00015, .0003, .0003))
    elem(arm.find('inertial'), 'pose', '.045 .025 0 0 0 0')
    part(arm, 'yaw_motor', 'cylinder', {'radius': .023, 'length': .025}, f'.020 0 0 0 {math.pi/2} 0')
    cx, cy, cz = c['camera_center_m']
    side_y = c['camera_radius_m'] + .013
    part(arm, 'single_side_arm', 'box', {'size': f'{cx-.02} .014 .025'}, f'{(cx+.02)/2} {side_y} 0 0 0 0')
    part(arm, 'arm_root', 'box', {'size': f'.018 {side_y} .025'}, f'.026 {side_y/2} 0 0 0 0')
    part(arm, 'pitch_motor', 'cylinder', {'radius': .017, 'length': .018}, f'{cx} {side_y} {cz} {math.pi/2} 0 0')
    part(arm, 'pitch_axle', 'cylinder', {'radius': .008, 'length': .014}, f'{cx} {side_y-.013} {cz} {math.pi/2} 0 0', collision=False)
    r = c['camera_radius_m']
    camera = link(m, 'camera_link', .1, (cx, cy, cz), (2*.1*r*r/5,)*3)
    part(camera, 'camera_shell', 'sphere', {'radius': r})
    part(camera, 'lens_bezel', 'cylinder', {'radius': .017, 'length': .008}, f'0 0 {r-.003} 0 0 0', '.20 .20 .21 1', False)
    part(camera, 'lens_glass', 'cylinder', {'radius': .013, 'length': .001}, f'0 0 {r+.0015} 0 0 0', '.12 .22 .30 1', False)
    sensor = elem(camera, 'sensor', name='camera', type='camera')
    elem(sensor, 'pose', f'0 0 {r+.002} {math.pi} {-math.pi/2} 0')
    elem(sensor, 'always_on', 'true')
    elem(sensor, 'update_rate', c['camera']['fps'])
    cam = elem(sensor, 'camera')
    elem(cam, 'horizontal_fov', math.radians(c['camera']['hfov_deg']))
    image = elem(cam, 'image')
    for k in ('width', 'height'):
        elem(image, k, c['camera'][k])
    elem(image, 'format', 'R8G8B8')
    clip = elem(cam, 'clip')
    elem(clip, 'near', c['camera']['near_m'])
    elem(clip, 'far', c['camera']['far_m'])
    joint(m, 'yaw_joint', 'mount_link', 'yaw_arm_link', '1 0 0', c['yaw_limits_deg'], c['max_rate_dps'])
    joint(m, 'pitch_joint', 'yaw_arm_link', 'camera_link', '0 1 0', c['pitch_limits_deg'], c['max_rate_dps'])
    plugin = elem(m, 'plugin', filename='gz-two-axis-pod-controller', name='two_axis::GimbalController')
    settings = dict(yaw_joint='yaw_joint', pitch_joint='pitch_joint', mount_link='mount_link', camera_link='camera_link', camera_sensor='camera', command_topic='/two_axis_pod/command', telemetry_topic='/two_axis_pod/telemetry', command_result_topic='/two_axis_pod/command_result', control_rate=100, telemetry_rate=50, max_rate_dps=c['max_rate_dps'], max_acc_dps2=c['max_acc_dps2'], yaw_min_deg=c['yaw_limits_deg'][0], yaw_max_deg=c['yaw_limits_deg'][1], pitch_min_deg=c['pitch_limits_deg'][0], pitch_max_deg=c['pitch_limits_deg'][1])
    for k,v in settings.items():
        elem(plugin, k, v)
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(ROOT / 'model.sdf', encoding='utf-8', xml_declaration=True)
    world = ET.parse(ROOT / 'tests/world_base.sdf')
    fixture_pod = copy.deepcopy(m)
    elem(fixture_pod, 'pose', '0 0 1.5 0 0 0')
    fixed = elem(fixture_pod, 'joint', name='world_mount', type='fixed')
    elem(fixed, 'parent', 'world')
    elem(fixed, 'child', 'mount_link')
    world.getroot().find('world').append(fixture_pod)
    target = world.getroot().find("world/model[@name='tracking_target']/link")
    # 非对称纹理便于选框和观察画面旋转。
    for row in range(5):
        for col in range(5):
            if (row + col) % 2 == 0:
                part(target, f'tile_{row}_{col}', 'box', {'size': '.085 .085 .003'},
                     f'{(row-2)*.105} {(col-2)*.105} -.007 0 0 0',
                     '.08 .12 .18 1', False)
    part(target, 'orientation_mark', 'box', {'size': '.06 .18 .003'},
         '.27 .17 -.008 0 0 0', '.8 .15 .08 1', False)
    ET.indent(world, space='  ')
    world.write(ROOT / 'tests/world.sdf', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    generate()
