#!/usr/bin/env python3
"""组合火箭机与两轴吊舱的实际视觉资产，生成静态造型评审模型。"""
from __future__ import annotations

import argparse
import copy
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
ROCKET = REPO / 'models/vehicles/rocket_racing_quad'
POD = REPO / 'models/gimbals/two_axis_pod'
NAME = 'rocket_two_axis_pod'


def node(parent, tag, value=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if value is not None:
        child.text = str(value)
    return child


def write_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def configuration():
    return tuple(json.loads(p.read_text()) for p in
                 (ROOT / 'parameters.json', ROCKET / 'parameters.json', POD / 'parameters.json'))


def dimensions():
    assembly, rocket, pod = configuration()
    length = assembly['overall_length_m']
    if not math.isclose(length, rocket['geometry']['body_length_m']):
        raise ValueError('静态装配保留原机尾、电机与翼面位置，总长必须与原机身长度一致')
    mount_z = length / 2 - pod['overall_height_m']
    shoulder_start = mount_z - assembly['shoulder_length_m']
    if not -length / 2 < shoulder_start < mount_z:
        raise ValueError('过渡肩部必须位于保留的机身内')
    if mount_z <= -length / 2 + rocket['geometry']['fin_root_length_m']:
        raise ValueError('顶部替换不得切入翼根')
    return length, mount_z, shoulder_start


def rocket_mesh_tools():
    spec = importlib.util.spec_from_file_location('rocket_mesh_source', ROCKET / 'tools/generate_meshes.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cut_fuselage():
    """沿用原机身下段剖面，替换头部并过渡至吊舱底环。"""
    _, rocket, pod = configuration()
    _, mount_z, shoulder_start = dimensions()
    tools = rocket_mesh_tools()
    vertices = tools.build_fuselage(rocket['geometry']).reshape(-1, 3)
    rings = {}
    for x, y, z in vertices:
        key = round(float(z), 12)
        rings[key] = max(rings.get(key, 0), float(np.hypot(x, y)))
    source = sorted(rings.items())
    z, radius = np.array(source).T
    start_radius = float(np.interp(shoulder_start, z, radius))
    profile = [(zi, ri) for zi, ri in source if zi < shoulder_start]
    profile.append((shoulder_start, start_radius))
    profile.append((mount_z, pod['housing_diameter_m'] / 2))
    profile.append((mount_z, 0.0))
    return tools.revolve_around_x(profile) @ tools.X_AXIS_TO_Z_AXIS.T


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def transform(pose):
    vals = [float(v) for v in (pose or '0 0 0 0 0 0').split()]
    result = np.eye(4)
    result[:3, :3] = rotation(vals[3:])
    result[:3, 3] = vals[:3]
    return result


def pose_text(matrix):
    r = matrix[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(pitch)) > 1e-8:
        roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    else:
        roll, yaw = 0., math.atan2(-r[0, 1], r[1, 1])
    return ' '.join(f'{v:.12g}' for v in (*matrix[:3, 3], roll, pitch, yaw))


def build_model(yaw_deg=0.0, pitch_deg=0.0):
    _, _, pod = configuration()
    _, mount_z, _ = dimensions()
    for value, key in ((yaw_deg, 'yaw_limits_deg'), (pitch_deg, 'pitch_limits_deg')):
        if not math.isfinite(value) or not pod[key][0] <= value <= pod[key][1]:
            raise ValueError(f'{key} 超出机械限位')
    root = ET.Element('sdf', version='1.9')
    model = node(root, 'model', name=NAME)
    node(model, 'static', 'true')
    link = node(model, 'link', name='appearance_link')
    mount = np.eye(4)
    mount[2, 3] = mount_z
    yaw = transform(f'0 0 0 0 0 {math.radians(yaw_deg)}')
    pitch = transform(f'0 0 0 0 {math.radians(pitch_deg)} 0')
    camera_center = np.eye(4)
    camera_center[:3, 3] = pod['camera_center_m']
    for source_dir, prefix in ((ROCKET, 'rocket'), (POD, 'pod')):
        source = ET.parse(source_dir / 'model.sdf').getroot().find('model')
        for source_link in source.findall('link'):
            link_tf = transform(source_link.findtext('pose'))
            if prefix == 'pod':
                if source_link.get('name') == 'yaw_arm_link':
                    link_tf = mount @ yaw @ link_tf
                elif source_link.get('name') == 'camera_link':
                    link_tf = mount @ yaw @ camera_center @ pitch
                else:
                    link_tf = mount @ link_tf
            for source_visual in source_link.findall('visual'):
                visual = copy.deepcopy(source_visual)
                visual.set('name', f'{prefix}_{source_link.get("name")}_{visual.get("name")}')
                vis_pose = visual.find('pose')
                if vis_pose is None:
                    vis_pose = node(visual, 'pose')
                vis_pose.text = pose_text(link_tf @ transform(source_visual.findtext('pose')))
                if prefix == 'rocket' and source_visual.get('name') == 'fuselage_visual':
                    visual.find('geometry/mesh/uri').text = f'model://{NAME}/meshes/fuselage_cut.stl'
                link.append(visual)
    return root


def marker(world):
    model = node(world, 'model', name='ROCKET_POD_LOCATION_MARKER')
    node(model, 'static', 'true')
    link = node(model, 'link', name='arrow')
    for name, position, size in (
        ('shaft', '-.20 0 .41 0 0 0', '.12 .009 .009'),
        ('head_upper', '-.125 0 .42 0 .7853981634 0', '.04 .009 .009'),
        ('head_lower', '-.125 0 .40 0 -.7853981634 0', '.04 .009 .009'),
    ):
        visual = node(link, 'visual', name=name)
        node(visual, 'pose', position)
        node(node(node(visual, 'geometry'), 'box'), 'size', size)
        mat = node(visual, 'material')
        node(mat, 'ambient', '1 .35 .02 1')
        node(mat, 'diffuse', '1 .35 .02 1')


def build_world(view='overall'):
    assembly, _, _ = configuration()
    root = ET.Element('sdf', version='1.9')
    world = node(root, 'world', name='rocket_two_axis_pod_preview')
    node(world, 'gravity', '0 0 -9.8')
    physics = node(world, 'physics', name='physics', type='ignored')
    node(physics, 'max_step_size', '.004')
    node(physics, 'real_time_factor', '1')
    for filename, name in (('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')):
        node(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
    scene = node(world, 'scene')
    node(scene, 'ambient', '.7 .7 .7 1')
    node(scene, 'background', '.84 .88 .93 1')
    node(scene, 'grid', 'false')
    node(scene, 'shadows', 'true')
    gui = node(world, 'gui', fullscreen='false')
    render = node(gui, 'plugin', filename='MinimalScene', name='3D View')
    node(render, 'engine', 'ogre2')
    node(render, 'scene', 'scene')
    camera = assembly['view']['camera_pose'] if view == 'overall' else [.23, -.28, .54, 0, .3, 2.258]
    node(render, 'camera_pose', ' '.join(str(v) for v in camera))
    clip = node(render, 'camera_clip')
    node(clip, 'near', assembly['view']['near_clip_m'])
    node(clip, 'far', assembly['view']['far_clip_m'])
    for filename, name in (('GzSceneManager', 'Scene Manager'), ('InteractiveViewControl', 'Interactive view control'),
                           ('CameraTracking', 'Camera Tracking'), ('EntityTree', 'Entity tree'), ('WorldControl', 'World control')):
        node(gui, 'plugin', filename=filename, name=name)
    light = node(world, 'light', name='sun', type='directional')
    node(light, 'pose', '0 0 3 0 0 0')
    node(light, 'diffuse', '.8 .8 .8 1')
    node(light, 'specular', '.3 .3 .3 1')
    node(light, 'direction', '-.4 .5 -1')
    include = node(world, 'include')
    node(include, 'uri', f'model://{NAME}')
    node(include, 'name', 'ROCKET_TWO_AXIS_POD')
    node(include, 'pose', '0 0 .24 0 0 0')
    floor = node(world, 'model', name='preview_floor')
    node(floor, 'static', 'true')
    visual = node(node(floor, 'link', name='floor'), 'visual', name='floor')
    plane = node(node(visual, 'geometry'), 'plane')
    node(plane, 'normal', '0 0 1')
    node(plane, 'size', '4 4')
    material = node(visual, 'material')
    node(material, 'ambient', '.72 .74 .77 1')
    node(material, 'diffuse', '.72 .74 .77 1')
    marker(world)
    return root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=ROOT)
    parser.add_argument('--yaw-deg', type=float)
    parser.add_argument('--pitch-deg', type=float)
    args = parser.parse_args()
    assembly, _, _ = configuration()
    yaw = assembly['yaw_deg'] if args.yaw_deg is None else args.yaw_deg
    pitch = assembly['pitch_deg'] if args.pitch_deg is None else args.pitch_deg
    root = build_model(yaw, pitch)
    rocket_mesh_tools().write_stl(args.output_dir / 'meshes/fuselage_cut.stl', cut_fuselage())
    write_xml(args.output_dir / 'model.sdf', root)
    config = ET.Element('model')
    node(config, 'name', NAME)
    node(config, 'version', '1.0')
    node(config, 'sdf', 'model.sdf', version='1.9')
    node(config, 'description', '450 mm 火箭机与顶部两轴吊舱静态造型预览；无飞行动力学与活动控制。')
    write_xml(args.output_dir / 'model.config', config)
    for view, filename in (('overall', 'world.sdf'), ('close', 'closeup.sdf')):
        write_xml(args.output_dir / 'tests' / filename, build_world(view))
    print(f'造型资产：{args.output_dir.resolve()}；yaw={yaw:g}°，pitch={pitch:g}°')
    print(f'总长 {dimensions()[0]*1000:g} mm；底环安装面 z={dimensions()[1]*1000:g} mm')


if __name__ == '__main__':
    main()
