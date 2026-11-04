#!/usr/bin/env python3
"""组合原生火箭机与中科D80，生成顶部水平安装的静态造型资产。"""
from __future__ import annotations

import argparse
import copy
import configparser
import shutil
import importlib.util
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
ROCKET = REPO / 'models/vehicles/rocket_racing_quad'
POD = REPO / 'models/gimbals/zhongke-d80'
NAME = 'rocket_zhongke_d80'
_kinematics_spec = importlib.util.spec_from_file_location('rocket_pod_kinematics', ROOT / 'tools/pod_kinematics.py')
kinematics = importlib.util.module_from_spec(_kinematics_spec)
_kinematics_spec.loader.exec_module(kinematics)


def node(parent, tag, value=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if value is not None:
        child.text = str(value)
    return child


def write_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def load_component_module():
    spec = importlib.util.spec_from_file_location('rocket_zhongke_component_generator', POD / 'tools/generate_model.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


component = load_component_module()


def component_configuration(config_path=None):
    parser = configparser.ConfigParser(interpolation=None)
    parser.read(ROOT / 'parameters.ini')
    path = Path(config_path) if config_path else Path(parser['component']['parameters_file'])
    if not path.is_absolute():
        path = ROOT / path
    return component.load_config(path)


def component_model(config_path=None):
    c = component_configuration(config_path)
    parameters = component.to_model_parameters(c)
    _, model = component.create_model(c, parameters, component.load_geometry())
    return c, parameters, model


def configuration(config_path=None):
    parser = configparser.ConfigParser(interpolation=None, strict=True)
    with (ROOT / 'parameters.ini').open() as stream:
        parser.read_file(stream)
    assembly = {key: parser.getfloat('assembly', key) for key in
                ('overall_length_m', 'shoulder_length_m', 'yaw_deg', 'pitch_deg')}
    assembly['dynamic'] = dict(airframe_id=parser.getint('dynamic', 'airframe_id'),
                               topic_prefix=parser.get('dynamic', 'topic_prefix'))
    assembly['view'] = {key: ([float(x) for x in value.split(',')] if key.endswith('pose') else float(value))
                        for key, value in parser['view'].items()}
    assembly['px4'] = {'parameters': {key.upper(): float(value) for key, value in parser['px4'].items()}}
    stability = {}
    for key, value in parser['stability_validation'].items():
        stability[key] = ([float(x) for x in value.split(',')] if ',' in value else
                          parser.getboolean('stability_validation', key) if value in ('true', 'false') else float(value))
    assembly['stability_validation'] = stability
    for key, value in assembly.items():
        if isinstance(value, (int, float)) and not math.isfinite(value):
            raise ValueError(f'装配配置{key}必须有限')
    return assembly, source_geometry(), component.to_model_parameters(component_configuration(config_path))


def read_stl(path):
    """读取当前资产，不从基础模型的离线生成参数推导几何。"""
    raw = path.read_bytes()
    if len(raw) >= 84:
        count = struct.unpack_from('<I', raw, 80)[0]
        if len(raw) == 84 + count * 50:
            dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
            return np.frombuffer(raw, dtype=dtype, count=count, offset=84)['vertices'].astype(float)
    vertices = [list(map(float, line.split()[1:])) for line in raw.decode('ascii').splitlines()
                if line.strip().startswith('vertex ')]
    result = np.asarray(vertices).reshape(-1, 3, 3)
    if not len(result) or not np.isfinite(result).all():
        raise ValueError(f'无效的 STL 网格：{path}')
    return result


def source_mesh(visual_name):
    """返回实际基础视觉网格在模型坐标系中的三角面，以及原网格变换。"""
    source = ET.parse(ROCKET / 'model.sdf').getroot().find('model')
    matches = [(link, visual) for link in source.findall('link')
               for visual in link.findall('visual') if visual.get('name') == visual_name]
    if len(matches) != 1:
        raise ValueError(f'基础模型必须有唯一的 {visual_name}')
    link, visual = matches[0]
    mesh = visual.find('geometry/mesh')
    uri = mesh.findtext('uri')
    if uri.startswith('model://rocket_racing_quad/'):
        path = ROCKET / uri.split('/', 3)[3]
    else:
        path = Path(uri.removeprefix('file://'))
        if not path.is_absolute():
            path = ROCKET / path
    if path.suffix.lower() != '.stl':
        raise ValueError(f'装配裁切目前需要基础 STL 资产：{path}')
    scale = np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
    if len(scale) != 3 or not np.isfinite(scale).all() or np.any(scale <= 0):
        raise ValueError('基础网格缩放必须是三个正数')
    pose = transform(link.findtext('pose')) @ transform(visual.findtext('pose'))
    vertices = read_stl(path) * scale
    return vertices @ pose[:3, :3].T + pose[:3, 3], pose, scale


def source_geometry():
    """读取原生机身和尾翼的实际包络；机身原点不随总长变化而移动。"""
    body, _, _ = source_mesh('fuselage_visual')
    tail, _, _ = source_mesh('tail_visual')
    min_z, max_z = float(body[:, :, 2].min()), float(body[:, :, 2].max())
    return {'min_z_m': min_z, 'max_z_m': max_z, 'length_m': max_z - min_z,
            'fin_max_z_m': float(tail[:, :, 2].max())}


def dimensions():
    assembly, rocket, pod = configuration()
    length = assembly['overall_length_m']
    if not math.isclose(length, rocket['length_m'], abs_tol=1e-9):
        raise ValueError('静态装配保留原机尾、电机与翼面位置，总长必须与原机身长度一致')
    mount_z = rocket['max_z_m'] - pod['overall_height_m']
    shoulder_start = mount_z - assembly['shoulder_length_m']
    if not rocket['min_z_m'] < shoulder_start < mount_z:
        raise ValueError('过渡肩部必须位于保留的机身内')
    if shoulder_start <= rocket['fin_max_z_m']:
        raise ValueError('顶部替换不得切入翼根')
    return length, mount_z, shoulder_start


def rocket_mesh_tools():
    spec = importlib.util.spec_from_file_location('rocket_mesh_source', ROCKET / 'tools/generate_meshes.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cut_fuselage():
    """裁切真实机身三角面，保留下段并以肩部接到底环；返回原网格局部坐标。"""
    _, _, pod = configuration()
    _, mount_z, shoulder_start = dimensions()
    vertices, source_pose, source_scale = source_mesh('fuselage_visual')
    triangles, boundary = [], {}
    for triangle in vertices:
        polygon = []
        for i, point in enumerate(triangle):
            following = triangle[(i + 1) % 3]
            inside, next_inside = point[2] <= shoulder_start, following[2] <= shoulder_start
            if inside:
                polygon.append(point)
            if inside != next_inside:
                crossing = point + (following - point) * ((shoulder_start - point[2]) /
                                                           (following[2] - point[2]))
                crossing[2] = shoulder_start
                polygon.append(crossing)
        for point in polygon:
            if abs(point[2] - shoulder_start) < 1e-10:
                boundary[tuple(np.round(point, 11))] = point
        for i in range(1, len(polygon) - 1):
            candidate = np.asarray([polygon[0], polygon[i], polygon[i + 1]])
            if np.linalg.norm(np.cross(candidate[1] - candidate[0], candidate[2] - candidate[0])) > 1e-14:
                triangles.append(candidate)
    if len(boundary) < 3:
        raise ValueError('机身裁切平面没有闭合轮廓')
    ring = np.asarray(sorted(boundary.values(), key=lambda point: math.atan2(point[1], point[0])))
    radii = np.linalg.norm(ring[:, :2], axis=1)
    if radii.min() <= 0 or np.max(np.abs(np.mean(ring[:, :2], axis=0))) > 1e-5:
        raise ValueError('当前同轴安装要求裁切轮廓以机体 Z 轴为中心')
    upper = ring.copy()
    upper[:, :2] *= (pod['housing_diameter_m'] / 2 / radii)[:, None]
    upper[:, 2] = mount_z
    center = np.array([0., 0., mount_z])
    for i in range(len(ring)):
        j = (i + 1) % len(ring)
        triangles.extend(([ring[i], ring[j], upper[j]], [ring[i], upper[j], upper[i]],
                          [center, upper[i], upper[j]]))
    # 保持 source visual 的 pose / scale，避免装配侧改变原生坐标和缩放。
    return ((np.asarray(triangles) - source_pose[:3, 3]) @ source_pose[:3, :3]) / source_scale


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


def build_model(yaw_deg=0.0, pitch_deg=0.0, config_path=None):
    _, pod, source_pod = component_model(config_path)
    _, mount_z, _ = dimensions()
    for value, key in ((yaw_deg, 'yaw_limits_deg'), (pitch_deg, 'pitch_limits_deg')):
        if not math.isfinite(value) or not pod[key][0] <= value <= pod[key][1]:
            raise ValueError(f'{key} 超出机械限位')
    root = ET.Element('sdf', version='1.10')
    model = node(root, 'model', name=NAME)
    node(model, 'static', 'true')
    link = node(model, 'link', name='appearance_link')
    mount = np.eye(4)
    mount[2, 3] = mount_z
    pod_poses = kinematics.PodKinematics(model=source_pod).link_poses(yaw_deg, pitch_deg)
    for source_dir, prefix in ((ROCKET, 'rocket'), (POD, 'pod')):
        source = source_pod if prefix == 'pod' else ET.parse(source_dir / 'model.sdf').getroot().find('model')
        for source_link in source.findall('link'):
            link_tf = transform(source_link.findtext('pose'))
            if prefix == 'pod':
                link_tf = mount @ pod_poses[source_link.get('name')]
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


def preview_spawn_z():
    return -source_geometry()['min_z_m'] + .015


def marker(world):
    model = node(world, 'model', name='ROCKET_ZHONGKE_D80_LOCATION_MARKER')
    node(model, 'static', 'true')
    link = node(model, 'link', name='arrow')
    height = preview_spawn_z() + dimensions()[1] + .036
    for name, position, size in (
        ('shaft', f'-.20 0 {height:.12g} 0 0 0', '.12 .009 .009'),
        ('head_upper', f'-.125 0 {height + .01:.12g} 0 .7853981634 0', '.04 .009 .009'),
        ('head_lower', f'-.125 0 {height - .01:.12g} 0 -.7853981634 0', '.04 .009 .009'),
    ):
        visual = node(link, 'visual', name=name)
        node(visual, 'pose', position)
        node(node(node(visual, 'geometry'), 'box'), 'size', size)
        mat = node(visual, 'material')
        node(mat, 'ambient', '1 .35 .02 1')
        node(mat, 'diffuse', '1 .35 .02 1')


def build_world(view='overall'):
    assembly, _, _ = configuration()
    root = ET.Element('sdf', version='1.10')
    world = node(root, 'world', name='rocket_zhongke_d80_preview')
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
    camera = assembly['view']['camera_pose' if view == 'overall' else 'close_camera_pose']
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
    node(include, 'name', 'ROCKET_ZHONGKE_D80')
    node(include, 'pose', f'0 0 {preview_spawn_z():.12g} 0 0 0')
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


def copy_meshes(root, output):
    """将引用网格复制到新装配目录，模型URI不再依赖旧装配。"""
    for uri in root.findall('.//mesh/uri'):
        for source_name, prefix, source in (('rocket_racing_quad', 'rocket', ROCKET),
                                             ('zhongke-d80', 'pod', POD)):
            source_prefix = f'model://{source_name}/meshes/'
            if uri.text.startswith(source_prefix):
                relative = uri.text[len(source_prefix):]
                uri.text = f'model://{NAME}/meshes/{prefix}_{relative}'
        relative = uri.text.removeprefix(f'model://{NAME}/meshes/')
        if relative == uri.text:
            raise ValueError(f'装配网格URI未闭合：{uri.text}')
        for prefix, source in (('rocket_', ROCKET), ('pod_', POD)):
            if relative.startswith(prefix):
                destination = Path(output) / 'meshes' / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source / 'meshes' / relative[len(prefix):], destination)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=ROOT)
    parser.add_argument('--config', type=Path, help='完整中科性能INI，不修改基础组件')
    parser.add_argument('--yaw-deg', type=float)
    parser.add_argument('--pitch-deg', type=float)
    args = parser.parse_args()
    output = args.output_dir.resolve()
    if output != ROOT and not output.is_relative_to(ROOT):
        raise ValueError('静态输出必须位于本装配目录')
    assembly, _, _ = configuration(args.config)
    yaw = assembly['yaw_deg'] if args.yaw_deg is None else args.yaw_deg
    pitch = assembly['pitch_deg'] if args.pitch_deg is None else args.pitch_deg
    root = build_model(yaw, pitch, args.config)
    rocket_mesh_tools().write_stl(output / 'meshes/fuselage_cut.stl', cut_fuselage())
    copy_meshes(root, output)
    write_xml(output / 'model.sdf', root)
    config = ET.Element('model')
    node(config, 'name', NAME)
    node(config, 'version', '1.0')
    node(config, 'sdf', 'model.sdf', version='1.10')
    node(config, 'description', f'{dimensions()[0]*1000:g} mm 火箭机与顶部中科D80静态造型预览；无飞行动力学与活动控制。')
    write_xml(output / 'model.config', config)
    for view, filename in (('overall', 'world.sdf'), ('close', 'closeup.sdf')):
        write_xml(output / 'tests' / filename, build_world(view))
    print(f'造型资产：{output}；yaw={yaw:g}°，pitch={pitch:g}°')
    print(f'总长 {dimensions()[0]*1000:g} mm；底环安装面 z={dimensions()[1]*1000:g} mm')


if __name__ == '__main__':
    main()
