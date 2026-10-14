#!/usr/bin/env python3
"""生成保留真实关节、传感器和 PX4 动力接口的 450 mm 动态装配模型。"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
import shutil
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
ROCKET = REPO / 'models/vehicles/rocket_racing_quad'
POD = REPO / 'models/gimbals/two_axis_pod'
NAME = 'rocket_two_axis_pod'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


appearance = load_module('rocket_pod_appearance', ROOT / 'tools/generate_model.py')
mesh_tools = appearance.rocket_mesh_tools()
# 基础生成器使用同目录 import；暂存模块以便被其他工具导入时不污染它们。
previous_mesh_module = sys.modules.get('generate_meshes')
sys.modules['generate_meshes'] = mesh_tools
try:
    rocket_tools = load_module('rocket_pod_vehicle', ROCKET / 'tools/generate_model.py')
finally:
    if previous_mesh_module is None:
        del sys.modules['generate_meshes']
    else:
        sys.modules['generate_meshes'] = previous_mesh_module


def set_text(parent, tag, value):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def settings(body_mass_kg=None, airframe_id=None, topic_prefix=None):
    assembly, rocket, pod = appearance.configuration()
    rocket_tools.validate(rocket)
    length, mount_z, _ = appearance.dimensions()
    retained_length = length / 2 + mount_z
    dynamic = assembly.get('dynamic', {})
    configured_mass = dynamic.get('body_mass_kg')
    mass = body_mass_kg if body_mass_kg is not None else configured_mass
    if mass is None:
        mass = rocket['mass']['body_kg'] * retained_length / length
    if isinstance(mass, bool) or not isinstance(mass, (int, float)) or not math.isfinite(mass) or mass <= 0:
        raise ValueError('body_mass_kg 必须为有限正数')
    airframe = airframe_id if airframe_id is not None else dynamic.get('airframe_id', 4023)
    if isinstance(airframe, bool) or not isinstance(airframe, int) or airframe <= 0:
        raise ValueError('airframe_id 必须为正整数')
    prefix = topic_prefix if topic_prefix is not None else dynamic.get('topic_prefix', f'/{NAME}')
    if not isinstance(prefix, str) or not re.fullmatch(r'/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*', prefix):
        raise ValueError('topic_prefix 必须为绝对 Gazebo 话题前缀，不含末尾斜线')
    idle = rocket['motor']['idle_velocity_rad_s']
    maximum = rocket['motor']['max_velocity_rad_s']
    if (isinstance(idle, bool) or not isinstance(idle, (int, float)) or
            not math.isfinite(idle) or idle != int(idle) or not 0 < idle < maximum):
        raise ValueError('基础火箭模型的 idle_velocity_rad_s 必须为小于电机最大转速的正整数')
    return dict(assembly=assembly, rocket=rocket, pod=pod, mount_z_m=mount_z,
                retained_body_length_m=retained_length, body_mass_kg=float(mass),
                body_mass_method='override' if body_mass_kg is not None or configured_mass is not None else 'retained_length_ratio',
                airframe_id=airframe, topic_prefix=prefix, armed_idle_velocity_rad_s=int(idle))


def build_model(config):
    root = rocket_tools.build_model(config['rocket'])
    model = root.find('model')
    model.set('name', NAME)
    model.set('canonical_link', 'base_link')
    set_text(model, 'static', 'false')
    base = model.find("link[@name='base_link']")
    retained = config['retained_body_length_m']
    center_z = config['mount_z_m'] - retained / 2
    mass = config['body_mass_kg']
    radius = config['rocket']['geometry']['body_radius_m']
    inertia = base.find('inertial')
    set_text(inertia, 'mass', mass)
    set_text(inertia, 'pose', f'0 0 {center_z:.12g} 0 0 0')
    transverse = mass * (3 * radius**2 + retained**2) / 12
    for axis, value in (('ixx', transverse), ('iyy', transverse), ('izz', mass * radius**2 / 2)):
        set_text(inertia.find('inertia'), axis, value)
    collision = base.find("collision[@name='body_collision']")
    set_text(collision, 'pose', f'0 0 {center_z:.12g} 0 0 0')
    set_text(collision.find('geometry/cylinder'), 'length', retained)
    base.find("visual[@name='fuselage_visual']/geometry/mesh/uri").text = f'model://{NAME}/meshes/fuselage_cut.stl'
    pod_model = ET.parse(POD / 'model.sdf').getroot().find('model')
    for child in pod_model:
        if child.tag not in ('link', 'joint', 'plugin'):
            continue
        element = copy.deepcopy(child)
        if element.tag == 'link':
            pose = element.find('pose')
            if pose is not None and pose.get('relative_to'):
                raise ValueError('吊舱 link 位姿必须相对模型坐标系')
            transform = appearance.transform(element.findtext('pose'))
            transform[2, 3] += config['mount_z_m']
            set_text(element, 'pose', appearance.pose_text(transform))
        elif element.tag == 'plugin' and element.get('name') == 'two_axis::GimbalController':
            set_text(element, 'mount_dynamic', 'true')
            set_text(element, 'image_topic', config['topic_prefix'] + '/image')
            for key in ('command', 'telemetry', 'command_result'):
                set_text(element, key + '_topic', config['topic_prefix'] + '/' + key)
        model.append(element)
    fixed = ET.SubElement(model, 'joint', name='pod_mount_joint', type='fixed')
    set_text(fixed, 'parent', 'base_link')
    set_text(fixed, 'child', 'mount_link')
    camera = model.find("link[@name='camera_link']/sensor[@name='camera']")
    set_text(camera, 'topic', config['topic_prefix'] + '/image')
    # 输出包含实际使用的基础网格，可作为独立 model 放入 Gazebo 资源路径。
    for uri in model.findall('.//mesh/uri'):
        for name, prefix in (('rocket_racing_quad', 'rocket'), ('two_axis_pod', 'pod')):
            source_prefix = f'model://{name}/meshes/'
            if uri.text.startswith(source_prefix):
                uri.text = f'model://{NAME}/meshes/{prefix}_{uri.text[len(source_prefix):]}'
    return root


def source_fingerprints():
    files = [ROOT / 'parameters.json', ROOT / 'tools/generate_model.py',
             ROCKET / 'parameters.json', ROCKET / 'model.sdf',
             ROCKET / 'tools/generate_model.py', ROCKET / 'tools/generate_meshes.py',
             POD / 'parameters.json', POD / 'model.sdf', POD / 'tools/generate_model.py']
    files.extend(sorted((ROCKET / 'meshes').glob('*')))
    files.extend(sorted((POD / 'meshes').glob('*')))
    return {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files if path.is_file()}


def generate(output_dir, *, body_mass_kg=None, airframe_id=None, topic_prefix=None):
    output = Path(output_dir).resolve()
    if output == ROOT.resolve() or output.is_relative_to((REPO / 'models').resolve()):
        raise ValueError('动态模型不得覆盖已确认的静态装配或基础组件目录')
    config = settings(body_mass_kg, airframe_id, topic_prefix)
    before = source_fingerprints()
    root = build_model(config)
    total_mass = sum(float(link.findtext('inertial/mass')) for link in root.findall('model/link'))
    flight = copy.deepcopy(config['rocket'])
    flight['mass']['body_kg'] = config['body_mass_kg']
    idle = config['armed_idle_velocity_rad_s']
    maximum = flight['motor']['max_velocity_rad_s']
    flight['motor']['idle_velocity_rad_s'] = idle
    flight['px4']['airframe_id'] = config['airframe_id']
    if flight['px4']['parameters'].get('THR_MDL_FAC', 0) != 1:
        raise ValueError('装配悬停推力归一化需要 THR_MDL_FAC=1')
    required_velocity = math.sqrt(total_mass * 9.8 / (4 * flight['motor']['thrust_constant']))
    if not idle < required_velocity < maximum:
        raise ValueError('悬停所需转速必须位于解锁怠速与最大转速之间')
    # PX4 FunctionMotors 将推力开方，再由 MixingOutput 映射到 [MIN, MAX]。
    # 保持实际悬停转速不变，需要把非零解锁怠速从输出区间中扣除。
    flight['px4']['parameters']['MPC_THR_HOVER'] = ((required_velocity - idle) / (maximum - idle)) ** 2
    for channel in range(1, 5):
        flight['px4']['parameters'][f'SIM_GZ_EC_DIS{channel}'] = 0
    # 所有参数先验证，通过后才写运行资产；不可飞行的质量不能留下部分结果。
    rocket_tools.validate(flight)
    flight['assembly'] = dict(model_name=NAME, mount_z_m=config['mount_z_m'],
                              retained_body_length_m=config['retained_body_length_m'],
                              pod_mass_kg=sum(float(link.findtext('inertial/mass'))
                                              for link in root.findall('model/link')
                                              if link.get('name') in ('mount_link', 'yaw_arm_link', 'camera_link')),
                              total_mass_kg=total_mass)
    output.mkdir(parents=True, exist_ok=True)
    meshes = output / 'meshes'
    meshes.mkdir(exist_ok=True)
    mesh_tools.write_stl(meshes / 'fuselage_cut.stl', appearance.cut_fuselage())
    # 仅复制模型引用的网格；不修改基础组件资产。
    for uri in root.findall('.//mesh/uri'):
        relative = uri.text.removeprefix(f'model://{NAME}/meshes/')
        for prefix, source in (('rocket_', ROCKET), ('pod_', POD)):
            if relative.startswith(prefix):
                shutil.copyfile(source / 'meshes' / relative[len(prefix):], meshes / relative)
    appearance.write_xml(output / 'model.sdf', root)
    model_config = ET.Element('model')
    for tag, value in (('name', NAME), ('version', '1.0'),
                       ('description', '450 mm 火箭穿越机与顶部两轴吊舱动态装配，包含 PX4 动力接口与真实云台关节。')):
        set_text(model_config, tag, value)
    ET.SubElement(model_config, 'sdf', version='1.9').text = 'model.sdf'
    appearance.write_xml(output / 'model.config', model_config)
    (output / 'parameters.json').write_text(json.dumps(flight, indent=2, ensure_ascii=False) + '\n')
    text = rocket_tools.airframe(flight).replace('gz_rocket_racing_quad', f'gz_{NAME}')
    text = text.replace('# @name Gazebo rocket racing quad', '# @name Gazebo rocket with two axis pod')
    px4_dir = output / 'px4'
    px4_dir.mkdir(exist_ok=True)
    airframe_path = px4_dir / f"{config['airframe_id']}_gz_{NAME}"
    airframe_path.write_text(text)
    after = source_fingerprints()
    if before != after:
        raise RuntimeError('生成期间基础组件或装配参数发生变动，请重新生成')
    manifest = {key: config[key] for key in ('mount_z_m', 'retained_body_length_m', 'body_mass_kg',
                                           'body_mass_method', 'airframe_id', 'topic_prefix',
                                           'armed_idle_velocity_rad_s')}
    manifest.update(model_name=NAME, total_mass_kg=total_mass,
                    mass_approximation='截短机身采用均匀圆柱惯量与截短中心质心；其余动力舱、旋翼、吊舱沿用组件参数。',
                    image_topic=config['topic_prefix'] + '/image',
                    px4_hover_thrust=flight['px4']['parameters']['MPC_THR_HOVER'],
                    hover_motor_velocity_rad_s=required_velocity,
                    disarmed_velocity_rad_s=0,
                    armed_idle_total_thrust_n=4 * flight['motor']['thrust_constant'] * idle**2,
                    source_sha256=before)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'runtime' / NAME)
    parser.add_argument('--body-mass-kg', type=float)
    parser.add_argument('--airframe-id', type=int)
    parser.add_argument('--topic-prefix')
    args = parser.parse_args()
    manifest = generate(args.output_dir, body_mass_kg=args.body_mass_kg,
                        airframe_id=args.airframe_id, topic_prefix=args.topic_prefix)
    print(f"动态装配：{args.output_dir.resolve()}；质量 {manifest['total_mass_kg']:.6g} kg；PX4 airframe {manifest['airframe_id']}")


if __name__ == '__main__':
    main()
