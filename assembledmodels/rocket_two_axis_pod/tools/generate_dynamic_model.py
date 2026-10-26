#!/usr/bin/env python3
"""从当前原生火箭机与吊舱资产生成 590 mm 动态装配及 PX4 运行配置。"""
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
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
ROCKET = REPO / 'models/vehicles/rocket_racing_quad'
POD = REPO / 'models/gimbals/two_axis_pod'
NAME = 'rocket_two_axis_pod'
MOTOR = 'gz::sim::systems::MulticopterMotorModel'
AERO = 'gz::sim::systems::AdvancedLiftDrag'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


appearance = load_module('rocket_pod_appearance', ROOT / 'tools/generate_model.py')
mesh_tools = appearance.rocket_mesh_tools()
native = load_module('rocket_native_runtime', ROCKET / 'tools/prepare_runtime.py')


def set_text(parent, tag, value):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def settings(body_mass_kg=None, airframe_id=None, topic_prefix=None):
    assembly, _, pod = appearance.configuration()
    geometry = appearance.source_geometry()
    length, mount_z, _ = appearance.dimensions()
    source = ET.parse(ROCKET / 'model.sdf').getroot()
    base = source.find("model/link[@name='base_link']")
    mass = native.number(base.findtext('inertial/mass'))
    configured_mass = assembly.get('dynamic', {}).get('body_mass_kg')
    for value in (body_mass_kg, configured_mass):
        if value is not None and (isinstance(value, bool) or not isinstance(value, (int, float)) or
                                  not math.isfinite(value) or not math.isclose(value, mass, abs_tol=1e-12)):
            raise ValueError('当前装配完整继承基础质量与惯量，不能单独覆盖 body_mass_kg')
    dynamic = assembly.get('dynamic', {})
    airframe = airframe_id if airframe_id is not None else dynamic.get('airframe_id', 4023)
    if isinstance(airframe, bool) or not isinstance(airframe, int) or airframe <= 0:
        raise ValueError('airframe_id 必须为正整数')
    prefix = topic_prefix if topic_prefix is not None else dynamic.get('topic_prefix', f'/{NAME}')
    if not isinstance(prefix, str) or not re.fullmatch(r'/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*', prefix):
        raise ValueError('topic_prefix 必须为绝对 Gazebo 话题前缀，不含末尾斜线')
    airframes = sorted((ROCKET / 'px4').glob('*_gz_rocket_racing_quad'))
    if len(airframes) != 1 or not re.fullmatch(r'[0-9]+_gz_rocket_racing_quad', airframes[0].name):
        raise ValueError('基础模型须包含唯一的原生 PX4 机体文件')
    text = airframes[0].read_bytes().decode('utf-8')
    parameters = native.parameter_values(text)
    overrides = assembly.get('px4', {}).get('parameters', {})
    if not isinstance(overrides, dict):
        raise ValueError('px4.parameters 必须为参数名到有限数值的对象')
    derived = {'MPC_THR_HOVER'} | {
        prefix + axis
        for prefix in ([f'CA_ROTOR{channel}_P' for channel in range(4)] +
                       ['EKF2_IMU_POS_', 'EKF2_GPS_POS_'])
        for axis in 'XYZ'
    }
    for key, value in overrides.items():
        if not isinstance(key, str) or not re.fullmatch(r'[A-Z][A-Z0-9_]{1,30}', key):
            raise ValueError(f'无效的 PX4 参数名：{key!r}')
        if key in derived:
            raise ValueError(f'PX4 参数 {key} 由动态装配自动派生，不能手动覆盖')
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'PX4 参数 {key} 必须为有限数值')
    stability = assembly.get('stability_validation')
    if not isinstance(stability, dict) or not stability:
        raise ValueError('stability_validation 必须为非空对象')
    defaults = json.loads((ROCKET / 'parameters.json').read_text())
    return dict(assembly=assembly, pod=pod, geometry=geometry, source_model=source,
                source_airframe=airframes[0], airframe_text=text, px4_parameters=parameters,
                px4_parameter_overrides=overrides,
                stability_validation=copy.deepcopy(stability),
                flight_test=defaults['flight_test'], imu_wait_timeout_s=defaults['simulation']['imu_wait_timeout_s'],
                overall_length_m=length, mount_z_m=mount_z,
                retained_body_length_m=mount_z - geometry['min_z_m'],
                body_mass_kg=mass, body_mass_method='native_inertial_preserved',
                airframe_id=airframe, topic_prefix=prefix,
                armed_idle_velocity_rad_s=parameters['SIM_GZ_EC_MIN1'])


def build_model(config):
    root = copy.deepcopy(config['source_model'])
    model = root.find('model')
    model.set('name', NAME)
    model.set('canonical_link', 'base_link')
    set_text(model, 'static', 'false')
    base = model.find("link[@name='base_link']")
    collision = base.find("collision[@name='body_collision']")
    pose = [float(value) for value in collision.findtext('pose', '0 0 0 0 0 0').split()]
    pose[2] = (config['geometry']['min_z_m'] + config['mount_z_m']) / 2
    set_text(collision, 'pose', ' '.join(f'{value:.12g}' for value in pose))
    set_text(collision.find('geometry/cylinder'), 'length', f"{config['retained_body_length_m']:.12g}")
    base.find("visual[@name='fuselage_visual']/geometry/mesh/uri").text = f'model://{NAME}/meshes/fuselage_cut.stl'
    # 显示与碰撞截短；原生机身质量、惯性位姿和完整惯量张量保持不变。
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
    for uri in model.findall('.//mesh/uri'):
        for name, prefix in (('rocket_racing_quad', 'rocket'), ('two_axis_pod', 'pod')):
            source_prefix = f'model://{name}/meshes/'
            if uri.text.startswith(source_prefix):
                uri.text = f'model://{NAME}/meshes/{prefix}_{uri.text[len(source_prefix):]}'
    gimbal = model.find("plugin[@name='two_axis::GimbalController']")
    if gimbal is None:
        raise ValueError('动态装配缺少两轴吊舱控制器')
    # 固定零关节名义重心与 PX4 位置参考同源，不使用随关节运动变化的瞬时重心。
    set_text(gimbal, 'los_observer_cg_model',
             ' '.join(format(value, '.17g') for value in center_of_mass(model)))
    return root


def source_fingerprints():
    files = [ROOT / 'parameters.json', ROOT / 'tools/generate_model.py',
             ROOT / 'tools/pod_kinematics.py', Path(__file__),
             ROOT / 'tools/analyze_flight_stability.py',
             ROOT / 'tools/capture_gimbal_telemetry.py',
             ROOT / 'tests/run_flight_check.sh',
             ROCKET / 'parameters.json', ROCKET / 'tools/prepare_runtime.py',
             ROCKET / 'tools/generate_meshes.py', ROCKET / 'tests/flight_check.py',
             ROCKET / 'tests/gazebo_truth.py', POD / 'parameters.json', POD / 'model.sdf']
    controller = POD / 'controller'
    files += [controller / path for path in (
        'CMakeLists.txt', 'service/telemetry.json',
        'include/two_axis/control.hh', 'include/two_axis/control_schedule.hh',
        'include/two_axis/inertial_los.hh',
        'include/two_axis/kinematics.hh', 'include/two_axis/telemetry.hh',
        'include/two_axis/truth_tracking.hh', 'src/control.cc',
        'src/gimbal_controller.cc', 'src/inertial_los.cc', 'src/kinematics.cc', 'src/truth_tracking.cc')]
    shared = POD.parent / 'three_axis_pod/controller'
    files += [shared / path for path in (
        'include/gimbal_controller/kinematics.hh',
        'include/gimbal_controller/mini_json.hh',
        'include/gimbal_controller/tracker.hh', 'src/kinematics.cc',
        'src/mini_json.cc', 'src/tracker.cc')]
    files += native.asset_paths(ROCKET)
    files += sorted((POD / 'meshes').rglob('*'))
    return {str(path.relative_to(REPO)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in files if path.is_file()}


def hover_configuration(model, parameters, gravity):
    if parameters.get('THR_MDL_FAC', 0) != 1:
        raise ValueError('装配悬停推力归一化需要 THR_MDL_FAC=1')
    total_mass = sum(native.number(link.findtext('inertial/mass')) for link in model.findall('link'))
    motors = sorted(model.findall(f"plugin[@name='{MOTOR}']"), key=lambda p: int(p.findtext('motorNumber')))
    if [int(plugin.findtext('motorNumber')) for plugin in motors] != list(range(4)):
        raise ValueError('当前装配需要原生四电机通道 0～3')
    coefficient_a = coefficient_b = idle_thrust = 0.
    limits = []
    for channel, motor in enumerate(motors, 1):
        idle = parameters[f'SIM_GZ_EC_MIN{channel}']
        maximum = parameters[f'SIM_GZ_EC_MAX{channel}']
        constant = native.number(motor.findtext('motorConstant'))
        sdf_maximum = native.number(motor.findtext('maxRotVelocity'))
        if not 0 < idle < maximum <= sdf_maximum or constant <= 0:
            raise ValueError('原生电机怠速、最大转速或推力系数不支持悬停标定')
        delta = maximum - idle
        coefficient_a += constant * delta**2
        coefficient_b += 2 * constant * idle * delta
        idle_thrust += constant * idle**2
        limits.append((idle, maximum, constant))
    weight = total_mass * gravity
    if not idle_thrust < weight < coefficient_a + coefficient_b + idle_thrust:
        raise ValueError('悬停所需推力必须位于原生怠速与最大推力之间')
    command = (2 * (weight - idle_thrust) /
               (coefficient_b + math.sqrt(coefficient_b**2 + 4 * coefficient_a * (weight - idle_thrust))))
    hover = command**2
    if not 0 <= parameters['MPC_THR_MIN'] < hover < parameters['MPC_THR_MAX'] <= 1:
        raise ValueError('组合悬停推力必须位于 PX4 推力上下限之间')
    velocities = [idle + (maximum - idle) * command for idle, maximum, _ in limits]
    return dict(total_mass_kg=total_mass, px4_hover_thrust=hover,
                hover_motor_velocity_rad_s=velocities[0], hover_motor_velocities_rad_s=velocities,
                armed_idle_total_thrust_n=idle_thrust,
                disarmed_velocity_rad_s=parameters['SIM_GZ_EC_DIS1'])


def center_of_mass(model):
    """零关节位姿下的模型坐标系重心；惯性位姿相对所在 link。"""
    total = 0.
    moment = [0., 0., 0.]
    for link in model.findall('link'):
        mass = native.number(link.findtext('inertial/mass'))
        if mass <= 0:
            raise ValueError('装配重心计算需要每个 link 的质量为正')
        for pose in (link.find('pose'), link.find('inertial/pose')):
            if pose is not None and pose.get('relative_to'):
                raise ValueError('装配重心计算需要 link/model 与 inertial/link 的直接位姿')
        transform = (appearance.transform(link.findtext('pose')) @
                     appearance.transform(link.findtext('inertial/pose')))
        for axis in range(3):
            position = float(transform[axis, 3])
            if not math.isfinite(position):
                raise ValueError('装配重心计算需要有限惯性位置')
            moment[axis] += mass * position
        total += mass
    if total <= 0:
        raise ValueError('装配重心计算需要显式 link 质量')
    return [value / total for value in moment]


def px4_reference_configuration(source_model, assembly_model, source_parameters):
    base_center = center_of_mass(source_model)
    assembly_center = center_of_mass(assembly_model)
    translation = [(base_center[axis] - assembly_center[axis]) * sign
                   for axis, sign in enumerate((1., -1., -1.))]
    prefixes = [f'CA_ROTOR{channel}_P' for channel in range(4)] + ['EKF2_IMU_POS_', 'EKF2_GPS_POS_']
    original, derived = {}, {}
    for prefix in prefixes:
        for axis, suffix in enumerate('XYZ'):
            key = prefix + suffix
            if key not in source_parameters:
                raise ValueError(f'原生机体须显式配置重心参考参数 {key}')
            original[key] = source_parameters[key]
            derived[key] = original[key] + translation[axis]
    return dict(reference_pose='zero_joint_positions', base_cg_flu_m=base_center,
                assembly_nominal_cg_flu_m=assembly_center, offset_translation_frd_m=translation,
                source_parameters=original, parameters=derived)


def assembly_airframe(source, hover, reference_parameters, overrides):
    text = source.replace('gz_rocket_racing_quad', f'gz_{NAME}')
    text = text.replace('# @name Gazebo rocket racing quad', '# @name Gazebo rocket with two axis pod')
    for key, value in {'MPC_THR_HOVER': hover, **reference_parameters}.items():
        text, count = re.subn(r'^(param[ \t]+(?:set-default|set)[ \t]+' + re.escape(key) + r'[ \t]+)[^\r\n]+',
                              lambda match: match[1] + format(value, '.16g'), text, flags=re.MULTILINE)
        if count != 1:
            raise ValueError(f'原生机体文件须显式且仅一次配置 {key}')
    additions = []
    for key, value in overrides.items():
        replacement = f'param set-default {key} {format(value, ".16g")}'
        text, count = re.subn(r'^param[ \t]+(?:set-default|set)[ \t]+' + re.escape(key) +
                              r'[ \t]+[^\r\n]+$', replacement, text, flags=re.MULTILINE)
        if count > 1:
            raise ValueError(f'原生机体文件重复配置 PX4 参数 {key}')
        if count == 0:
            additions.append(replacement)
    if additions:
        text = text.rstrip() + '\n# 组合体飞控参数\n' + '\n'.join(additions) + '\n'
    return text


def component_world():
    root = ET.parse(ROCKET / 'tests/world.sdf').getroot()
    world = root.find('world')
    world.set('name', NAME + '_test')
    include = world.find("include[uri='model://rocket_racing_quad']")
    if include is None:
        raise ValueError('基础自验证世界缺少原生火箭模型 include')
    include.find('uri').text = f'model://{NAME}'
    return root


def model_snapshot(model):
    links = {}
    for link in model.findall('link'):
        inertial = link.find('inertial')
        links[link.get('name')] = dict(
            pose=link.findtext('pose', '0 0 0 0 0 0'),
            inertial_pose=inertial.findtext('pose', '0 0 0 0 0 0'),
            mass_kg=native.number(inertial.findtext('mass')),
            inertia={axis: native.number(inertial.findtext(f'inertia/{axis}'))
                     for axis in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')})
    return dict(name=NAME, links=links, total_mass_kg=sum(link['mass_kg'] for link in links.values()),
                motors=[native.plugin_values(p) for p in model.findall(f"plugin[@name='{MOTOR}']")],
                aerodynamics=[native.plugin_values(p) for p in model.findall(f"plugin[@name='{AERO}']")])


def generate(output_dir, *, body_mass_kg=None, airframe_id=None, topic_prefix=None):
    output = Path(output_dir).resolve()
    if output == ROOT.resolve() or ROOT.resolve().is_relative_to(output) or output.is_relative_to((REPO / 'models').resolve()):
        raise ValueError('动态模型不得覆盖已确认的静态装配或基础组件目录')
    before = source_fingerprints()
    config = settings(body_mass_kg, airframe_id, topic_prefix)
    root = build_model(config)
    world = component_world()
    gravity = math.sqrt(sum(float(value)**2 for value in world.findtext('world/gravity').split()))
    hover = hover_configuration(root.find('model'), config['px4_parameters'], gravity)
    reference = px4_reference_configuration(config['source_model'].find('model'), root.find('model'),
                                            config['px4_parameters'])
    los_reference = dict(kind='nominal_cg', reference_pose=reference['reference_pose'],
                         body_offset_flu_m=reference['assembly_nominal_cg_flu_m'])
    airframe_text = assembly_airframe(config['airframe_text'], hover['px4_hover_thrust'],
                                      reference['parameters'], config['px4_parameter_overrides'])
    parameters = native.parameter_values(airframe_text)
    assembly = dict(model_name=NAME, mount_z_m=config['mount_z_m'],
                    retained_body_length_m=config['retained_body_length_m'],
                    pod_mass_kg=sum(native.number(link.findtext('inertial/mass'))
                                    for link in ET.parse(POD / 'model.sdf').getroot().findall('model/link')),
                    total_mass_kg=hover['total_mass_kg'], px4_reference=reference,
                    los_observer_reference=los_reference)
    world_element = world.find('world')
    snapshot = dict(model=model_snapshot(root.find('model')),
                    px4=dict(airframe_id=config['airframe_id'], parameters=parameters),
                    flight_test=copy.deepcopy(config['flight_test']),
                    stability_validation=copy.deepcopy(config['stability_validation']),
                    simulation=dict(imu_wait_timeout_s=config['imu_wait_timeout_s'],
                                    world_name=world_element.get('name'),
                                    physics={child.tag: child.text for child in world_element.find('physics')},
                                    gravity=world_element.findtext('gravity'),
                                    spawn_pose=world_element.findtext(f"include[uri='model://{NAME}']/pose")),
                    assembly=assembly, runtime=dict(source_sha256=before))
    cut_mesh = appearance.cut_fuselage()
    if before != source_fingerprints():
        raise RuntimeError('生成期间基础组件或装配参数发生变动，请重新生成')
    output.mkdir(parents=True, exist_ok=True)
    mesh_tools.write_stl(output / 'meshes/fuselage_cut.stl', cut_mesh)
    for uri in root.findall('.//mesh/uri'):
        relative = uri.text.removeprefix(f'model://{NAME}/meshes/')
        for prefix, source in (('rocket_', ROCKET), ('pod_', POD)):
            if relative.startswith(prefix):
                target = output / 'meshes' / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(source / 'meshes' / relative[len(prefix):], target)
    appearance.write_xml(output / 'model.sdf', root)
    appearance.write_xml(output / 'tests/world.sdf', world)
    model_config = ET.Element('model')
    for tag, value in (('name', NAME), ('version', '1.0'),
                       ('description', f"{config['overall_length_m']*1000:g} mm 火箭穿越机与顶部两轴吊舱动态装配，包含 PX4 动力接口与真实云台关节。")):
        set_text(model_config, tag, value)
    ET.SubElement(model_config, 'sdf', version='1.9').text = 'model.sdf'
    appearance.write_xml(output / 'model.config', model_config)
    (output / 'parameters.json').write_text(json.dumps(snapshot, indent=2, ensure_ascii=False) + '\n')
    px4_dir = output / 'px4'
    px4_dir.mkdir(exist_ok=True)
    airframe_path = px4_dir / f"{config['airframe_id']}_gz_{NAME}"
    airframe_path.write_bytes(airframe_text.encode('utf-8'))
    airframe_path.chmod(config['source_airframe'].stat().st_mode & 0o777)
    if before != source_fingerprints():
        raise RuntimeError('生成期间基础组件或装配参数发生变动，请重新生成')
    manifest = {key: config[key] for key in ('mount_z_m', 'retained_body_length_m', 'body_mass_kg',
                                           'body_mass_method', 'airframe_id', 'topic_prefix',
                                           'armed_idle_velocity_rad_s')}
    manifest.update(model_name=NAME,
                    mass_approximation='完整保留基础机质量、重心和惯量，另加原生吊舱；顶部外形和碰撞截短不扣除机头质量。',
                    image_topic=config['topic_prefix'] + '/image', gravity_m_s2=gravity,
                    source_sha256=before, px4_reference=reference,
                    los_observer_reference=los_reference, **hover)
    (output / 'manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'runtime' / NAME)
    parser.add_argument('--body-mass-kg', type=float, help='只能与原生机身质量相同，完整继承惯性')
    parser.add_argument('--airframe-id', type=int)
    parser.add_argument('--topic-prefix')
    args = parser.parse_args()
    manifest = generate(args.output_dir, body_mass_kg=args.body_mass_kg,
                        airframe_id=args.airframe_id, topic_prefix=args.topic_prefix)
    print(f"动态装配：{args.output_dir.resolve()}；质量 {manifest['total_mass_kg']:.6g} kg；PX4 airframe {manifest['airframe_id']}")


if __name__ == '__main__':
    main()
