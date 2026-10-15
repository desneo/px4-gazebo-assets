#!/usr/bin/env python3
"""从 parameters.json 生成 Gazebo 资产和可安装的 PX4 机体配置。"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import generate_meshes

ROOT = Path(__file__).resolve().parents[1]
NAME = 'rocket_racing_quad'
POSITIONS = ((1, -1), (-1, 1), (1, 1), (-1, -1))


def node(parent, tag, value=None, **attrs):
    result = ET.SubElement(parent, tag, attrs)
    if value is not None:
        result.text = str(value)
    return result


def pose(parent, xyz, rpy=(0, 0, 0)):
    node(parent, 'pose', ' '.join(f'{v:.10g}' for v in (*xyz, *rpy)))


def cylinder(parent, radius, length):
    geom = node(parent, 'geometry')
    shape = node(geom, 'cylinder')
    node(shape, 'radius', radius)
    node(shape, 'length', length)


def inertial(link, mass, radius, length):
    element = node(link, 'inertial')
    node(element, 'mass', mass)
    tensor = node(element, 'inertia')
    transverse = mass * (3 * radius**2 + length**2) / 12
    for name, value in {'ixx': transverse, 'iyy': transverse, 'izz': mass * radius**2 / 2,
                        'ixy': 0, 'ixz': 0, 'iyz': 0}.items():
        node(tensor, name, value)


def mesh(link, name, file):
    visual = node(link, 'visual', name=name)
    geom = node(visual, 'geometry')
    node(node(geom, 'mesh'), 'uri', f'model://{NAME}/meshes/{file}.stl')
    material = node(visual, 'material')
    node(material, 'ambient', '0.16 0.18 0.21 1')
    node(material, 'diffuse', '0.24 0.29 0.34 1')


def write_xml(path, root):
    ET.indent(root, space='  ')
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding='UTF-8', xml_declaration=True)


def validate(config):
    generate_meshes.validate_geometry(config['geometry'])
    for section in ('mass', 'motor', 'sensors'):
        for key, value in config[section].items():
            allow_zero = section == 'motor' and key in ('idle_velocity_rad_s', 'rotor_drag_coefficient', 'rolling_moment_coefficient')
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{section}.{key} 必须为有限数值')
            if value < 0 or (value == 0 and not allow_zero):
                raise ValueError(f'{section}.{key} 必须为{"非负数" if allow_zero else "正数"}')
    motor = config['motor']
    if motor['idle_velocity_rad_s'] >= motor['max_velocity_rad_s']:
        raise ValueError('空闲转速必须小于最大转速')
    aero = config['aerodynamics']
    for key, value in aero.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0 or (value == 0 and key != 'drag_coefficient'):
            raise ValueError(f'aerodynamics.{key} 数值无效（阻力系数可为0，其余必须为正）')
    sim = config['simulation']
    for key, value in sim.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'simulation.{key} 必须为有限数值')
    if sim['imu_wait_timeout_s'] <= 0:
        raise ValueError('IMU 等待超时必须为正')
    if sim['ground_size_m'] <= 0:
        raise ValueError('地面尺寸必须为正')
    if sim['step_size_s'] <= 0 or sim['real_time_factor'] <= 0 or sim['spawn_clearance_m'] < 0:
        raise ValueError('仿真步长、实时倍率必须为正，离地间隙不能为负')
    if config['sensors']['imu_rate_hz'] > 1 / sim['step_size_s']:
        raise ValueError('仿真步长必须足以支持IMU频率')
    if not -90 <= sim['latitude_deg'] <= 90 or not -180 <= sim['longitude_deg'] <= 180:
        raise ValueError('地理原点越界')
    g = config['geometry']
    if g['nacelle_z_m'] - g['arm_radius_m'] < -g['body_length_m'] / 2:
        raise ValueError('连接臂不得低于机尾落地平面')
    if g['rotor_z_m'] + 0.014 > g['nacelle_z_m'] - g['nacelle_length_m'] / 2:
        raise ValueError('下置旋翼轮毂顶部不得进入电机整流罩')
    if g['nacelle_z_m'] - g['arm_radius_m'] < g['rotor_z_m'] + 0.0035:
        raise ValueError('翼内连接臂不得进入下置桨叶旋转平面')
    if isinstance(config['px4']['airframe_id'], bool) or not isinstance(config['px4']['airframe_id'], int) or config['px4']['airframe_id'] <= 0:
        raise ValueError('机体编号必须为正整数')
    for key, value in config['px4']['parameters'].items():
        if not re.fullmatch(r'[A-Z][A-Z0-9_]*', key) or isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'无效PX4参数: {key}')
    # 与当前 PX4 v1.17 参数定义的默认值一致；显式配置优先。
    px4 = config['px4']['parameters']
    minimum = px4.get('MPC_THR_MIN', 0.12)
    hover = px4.get('MPC_THR_HOVER', 0.5)
    maximum = px4.get('MPC_THR_MAX', 1.0)
    if not 0 <= minimum < hover < maximum <= 1:
        raise ValueError('PX4 推力比例必须满足 0 ≤ MPC_THR_MIN < MPC_THR_HOVER < MPC_THR_MAX ≤ 1')
    if not 0 <= px4.get('THR_MDL_FAC', 0.0) <= 1:
        raise ValueError('THR_MDL_FAC 必须位于 [0, 1]')


def build_model(config):
    g, masses, motors = (config[k] for k in ('geometry', 'mass', 'motor'))
    root = ET.Element('sdf', version='1.9')
    model = node(root, 'model', name=NAME)
    node(model, 'self_collide', 'false')
    base = node(model, 'link', name='base_link')
    inertial(base, masses['body_kg'], g['body_radius_m'], g['body_length_m'])
    mesh(base, 'fuselage_visual', 'fuselage')
    mesh(base, 'tail_visual', 'tail')
    cylinder(node(base, 'collision', name='body_collision'), g['body_radius_m'], g['body_length_m'])
    # 翼面与连接臂并入机身质量，机尾落地接触采用机身圆柱碰撞近似。
    for i, (sx, sy) in enumerate(POSITIONS):
        a = g['diagonal_wheelbase_m'] / (2 * math.sqrt(2))
        x, y = sx * a, sy * a
        radial = math.hypot(x, y)
        for tag in ('collision',):
            arm = node(base, tag, name=f'arm_{i}_{tag}')
            pose(arm, (x / 2, y / 2, g['nacelle_z_m']), (0, math.pi / 2, math.atan2(y, x)))
            cylinder(arm, g['arm_radius_m'], radial)
        pod = node(model, 'link', name=f'nacelle_{i}')
        pose(pod, (x, y, g['nacelle_z_m']))
        inertial(pod, masses['nacelle_kg'], g['nacelle_radius_m'], g['nacelle_length_m'])
        mesh(pod, f'nacelle_{i}_visual', 'nacelle')
        cylinder(node(pod, 'collision', name=f'nacelle_{i}_collision'), g['nacelle_radius_m'], g['nacelle_length_m'])
        pod_bottom = -g['nacelle_length_m'] / 2
        hub_top = g['rotor_z_m'] - g['nacelle_z_m'] + 0.014
        if pod_bottom > hub_top:
            for tag in ('visual', 'collision'):
                shaft = node(pod, tag, name=f'motor_shaft_{i}_{tag}')
                pose(shaft, (0, 0, (pod_bottom + hub_top) / 2))
                cylinder(shaft, min(g['arm_radius_m'], 0.006), pod_bottom - hub_top)
        joint = node(model, 'joint', name=f'nacelle_{i}_joint', type='fixed')
        node(joint, 'parent', 'base_link')
        node(joint, 'child', f'nacelle_{i}')
        rotor = node(model, 'link', name=f'rotor_{i}')
        pose(rotor, (x, y, g['rotor_z_m']))
        inertial(rotor, masses['propeller_kg'], g['propeller_diameter_m'] / 2, 0.007)
        mesh(rotor, f'rotor_{i}_visual', 'propeller_10in')
        # 旋转圆盘碰撞包络覆盖所有桨叶方位。
        cylinder(node(rotor, 'collision', name=f'rotor_{i}_collision'), g['propeller_diameter_m'] / 2, 0.007)
        joint = node(model, 'joint', name=f'rotor_{i}_joint', type='revolute')
        node(joint, 'parent', 'base_link')
        node(joint, 'child', f'rotor_{i}')
        axis = node(joint, 'axis')
        node(axis, 'xyz', '0 0 1')
        limit = node(axis, 'limit')
        node(limit, 'lower', '-1e16')
        node(limit, 'upper', '1e16')
        plugin = node(model, 'plugin', filename='gz-sim-multicopter-motor-model-system', name='gz::sim::systems::MulticopterMotorModel')
        values = {'jointName': f'rotor_{i}_joint', 'linkName': f'rotor_{i}', 'turningDirection': 'ccw' if i < 2 else 'cw',
                  'commandSubTopic': 'command/motor_speed', 'motorNumber': i, 'motorType': 'velocity'}
        mapping = {'maxRotVelocity': 'max_velocity_rad_s', 'motorConstant': 'thrust_constant', 'momentConstant': 'moment_constant_m',
                   'timeConstantUp': 'time_constant_up_s', 'timeConstantDown': 'time_constant_down_s',
                   'rotorDragCoefficient': 'rotor_drag_coefficient', 'rollingMomentCoefficient': 'rolling_moment_coefficient',
                   'rotorVelocitySlowdownSim': 'visual_slowdown'}
        values.update({tag: motors[key] for tag, key in mapping.items()})
        for tag, value in values.items():
            node(plugin, tag, value)
    sensor_types = {'imu': 'imu', 'air_pressure': 'air_pressure', 'magnetometer': 'magnetometer', 'navsat': 'navsat'}
    for name, kind in sensor_types.items():
        sensor = node(base, 'sensor', name=f'{name}_sensor', type=kind)
        node(sensor, 'gz_frame_id', 'base_link')
        node(sensor, 'always_on', 1)
        node(sensor, 'update_rate', config['sensors'][f'{name}_rate_hz'])
        if name == 'imu':
            imu = node(sensor, 'imu')
            for quantity, stddev in (('angular_velocity', 0.0008726646), ('linear_acceleration', 0.00637)):
                quantity_node = node(imu, quantity)
                for axis_name in ('x', 'y', 'z'):
                    noise = node(node(quantity_node, axis_name), 'noise', type='gaussian')
                    node(noise, 'mean', 0)
                    node(noise, 'stddev', stddev)
        elif name == 'air_pressure':
            pressure = node(node(sensor, 'air_pressure'), 'pressure')
            noise = node(pressure, 'noise', type='gaussian')
            node(noise, 'mean', 0)
            node(noise, 'stddev', config['sensors']['air_pressure_noise_stddev_pa'])
        elif name == 'magnetometer':
            magnetometer = node(sensor, 'magnetometer')
            for axis_name in ('x', 'y', 'z'):
                noise = node(node(magnetometer, axis_name), 'noise', type='gaussian')
                node(noise, 'mean', 0)
                node(noise, 'stddev', config['sensors']['magnetometer_noise_stddev_t'])
    aero = config['aerodynamics']
    plugin = node(model, 'plugin', filename='gz-sim-advanced-lift-drag-system', name='gz::sim::systems::AdvancedLiftDrag')
    for tag, value in {'link_name': 'base_link', 'air_density': aero['air_density_kg_m3'], 'forward': '0 0 1',
                       'upward': '1 0 0', 'CL0': 0, 'CLa': 0, 'CD0': aero['drag_coefficient'],
                       'alpha_stall': math.pi / 2, 'AR': 1, 'eff': 1, 'area': aero['reference_area_m2'], 'cp': '0 0 0'}.items():
        node(plugin, tag, value)
    return root


def build_world(config):
    # 世界模板保留场景显示与原生插件，仅修改参数化字段。
    root = ET.parse(ROOT / 'tools' / 'world_template.sdf').getroot()
    world = root.find('world')
    sim = config['simulation']
    world.find('physics/max_step_size').text = str(sim['step_size_s'])
    world.find('physics/real_time_factor').text = str(sim['real_time_factor'])
    world.find('physics/real_time_update_rate').text = str(1 / sim['step_size_s'])
    for tag, key in (('latitude_deg', 'latitude_deg'), ('longitude_deg', 'longitude_deg'), ('elevation', 'elevation_m')):
        world.find(f'spherical_coordinates/{tag}').text = str(sim[key])
    world.find('include/pose').text = f"0 0 {config['geometry']['body_length_m'] / 2 + sim['spawn_clearance_m']} 0 0 0"
    ground = world.find("model[@name='ground_plane']/link")
    ground.find('visual/geometry/plane/size').text = f"{sim['ground_size_m']} {sim['ground_size_m']}"
    collision = ground.find('collision')
    geometry = collision.find('geometry')
    geometry.clear()
    node(node(geometry, 'box'), 'size', f"{sim['ground_size_m']} {sim['ground_size_m']} 0.1")
    pose(collision, (0, 0, -0.05))
    return root


def airframe(config):
    g, motor, px4 = (config[k] for k in ('geometry', 'motor', 'px4'))
    lines = ['#!/bin/sh', '# @name Gazebo rocket racing quad', '# @type Quadrotor',
             '# 从 parameters.json 生成；修改配置后重新生成。', '. ${R}etc/init.d/rc.mc_defaults',
             'PX4_SIMULATOR=${PX4_SIMULATOR:=gz}', 'PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}',
             'PX4_SIM_MODEL=${PX4_SIM_MODEL:=gz_rocket_racing_quad}', 'param set-default SIM_GZ_EN 1',
             'param set-default CA_AIRFRAME 0', 'param set-default CA_ROTOR_COUNT 4']
    a = g['diagonal_wheelbase_m'] / (2 * math.sqrt(2))
    for i, (sx, sy) in enumerate(POSITIONS):
        # Gazebo FLU -> PX4 FRD：y/z 符号翻转。
        parameters = {f'CA_ROTOR{i}_PX': sx * a, f'CA_ROTOR{i}_PY': -sy * a,
                      f'CA_ROTOR{i}_PZ': -g['rotor_z_m'],
                      f'CA_ROTOR{i}_KM': motor['moment_constant_m'] * (1 if i < 2 else -1),
                      f'SIM_GZ_EC_FUNC{i+1}': 101 + i, f'SIM_GZ_EC_MIN{i+1}': motor['idle_velocity_rad_s'],
                      f'SIM_GZ_EC_MAX{i+1}': motor['max_velocity_rad_s'], f'SIM_GZ_EC_DIS{i+1}': 0}
        lines += [f'param set-default {key} {value:.10g}' for key, value in parameters.items()]
    lines += [f'param set-default {key} {value:.10g}' for key, value in px4['parameters'].items()]
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'parameters.json')
    parser.add_argument('--output-dir', type=Path, default=ROOT, help='另存完整资产用于参数对比，默认更新模型目录')
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    validate(config)
    output = args.output_dir
    output.mkdir(parents=True, exist_ok=True)
    generate_meshes.generate(config, output / 'meshes')
    write_xml(output / 'model.sdf', build_model(config))
    write_xml(output / 'tests/world.sdf', build_world(config))
    (output / 'model.config').write_bytes((ROOT / 'model.config').read_bytes())
    if args.config.resolve() != (output / 'parameters.json').resolve():
        (output / 'parameters.json').write_text(json.dumps(config, indent=2) + '\n')
    generated = output / 'px4' / f"{config['px4']['airframe_id']}_gz_{NAME}"
    generated.parent.mkdir(exist_ok=True)
    generated.write_text(airframe(config))
    total = config['mass']['body_kg'] + 4 * (config['mass']['nacelle_kg'] + config['mass']['propeller_kg'])
    thrust = 4 * config['motor']['thrust_constant'] * config['motor']['max_velocity_rad_s'] ** 2
    print(f'生成完成：{output.resolve()}；质量 {total:.3f} kg；峰值推重比 {thrust / (total * 9.8):.2f}')


if __name__ == '__main__':
    main()
