#!/usr/bin/env python3
"""验证动态装配的来源一致性、物理结构、质量与机械轴向，不启动仿真。"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

import generate_dynamic_model as dynamic


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(),
            tuple(canonical(child) for child in element))


def scalar(element, path):
    return float(element.findtext(path))


def validate_source_consistency(temp):
    source = ET.parse(dynamic.ROCKET / 'model.sdf').getroot()
    generated = dynamic.rocket_tools.build_model(dynamic.settings()['rocket'])
    assert canonical(source) == canonical(generated), '基础火箭 SDF 与参数生成结果不一致'
    rocket_meshes = temp / 'source_rocket_meshes'
    dynamic.mesh_tools.generate(dynamic.settings()['rocket'], rocket_meshes)
    for mesh in rocket_meshes.iterdir():
        assert mesh.read_bytes() == (dynamic.ROCKET / 'meshes' / mesh.name).read_bytes(), f'火箭网格与参数不一致：{mesh.name}'
    # 吊舱生成器只向临时副本写入，验证实际活动关节和网格来自当前组件参数。
    pod_temp = temp / 'source_pod'
    (pod_temp / 'tests').mkdir(parents=True)
    for relative in ('parameters.json', 'tests/world_base.sdf'):
        shutil.copyfile(dynamic.POD / relative, pod_temp / relative)
    pod_tools = dynamic.load_module('validate_pod_source', dynamic.POD / 'tools/generate_model.py')
    pod_tools.ROOT = pod_temp
    pod_tools.generate()
    assert canonical(ET.parse(pod_temp / 'model.sdf').getroot()) == canonical(ET.parse(dynamic.POD / 'model.sdf').getroot()), '基础吊舱 SDF 与参数生成结果不一致'
    for mesh in (pod_temp / 'meshes').iterdir():
        assert mesh.read_bytes() == (dynamic.POD / 'meshes' / mesh.name).read_bytes(), f'吊舱网格与参数不一致：{mesh.name}'


def check_output(output, manifest):
    config = dynamic.settings()
    model = ET.parse(output / 'model.sdf').getroot().find('model')
    assert model.get('name') == dynamic.NAME
    assert model.get('canonical_link') == 'base_link'
    assert model.findtext('static') == 'false'
    links = {link.get('name'): link for link in model.findall('link')}
    assert len(links) == 12 and len(model.findall('link')) == len(links)
    joints = model.findall('joint')
    assert len(joints) == len(links) - 1
    children = [joint.findtext('child') for joint in joints]
    assert len(set(children)) == len(children)
    reachable = {'base_link'}
    while True:
        updated = reachable | {joint.findtext('child') for joint in joints if joint.findtext('parent') in reachable}
        if updated == reachable:
            break
        reachable = updated
    assert reachable == set(links), '所有部件必须连接到 base_link，不能固定到 world'
    fixed = model.find("joint[@name='pod_mount_joint']")
    assert fixed.get('type') == 'fixed'
    assert (fixed.findtext('parent'), fixed.findtext('child')) == ('base_link', 'mount_link')
    pod_source = ET.parse(dynamic.POD / 'model.sdf').getroot().find('model')
    for name, axis in (('yaw_joint', '0 0 1'), ('pitch_joint', '0 1 0')):
        joint = model.find(f"joint[@name='{name}']")
        assert canonical(joint) == canonical(pod_source.find(f"joint[@name='{name}']"))
        assert joint.findtext('axis/xyz') == axis
    for name in ('mount_link', 'yaw_arm_link', 'camera_link'):
        actual = np.fromstring(links[name].findtext('pose'), sep=' ')
        expected = np.fromstring(pod_source.find(f"link[@name='{name}']/pose").text, sep=' ')
        expected[2] += config['mount_z_m']
        np.testing.assert_allclose(actual, expected, atol=1e-12)
        assert canonical(links[name].find('inertial')) == canonical(pod_source.find(f"link[@name='{name}']/inertial"))
    controller = model.find("plugin[@name='two_axis::GimbalController']")
    assert controller.findtext('mount_dynamic') == 'true'
    for key in ('command', 'telemetry', 'command_result'):
        assert controller.findtext(key + '_topic') == manifest['topic_prefix'] + '/' + key
    assert controller.findtext('image_topic') == manifest['image_topic']
    assert links['camera_link'].findtext("sensor[@name='camera']/topic") == manifest['image_topic']
    total = 0.
    for link in links.values():
        inertial = link.find('inertial')
        mass = scalar(inertial, 'mass')
        assert math.isfinite(mass) and mass > 0
        total += mass
        inertia = inertial.find('inertia')
        tensor = np.array([[scalar(inertia, 'ixx'), scalar(inertia, 'ixy'), scalar(inertia, 'ixz')],
                           [scalar(inertia, 'ixy'), scalar(inertia, 'iyy'), scalar(inertia, 'iyz')],
                           [scalar(inertia, 'ixz'), scalar(inertia, 'iyz'), scalar(inertia, 'izz')]])
        principal = np.linalg.eigvalsh(tensor)
        assert np.all(np.isfinite(principal)) and np.all(principal > 0)
        assert principal[-1] <= sum(principal[:2]) + 1e-12, '主惯量需满足三角不等式'
    pod_mass = sum(float(link.findtext('inertial/mass')) for link in pod_source.findall('link'))
    expected_mass = config['body_mass_kg'] + 4 * sum(config['rocket']['mass'][key] for key in ('nacelle_kg', 'propeller_kg')) + pod_mass
    assert math.isclose(total, expected_mass, abs_tol=1e-12)
    assert math.isclose(total, manifest['total_mass_kg'], abs_tol=1e-12)
    base = links['base_link']
    body = base.find("collision[@name='body_collision']")
    length = scalar(body, 'geometry/cylinder/length')
    center_z = float(body.findtext('pose').split()[2])
    assert math.isclose(center_z + length / 2, config['mount_z_m'], abs_tol=1e-12)
    assert math.isclose(center_z - length / 2, -.225, abs_tol=1e-12)
    assert math.isclose(config['mount_z_m'] + config['pod']['overall_height_m'], .225, abs_tol=1e-12)
    assert base.findtext('inertial/pose') == body.findtext('pose')
    cut_vertices = dynamic.appearance.cut_fuselage().reshape(-1, 3)
    np.testing.assert_allclose(cut_vertices[:, 2].min(), -.225, atol=1e-12)
    np.testing.assert_allclose(cut_vertices[:, 2].max(), config['mount_z_m'], atol=1e-12)
    source_rocket = dynamic.rocket_tools.build_model(config['rocket']).find('model')
    for source_link in source_rocket.findall('link'):
        if source_link.get('name') == 'base_link':
            for sensor in source_link.findall('sensor'):
                assert canonical(sensor) == canonical(base.find(f"sensor[@name='{sensor.get('name')}']"))
            continue
        actual = links[source_link.get('name')]
        assert canonical(actual.find('inertial')) == canonical(source_link.find('inertial'))
        assert actual.findtext('pose') == source_link.findtext('pose')
    motor_plugins = model.findall("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
    expected_motors = source_rocket.findall("plugin[@name='gz::sim::systems::MulticopterMotorModel']")
    assert [canonical(plugin) for plugin in motor_plugins] == [canonical(plugin) for plugin in expected_motors]
    for uri in model.findall('.//mesh/uri'):
        assert uri.text.startswith(f'model://{dynamic.NAME}/')
        assert (output / uri.text.removeprefix(f'model://{dynamic.NAME}/')).is_file()
    airframe = (output / 'px4' / f"{manifest['airframe_id']}_gz_{dynamic.NAME}").read_text()
    assert 'PX4_SIM_MODEL=${PX4_SIM_MODEL:=gz_rocket_two_axis_pod}' in airframe
    for i in range(4):
        assert f'param set-default SIM_GZ_EC_FUNC{i+1} {101+i}' in airframe
    parameters = json.loads((output / 'parameters.json').read_text())
    assert parameters['mass']['body_kg'] == manifest['body_mass_kg']
    assert math.isclose(parameters['assembly']['total_mass_kg'], total, abs_tol=1e-12)
    assert parameters['px4']['parameters']['MPC_THR_HOVER'] == manifest['px4_hover_thrust']
    airframe_parameters = {}
    for line in airframe.splitlines():
        parts = line.split()
        if parts[:2] == ['param', 'set-default']:
            airframe_parameters[parts[2]] = float(parts[3])
    motor = parameters['motor']
    hover = airframe_parameters['MPC_THR_HOVER']
    assert math.isclose(hover, parameters['px4']['parameters']['MPC_THR_HOVER'], rel_tol=1e-9)
    assert airframe_parameters['THR_MDL_FAC'] == 1
    for channel in range(1, 5):
        idle = airframe_parameters[f'SIM_GZ_EC_MIN{channel}']
        assert idle == manifest['armed_idle_velocity_rad_s'] == motor['idle_velocity_rad_s'] > 0
        assert airframe_parameters[f'SIM_GZ_EC_DIS{channel}'] == 0
        assert parameters['px4']['parameters'][f'SIM_GZ_EC_DIS{channel}'] == 0
        maximum = airframe_parameters[f'SIM_GZ_EC_MAX{channel}']
        actual_velocity = idle + (maximum - idle) * math.sqrt(hover)
        channel_thrust = motor['thrust_constant'] * actual_velocity**2
        assert math.isclose(4 * channel_thrust, total * 9.8, rel_tol=1e-9), '归一化悬停输出必须恢复实际整机重量'
        # PX4 对速度取整；误差不得超过半个速度刻度产生的推力差。
        rounded_thrust = 4 * motor['thrust_constant'] * round(actual_velocity)**2
        quantization_bound = 4 * motor['thrust_constant'] * (actual_velocity + .25)
        assert abs(rounded_thrust - total * 9.8) <= quantization_bound
    assert math.isclose(manifest['armed_idle_total_thrust_n'], 4 * motor['thrust_constant'] * motor['idle_velocity_rad_s']**2)
    return total


def main():
    before = dynamic.source_fingerprints()
    static = dynamic.ROOT / 'model.sdf'
    static_hash = hashlib.sha256(static.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory(prefix='rocket-pod-structure-') as directory:
        temp = Path(directory)
        validate_source_consistency(temp)
        first, second = temp / 'first', temp / 'second'
        manifest = dynamic.generate(first)
        total = check_output(first, manifest)
        dynamic.generate(second)
        for file in first.rglob('*'):
            if file.is_file():
                assert file.read_bytes() == (second / file.relative_to(first)).read_bytes(), f'生成不确定：{file.name}'
        override = dynamic.generate(temp / 'override', body_mass_kg=.9, airframe_id=4999, topic_prefix='/test/rocket_pod')
        assert override['body_mass_kg'] == .9 and override['airframe_id'] == 4999
        assert override['image_topic'] == '/test/rocket_pod/image'
        invalid_output = temp / 'invalid'
        try:
            dynamic.generate(invalid_output, body_mass_kg=1000)
        except ValueError:
            pass
        else:
            raise AssertionError('不可行的悬停推力未被拒绝')
        assert not invalid_output.exists(), '无效配置留下部分运行资产'
        for value in (-1, 0, float('nan'), float('inf'), True):
            try:
                dynamic.settings(body_mass_kg=value)
            except ValueError:
                pass
            else:
                raise AssertionError('无效质量未被拒绝')
        for path in (dynamic.ROOT, dynamic.ROCKET, dynamic.POD / 'runtime'):
            try:
                dynamic.generate(path)
            except ValueError:
                pass
            else:
                raise AssertionError('源目录覆盖未被拒绝')
    assert before == dynamic.source_fingerprints(), '基础组件被修改'
    assert static_hash == hashlib.sha256(static.read_bytes()).hexdigest(), '确认造型被覆盖'
    print(f'动态装配验证通过：12 links，11 joints，yaw +Z / pitch +Y，总质量 {total:.9f} kg；源资产不变、生成确定、所有惯量正定。')


if __name__ == '__main__':
    main()
