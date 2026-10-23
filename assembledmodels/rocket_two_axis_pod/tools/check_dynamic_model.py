#!/usr/bin/env python3
"""验证原生资产继承、动态装配、质量惯量、PX4 与控制接口，不启动仿真。"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

import generate_dynamic_model as dynamic


def canonical(element):
    return (element.tag, tuple(sorted(element.attrib.items())), (element.text or '').strip(),
            tuple(canonical(child) for child in element))


def scalar(element, path):
    return float(element.findtext(path))


def mass_and_center(model):
    mass = 0.
    moment = np.zeros(3)
    for link in model.findall('link'):
        link_mass = scalar(link, 'inertial/mass')
        transform = (dynamic.appearance.transform(link.findtext('pose')) @
                     dynamic.appearance.transform(link.findtext('inertial/pose')))
        mass += link_mass
        moment += link_mass * transform[:3, 3]
    return mass, moment / mass


def native_mesh_uris(element):
    element = copy.deepcopy(element)
    for uri in element.findall('.//mesh/uri'):
        for prefix, name in (('rocket_', 'rocket_racing_quad'), ('pod_', 'two_axis_pod')):
            source = f'model://{dynamic.NAME}/meshes/{prefix}'
            if uri.text.startswith(source):
                uri.text = f'model://{name}/meshes/{uri.text[len(source):]}'
    return element


def check_output(output, manifest):
    config = dynamic.settings(airframe_id=manifest['airframe_id'], topic_prefix=manifest['topic_prefix'])
    model = ET.parse(output / 'model.sdf').getroot().find('model')
    source = ET.parse(dynamic.ROCKET / 'model.sdf').getroot().find('model')
    pod = ET.parse(dynamic.POD / 'model.sdf').getroot().find('model')
    assert model.get('name') == dynamic.NAME and model.get('canonical_link') == 'base_link'
    assert model.findtext('static') == 'false'
    links = {link.get('name'): link for link in model.findall('link')}
    assert len(links) == len(model.findall('link')) == 12
    joints = model.findall('joint')
    assert len(joints) == 11
    assert len({joint.findtext('child') for joint in joints}) == len(joints)
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
    # 原生火箭所有 link 的质量、COM、完整惯量及其他属性均继承。
    for original in source.findall('link'):
        actual = native_mesh_uris(links[original.get('name')])
        expected = copy.deepcopy(original)
        assert canonical(actual.find('inertial')) == canonical(expected.find('inertial'))
        if original.get('name') == 'base_link':
            for element in (actual, expected):
                element.remove(element.find("collision[@name='body_collision']"))
                element.remove(element.find("visual[@name='fuselage_visual']"))
        assert canonical(actual) == canonical(expected), f'原生 link 被改变：{original.get("name")}'
    for original in source.findall('joint'):
        assert canonical(model.find(f"joint[@name='{original.get('name')}']")) == canonical(original)
    source_plugins = source.findall('plugin')
    assert [canonical(p) for p in model.findall('plugin')[:len(source_plugins)]] == [canonical(p) for p in source_plugins]
    base = links['base_link']
    collision = base.find("collision[@name='body_collision']")
    length = scalar(collision, 'geometry/cylinder/length')
    center_z = float(collision.findtext('pose').split()[2])
    expected_length = config['mount_z_m'] - config['geometry']['min_z_m']
    expected_center_z = (config['mount_z_m'] + config['geometry']['min_z_m']) / 2
    assert math.isclose(length, expected_length, abs_tol=1e-12)
    assert math.isclose(center_z, expected_center_z, abs_tol=1e-12)
    assert math.isclose(center_z - length / 2, config['geometry']['min_z_m'], abs_tol=1e-12)
    assert math.isclose(center_z + length / 2, config['mount_z_m'], abs_tol=1e-12)
    assert math.isclose(config['mount_z_m'] + config['pod']['overall_height_m'], config['geometry']['max_z_m'], abs_tol=1e-12)
    expected_visual = copy.deepcopy(source.find("link[@name='base_link']/visual[@name='fuselage_visual']"))
    expected_visual.find('geometry/mesh/uri').text = f'model://{dynamic.NAME}/meshes/fuselage_cut.stl'
    assert canonical(base.find("visual[@name='fuselage_visual']")) == canonical(expected_visual)
    visual = base.find("visual[@name='fuselage_visual']")
    mesh = visual.find('geometry/mesh')
    vertices = dynamic.appearance.read_stl(output / 'meshes/fuselage_cut.stl').reshape(-1, 3)
    vertices *= np.fromstring(mesh.findtext('scale', '1 1 1'), sep=' ')
    transform = dynamic.appearance.transform(base.findtext('pose')) @ dynamic.appearance.transform(visual.findtext('pose'))
    vertices = vertices @ transform[:3, :3].T + transform[:3, 3]
    assert math.isclose(vertices[:, 2].min(), config['geometry']['min_z_m'], abs_tol=1e-7)
    assert math.isclose(vertices[:, 2].max(), config['mount_z_m'], abs_tol=1e-7)
    # 吊舱活动关节、限位、状态机参数、相机参数均保持组件原值。
    for original in pod:
        if original.tag == 'link':
            expected = copy.deepcopy(original)
            transform = dynamic.appearance.transform(original.findtext('pose'))
            transform[2, 3] += config['mount_z_m']
            dynamic.set_text(expected, 'pose', dynamic.appearance.pose_text(transform))
            camera = expected.find("sensor[@name='camera']")
            if camera is not None:
                dynamic.set_text(camera, 'topic', manifest['image_topic'])
            assert canonical(native_mesh_uris(links[original.get('name')])) == canonical(expected)
        elif original.tag == 'joint':
            assert canonical(model.find(f"joint[@name='{original.get('name')}']")) == canonical(original)
        elif original.tag == 'plugin':
            expected = copy.deepcopy(original)
            if original.get('name') == 'two_axis::GimbalController':
                dynamic.set_text(expected, 'mount_dynamic', 'true')
                dynamic.set_text(expected, 'image_topic', manifest['image_topic'])
                for key in ('command', 'telemetry', 'command_result'):
                    dynamic.set_text(expected, key + '_topic', manifest['topic_prefix'] + '/' + key)
            assert canonical(model.find(f"plugin[@name='{original.get('name')}']")) == canonical(expected)
    # 同时校验对外机械角约定，不能只依赖源资产相等而漏掉轴方向回归。
    for name, axis in (('yaw_joint', '1 0 0'), ('pitch_joint', '0 -1 0')):
        joint = model.find(f"joint[@name='{name}']")
        assert joint.findtext('axis/xyz') == axis
    pitch_limit = model.find("joint[@name='pitch_joint']/axis/limit")
    assert math.isclose(scalar(pitch_limit, 'lower'), math.radians(-80), abs_tol=1e-12)
    assert math.isclose(scalar(pitch_limit, 'upper'), math.radians(30), abs_tol=1e-12)
    yaw_limit = model.find("joint[@name='yaw_joint']/axis/limit")
    assert math.isclose(scalar(yaw_limit, 'lower'), math.radians(-60), abs_tol=1e-12)
    assert math.isclose(scalar(yaw_limit, 'upper'), math.radians(60), abs_tol=1e-12)
    total = 0.
    for link in links.values():
        inertial = link.find('inertial')
        mass = scalar(inertial, 'mass')
        assert math.isfinite(mass) and mass > 0
        total += mass
        tensor = inertial.find('inertia')
        matrix = np.array([[scalar(tensor, 'ixx'), scalar(tensor, 'ixy'), scalar(tensor, 'ixz')],
                           [scalar(tensor, 'ixy'), scalar(tensor, 'iyy'), scalar(tensor, 'iyz')],
                           [scalar(tensor, 'ixz'), scalar(tensor, 'iyz'), scalar(tensor, 'izz')]])
        principal = np.linalg.eigvalsh(matrix)
        assert np.all(np.isfinite(principal)) and np.all(principal > 0)
        assert principal[-1] <= sum(principal[:2]) + 1e-12
    expected_mass = sum(scalar(link, 'inertial/mass') for parent in (source, pod) for link in parent.findall('link'))
    assert math.isclose(total, expected_mass, abs_tol=1e-12)
    assert math.isclose(total, manifest['total_mass_kg'], abs_tol=1e-12)
    source_mass, source_center = mass_and_center(source)
    pod_mass, pod_center = mass_and_center(pod)
    expected_center = (source_mass * source_center + pod_mass *
                       (pod_center + np.array([0., 0., config['mount_z_m']]))) / expected_mass
    actual_mass, actual_center = mass_and_center(model)
    assert math.isclose(actual_mass, expected_mass, abs_tol=1e-12)
    np.testing.assert_allclose(actual_center, expected_center, rtol=0, atol=1e-12,
                               err_msg='组合 COM 必须由原生惯性位姿与吊舱安装位置合成')
    for uri in model.findall('.//mesh/uri'):
        assert uri.text.startswith(f'model://{dynamic.NAME}/')
        path = output / uri.text.removeprefix(f'model://{dynamic.NAME}/')
        assert path.is_file()
        for prefix, origin in (('rocket_', dynamic.ROCKET), ('pod_', dynamic.POD)):
            if path.name.startswith(prefix):
                assert path.read_bytes() == (origin / 'meshes' / path.name[len(prefix):]).read_bytes()
    parameters = json.loads((output / 'parameters.json').read_text())
    assert set(parameters) == {'model', 'px4', 'simulation', 'flight_test',
                               'stability_validation', 'runtime', 'assembly'}
    assert parameters['stability_validation'] == config['stability_validation']
    assert parameters['runtime']['source_sha256'] == dynamic.source_fingerprints() == manifest['source_sha256']
    assert math.isclose(parameters['model']['total_mass_kg'], total, abs_tol=1e-12)
    assert parameters['model']['links']['base_link']['inertial_pose'] == source.findtext("link[@name='base_link']/inertial/pose")
    defaults = json.loads((dynamic.ROCKET / 'parameters.json').read_text())
    assert parameters['flight_test'] == defaults['flight_test']
    world = ET.parse(output / 'tests/world.sdf').getroot().find('world')
    assert parameters['simulation']['world_name'] == world.get('name') == dynamic.NAME + '_test'
    assert parameters['simulation']['gravity'] == world.findtext('gravity')
    assert parameters['simulation']['spawn_pose'] == world.findtext(f"include[uri='model://{dynamic.NAME}']/pose")
    # 坐标来自原生 CG，平移到组合零位 CG；只变参考原点，物理安装点保持不变。
    signs = np.array([1., -1., -1.])
    translation = signs * (source_center - expected_center)
    reference_keys = [prefix + axis
                      for prefix in ([f'CA_ROTOR{index}_P' for index in range(4)] +
                                     ['EKF2_IMU_POS_', 'EKF2_GPS_POS_']) for axis in 'XYZ']
    expected_reference = {key: config['px4_parameters'][key] + translation[index % 3]
                          for index, key in enumerate(reference_keys)}
    reference = manifest['px4_reference']
    assert reference == parameters['assembly']['px4_reference']
    assert set(reference) == {'reference_pose', 'base_cg_flu_m', 'assembly_nominal_cg_flu_m',
                              'offset_translation_frd_m', 'source_parameters', 'parameters'}
    assert reference['reference_pose'] == 'zero_joint_positions'
    np.testing.assert_allclose(reference['base_cg_flu_m'], source_center, rtol=0, atol=1e-12)
    np.testing.assert_allclose(reference['assembly_nominal_cg_flu_m'], expected_center, rtol=0, atol=1e-12)
    np.testing.assert_allclose(reference['offset_translation_frd_m'], translation, rtol=0, atol=1e-12)
    assert reference['source_parameters'] == {key: config['px4_parameters'][key] for key in reference_keys}
    assert set(reference['parameters']) == set(expected_reference)
    for key, value in expected_reference.items():
        assert math.isclose(reference['parameters'][key], value, rel_tol=0, abs_tol=1e-12), key
    airframe = (output / 'px4' / f"{manifest['airframe_id']}_gz_{dynamic.NAME}").read_bytes().decode()
    assert airframe == dynamic.assembly_airframe(config['airframe_text'], manifest['px4_hover_thrust'],
                                                reference['parameters'], config['px4_parameter_overrides'])
    actual_px4 = dynamic.native.parameter_values(airframe)
    assert actual_px4 == parameters['px4']['parameters']
    assert set(actual_px4) == set(config['px4_parameters']) | set(config['px4_parameter_overrides'])
    for key, value in config['px4_parameters'].items():
        if key in expected_reference:
            assert math.isclose(actual_px4[key], expected_reference[key], rel_tol=0, abs_tol=1e-12), key
            axis = reference_keys.index(key) % 3
            assert math.isclose(actual_px4[key] + signs[axis] * actual_center[axis],
                                value + signs[axis] * source_center[axis], rel_tol=0, abs_tol=1e-12), key
        elif key != 'MPC_THR_HOVER' and key not in config['px4_parameter_overrides']:
            assert actual_px4[key] == value, f'原生 PX4 参数被改变：{key}'
    for key, value in config['px4_parameter_overrides'].items():
        assert actual_px4[key] == value, f'装配 PX4 参数未生效：{key}'
    motor_plugins = model.findall(f"plugin[@name='{dynamic.MOTOR}']")
    thrust = 0.
    for motor in motor_plugins:
        channel = int(motor.findtext('motorNumber')) + 1
        idle, maximum = (actual_px4[f'SIM_GZ_EC_{tag}{channel}'] for tag in ('MIN', 'MAX'))
        velocity = idle + (maximum - idle) * math.sqrt(actual_px4['MPC_THR_HOVER'])
        thrust += scalar(motor, 'motorConstant') * velocity**2
    assert math.isclose(thrust, total * manifest['gravity_m_s2'], rel_tol=1e-12)
    return total


def main():
    before = dynamic.source_fingerprints()
    static_path = dynamic.ROOT / 'model.sdf'
    static_hash = hashlib.sha256(static_path.read_bytes()).hexdigest()
    with tempfile.TemporaryDirectory(prefix='rocket-pod-structure-') as directory:
        temp = Path(directory)
        first, second = temp / 'first', temp / 'second'
        manifest = dynamic.generate(first)
        total = check_output(first, manifest)
        dynamic.generate(second)
        for file in first.rglob('*'):
            if file.is_file():
                assert file.read_bytes() == (second / file.relative_to(first)).read_bytes(), f'生成不确定：{file.name}'
        override = dynamic.generate(temp / 'interface', airframe_id=4999, topic_prefix='/test/rocket_pod')
        check_output(temp / 'interface', override)
        assert override['image_topic'] == '/test/rocket_pod/image'
        source_body_mass = scalar(ET.parse(dynamic.ROCKET / 'model.sdf').getroot(),
                                  "model/link[@name='base_link']/inertial/mass")
        for value in (-1, 0, source_body_mass / 2, source_body_mass * 2, float('nan'), float('inf'), True):
            try:
                dynamic.generate(temp / 'invalid', body_mass_kg=value)
            except ValueError:
                pass
            else:
                raise AssertionError('不得单独覆盖已确认的原生质量与惯性')
        assert not (temp / 'invalid').exists()
        for path in (dynamic.ROOT, dynamic.ROCKET, dynamic.POD / 'runtime'):
            try:
                dynamic.generate(path)
            except ValueError:
                pass
            else:
                raise AssertionError('源目录覆盖未被拒绝')
    assert before == dynamic.source_fingerprints(), '基础组件被修改'
    assert static_hash == hashlib.sha256(static_path.read_bytes()).hexdigest(), '静态造型被覆盖'
    print(f'动态装配验证通过：12 links，11 joints，yaw +X / pitch -Y，总质量 {total:.9f} kg；原生惯性/PX4继承、接口完整、生成确定、源资产不变。')


if __name__ == '__main__':
    main()
