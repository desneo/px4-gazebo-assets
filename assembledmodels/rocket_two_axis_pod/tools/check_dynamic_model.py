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
    assert math.isclose(length, .499, abs_tol=1e-12)
    assert math.isclose(center_z, .0245, abs_tol=1e-12)
    assert math.isclose(center_z - length / 2, config['geometry']['min_z_m'], abs_tol=1e-12)
    assert math.isclose(center_z + length / 2, config['mount_z_m'], abs_tol=1e-12)
    assert math.isclose(config['mount_z_m'] + config['pod']['overall_height_m'], config['geometry']['max_z_m'], abs_tol=1e-12)
    assert base.findtext('inertial/pose') != collision.findtext('pose'), 'COM 不得跟随截短碰撞几何中心'
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
    assert math.isclose(total, expected_mass, abs_tol=1e-12) and math.isclose(total, 7.94, abs_tol=1e-12)
    assert math.isclose(total, manifest['total_mass_kg'], abs_tol=1e-12)
    for uri in model.findall('.//mesh/uri'):
        assert uri.text.startswith(f'model://{dynamic.NAME}/')
        path = output / uri.text.removeprefix(f'model://{dynamic.NAME}/')
        assert path.is_file()
        for prefix, origin in (('rocket_', dynamic.ROCKET), ('pod_', dynamic.POD)):
            if path.name.startswith(prefix):
                assert path.read_bytes() == (origin / 'meshes' / path.name[len(prefix):]).read_bytes()
    parameters = json.loads((output / 'parameters.json').read_text())
    assert set(parameters) == {'model', 'px4', 'simulation', 'flight_test', 'runtime', 'assembly'}
    assert parameters['runtime']['source_sha256'] == dynamic.source_fingerprints() == manifest['source_sha256']
    assert math.isclose(parameters['model']['total_mass_kg'], total, abs_tol=1e-12)
    assert parameters['model']['links']['base_link']['inertial_pose'] == source.findtext("link[@name='base_link']/inertial/pose")
    defaults = json.loads((dynamic.ROCKET / 'parameters.json').read_text())
    assert parameters['flight_test'] == defaults['flight_test']
    world = ET.parse(output / 'tests/world.sdf').getroot().find('world')
    assert parameters['simulation']['world_name'] == world.get('name') == dynamic.NAME + '_test'
    assert parameters['simulation']['gravity'] == world.findtext('gravity')
    assert parameters['simulation']['spawn_pose'] == world.findtext(f"include[uri='model://{dynamic.NAME}']/pose")
    airframe = (output / 'px4' / f"{manifest['airframe_id']}_gz_{dynamic.NAME}").read_bytes().decode()
    assert airframe == dynamic.assembly_airframe(config['airframe_text'], manifest['px4_hover_thrust'])
    actual_px4 = dynamic.native.parameter_values(airframe)
    assert actual_px4 == parameters['px4']['parameters']
    assert set(actual_px4) == set(config['px4_parameters'])
    for key, value in config['px4_parameters'].items():
        if key != 'MPC_THR_HOVER':
            assert actual_px4[key] == value, f'原生 PX4 参数被改变：{key}'
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
        for value in (-1, 0, .9, 1000, float('nan'), float('inf'), True):
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
    print(f'动态装配验证通过：12 links，11 joints，总质量 {total:.9f} kg；原生惯性/PX4继承、接口完整、生成确定、源资产不变。')


if __name__ == '__main__':
    main()
