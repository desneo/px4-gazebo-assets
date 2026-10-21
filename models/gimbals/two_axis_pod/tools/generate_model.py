#!/usr/bin/env python3
"""从已导入 CAD 网格和质量属性生成两轴组件；安装系 X 前、Y 左、Z 上。"""
import copy
import hashlib
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


def mesh_part(link, name, uri, pose='0 0 0 0 0 0', color='.09 .09 .10 1'):
    visual = elem(link, 'visual', name=name + '_visual')
    elem(visual, 'pose', pose)
    geometry(visual, 'mesh', {'uri': uri})
    mat = elem(visual, 'material')
    elem(mat, 'ambient', color)
    elem(mat, 'diffuse', color)
    elem(mat, 'specular', '.35 .35 .35 1')


def write_dae(path, vertices, faces, name):
    """写入带平滑法向的确定性 COLLADA 网格，由 SDF 统一指定材质。"""
    triangles = []
    for face in faces:
        triangles.extend((face[0], face[index], face[index + 1])
                         for index in range(1, len(face) - 1))
    normals = [[0., 0., 0.] for _ in vertices]
    for face in triangles:
        a, b, c = [vertices[index] for index in face]
        u = [b[index] - a[index] for index in range(3)]
        v = [c[index] - a[index] for index in range(3)]
        normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
        for vertex in face:
            for index in range(3):
                normals[vertex][index] += normal[index]
    for index, normal in enumerate(normals):
        length = max(1e-12, math.sqrt(sum(value * value for value in normal)))
        normals[index] = [value / length for value in normal]
    namespace = 'http://www.collada.org/2005/11/COLLADASchema'
    ET.register_namespace('', namespace)
    def dae_elem(parent, tag, text=None, **attrs):
        node = ET.SubElement(parent, '{' + namespace + '}' + tag, attrs)
        if text is not None:
            node.text = text
        return node
    root = ET.Element('{' + namespace + '}COLLADA', version='1.4.1')
    asset = dae_elem(root, 'asset')
    dae_elem(asset, 'unit', name='meter', meter='1')
    dae_elem(asset, 'up_axis', 'Z_UP')
    geometry_library = dae_elem(root, 'library_geometries')
    geometry_node = dae_elem(geometry_library, 'geometry', id=name, name=name)
    mesh = dae_elem(geometry_node, 'mesh')
    for suffix, data in (('positions', vertices), ('normals', normals)):
        source = dae_elem(mesh, 'source', id=f'{name}-{suffix}')
        dae_elem(source, 'float_array',
                 ' '.join(f'{value:.9f}' for row in data for value in row),
                 id=f'{name}-{suffix}-array', count=str(len(data) * 3))
        accessor = dae_elem(dae_elem(source, 'technique_common'), 'accessor',
                            source=f'#{name}-{suffix}-array', count=str(len(data)), stride='3')
        for axis in 'XYZ':
            dae_elem(accessor, 'param', name=axis, type='float')
    vertices_node = dae_elem(mesh, 'vertices', id=f'{name}-vertices')
    dae_elem(vertices_node, 'input', semantic='POSITION', source=f'#{name}-positions')
    triangles_node = dae_elem(mesh, 'triangles', count=str(len(triangles)))
    dae_elem(triangles_node, 'input', semantic='VERTEX', source=f'#{name}-vertices', offset='0')
    dae_elem(triangles_node, 'input', semantic='NORMAL', source=f'#{name}-normals', offset='1')
    dae_elem(triangles_node, 'p',
             ' '.join(str(value) for face in triangles for vertex in face for value in (vertex, vertex)))
    scene_library = dae_elem(root, 'library_visual_scenes')
    scene = dae_elem(scene_library, 'visual_scene', id='Scene')
    node = dae_elem(scene, 'node', id=f'{name}-node')
    dae_elem(node, 'instance_geometry', url=f'#{name}')
    dae_elem(dae_elem(root, 'scene'), 'instance_visual_scene', url='#Scene')
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def number(value):
    return format(float(value), '.12g')


def vector(values):
    return ' '.join(number(v) for v in values)


def load_geometry():
    properties = json.loads((ROOT / 'cad/properties.json').read_text())
    for relative, expected in properties['sha256'].items():
        if hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f'CAD 与导出资产不一致，请运行 tools/import_cad.py：{relative}')
    return properties


def validate_configuration(c, properties):
    if c['camera_center_m'] != properties['groups']['camera_link']['origin_m']:
        raise ValueError('相机球心必须继承 CAD 导出属性')
    if c['kinematics']['yaw_origin_m'] != properties['groups']['yaw_arm_link']['origin_m']:
        raise ValueError('外轴轴心必须继承 CAD 导出属性')
    if c['kinematics']['yaw_axis'] != [-1, 0, 0] or c['kinematics']['pitch_axis'] != [0, -1, 0]:
        raise ValueError('机械轴必须与 CAD 及专用控制器一致')
    expected = [0, -math.pi/2, 0]
    rpy = c['camera']['sensor_rpy_rad']
    if len(rpy) != 3 or any(not math.isfinite(v) or abs(v-e) > 1e-10 for v, e in zip(rpy, expected)):
        raise ValueError('相机光学基底必须与 CAD 及专用控制器一致')
    if c['mass_distribution'] != 'uniform_effective_density_over_cad_solids':
        raise ValueError('不支持该质量分布方法')
    if not math.isfinite(c['mass_kg']) or c['mass_kg'] <= 0:
        raise ValueError('总质量必须为正有限数')


def link(model, name, group, mass):
    node = elem(model, 'link', name=name)
    elem(node, 'pose', vector([*group['origin_m'], 0, 0, 0]))
    inertial = elem(node, 'inertial')
    elem(inertial, 'pose', vector([*group['center_local_m'], 0, 0, 0]))
    elem(inertial, 'mass', number(mass))
    tensor = elem(inertial, 'inertia')
    inertia = group['unit_mass_inertia_m2']
    for key, i, j in [('ixx', 0, 0), ('iyy', 1, 1), ('izz', 2, 2),
                      ('ixy', 0, 1), ('ixz', 0, 2), ('iyz', 1, 2)]:
        elem(tensor, key, number(mass * inertia[i][j]))
    return node


def joint(model, name, parent, child, axis, limits, rate):
    node = elem(model, 'joint', name=name, type='revolute')
    elem(node, 'parent', parent)
    elem(node, 'child', child)
    # SDF joint pose 默认相对子 link；两个子 link 的原点均位于相应轴线上。
    elem(node, 'pose', '0 0 0 0 0 0')
    a = elem(node, 'axis')
    elem(a, 'xyz', vector(axis))
    limit = elem(a, 'limit')
    elem(limit, 'lower', number(math.radians(limits[0])))
    elem(limit, 'upper', number(math.radians(limits[1])))
    elem(limit, 'effort', 5)
    elem(limit, 'velocity', number(math.radians(rate)))
    elem(elem(a, 'dynamics'), 'damping', .1)


def generate():
    c = json.loads((ROOT / 'parameters.json').read_text())
    properties = load_geometry()
    validate_configuration(c, properties)
    total_mass = c['mass_kg']
    sdf = ET.Element('sdf', version='1.9')
    model = elem(sdf, 'model', name='two_axis_pod')
    elem(model, 'self_collide', 'false')
    links = {}
    for name, group in properties['groups'].items():
        mass = total_mass * group['volume_mm3'] / properties['volume_mm3']
        links[name] = node = link(model, name, group, mass)
        for part_name in group['parts']:
            part_data = properties['parts'][part_name]
            uri = 'model://two_axis_pod/' + part_data['mesh']
            mesh_part(node, part_name, uri, color=vector(part_data['rgba']))
            collision = elem(node, 'collision', name=part_name + '_collision')
            geometry(collision, 'mesh', {'uri': uri})
    camera = links['camera_link']
    sensor = elem(camera, 'sensor', name='camera', type='camera')
    elem(sensor, 'pose', vector(c['camera']['sensor_offset_m'] + c['camera']['sensor_rpy_rad']))
    elem(sensor, 'always_on', 'true')
    elem(sensor, 'update_rate', c['camera']['fps'])
    cam = elem(sensor, 'camera')
    elem(cam, 'horizontal_fov', number(math.radians(c['camera']['hfov_deg'])))
    image = elem(cam, 'image')
    for key in ('width', 'height'):
        elem(image, key, c['camera'][key])
    elem(image, 'format', 'R8G8B8')
    clip = elem(cam, 'clip')
    elem(clip, 'near', c['camera']['near_m'])
    elem(clip, 'far', c['camera']['far_m'])
    for axis, parent, child in [('yaw', 'mount_link', 'yaw_arm_link'),
                                 ('pitch', 'yaw_arm_link', 'camera_link')]:
        joint(model, axis + '_joint', parent, child, c['kinematics'][axis + '_axis'],
              c[axis + '_limits_deg'], c['max_rate_dps'])
    plugin = elem(model, 'plugin', filename='gz-two-axis-pod-controller', name='two_axis::GimbalController')
    settings = dict(yaw_joint='yaw_joint', pitch_joint='pitch_joint', mount_link='mount_link',
                    camera_link='camera_link', camera_sensor='camera', mount_mode='upright',
                    command_topic='/two_axis_pod/command', telemetry_topic='/two_axis_pod/telemetry',
                    command_result_topic='/two_axis_pod/command_result', control_rate=100,
                    telemetry_rate=50, max_rate_dps=c['max_rate_dps'], max_acc_dps2=c['max_acc_dps2'],
                    yaw_min_deg=c['yaw_limits_deg'][0], yaw_max_deg=c['yaw_limits_deg'][1],
                    pitch_min_deg=c['pitch_limits_deg'][0], pitch_max_deg=c['pitch_limits_deg'][1])
    for key, value in settings.items():
        elem(plugin, key, value)
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(ROOT / 'model.sdf', encoding='utf-8', xml_declaration=True)
    world = ET.parse(ROOT / 'tests/world_base.sdf')
    fixture_pod = copy.deepcopy(model)
    elem(fixture_pod, 'pose', '0 0 1.5 0 0 0')
    fixed = elem(fixture_pod, 'joint', name='world_mount', type='fixed')
    elem(fixed, 'parent', 'world')
    elem(fixed, 'child', 'mount_link')
    world.getroot().find('world').append(fixture_pod)
    target = world.getroot().find("world/model[@name='tracking_target']/link")
    for row in range(5):
        for col in range(5):
            if (row + col) % 2 == 0:
                part(target, f'tile_{row}_{col}', 'box', {'size': '.085 .085 .003'},
                     f'{(row-2)*.105} {(col-2)*.105} -.007 0 0 0', '.08 .12 .18 1', False)
    part(target, 'orientation_mark', 'box', {'size': '.06 .18 .003'},
         '.27 .17 -.008 0 0 0', '.8 .15 .08 1', False)
    ET.indent(world, space='  ')
    world.write(ROOT / 'tests/world.sdf', encoding='utf-8', xml_declaration=True)


if __name__ == '__main__':
    generate()
