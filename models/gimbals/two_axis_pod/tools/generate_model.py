#!/usr/bin/env python3
"""由参数生成立装两轴吊舱；坐标 X 朝前、Y 朝左、Z 朝天。"""
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


def ring_mesh(path, z0, z1, outer_radius, inner_radius, segments=96):
    vertices = []
    # 每个面使用独立顶点，避免圆柱面与端面共享法向产生放射状阴影。
    rings = ((z0, outer_radius), (z1, outer_radius),
             (z0, inner_radius), (z1, inner_radius),
             (z0, outer_radius), (z0, inner_radius),
             (z1, outer_radius), (z1, inner_radius))
    for z, radius in rings:
        for index in range(segments):
            angle = 2 * math.pi * index / segments
            vertices.append((-radius * math.sin(angle), radius * math.cos(angle), z))
    faces = []
    for index in range(segments):
        next_index = (index + 1) % segments
        outer0, outer1 = index, segments + index
        outer0n, outer1n = next_index, segments + next_index
        inner0, inner1 = 2 * segments + index, 3 * segments + index
        inner0n, inner1n = 2 * segments + next_index, 3 * segments + next_index
        back_outer, back_inner = 4 * segments + index, 5 * segments + index
        back_outer_n, back_inner_n = 4 * segments + next_index, 5 * segments + next_index
        front_outer, front_inner = 6 * segments + index, 7 * segments + index
        front_outer_n, front_inner_n = 6 * segments + next_index, 7 * segments + next_index
        faces.extend([
            (outer0, outer0n, outer1n, outer1),
            (inner0, inner1, inner1n, inner0n),
            (back_outer, back_inner, back_inner_n, back_outer_n),
            (front_outer, front_outer_n, front_inner_n, front_inner),
        ])
    write_dae(path, vertices, faces, 'carrier_ring')


def shell_mesh(path, overall_height, segments=64):
    """图纸轴向竖直放置：底环水平，后部弧形罩壳向上延伸至 91 mm。"""
    sections = [
        (.018, .0400, .0060, .0410),
        (.040, .0400, .0070, .0410),
        (.060, .0380, .0090, .0390),
        (.075, .0340, .0100, .0330),
        (.087, .0270, .0110, .0230),
        (overall_height, .0180, .0120, .0150),
    ]
    vertices = []
    for z, half_width, bottom, top in sections:
        center = (bottom + top) / 2
        half_height = (top - bottom) / 2
        for index in range(segments):
            angle = 2 * math.pi * index / segments
            vertices.append((-(center + half_height * math.sin(angle)),
                             half_width * math.cos(angle), z))
    faces = []
    for section in range(len(sections) - 1):
        start = section * segments
        following = (section + 1) * segments
        for index in range(segments):
            next_index = (index + 1) % segments
            faces.append((start + index, start + next_index,
                          following + next_index, following + index))
    faces.append(tuple(reversed(range(segments))))
    last = (len(sections) - 1) * segments
    faces.append(tuple(last + index for index in range(segments)))
    write_dae(path, vertices, faces, 'rear_shell')


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
    mesh_dir = ROOT / 'meshes'
    outer_radius = c['housing_diameter_m'] / 2
    flange = c['flange_thickness_m']
    ring_mesh(mesh_dir / 'carrier_ring.dae', flange, .028, outer_radius - .0005, .030)
    shell_mesh(mesh_dir / 'rear_shell.dae', c['overall_height_m'])
    sdf = ET.Element('sdf', version='1.9')
    m = elem(sdf, 'model', name='two_axis_pod')
    elem(m, 'self_collide', 'false')
    mount_mass = .10
    mount = link(m, 'mount_link', mount_mass, (0, 0, 0), (.00009, .00009, .00009))
    elem(mount.find('inertial'), 'pose', f'0 0 {flange/2} 0 0 0')
    part(mount, 'mounting_flange', 'cylinder',
         {'radius': outer_radius, 'length': flange},
         f'0 0 {flange/2} 0 0 0')
    holes = c['mounting_holes']
    hole_radius = holes['thread_diameter_m'] / 2
    hole_circle = holes['circle_diameter_m'] / 2
    for index in range(holes['count']):
        angle = 2 * math.pi * index / holes['count']
        x = hole_circle * math.cos(angle)
        y = hole_circle * math.sin(angle)
        part(mount, f'm3_hole_{index + 1}', 'cylinder',
             {'radius': hole_radius, 'length': .0008},
             f'{x} {y} {flange+.0004} 0 0 0', '.018 .018 .020 1', False)
    cx, cy, cz = c['camera_center_m']
    r = c['camera_diameter_m'] / 2
    side_y = r + .004
    arm = link(m, 'yaw_arm_link', .04, (0, 0, flange), (.00004, .00005, .00005))
    elem(arm.find('inertial'), 'pose', f'-.012 0 {cz/2-flange} 0 0 0')
    mesh_part(arm, 'carrier_ring', 'model://two_axis_pod/meshes/carrier_ring.dae',
              f'0 0 {-flange} 0 0 0')
    mesh_part(arm, 'rear_shell', 'model://two_axis_pod/meshes/rear_shell.dae',
              f'0 0 {-flange} 0 0 0')
    # 外壳、支架整体随偏航绕底环中心转动，球体在支架内俯仰。
    part(arm, 'yaw_drive', 'cylinder', {'radius': .012, 'length': .018},
         '0 0 .009 0 0 0')
    part(arm, 'rear_support', 'box', {'size': '.010 .018 .030'},
         f'-.025 0 {.034-flange} 0 0 0')
    part(arm, 'yoke_crossbar', 'box', {'size': f'.036 {2*side_y} .006'},
         f'-.003 0 {cz-.018-flange} 0 0 0')
    for side in (-1, 1):
        y = side * side_y
        part(arm, f'yoke_arm_{"left" if side > 0 else "right"}', 'box',
             {'size': '.008 .006 .024'}, f'{cx} {y} {cz-.008-flange} 0 0 0')
        part(arm, f'pitch_motor_{"left" if side > 0 else "right"}', 'cylinder',
             {'radius': .010, 'length': .006}, f'{cx} {y} {cz-flange} {math.pi/2} 0 0')
    part(arm, 'pitch_axle', 'cylinder', {'radius': .0045, 'length': f'{2*side_y}'},
         f'{cx} 0 {cz-flange} {math.pi/2} 0 0', collision=False)
    collision = elem(arm, 'collision', name='rear_shell_collision')
    elem(collision, 'pose', f'-.028 0 {.049-flange} 0 0 0')
    geometry(collision, 'box', {'size': '.015 .060 .065'})
    camera = link(m, 'camera_link', .1, (cx, cy, cz), (2*.1*r*r/5,)*3)
    part(camera, 'camera_shell', 'sphere', {'radius': r})
    part(camera, 'lens_bezel', 'cylinder', {'radius': .013, 'length': .005}, f'0 0 {r-.0015} 0 0 0', '.20 .20 .21 1', False)
    part(camera, 'lens_glass', 'cylinder', {'radius': .0105, 'length': .001}, f'0 0 {r+.0015} 0 0 0', '.12 .22 .30 1', False)
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
    joint(m, 'yaw_joint', 'mount_link', 'yaw_arm_link', '0 0 1', c['yaw_limits_deg'], c['max_rate_dps'])
    joint(m, 'pitch_joint', 'yaw_arm_link', 'camera_link', '0 -1 0', c['pitch_limits_deg'], c['max_rate_dps'])
    plugin = elem(m, 'plugin', filename='gz-two-axis-pod-controller', name='two_axis::GimbalController')
    settings = dict(yaw_joint='yaw_joint', pitch_joint='pitch_joint', mount_link='mount_link', camera_link='camera_link', camera_sensor='camera', mount_mode='upright', command_topic='/two_axis_pod/command', telemetry_topic='/two_axis_pod/telemetry', command_result_topic='/two_axis_pod/command_result', control_rate=100, telemetry_rate=50, max_rate_dps=c['max_rate_dps'], max_acc_dps2=c['max_acc_dps2'], yaw_min_deg=c['yaw_limits_deg'][0], yaw_max_deg=c['yaw_limits_deg'][1], pitch_min_deg=c['pitch_limits_deg'][0], pitch_max_deg=c['pitch_limits_deg'][1])
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
