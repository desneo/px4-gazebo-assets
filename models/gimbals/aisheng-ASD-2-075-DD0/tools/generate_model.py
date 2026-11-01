#!/usr/bin/env python3
"""从 two_axis_pod 的已导入 CAD 网格生成 ASD-2-075-DD0 外观/转动预览。

预览沿用 two_axis_pod 的内部机械链：机械零位 +Z（pod 内部朝上），
pitch 负值转向 +X；整机安装绕 Y 轴 +90°，使 pod 内部 +Z 对载机 +X。
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL_NAME = 'aisheng_seeker'
MODEL_URI = 'model://aisheng-ASD-2-075-DD0'


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


def number(value):
    return format(float(value), '.12g')


def vector(values):
    return ' '.join(number(v) for v in values)


def load_geometry():
    properties = json.loads((ROOT / 'cad/properties.json').read_text())
    for relative, expected in properties['sha256'].items():
        if hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f'CAD 与导出资产不一致：{relative}')
    return properties


def validate_configuration(c, properties):
    rate = c.get('max_rate_dps')
    if (type(rate) not in (int, float) or not 0 < rate <= 60
            or not math.isfinite(rate)):
        raise ValueError('max_rate_dps 必须为 (0, 60] 的有限数值')
    if c['camera_center_m'] != properties['groups']['camera_link']['origin_m']:
        raise ValueError('相机球心必须继承 CAD 导出属性')
    if c['kinematics']['yaw_origin_m'] != properties['groups']['yaw_arm_link']['origin_m']:
        raise ValueError('外轴轴心必须继承 CAD 导出属性')
    if c['kinematics']['yaw_axis'] != [1, 0, 0] or c['kinematics']['pitch_axis'] != [0, -1, 0]:
        raise ValueError('机械轴必须为外轴 +X 及内轴 -Y')
    expected = [0, -math.pi / 2, 0]
    for key in ('camera', 'tele_camera'):
        rpy = c[key]['sensor_rpy_rad']
        if len(rpy) != 3 or any(not math.isfinite(v) or abs(v - e) > 1e-10
                                for v, e in zip(rpy, expected)):
            raise ValueError(f'{key} 光学基底必须与 two_axis_pod 一致')
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
    elem(node, 'pose', '0 0 0 0 0 0')
    a = elem(node, 'axis')
    elem(a, 'xyz', vector(axis))
    limit = elem(a, 'limit')
    elem(limit, 'lower', number(math.radians(limits[0])))
    elem(limit, 'upper', number(math.radians(limits[1])))
    elem(limit, 'effort', 5)
    elem(limit, 'velocity', number(math.radians(rate)))
    elem(elem(a, 'dynamics'), 'damping', .1)


def camera_sensor(link, name, cfg, topic):
    sensor = elem(link, 'sensor', name=name, type='camera')
    elem(sensor, 'pose', vector(cfg['sensor_offset_m'] + cfg['sensor_rpy_rad']))
    elem(sensor, 'always_on', 'true')
    elem(sensor, 'update_rate', cfg['fps'])
    elem(sensor, 'topic', topic)
    cam = elem(sensor, 'camera')
    elem(cam, 'horizontal_fov', number(math.radians(cfg['hfov_deg'])))
    image = elem(cam, 'image')
    for key in ('width', 'height'):
        elem(image, key, cfg[key])
    elem(image, 'format', 'R8G8B8')
    clip = elem(cam, 'clip')
    elem(clip, 'near', cfg['near_m'])
    elem(clip, 'far', cfg['far_m'])


def visual_box(link, name, size, pose, color):
    part(link, name, 'box', {'size': size}, pose, color, False)


def add_tele_lens_visual(link):
    """在球头前下方增加长焦镜圈；仅视觉，无碰撞、无质量。"""
    barrel = elem(link, 'visual', name='tele_lens_barrel_visual')
    elem(barrel, 'pose', '0.013 0 0.0205 0 0 0')
    geometry(barrel, 'cylinder', {'radius': '.0082', 'length': '.004'})
    mat = elem(barrel, 'material')
    elem(mat, 'ambient', '.1333 .1725 .2118 1')
    elem(mat, 'diffuse', '.1333 .1725 .2118 1')
    elem(mat, 'specular', '.45 .45 .45 1')
    window = elem(link, 'visual', name='tele_lens_window_visual')
    elem(window, 'pose', '0.013 0 0.0228 0 0 0')
    geometry(window, 'cylinder', {'radius': '.0054', 'length': '.001'})
    mat = elem(window, 'material')
    elem(mat, 'ambient', '.2039 .3804 .4784 1')
    elem(mat, 'diffuse', '.2039 .3804 .4784 1')
    elem(mat, 'specular', '.55 .55 .55 1')


def generate_preview_world(model):
    sdf = ET.Element('sdf', version='1.9')
    world = elem(sdf, 'world', name='aisheng_seeker_preview')
    elem(world, 'gravity', '0 0 -9.8')
    physics = elem(world, 'physics', type='ode')
    elem(physics, 'max_step_size', '0.004')
    elem(physics, 'real_time_factor', '1')
    for filename, name in [
            ('gz-sim-physics-system', 'gz::sim::systems::Physics'),
            ('gz-sim-user-commands-system', 'gz::sim::systems::UserCommands'),
            ('gz-sim-scene-broadcaster-system', 'gz::sim::systems::SceneBroadcaster')]:
        elem(world, 'plugin', filename=filename, name=name)
    sensors = elem(world, 'plugin', filename='gz-sim-sensors-system',
                   name='gz::sim::systems::Sensors')
    elem(sensors, 'render_engine', 'ogre2')

    scene = elem(world, 'scene')
    elem(scene, 'ambient', '.65 .65 .65 1')
    elem(scene, 'background', '.82 .87 .93 1')
    elem(scene, 'grid', 'false')
    elem(scene, 'shadows', 'true')

    gui = elem(world, 'gui', fullscreen='false')
    minimal = elem(gui, 'plugin', filename='MinimalScene', name='3D View')
    elem(minimal, 'engine', 'ogre2')
    elem(minimal, 'scene', 'scene')
    elem(minimal, 'camera_pose', '0.55 -0.50 1.62 0 0.18 2.30')
    clip = elem(minimal, 'camera_clip')
    elem(clip, 'near', '0.005')
    elem(clip, 'far', '100.0')
    for filename, name in [
            ('GzSceneManager', 'Scene Manager'),
            ('InteractiveViewControl', 'Interactive view control'),
            ('CameraTracking', 'Camera Tracking'),
            ('EntityTree', 'Entity tree'),
            ('WorldControl', 'World control')]:
        elem(gui, 'plugin', filename=filename, name=name)

    light = elem(world, 'light', name='sun', type='directional')
    elem(light, 'pose', '0 0 3 0 0 0')
    elem(light, 'diffuse', '.85 .85 .85 1')
    elem(light, 'specular', '.3 .3 .3 1')
    elem(light, 'direction', '-.4 .5 -1')

    ground = elem(world, 'model', name='ground')
    elem(ground, 'static', 'true')
    glink = elem(ground, 'link', name='link')
    part(glink, 'ground', 'plane', {'normal': '0 0 1', 'size': '20 20'},
         '0 0 0 0 0 0', '.40 .42 .45 1', False)

    pedestal = elem(world, 'model', name='seeker_pedestal')
    elem(pedestal, 'static', 'true')
    plink = elem(pedestal, 'link', name='link')
    visual_box(plink, 'post', '.18 .18 1.46', '0 0 .73 0 0 0', '.30 .32 .35 1')

    wall = elem(world, 'model', name='target_wall')
    elem(wall, 'static', 'true')
    elem(wall, 'pose', '4 0 1.5 0 0 0')
    wlink = elem(wall, 'link', name='link')
    visual_box(wlink, 'panel', '.02 2.4 2.4', '0 0 0 0 0 0', '.90 .90 .90 1')
    colors = ['.12 .28 .45 1', '.80 .55 .20 1', '.20 .55 .35 1', '.55 .25 .55 1']
    for row in range(5):
        for col in range(5):
            visual_box(wlink, f'tile_{row}_{col}', '.012 .35 .35',
                       f'-.016 {(col-2)*.40:.3f} {(row-2)*.40:.3f} 0 0 0',
                       colors[(row + col) % len(colors)])
    visual_box(wlink, 'center_vertical', '.012 .06 .60', '-.020 0 0 0 0 0', '.85 .08 .08 1')
    visual_box(wlink, 'center_horizontal', '.012 .60 .06', '-.020 0 0 0 0 0', '.85 .08 .08 1')

    overview = elem(world, 'model', name='overview')
    elem(overview, 'static', 'true')
    elem(overview, 'pose', '0.35 -0.35 1.62 0 0.25 2.356')
    olink = elem(overview, 'link', name='link')
    overview_cfg = {
        'width': 960, 'height': 720, 'fps': 10, 'hfov_deg': 37,
        'near_m': 0.005, 'far_m': 100,
        'sensor_offset_m': [0, 0, 0], 'sensor_rpy_rad': [0, 0, 0]}
    camera_sensor(olink, 'overview', overview_cfg, '/aisheng_seeker/overview')

    for name, pose, color in [
            ('target_aircraft', '3.7 0.55 1.5 0 0 0', '.95 .35 .05 1'),
            ('target_aircraft_2', '3.7 -0.70 1.70 0 0 0', '.10 .55 .95 1')]:
        target = elem(world, 'model', name=name)
        elem(target, 'static', 'true')
        elem(target, 'pose', pose)
        tlink = elem(target, 'link', name='link')
        visual_box(tlink, 'body', '.36 .10 .10', '0 0 0 0 0 0', color)
        visual_box(tlink, 'wing', '.12 .52 .035', '0 0 0 0 0 0', '.92 .92 .92 1')
        visual_box(tlink, 'tail', '.10 .18 .035', '-.18 0 0 0 0 0', color)
        visual_box(tlink, 'nose', '.08 .07 .07', '.20 0 0 0 0 0', '.95 .95 .95 1')

    marker = elem(world, 'model', name='AISHENG_SEEKER_LOCATION_MARKER')
    elem(marker, 'static', 'true')
    elem(marker, 'pose', '-0.12 -0.28 1.55 0 0 0')
    mlink = elem(marker, 'link', name='link')
    visual_box(mlink, 'shaft', '.02 .22 .02', '0 0 0 0 0 0', '1 .35 .02 1')
    cone = elem(mlink, 'visual', name='arrowhead_visual')
    elem(cone, 'pose', '0 .17 0 -1.5707963267948966 0 0')
    geometry(cone, 'cone', {'radius': '.05', 'length': '.12'})
    mat = elem(cone, 'material')
    elem(mat, 'ambient', '1 .35 .02 1')
    elem(mat, 'diffuse', '1 .35 .02 1')
    elem(mat, 'specular', '.35 .35 .35 1')

    fixture = copy.deepcopy(model)
    elem(fixture, 'pose', '0 0 1.5 0 1.5707963267948966 0')
    fixed = elem(fixture, 'joint', name='world_mount', type='fixed')
    elem(fixed, 'parent', 'world')
    elem(fixed, 'child', 'mount_link')
    world.append(fixture)

    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(ROOT / 'tests/preview.world.sdf',
                              encoding='utf-8', xml_declaration=True)

    los_sdf = copy.deepcopy(sdf)
    los_world = los_sdf.find('world')
    motion = elem(los_world, 'plugin', filename='gz-aisheng-target-motion',
                  name='aisheng::TargetMotion')
    for key, value in [
            ('target_name', 'target_aircraft'),
            ('center_x', '3.7'), ('center_y', '0.55'), ('center_z', '1.5'),
            ('amp_y', '0.55'), ('amp_z', '0'),
            ('period_y', '2.0'), ('period_z', '8.0'),
            ('phase_y', '0'), ('phase_z', '0')]:
        elem(motion, key, value)
    ET.indent(los_sdf, space='  ')
    ET.ElementTree(los_sdf).write(ROOT / 'tests/los25.world.sdf',
                                  encoding='utf-8', xml_declaration=True)


def generate():
    c = json.loads((ROOT / 'parameters.json').read_text())
    properties = load_geometry()
    validate_configuration(c, properties)
    total_mass = c['mass_kg']
    sdf = ET.Element('sdf', version='1.9')
    model = elem(sdf, 'model', name=MODEL_NAME)
    elem(model, 'self_collide', 'false')
    links = {}
    for name, group in properties['groups'].items():
        mass = total_mass * group['volume_mm3'] / properties['volume_mm3']
        links[name] = node = link(model, name, group, mass)
        for part_name in group['parts']:
            part_data = properties['parts'][part_name]
            uri = MODEL_URI + '/' + part_data['mesh']
            mesh_part(node, part_name, uri, color=vector(part_data['rgba']))
            collision = elem(node, 'collision', name=part_name + '_collision')
            geometry(collision, 'mesh', {'uri': uri})

    camera = links['camera_link']
    add_tele_lens_visual(camera)
    camera_sensor(camera, 'camera', c['camera'], '/aisheng_seeker/camera')
    camera_sensor(camera, 'camera_tele', c['tele_camera'], '/aisheng_seeker/camera_tele')

    for axis, parent, child in [('yaw', 'mount_link', 'yaw_arm_link'),
                                ('pitch', 'yaw_arm_link', 'camera_link')]:
        joint(model, axis + '_joint', parent, child, c['kinematics'][axis + '_axis'],
              c[axis + '_limits_deg'], c['max_rate_dps'])

    plugin = elem(model, 'plugin', filename='gz-aisheng-seeker-controller',
                  name='two_axis::GimbalController')
    settings = dict(
        yaw_joint='yaw_joint', pitch_joint='pitch_joint', mount_link='mount_link',
        camera_link='camera_link', camera_sensor='camera',
        camera_sensor_tele='camera_tele', mount_mode='upright',
        command_topic='/aisheng_seeker/command',
        telemetry_topic='/aisheng_seeker/telemetry',
        command_result_topic='/aisheng_seeker/command_result',
        image_topic='/aisheng_seeker/camera',
        image_topic_tele='/aisheng_seeker/camera_tele',
        control_rate=100, tracking_backend='sim_truth',
        telemetry_config=MODEL_URI + '/controller/service/telemetry.json',
        max_rate_dps=c['max_rate_dps'], max_acc_dps2=c['max_acc_dps2'],
        servo_kp=c.get('servo_kp', 4),
        yaw_min_deg=c['yaw_limits_deg'][0], yaw_max_deg=c['yaw_limits_deg'][1],
        pitch_min_deg=c['pitch_limits_deg'][0], pitch_max_deg=c['pitch_limits_deg'][1])
    for key, value in settings.items():
        elem(plugin, key, value)
    if c.get('los_observer_cg_model') is not None:
        elem(plugin, 'los_observer_cg_model', vector(c['los_observer_cg_model']))
    for target_name in c.get('truth_target_names', []):
        elem(plugin, 'sim_truth_target_name', target_name)

    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(ROOT / 'model.sdf', encoding='utf-8', xml_declaration=True)
    generate_preview_world(model)


if __name__ == '__main__':
    generate()
