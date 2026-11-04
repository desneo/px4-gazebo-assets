#!/usr/bin/env python3
"""使用中科统一INI、复制CAD及质量属性生成前视两轴组件与独立演示。"""
from __future__ import annotations
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common.config import load_config, to_model_parameters

MODEL_URI = 'model://zhongke-d80'


def elem(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def number(value):
    return format(float(value), '.15g')


def vector(values):
    return ' '.join(number(x) for x in values)


def geometry(parent, shape, values):
    node = elem(elem(parent, 'geometry'), shape)
    for key, value in values.items():
        elem(node, key, value)


def visual(parent, name, shape, values, pose, color):
    node = elem(parent, 'visual', name=name)
    elem(node, 'pose', pose)
    geometry(node, shape, values)
    material = elem(node, 'material')
    elem(material, 'ambient', color)
    elem(material, 'diffuse', color)
    return node


def load_geometry():
    properties = json.loads((ROOT / 'cad/properties.json').read_text())
    for relative, expected in properties['sha256'].items():
        if hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() != expected:
            raise ValueError(f'复制CAD或网格校验失败：{relative}')
    return properties


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


def camera_sensor(node, config, topic):
    sensor = elem(node, 'sensor', name='camera', type='camera')
    elem(sensor, 'pose', vector(config['sensor_offset_m'] + config['sensor_rpy_rad']))
    elem(sensor, 'always_on', 'true')
    elem(sensor, 'update_rate', number(config['fps']))
    elem(sensor, 'topic', topic)
    camera = elem(sensor, 'camera')
    elem(camera, 'camera_info_topic', topic.rsplit('/', 1)[0] + '/camera_info')
    # SDFormat的兼容HFOV最小为0.1rad；实际投影由独立fx/fy覆盖。
    elem(camera, 'horizontal_fov', number(max(.1, math.radians(config['hfov_deg']))))
    image = elem(camera, 'image')
    elem(image, 'width', config['width'])
    elem(image, 'height', config['height'])
    elem(image, 'format', 'R8G8B8')
    clip = elem(camera, 'clip')
    elem(clip, 'near', number(config['near_m']))
    elem(clip, 'far', number(config['far_m']))
    lens = elem(camera, 'lens')
    elem(lens, 'type', 'gnomonical')
    elem(lens, 'scale_to_hfov', 'false')
    intrinsics = elem(lens, 'intrinsics')
    for key in ('fx', 'fy', 'cx', 'cy'):
        elem(intrinsics, key, number(config[key]))
    elem(intrinsics, 's', '0')


def create_model(c, cfg, properties):
    sdf = ET.Element('sdf', version='1.10')
    model = elem(sdf, 'model', name=cfg['model_name'])
    # 整机安装姿态只在模型级施加一次；内部几何和轴向不变。
    elem(model, 'pose', vector([0, 0, 0, 0, math.pi / 2, 0]))
    elem(model, 'self_collide', 'false')
    links = {}
    for name, group in properties['groups'].items():
        mass = cfg['mass_kg'] * group['volume_mm3'] / properties['volume_mm3']
        node = links[name] = link(model, name, group, mass)
        for part_name in group['parts']:
            part = properties['parts'][part_name]
            uri = MODEL_URI + '/' + part['mesh']
            visual(node, part_name + '_visual', 'mesh', {'uri': uri},
                   '0 0 0 0 0 0', vector(part['rgba']))
            geometry(elem(node, 'collision', name=part_name + '_collision'), 'mesh', {'uri': uri})
    topic_prefix = '/' + cfg['model_name']
    camera_sensor(links['camera_link'], cfg['camera'], topic_prefix + '/camera')
    for axis, parent, child in [('yaw', 'mount_link', 'yaw_arm_link'),
                                ('pitch', 'yaw_arm_link', 'camera_link')]:
        joint = elem(model, 'joint', name=axis + '_joint', type='revolute')
        elem(joint, 'parent', parent)
        elem(joint, 'child', child)
        elem(joint, 'pose', '0 0 0 0 0 0')
        axis_node = elem(joint, 'axis')
        elem(axis_node, 'xyz', vector(cfg['kinematics'][axis + '_axis']))
        limits = elem(axis_node, 'limit')
        elem(limits, 'lower', number(math.radians(cfg[axis + '_limits_deg'][0])))
        elem(limits, 'upper', number(math.radians(cfg[axis + '_limits_deg'][1])))
        elem(limits, 'effort', 5)
        elem(limits, 'velocity', number(math.radians(cfg['max_rate_dps'])))
        elem(elem(axis_node, 'dynamics'), 'damping', '.1')
    plugin = elem(model, 'plugin', filename='gz-zhongke-d80-controller', name='zhongke_d80::GimbalController')
    settings = dict(yaw_joint='yaw_joint', pitch_joint='pitch_joint',
        mount_link='mount_link', camera_link='camera_link', camera_sensor='camera',
        mount_mode='upright', command_topic=topic_prefix + '/command',
        telemetry_topic=topic_prefix + '/telemetry', command_result_topic=topic_prefix + '/command_result',
        image_topic=topic_prefix + '/camera', control_rate=100, tracking_backend='sim_truth',
        telemetry_hz=cfg['telemetry_hz'], max_rate_dps=cfg['max_rate_dps'],
        max_acc_dps2=cfg['max_acc_dps2'], servo_kp=cfg['kp'],
        stop_margin_deg=cfg['stop_margin_deg'], manual_timeout_s=cfg['manual_timeout_s'],
        discovery_range_m=cfg['discovery_range_m'], config_hash=cfg['config_hash'],
        model_name=cfg['model_name'], occlusion_timeout_s=c['tracking']['lost_timeout_s'],
        yaw_min_deg=cfg['yaw_limits_deg'][0], yaw_max_deg=cfg['yaw_limits_deg'][1],
        pitch_min_deg=cfg['pitch_limits_deg'][0], pitch_max_deg=cfg['pitch_limits_deg'][1])
    for key, value in settings.items():
        elem(plugin, key, value)
    for name in cfg['target_names']:
        elem(plugin, 'sim_truth_target_name', name)
    return sdf, model


def create_world(c, cfg, model):
    sdf = ET.Element('sdf', version='1.10')
    world = elem(sdf, 'world', name=cfg['world_name'])
    elem(world, 'gravity', '0 0 -9.8')
    physics = elem(world, 'physics', type='ode')
    elem(physics, 'max_step_size', '.004')
    elem(physics, 'real_time_factor', '1')
    for filename, name in [('gz-sim-physics-system', 'Physics'),
        ('gz-sim-user-commands-system', 'UserCommands'),
        ('gz-sim-scene-broadcaster-system', 'SceneBroadcaster')]:
        elem(world, 'plugin', filename=filename, name='gz::sim::systems::' + name)
    sensors = elem(world, 'plugin', filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
    elem(sensors, 'render_engine', 'ogre2')
    scene = elem(world, 'scene')
    elem(scene, 'ambient', '.7 .7 .7 1')
    elem(scene, 'background', '.65 .8 .95 1')
    gui = elem(world, 'gui', fullscreen='false')
    view = elem(gui, 'plugin', filename='MinimalScene', name='3D View')
    elem(view, 'engine', 'ogre2')
    elem(view, 'scene', 'scene')
    elem(view, 'camera_pose', '.55 -.45 1.65 0 .2 2.4')
    clip = elem(view, 'camera_clip')
    elem(clip, 'near', c['viewer']['near_clip_m'])
    elem(clip, 'far', '30000')
    for filename in ('GzSceneManager', 'InteractiveViewControl', 'CameraTracking', 'EntityTree', 'WorldControl'):
        elem(gui, 'plugin', filename=filename, name=filename)
    light = elem(world, 'light', name='sun', type='directional')
    elem(light, 'diffuse', '.9 .9 .9 1')
    elem(light, 'direction', '-.3 .1 -1')
    ground = elem(world, 'model', name='ground')
    elem(ground, 'static', 'true')
    visual(elem(ground, 'link', name='link'), 'surface', 'plane',
           {'normal': '0 0 1', 'size': '1000 1000'}, '0 0 0 0 0 0', '.35 .4 .35 1')
    fixture = copy.deepcopy(model)
    fixture.find('pose').text = vector([0, 0, 1.5, 0, math.pi / 2, 0])
    fixed = elem(fixture, 'joint', name='world_mount', type='fixed')
    elem(fixed, 'parent', 'world')
    elem(fixed, 'child', 'mount_link')
    world.append(fixture)
    pedestal = elem(world, 'model', name='zhongke_d80_pedestal')
    elem(pedestal, 'static', 'true')
    visual(elem(pedestal, 'link', name='link'), 'post', 'box', {'size': '.12 .12 1.44'},
           '-.08 0 .72 0 0 0', '.3 .3 .35 1')
    marker = elem(world, 'model', name=c['demo']['marker_name'])
    elem(marker, 'static', 'true')
    marker_link = elem(marker, 'link', name='link')
    visual(marker_link, 'shaft', 'box', {'size': '.025 .23 .025'},
           '-.04 -.26 1.52 0 0 0', '1 .2 .01 1')
    visual(marker_link, 'arrowhead', 'cone', {'radius': '.05', 'length': '.12'},
           '-.04 -.1 1.52 -1.5707963267948966 0 0', '1 .2 .01 1')
    for index, name in enumerate(cfg['target_names']):
        target = elem(world, 'model', name=name)
        elem(target, 'static', 'true')
        # 中央和左右目标在默认7.03°窄视场内，仍以实体原点作为跟踪参考。
        offset = (index + 1) // 2 * (1 if index % 2 else -1) if index else 0
        elem(target, 'pose', vector([50, offset * 1.4, 1.485, 0, 0, 0]))
        node = elem(target, 'link', name='link')
        color = ['.95 .15 .02 1', '.02 .3 .95 1', '.95 .85 .05 1'][index % 3]
        visual(node, 'body', 'box', {'size': '.4 .2 .2'}, '0 0 0 0 0 0', color)
        visual(node, 'wing', 'box', {'size': '.16 1 .04'}, '0 0 0 0 0 0', color)
    return sdf


def write_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def generate(config_path=None, output_dir=None):
    c = load_config(config_path or ROOT / 'parameters.ini')
    cfg = to_model_parameters(c)
    properties = load_geometry()
    destination = Path(output_dir).resolve() if output_dir else ROOT
    sdf, model = create_model(c, cfg, properties)
    write_xml(destination / 'model.sdf', sdf)
    write_xml(destination / 'tests/world.sdf', create_world(c, cfg, model))
    metadata = dict(config_path=c['config_path'], config_hash=c['config_hash'],
                    mass_kg=cfg['mass_kg'], camera=cfg['camera'])
    (destination / 'generated.json').write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + '\n')
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'parameters.ini')
    parser.add_argument('--output-dir', type=Path, default=ROOT)
    args = parser.parse_args()
    try:
        destination = generate(args.config, args.output_dir)
        print(f'中科模型与演示已生成：{destination}')
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(2, f'中科模型生成失败：{exc}\n')


if __name__ == '__main__':
    main()
