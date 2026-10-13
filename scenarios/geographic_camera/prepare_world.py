#!/usr/bin/env python3
"""由真实地理环境生成固定吊舱视觉验证运行世界。"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET
from scene_geometry import target_x

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'worlds/geo_32_217857_119_105878/world.sdf'
WORLD = 'geographic_camera'


def set_text(parent, tag, value):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = value


def box(name, pose, size, color, collision=False):
    model = ET.fromstring(f'''<model name="{name}"><static>true</static><pose>{pose}</pose>
      <link name="body"><visual name="surface"><geometry><box><size>{size}</size></box></geometry>
      <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link></model>''')
    if collision:
        body = model.find('link')
        shape = ET.SubElement(body, 'collision', name='surface_collision')
        geometry = ET.SubElement(shape, 'geometry')
        set_text(ET.SubElement(geometry, 'box'), 'size', size)
    return model


def prepare(output, mount_mode='skyward'):
    tree = ET.parse(SOURCE)
    world = tree.getroot().find('world')
    world.set('name', WORLD)
    # 副本位于运行目录，显式解析原世界的地形及影像路径。
    for tag in ('uri', 'albedo_map'):
        for element in world.iter(tag):
            if element.text and element.text.strip().startswith('assets/'):
                element.text = str((SOURCE.parent / element.text.strip()).resolve())
    pod = ET.parse(ROOT / 'models/gimbals/three_axis_pod/model.sdf').getroot().find('model')
    poses = {'skyward': '0 0 50 0 0 0', 'downward': '0 0 50 3.141592653589793 0 0',
             'sideward': '0 0 50 0 1.5707963267948966 0'}
    set_text(pod, 'pose', poses[mount_mode])
    set_text(pod.find('plugin'), 'mount_mode', 'downward' if mount_mode == 'downward' else 'skyward')
    if mount_mode == 'downward':
        sensor_pose = pod.find('./link[@name="camera_link"]/sensor[@name="camera"]/pose')
        values = sensor_pose.text.split()
        sensor_pose.text = ' '.join(values[:3] + ['0', '-1.5707963267948966', '0'])
    x = target_x(mount_mode)
    pod.append(ET.fromstring('''<joint name="fixed_installation" type="fixed">
      <parent>world</parent><child>cgo3_mount_link</child></joint>'''))
    world.append(pod)
    # 目标平台悬于地形上方，避免 DEM 高程/物理接触影响视觉实验。
    world.append(box('target_platform', f'{x} 0 29.8 0 0 0', '50 50 0.4', '0.35 0.4 0.42 1', collision=True))
    car = ET.parse(ROOT / 'models/props/test_car/model.sdf').getroot().find('model')
    car.set('name', 'tracking_target')
    set_text(car, 'static', 'false')
    set_text(car, 'pose', f'{x} 0 30.6 0 0 0')
    # Gazebo VelocityControl 驱动动态刚体；原模型轮子仅用于显示。
    car.find('./plugin/topic').text = '/model/tracking_target/cmd_vel'
    car.append(ET.fromstring('''<plugin filename="gz-sim-odometry-publisher-system"
      name="gz::sim::systems::OdometryPublisher"><dimensions>3</dimensions>
      <odom_publish_frequency>20</odom_publish_frequency>
      <odom_topic>/model/tracking_target/odometry</odom_topic></plugin>'''))
    world.append(car)
    world.append(box('occlusion_panel', '40 0 40 0 0 0', '20 20 0.2', '0.22 0.22 0.22 1'))
    camera_pose = world.find('./gui/plugin/camera_pose')
    if camera_pose is not None:
        camera_pose.text = '-35 -35 65 0 0.65 0.785398'
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'runs/geographic_camera/world.sdf')
    parser.add_argument('--mount-mode', choices=['skyward', 'downward', 'sideward'], default='skyward')
    args = parser.parse_args()
    print(prepare(args.output.resolve(), args.mount_mode))
