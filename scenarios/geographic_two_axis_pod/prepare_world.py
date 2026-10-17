#!/usr/bin/env python3
"""复制完整地理环境，加入标准立装两轴吊舱和空中视觉测试夹具。"""
import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'worlds/geo_32_217857_119_105878/world.sdf'
WORLD = 'geographic_two_axis_pod'


def set_text(parent, tag, value):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = value


def visual(link, name, pose, size, color):
    element = ET.SubElement(link, 'visual', name=name)
    set_text(element, 'pose', pose)
    geometry = ET.SubElement(element, 'geometry')
    set_text(ET.SubElement(geometry, 'box'), 'size', size)
    material = ET.SubElement(element, 'material')
    set_text(material, 'ambient', color)
    set_text(material, 'diffuse', color)
    return element


def aircraft_visuals(link, scale=5.0):
    """使用仓库内小蝙蝠固定翼外形构造无传感器的空中视觉夹具。"""
    for side in (-1, 1):
        element = ET.SubElement(link, 'visual', name=f'aircraft_wing_{side}')
        geometry = ET.SubElement(element, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        set_text(mesh, 'uri', f'model://xiaobianfu/meshes/wing_{side}.stl')
        set_text(mesh, 'scale', f'{scale} {scale} {scale}')
        material = ET.SubElement(element, 'material')
        set_text(material, 'ambient', '.95 .96 .92 1')
        set_text(material, 'diffuse', '.95 .96 .92 1')
        y = side * .27 * scale
        visual(link, f'aircraft_stripe_{side}', f'{.005*scale} {y} {.014*scale} 0 0 {-side*.43}',
               f'{.045*scale} {.42*scale} {.003*scale}', '.03 .6 .03 1')
        visual(link, f'aircraft_winglet_{side}',
               f'{-.115*scale} {side*.492*scale} {.045*scale} 0 0 0',
               f'{.13*scale} {.012*scale} {.10*scale}', '.03 .6 .03 1')
    pod = ET.SubElement(link, 'visual', name='aircraft_central_pod')
    set_text(pod, 'pose', f'{.025*scale} 0 {.012*scale} 0 0 0')
    geometry = ET.SubElement(pod, 'geometry')
    set_text(ET.SubElement(geometry, 'ellipsoid'), 'radii',
             f'{.175*scale} {.075*scale} {.04*scale}')
    material = ET.SubElement(pod, 'material')
    set_text(material, 'ambient', '.03 .6 .03 1')
    set_text(material, 'diffuse', '.03 .6 .03 1')
    visual(link, 'aircraft_canopy', f'{.07*scale} 0 {.047*scale} 0 0 0',
           f'{.13*scale} {.07*scale} {.022*scale}', '.08 .10 .12 1')


def target_model():
    model = ET.fromstring('''<model name="tracking_target">
      <static>false</static><pose>30 0 70 0 0 3.14159265359</pose>
      <link name="body"><gravity>false</gravity>
        <inertial><mass>1</mass><inertia><ixx>0.96</ixx><iyy>0.483333333333</iyy>
          <izz>0.483333333333</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="body_collision"><geometry><box><size>1.8 5.2 0.55</size></box></geometry></collision>
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <topic>/model/tracking_target/cmd_vel</topic></plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <dimensions>3</dimensions><odom_publish_frequency>20</odom_publish_frequency>
        <odom_topic>/model/tracking_target/odometry</odom_topic></plugin>
    </model>''')
    link = model.find('link')
    aircraft_visuals(link)
    return model


def detection_aircraft(name, pose, scale=4.0):
    """额外静态飞机目标，用于验证多目标编号和中心最近选择。"""
    model = ET.fromstring(
        f'<model name="{name}"><static>true</static><pose>{pose}</pose>'
        '<link name="body"/></model>')
    aircraft_visuals(model.find('link'), scale)
    return model



def location_marker():
    """无碰撞的定位灯塔；顶部箭头指向原点处50m高的吊舱。"""
    model = ET.fromstring('<model name="POD_LOCATION_MARKER"><static>true</static><link name="marker"/></model>')
    link = model.find('link')
    orange, dark, yellow = '1 .22 .015 1', '.06 .06 .06 1', '1 .9 .02 1'
    visual(link, 'ground_locator', '-1.5 -2 .08 0 0 0', '4 4 .16', orange)
    visual(link, 'locator_mast', '-1.5 -2 24.1 0 0 0', '.35 .35 48.2', orange)
    for index in range(12):
        visual(link, f'contrast_band_{index}', f'-1.5 -2 {2+index*4} 0 0 0', '.38 .38 1.2', dark)
    # 大号十字定位牌和发光灯位于镜头下方，便于全景定位。
    visual(link, 'top_cross_horizontal', '-1.5 -2 48.6 0 0 0', '2.6 .22 .32', yellow)
    visual(link, 'top_cross_vertical', '-1.5 -2 48.6 0 0 0', '.22 2.6 .32', yellow)
    beacon = ET.SubElement(link, 'visual', name='locator_beacon')
    set_text(beacon, 'pose', '-1.5 -2 48.9 0 0 0')
    geometry = ET.SubElement(beacon, 'geometry')
    set_text(ET.SubElement(geometry, 'sphere'), 'radius', '.38')
    mat = ET.SubElement(beacon, 'material')
    for key in ('ambient', 'diffuse', 'emissive'):
        set_text(mat, key, '1 .02 .65 1')
    # 箭头在49.5m高处，沿平面指向吊舱旁，保留模型周边观察空间。
    start, tip = (-1.5, -2.), (0., -.3)
    dx, dy = tip[0]-start[0], tip[1]-start[1]
    yaw = math.atan2(dy, dx)
    visual(link, 'pod_pointer', f'{(start[0]+tip[0])/2} {(start[1]+tip[1])/2} 49.5 0 0 {yaw}',
           f'{math.hypot(dx,dy)} .10 .10', yellow)
    for sign in (-1, 1):
        angle = yaw + math.pi + sign * math.pi/5
        ex, ey = tip[0]+.5*math.cos(angle), tip[1]+.5*math.sin(angle)
        visual(link, f'arrow_head_{sign}', f'{(tip[0]+ex)/2} {(tip[1]+ey)/2} 49.5 0 0 {angle}',
               '.5 .14 .12', yellow)
    return model


def prepare(output):
    tree = ET.parse(SOURCE)
    world = tree.getroot().find('world')
    world.set('name', WORLD)
    for tag in ('uri', 'albedo_map'):
        for element in world.iter(tag):
            if element.text and element.text.strip().startswith('assets/'):
                element.text = str((SOURCE.parent / element.text.strip()).resolve())
    pod = ET.parse(ROOT / 'models/gimbals/two_axis_pod/model.sdf').getroot().find('model')
    set_text(pod, 'pose', '0 0 50 0 0 0')
    pod.append(ET.fromstring('''<joint name="fixed_installation" type="fixed">
      <parent>world</parent><child>mount_link</child></joint>'''))
    world.append(pod)
    fixture = ET.fromstring('''<model name="pod_mount_fixture"><static>true</static>
      <pose>0 0 49.996 0 0 0</pose><link name="platform"/></model>''')
    visual(fixture.find('link'), 'horizontal_platform', '0 0 0 0 0 0',
           '.12 .12 .008', '.25 .45 .65 1')
    world.append(fixture)
    world.append(location_marker())
    world.append(target_model())
    world.append(detection_aircraft('detection_aircraft_left',
                                    '34 -9 71 0 0 3.14159265359'))
    world.append(detection_aircraft('detection_aircraft_right',
                                    '38 11 73 0 0 3.14159265359'))
    panel = ET.fromstring('''<model name="occlusion_panel"><static>true</static>
      <pose>100 100 80 0 0 0</pose><link name="body">
        <collision name="panel"><geometry><box><size>0.2 20 20</size></box></geometry></collision>
      </link></model>''')
    visual(panel.find('link'), 'surface', '0 0 0 0 0 0', '0.2 20 20', '.22 .22 .22 1')
    world.append(panel)
    near_clip = world.find('./gui/plugin/camera_clip/near')
    if near_clip is not None:
        near_clip.text = '0.005'
    camera_pose = world.find('./gui/plugin/camera_pose')
    if camera_pose is not None:
        camera_pose.text = '-15 -35 65 0 .10 .75'
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'runs/geographic_two_axis_pod/world.sdf')
    args = parser.parse_args()
    print(prepare(args.output.resolve()))
