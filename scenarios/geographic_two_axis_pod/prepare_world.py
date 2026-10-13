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


def target_model():
    model = ET.fromstring('''<model name="tracking_target">
      <static>false</static><pose>30 0 70 0 0 0</pose>
      <link name="body"><gravity>false</gravity>
        <inertial><mass>1</mass><inertia><ixx>0.96</ixx><iyy>0.483333333333</iyy>
          <izz>0.483333333333</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="body_collision"><geometry><box><size>0.2 2.4 2.4</size></box></geometry></collision>
      </link>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
        <topic>/model/tracking_target/cmd_vel</topic></plugin>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <dimensions>3</dimensions><odom_publish_frequency>20</odom_publish_frequency>
        <odom_topic>/model/tracking_target/odometry</odom_topic></plugin>
    </model>''')
    link = model.find('link')
    visual(link, 'body_visual', '0 0 0 0 0 0', '0.2 2.4 2.4', '0.35 0.35 0.35 1')
    # 两个宽面使用 8×8 黑白格，薄侧面使用交替色条；几何材质无需外部贴图。
    for side in (-1, 1):
        for row in range(8):
            for col in range(8):
                color = '.025 .025 .025 1' if (row + col) % 2 == 0 else '.95 .95 .95 1'
                y, z = -1.05 + col * .3, -1.05 + row * .3
                visual(link, f'checker_{side}_{row}_{col}', f'{side * .102} {y} {z} 0 0 0', '.004 .3 .3', color)
        # 不同位置、不同颜色标记打破棋盘旋转对称性。
        visual(link, f'red_marker_{side}', f'{side * .106} -.9 .9 0 0 0', '.004 .22 .22', '.95 .05 .04 1')
        visual(link, f'blue_marker_{side}', f'{side * .106} .6 -.9 0 0 0', '.004 .38 .14', '.04 .12 .95 1')
        for index in range(8):
            position = -1.05 + index * .3
            color = '.04 .04 .04 1' if index % 2 else '.95 .8 .05 1'
            visual(link, f'edge_y_{side}_{index}', f'0 {side * 1.202} {position} 0 0 0', '.2 .004 .3', color)
            visual(link, f'edge_z_{side}_{index}', f'0 {position} {side * 1.202} 0 0 0', '.2 .3 .004', color)
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
