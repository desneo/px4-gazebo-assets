#!/usr/bin/env python3
"""复制完整地理环境，内联火箭机与两轴吊舱装配模型。"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from target_fixture import TARGET_POSITION, TARGET_SIZE, add_target

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'worlds/geo_32_217857_119_105878/world.sdf'
WORLD = 'geographic_rocket_two_axis_pod'
ENTITY = 'rocket_two_axis_pod'
MARKER = 'ROCKET_TAKEOFF_LOCATION_MARKER'


def set_text(parent, tag, value):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = str(value)
    return element


def absolute_resources(element, source_file):
    """运行副本仍使用原资产；不复制或改写世界和基础组件。"""
    model_roots = (
        source_file.parent.parent,
        ROOT / 'assembledmodels',
        ROOT / 'models/vehicles',
        ROOT / 'models/gimbals',
        ROOT / 'models/mounts',
        ROOT / 'models/props',
    )
    resource_tags = {'uri', 'albedo_map', 'normal_map', 'roughness_map',
                     'metalness_map', 'emissive_map', 'environment_map'}
    for item in element.iter():
        if item.tag not in resource_tags or not item.text:
            continue
        value = item.text.strip()
        if value.startswith('model://'):
            relative = value[len('model://'):]
            candidates = [directory / relative for directory in model_roots]
            path = next((path for path in candidates if path.exists()), None)
            if path is None:
                raise ValueError(f'模型资源不存在：{value}（来自 {source_file}）')
        elif value.startswith('file://'):
            path = Path(value[len('file://'):])
            if not path.is_absolute():
                path = source_file.parent / path
        elif '://' in value or not value:
            continue
        else:
            path = Path(value)
            if not path.is_absolute():
                path = source_file.parent / path
        path = path.resolve()
        if not path.is_file():
            raise ValueError(f'资源文件不存在：{path}')
        item.text = str(path)


def box(link, name, pose, size, color):
    visual = ET.SubElement(link, 'visual', name=name)
    set_text(visual, 'pose', pose)
    geometry = ET.SubElement(visual, 'geometry')
    set_text(ET.SubElement(geometry, 'box'), 'size', size)
    material = ET.SubElement(visual, 'material')
    for tag in ('ambient', 'diffuse'):
        set_text(material, tag, color)
    set_text(visual, 'cast_shadows', 'false')


def location_marker(x, y):
    """起飞点旁的无碰撞标志；箭头指向起飞点，不跟随飞机。"""
    marker = ET.fromstring(f'<model name="{MARKER}"><static>true</static>'
                           '<link name="marker"/></model>')
    set_text(marker, 'pose', f'{x} {y} 0 0 0 0')
    link = marker.find('link')
    orange, black, yellow = '1 .18 .01 1', '.03 .03 .03 1', '1 .9 .02 1'
    box(link, 'ground_locator', '-2 -2 .04 0 0 0', '1.2 1.2 .08', orange)
    box(link, 'locator_mast', '-2 -2 1.5 0 0 0', '.12 .12 3', orange)
    for index in range(5):
        box(link, f'contrast_band_{index}', f'-2 -2 {.3 + index * .6} 0 0 0',
            '.14 .14 .2', black)
    box(link, 'top_cross_horizontal', '-2 -2 3.1 0 0 0', '1.2 .14 .15', yellow)
    box(link, 'top_cross_vertical', '-2 -2 3.1 0 0 0', '.14 1.2 .15', yellow)
    # 指向原点附近，末端留出 0.6 m 空间，避开机体与旋翼。
    start, tip = (-2., -2.), (-.45, -.45)
    yaw = math.atan2(tip[1] - start[1], tip[0] - start[0])
    length = math.hypot(tip[0] - start[0], tip[1] - start[1])
    box(link, 'takeoff_pointer', f'{(start[0]+tip[0])/2} {(start[1]+tip[1])/2} .08 0 0 {yaw}',
        f'{length} .12 .08', yellow)
    for sign in (-1, 1):
        angle = yaw + math.pi + sign * math.pi / 5
        end = (tip[0] + .4 * math.cos(angle), tip[1] + .4 * math.sin(angle))
        box(link, f'pointer_head_{sign}', f'{(tip[0]+end[0])/2} {(tip[1]+end[1])/2} .08 0 0 {angle}',
            '.4 .16 .1', yellow)
    return marker


def prepare(output, model_path, altitude=.24, start_x=0., start_y=0.,
            view='overall', near_clip=.005, target_position=TARGET_POSITION, target_size=TARGET_SIZE):
    values = (altitude, start_x, start_y, near_clip)
    if not all(math.isfinite(value) for value in values) or near_clip <= 0:
        raise ValueError('位置必须是有限值，近裁剪距离必须大于零')
    if altitude < .225:
        raise ValueError('整机原点高度必须至少 0.225 m，避免机尾穿过原点地面')
    tree = ET.parse(SOURCE)
    world = tree.getroot().find('world')
    world.set('name', WORLD)
    absolute_resources(world, SOURCE)
    model_path = model_path.resolve()
    model = ET.parse(model_path).getroot().find('model')
    if model is None:
        raise ValueError(f'SDF 中没有顶层 model：{model_path}')
    if model.findtext('static', 'false').strip().lower() in ('true', '1'):
        raise ValueError('飞行场景必须输入动态装配模型，不能使用静态造型预览 SDF')
    if model.get('name') != ENTITY:
        raise ValueError(f'需要装配模型 {ENTITY}，实际为 {model.get("name")}')
    absolute_resources(model, model_path)
    model.set('name', ENTITY)
    set_text(model, 'pose', f'{start_x} {start_y} {altitude} 0 0 0')
    world.append(model)
    world.append(location_marker(start_x, start_y))
    add_target(world, dict(position=target_position, size=target_size))
    scene = world.find('./gui/plugin[@filename="MinimalScene"]')
    if scene is not None:
        clip = scene.find('camera_clip')
        if clip is None:
            clip = ET.SubElement(scene, 'camera_clip')
        set_text(clip, 'near', near_clip)
        if view == 'close':
            camera_pose = f'{start_x + .43} {start_y - .56} {altitude + .38} 0 .25 2.23'
        else:
            camera_pose = f'{start_x - 3} {start_y - 4} {altitude + 2} 0 .32 .927295'
        set_text(scene, 'camera_pose', camera_pose)
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space='  ')
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True,
                        help='generate_dynamic_model.py 生成的动态装配模型 SDF')
    parser.add_argument('--output', type=Path,
                        default=ROOT / f'runs/{WORLD}/world.sdf')
    parser.add_argument('--altitude', type=float, default=.24,
                        help='飞机原点的世界 Z（m），不是航线高度；默认起点地面附近')
    parser.add_argument('--start-x', type=float, default=0., help='起点 ENU 东向位置（m）')
    parser.add_argument('--start-y', type=float, default=0., help='起点 ENU 北向位置（m）')
    parser.add_argument('--view', choices=('overall', 'close'), default='overall')
    parser.add_argument('--near-clip', type=float, default=.005,
                        help='Gazebo 浏览相机近裁剪距离（m），与吊舱相机独立')
    parser.add_argument('--target', type=float, nargs=3, default=TARGET_POSITION,
                        help='空中纹理靶标世界 ENU 中心（m）；调整后跟踪检查也须指定同一 --target')
    parser.add_argument('--target-size', type=float, default=TARGET_SIZE)
    args = parser.parse_args()
    try:
        print(prepare(args.output, args.model, args.altitude, args.start_x,
                      args.start_y, args.view, args.near_clip, args.target, args.target_size))
    except (ValueError, OSError, ET.ParseError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
