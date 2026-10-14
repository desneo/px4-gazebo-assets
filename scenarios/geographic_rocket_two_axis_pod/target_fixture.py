#!/usr/bin/env python3
"""场景专用的静止空中视觉靶标；真值只用于验收，不提供跟踪控制。"""

import math
import xml.etree.ElementTree as ET

TARGET_NAME = 'AERIAL_TRACKING_TARGET'
TARGET_POSITION = (200., 150., 150.)
TARGET_SIZE = 8.


def fixture_config(config=None):
    config = {} if config is None else config
    position = tuple(float(value) for value in config.get('position', TARGET_POSITION))
    size = float(config.get('size', TARGET_SIZE))
    name = str(config.get('name', TARGET_NAME))
    if len(position) != 3 or not all(math.isfinite(value) for value in position):
        raise ValueError('靶标 position 必须含三个有限 ENU 坐标')
    if not math.isfinite(size) or size <= 0:
        raise ValueError('靶标 size 必须为正有限值')
    if not name or any(character in name for character in '/:'):
        raise ValueError('靶标 name 必须为非空的单层实体名称')
    return dict(name=name, position=position, size=size)


def _box(link, name, center, size, color):
    visual = ET.SubElement(link, 'visual', name=name)
    ET.SubElement(visual, 'pose').text = ' '.join(map(str, (*center, 0, 0, 0)))
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(ET.SubElement(geometry, 'box'), 'size').text = ' '.join(map(str, size))
    material = ET.SubElement(visual, 'material')
    for tag in ('ambient', 'diffuse'):
        ET.SubElement(material, tag).text = ' '.join(map(str, (*color, 1)))
    ET.SubElement(material, 'emissive').text = ' '.join(map(str, (*(channel * .18 for channel in color), 1)))
    ET.SubElement(visual, 'cast_shadows').text = 'false'


def target_model(config=None):
    """返回独立 SDF model，使用可渲染几何棋盘，运行时无需材质下载。"""
    config = fixture_config(config)
    size = config['size']
    model = ET.Element('model', name=config['name'])
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, (*config['position'], 0, 0, 0)))
    link = ET.SubElement(model, 'link', name='visual_fixture')
    _box(link, 'dark_core', (0, 0, 0), (size, size, size), (.025, .025, .04))
    # 各面 5×5 棋盘；不对称彩色角块防止不同面或重复棋盘混淆。
    palette = ((1., .14, .03), (.05, .75, 1.), (1., .8, .02),
               (.15, 1., .2), (1., .08, .65), (.4, .25, 1.))
    cell, thickness = size / 5, size * .001
    for face in range(6):
        axis, sign = face // 2, (-1 if face % 2 else 1)
        horizontal, vertical = ((axis + 1) % 3, (axis + 2) % 3)
        for row in range(5):
            for column in range(5):
                center = [0., 0., 0.]
                dimensions = [cell * .985] * 3
                center[axis] = sign * (size / 2 + thickness / 2)
                center[horizontal] = (column - 2) * cell
                center[vertical] = (row - 2) * cell
                dimensions[axis] = thickness
                color = (.96, .96, .88) if (row + column) % 2 == 0 else (.025, .025, .04)
                if (row, column) in ((0, 0), (0, 1), (1, 0), (4, 3)):
                    color = palette[face]
                if (row, column) == (3, 4):
                    color = palette[(face + 2) % 6]
                _box(link, f'face_{face}_tile_{row}_{column}', center, dimensions, color)
    return model


def add_target(world, config=None):
    """把靶标加入 world 并返回 model；已有同名实体时拒绝重复。"""
    model = target_model(config)
    if any(item.get('name') == model.get('name') for item in world.findall('model')):
        raise ValueError(f'世界中已有靶标实体：{model.get("name")}')
    world.append(model)
    return model
