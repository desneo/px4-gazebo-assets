"""已注册真值实体的初选规则；持续跟踪不调用发现距离筛选。"""
from __future__ import annotations

import math


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def choose_candidate(message: dict, config: dict, point=None):
    """按原图像素距离选择候选，返回(候选或None, 是否超提示半径)。"""
    width = config['optics']['width_px']
    height = config['optics']['height_px']
    location = (width / 2, height / 2) if point is None else point
    names = set(config['tracking']['target_names'])
    maximum = config['tracking']['discovery_range_m']
    eligible = []
    for candidate in message.get('truth_candidates', []):
        if not isinstance(candidate, dict):
            continue
        name = candidate.get('name')
        if name not in names or name == config['model']['name'] or candidate.get('valid') is not True:
            continue
        if any(type(candidate.get(k)) is not int or candidate[k] < 0
               for k in ('entity_id', 'frame_seq')):
            continue
        center = candidate.get('center_px')
        if not isinstance(center, (list, tuple)) or len(center) != 2:
            continue
        values = (*center, candidate.get('range_m'), candidate.get('depth_m'))
        if not all(_finite(v) for v in values):
            continue
        if not (0 <= center[0] < width and 0 <= center[1] < height
                and 0 <= candidate['range_m'] <= maximum and candidate['depth_m'] > 0):
            continue
        distance = (center[0] - location[0]) ** 2 + (center[1] - location[1]) ** 2
        eligible.append((distance, name, candidate))
    if not eligible:
        return None, False
    distance, _name, candidate = min(eligible, key=lambda value: value[:2])
    fallback = point is not None and distance > config['tracking']['point_fallback_radius_px'] ** 2
    return candidate, fallback
