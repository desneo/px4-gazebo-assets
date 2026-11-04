"""固定等效感光面的光学派生，角度输入/输出为度。"""
from __future__ import annotations
import math
from collections.abc import Mapping


def derive_optics(optics: Mapping) -> dict:
    f = float(optics['focal_length_mm'])
    f0 = float(optics['reference_focal_length_mm'])
    h0 = float(optics['reference_hfov_deg'])
    v0 = float(optics['reference_vfov_deg'])
    w, h = int(optics['width_px']), int(optics['height_px'])
    if not all(math.isfinite(x) for x in (f, f0, h0, v0)) or min(f, f0) <= 0:
        raise ValueError('焦距必须为正有限数')
    if not 0 < h0 < 180 or not 0 < v0 < 180 or min(w, h) <= 0:
        raise ValueError('标定视场角必须在(0,180)内，图像尺寸必须为正')
    zoom = f / f0
    if not 0 < zoom <= 409.5:
        raise ValueError('当前/标定焦距比例超出中科U12变焦字段范围')
    th, tv = math.tan(math.radians(h0) / 2), math.tan(math.radians(v0) / 2)
    # 先乘无量纲标定系数，避免2*f0、w*f等中间量无谓溢出。
    sw, sh = f0 * (2 * th), f0 * (2 * tv)
    if not all(math.isfinite(value) and value > 0 for value in (sw, sh, th, tv)):
        raise ValueError('派生感光面必须为正有限数，标定参数超出浮点表达范围')
    try:
        values = dict(sensor_width_mm=sw, sensor_height_mm=sh,
            fx=(w / (2 * th)) * zoom, fy=(h / (2 * tv)) * zoom,
            cx=w / 2, cy=h / 2,
            hfov_deg=math.degrees(2 * math.atan(th / zoom)),
            vfov_deg=math.degrees(2 * math.atan(tv / zoom)), zoom=zoom)
    except (OverflowError, ZeroDivisionError) as exc:
        raise ValueError('派生相机内参超出浮点表达范围') from exc
    if not all(math.isfinite(value) and value > 0 for value in values.values()):
        raise ValueError('派生相机内参与视场必须为正有限数')
    if not all(0 < values[key] < 180 for key in ('hfov_deg','vfov_deg')):
        raise ValueError('派生实际视场超出可表达的(0,180)度范围')
    return values
