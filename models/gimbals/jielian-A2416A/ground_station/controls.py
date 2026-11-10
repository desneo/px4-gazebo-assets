"""地面站交互到协议命令的纯函数映射。"""
from __future__ import annotations

from dataclasses import dataclass


WIDTH = 1920
HEIGHT = 1080
COMMANDS = {
    "recognition_on": 0x3F,
    "recognition_off": 0xF3,
    "auto_lock_on": 0x5F,
    "auto_lock_off": 0xF5,
    "cancel": 0x8E,
    "zoom_in": 0xB9,
    "zoom_out": 0x9B,
    "enhance_on": 0x6F,
    "enhance_off": 0xF6,
    "osd_full": 0x15,
    "osd_half": 0x16,
    "osd_off": 0x17,
    "video_on": 0x01,
    "video_off": 0x10,
}


def key_movement(key: int) -> tuple[int, int] | None:
    """按键重复会续租0xB1；不伪造无法可靠检测的按键释放事件。"""
    return {
        ord("a"): (-100, 0), ord("d"): (100, 0),
        ord("w"): (0, 100), ord("s"): (0, -100),
    }.get(key)


def view_to_source(x: float, y: float, view_width: int, view_height: int,
                   zoom: int = 1) -> tuple[int, int]:
    if zoom not in (1, 2, 4):
        raise ValueError("数字变倍只能是1、2或4")
    if not (0 <= x < view_width and 0 <= y < view_height):
        raise ValueError("点选坐标不在视频区域")
    crop_w, crop_h = WIDTH / zoom, HEIGHT / zoom
    left, top = (WIDTH - crop_w) / 2, (HEIGHT - crop_h) / 2
    u = left + x / view_width * crop_w
    v = top + y / view_height * crop_h
    return min(WIDTH - 1, max(0, round(u))), min(HEIGHT - 1, max(0, round(v)))


@dataclass(frozen=True)
class BoxSelection:
    x: int
    y: int
    width: int
    height: int


def box_to_source(start: tuple[float, float], end: tuple[float, float],
                  view_width: int, view_height: int, zoom: int = 1) -> BoxSelection:
    a = view_to_source(*start, view_width, view_height, zoom)
    b = view_to_source(*end, view_width, view_height, zoom)
    left, right = sorted((a[0], b[0]))
    top, bottom = sorted((a[1], b[1]))
    return BoxSelection(left, top, max(1, right - left + 1), max(1, bottom - top + 1))
