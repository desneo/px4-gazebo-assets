"""地面站 INI 配置读取与校验。"""
from __future__ import annotations

import configparser
import ipaddress
from pathlib import Path


DEFAULT_PATH = Path(__file__).with_name("config.ini")


def _port(value: str, name: str) -> int:
    if not str(value).strip().isdigit() or not 1 <= int(value) <= 65535:
        raise ValueError(f"{name}须在 1～65535 内")
    return int(value)


def load(path: str | Path = DEFAULT_PATH) -> dict:
    parser = configparser.ConfigParser(interpolation=None)
    if not parser.read(path, encoding="utf-8"):
        raise ValueError(f"无法读取地面站配置：{path}")
    required = {"network", "forward", "video", "display"}
    missing = required.difference(parser.sections())
    if missing:
        raise ValueError(f"地面站配置缺少段：{', '.join(sorted(missing))}")
    try:
        host = str(ipaddress.ip_address(parser["network"]["host"].strip()))
        forward_ip = str(ipaddress.ip_address(parser["forward"]["ip"].strip()))
        url = parser["video"]["rtsp_url"].strip()
        width = parser["video"].getint("width")
        height = parser["video"].getint("height")
        fps = parser["video"].getint("fps")
        display_width = parser["display"].getint("width")
        display_height = parser["display"].getint("height")
    except (KeyError, ValueError) as exc:
        raise ValueError(f"地面站配置值无效：{exc}") from exc
    if not url.startswith("rtsp://"):
        raise ValueError("video.rtsp_url 必须使用 rtsp://")
    if (width, height, fps) != (1920, 1080, 25):
        raise ValueError("A2416A 视频规格必须为 1920x1080@25Hz")
    if display_width <= 0 or display_height <= 0:
        raise ValueError("显示尺寸必须为正整数")
    return {
        "network": {"host": host, "port": _port(parser["network"]["port"], "网关端口")},
        "forward": {"ip": forward_ip, "port": _port(parser["forward"]["port"], "转发端口")},
        "video": {"rtsp_url": url, "width": width, "height": height, "fps": fps},
        "display": {
            "width": display_width,
            "height": display_height,
            "window_title": parser["display"].get("window_title", "A2416A GCS").strip(),
        },
    }


def with_forward_override(config: dict, host: str | None = None,
                          port: int | None = None) -> dict:
    """应用命令行转发端点覆盖，并保持传入配置不变。"""
    current = config["forward"]
    try:
        address = str(ipaddress.ip_address(
            current["ip"] if host is None else str(host).strip()))
        number = current["port"] if port is None else port
        number = _port(str(number), "艾晟报文端口")
    except (KeyError, ValueError) as exc:
        raise ValueError(f"艾晟报文目的地址无效：{exc}") from exc
    result = {section: dict(values) for section, values in config.items()}
    result["forward"] = {"ip": address, "port": number}
    return result
