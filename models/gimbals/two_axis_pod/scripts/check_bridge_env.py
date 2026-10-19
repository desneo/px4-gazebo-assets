#!/usr/bin/env python3
"""检查两轴吊舱网关的 Python、ONNX 与 NVENC 运行环境。"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


HERE = Path(__file__).resolve().parent
DEFAULT_VISION_CONFIG = HERE.parent / "controller/service/vision.json"


def resolve_model_path(config_path: Path) -> Path:
    config_path = config_path.expanduser().resolve()
    try:
        data = json.loads(config_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise RuntimeError(f"无法读取视觉配置 {config_path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"视觉配置不是有效 JSON {config_path}: {exc}") from exc
    model_value = data.get("model_path") if isinstance(data, dict) else None
    if not isinstance(model_value, str) or not model_value.strip():
        raise RuntimeError(f"视觉配置缺少有效 model_path: {config_path}")
    model_path = Path(model_value).expanduser()
    if not model_path.is_absolute():
        model_path = config_path.parent / model_path
    model_path = model_path.resolve()
    if not model_path.is_file():
        raise RuntimeError(f"ONNX 模型不存在: {model_path}")
    if model_path.suffix.lower() != ".onnx":
        raise RuntimeError(f"目标检测模型必须是 .onnx 文件: {model_path}")
    return model_path


def check_python_and_model(config_path: Path) -> tuple[str, str, Path]:
    try:
        import cv2
        import numpy
        import gi
        import gz.transport13  # noqa: F401
        import gz.msgs10.image_pb2  # noqa: F401
        import gz.msgs10.stringmsg_pb2  # noqa: F401
    except Exception as exc:  # 三方二进制模块也可能报 ABI/动态库错误
        raise RuntimeError(f"Python 缺少网关依赖: {exc}") from exc

    try:
        gi.require_version("Gst", "1.0")
        gi.require_version("GstRtspServer", "1.0")
        from gi.repository import Gst, GstRtspServer  # noqa: F401
    except Exception as exc:
        raise RuntimeError(f"Python 缺少 GStreamer/GstRtspServer 绑定: {exc}") from exc

    model_path = resolve_model_path(config_path)
    try:
        net = cv2.dnn.readNetFromONNX(str(model_path))
    except (AttributeError, cv2.error) as exc:
        raise RuntimeError(f"OpenCV DNN 无法加载 ONNX 模型 {model_path}: {exc}") from exc
    if hasattr(net, "empty") and net.empty():
        raise RuntimeError(f"OpenCV DNN 加载到空网络: {model_path}")
    return cv2.__version__, numpy.__version__, model_path


def check_nvenc() -> None:
    import gi

    gi.require_version("Gst", "1.0")
    from gi.repository import Gst

    Gst.init(None)
    if Gst.ElementFactory.find("nvcudah264enc") is None:
        raise RuntimeError(
            "未找到可用的 GStreamer nvcudah264enc；"
            "请检查 NVIDIA 驱动、nvidia-smi 与 nvcodec 插件。")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="检查两轴吊舱网关运行环境")
    parser.add_argument("--vision-config", type=Path, default=DEFAULT_VISION_CONFIG)
    args = parser.parse_args(argv)
    cv_version, numpy_version, model_path = check_python_and_model(args.vision_config)
    check_nvenc()
    print("两轴吊舱网关环境就绪:",
          "OpenCV", cv_version, "NumPy", numpy_version,
          "NVENC nvcudah264enc", "ONNX", model_path)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
