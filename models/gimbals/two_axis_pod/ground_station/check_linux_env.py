#!/usr/bin/env python3
"""检查 Linux Station 的 Python 与 NVDEC 运行依赖。"""
from __future__ import annotations

import cv2
import gi
import numpy
import PIL

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


def main() -> int:
    Gst.init(None)
    if Gst.ElementFactory.find("nvh264dec") is None:
        raise SystemExit(
            "未找到 GStreamer nvh264dec；请安装 NVIDIA 驱动及 NVDEC 插件。")
    print("Linux Station 环境就绪:", "OpenCV", cv2.__version__,
          "NumPy", numpy.__version__, "Pillow", PIL.__version__)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
