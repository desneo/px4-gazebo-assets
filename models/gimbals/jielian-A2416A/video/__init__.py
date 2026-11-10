"""A2416A H.264 硬件视频链路。"""

from .hardware import HardwareVideoError, require_nvcodec

__all__ = ["HardwareVideoError", "require_nvcodec"]
