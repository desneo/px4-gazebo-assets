"""NVIDIA H.264 GStreamer能力检查；不允许软件回退。"""
from __future__ import annotations

from dataclasses import dataclass

try:
    import gi
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    GST_IMPORT_ERROR = ""
except (ImportError, ValueError) as exc:
    Gst = None
    GST_IMPORT_ERROR = str(exc)


ENCODER_CANDIDATES = ("nvcudah264enc", "nvautogpuh264enc", "nvh264enc")
DECODER = "nvh264dec"


class HardwareVideoError(RuntimeError):
    pass


@dataclass(frozen=True)
class HardwareElements:
    encoder: str
    decoder: str = DECODER


def _loadable(name: str):
    factory = Gst.ElementFactory.find(name)
    if factory is None:
        return None
    try:
        element = factory.create(None)
    except Exception:  # noqa: BLE001 - GStreamer插件可以抛GLib异常
        return None
    if element is None:
        return None
    try:
        if element.set_state(Gst.State.READY) == Gst.StateChangeReturn.FAILURE:
            element.set_state(Gst.State.NULL)
            return None
        element.set_state(Gst.State.NULL)
    except Exception:  # noqa: BLE001 - CUDA设备初始化失败也必须视为不可用
        return None
    return element


def require_nvcodec(*, need_encoder: bool = True, need_decoder: bool = True) -> HardwareElements:
    """确认硬件编解码元素可实例化，否则给出明确前置条件错误。"""
    if Gst is None:
        raise HardwareVideoError(
            f"GStreamer/PyGObject不可用：{GST_IMPORT_ERROR}；A2416A不允许软件编解码回退")
    encoder = ""
    if need_encoder:
        encoder = next((name for name in ENCODER_CANDIDATES if _loadable(name) is not None), "")
        if not encoder:
            raise HardwareVideoError(
                "缺少可加载的NVIDIA H.264 NVENC元素（尝试了nvcudah264enc、"
                "nvautogpuh264enc、nvh264enc）；请恢复NVIDIA驱动并安装匹配的"
                "GStreamer nvcodec插件。A2416A不允许x264enc软件回退")
    if need_decoder and _loadable(DECODER) is None:
        raise HardwareVideoError(
            "缺少可加载的NVIDIA H.264解码元素nvh264dec；请恢复NVIDIA驱动并安装"
            "匹配的GStreamer nvcodec插件。A2416A不允许avdec_h264软件回退")
    return HardwareElements(encoder=encoder, decoder=DECODER if need_decoder else "")
