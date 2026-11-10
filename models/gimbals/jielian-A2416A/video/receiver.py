"""地面站 H.264 RTSP NVIDIA硬解接收器。"""
from __future__ import annotations

import time

import numpy as np

from .hardware import Gst, HardwareVideoError, require_nvcodec


def receiver_pipeline(url: str, decoder: str, display_width: int, display_height: int) -> str:
    return (
        f"rtspsrc location={url} protocols=tcp latency=100 "
        f"! rtph264depay ! h264parse ! {decoder} "
        f"! videoscale ! video/x-raw,width={display_width},height={display_height} "
        "! videoconvert ! video/x-raw,format=BGR "
        "! appsink name=sink emit-signals=true max-buffers=1 drop=true sync=false"
    )


def build_pipeline(url: str, *, display_width: int = 960, display_height: int = 540):
    elements = require_nvcodec(need_encoder=False, need_decoder=True)
    launch = receiver_pipeline(url, elements.decoder, display_width, display_height)
    try:
        pipeline = Gst.parse_launch(launch)
    except Exception as exc:  # noqa: BLE001
        raise HardwareVideoError(f"H.264 NVDEC RTSP管线创建失败：{exc}") from exc
    sink = pipeline.get_by_name("sink")
    if sink is None:
        raise HardwareVideoError("H.264 NVDEC RTSP管线缺少appsink")
    return pipeline, sink, launch


class VideoReceiver:
    def __init__(self, url: str, *, display_width: int = 960, display_height: int = 540):
        self.pipeline, self.sink, self.launch = build_pipeline(
            url, display_width=display_width, display_height=display_height)
        self.latest = None
        self.frames = 0
        self.samples_signaled = 0
        self.last_frame_monotonic = 0.0
        self.error = ""
        self.closing = False
        self.sink_handler_id = self.sink.connect("new-sample", self._on_sample)
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.sink.disconnect(self.sink_handler_id)
            self.pipeline.set_state(Gst.State.NULL)
            raise HardwareVideoError("H.264 NVDEC RTSP管线启动失败；请检查驱动和RTSP地址")

    def _on_sample(self, sink):
        if self.closing:
            return Gst.FlowReturn.EOS
        try:
            self.samples_signaled += 1
            sample = sink.emit("pull-sample")
            if sample is None:
                return Gst.FlowReturn.OK
            caps = sample.get_caps().get_structure(0)
            width, height = caps.get_value("width"), caps.get_value("height")
            buffer = sample.get_buffer()
            ok, info = buffer.map(Gst.MapFlags.READ)
            if not ok:
                return Gst.FlowReturn.OK
            try:
                frame = np.frombuffer(info.data, dtype=np.uint8).reshape(
                    height, width, 3).copy()
            finally:
                buffer.unmap(info)
            self.latest = frame
            self.frames += 1
            self.last_frame_monotonic = time.monotonic()
            return Gst.FlowReturn.OK
        except Exception as exc:  # noqa: BLE001 - 回调异常必须反馈到地面站
            self.error = f"H.264视频帧处理失败：{exc}"
            return Gst.FlowReturn.ERROR

    def close(self):
        self.closing = True
        self.pipeline.set_state(Gst.State.NULL)
        self.sink.disconnect(self.sink_handler_id)
