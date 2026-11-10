"""Gazebo RGB相机帧经NVENC H.264发送到本地RTP端口。"""
from __future__ import annotations

import queue
import threading
import time

import numpy as np

try:
    import cv2
    CV_IMPORT_ERROR = ""
except ImportError as exc:
    cv2 = None
    CV_IMPORT_ERROR = str(exc)

from .hardware import Gst, HardwareVideoError, require_nvcodec


WIDTH = 1920
HEIGHT = 1080
FPS = 25
DEFAULT_BITRATE_KBPS = 2000


def encoder_pipeline(encoder: str, host: str, port: int, bitrate_kbps: int) -> str:
    if not 0 < bitrate_kbps <= 8000:
        raise ValueError("H.264码率须在1～8000 kbps内")
    # nvcodec各包装器沿用NVIDIA编码器的公共低延迟属性。
    return (
        f"appsrc name=vsrc is-live=true format=time do-timestamp=false "
        f"caps=video/x-raw,format=RGB,width={WIDTH},height={HEIGHT},framerate={FPS}/1 "
        "max-buffers=3 leaky-type=downstream block=false "
        "! queue leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
        f"! videoconvert ! video/x-raw,format=NV12 ! {encoder} "
        "preset=low-latency-hq tune=ultra-low-latency "
        f"rate-control=cbr bitrate={bitrate_kbps} gop-size={FPS} "
        "zero-reorder-delay=true b-frames=0 "
        "! video/x-h264,profile=high,stream-format=byte-stream,alignment=au "
        "! h264parse ! rtph264pay pt=96 config-interval=1 mtu=1400 "
        f"! udpsink host={host} port={port} sync=false async=false"
    )


class VideoSender:
    """单帧邮箱发送器；处理开关由网关命令直接更新。"""

    def __init__(self, host: str = "127.0.0.1", port: int = 7001,
                 bitrate_kbps: int = DEFAULT_BITRATE_KBPS):
        if cv2 is None:
            raise HardwareVideoError(
                f"A2416A视频OSD与增强需要OpenCV：{CV_IMPORT_ERROR}；"
                "请使用资产仓.venv-linux环境")
        elements = require_nvcodec(need_encoder=True, need_decoder=False)
        self.encoder = elements.encoder
        self.host, self.port = host, int(port)
        self.bitrate_kbps = int(bitrate_kbps)
        self.pipeline_lock = threading.RLock()
        self.launch, self.pipeline, self.appsrc = self._new_pipeline(self.bitrate_kbps)
        self.mailbox = queue.Queue(maxsize=1)
        self.enabled = True
        self.enhance = False
        self.osd_mode = "full"
        self.crosshair = (WIDTH // 2, HEIGHT // 2)
        self.tracking_box = None
        self.detection_boxes = ()
        self.status_text = ""
        self.closing = threading.Event()
        self.base_pts = None
        self.frames = self.dropped = 0
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.pipeline.set_state(Gst.State.NULL)
            raise HardwareVideoError("H.264 NVENC管线启动失败；请检查NVIDIA驱动")
        self.thread = threading.Thread(target=self._push, name="a2416a-video-tx", daemon=True)
        self.thread.start()

    def _new_pipeline(self, bitrate_kbps: int):
        launch = encoder_pipeline(self.encoder, self.host, self.port, bitrate_kbps)
        try:
            pipeline = Gst.parse_launch(launch)
        except Exception as exc:  # noqa: BLE001
            raise HardwareVideoError(f"H.264 NVENC管线创建失败：{exc}") from exc
        appsrc = pipeline.get_by_name("vsrc")
        if appsrc is None:
            pipeline.set_state(Gst.State.NULL)
            raise HardwareVideoError("H.264 NVENC管线缺少appsrc")
        return launch, pipeline, appsrc

    def set_bitrate(self, bitrate_kbps: int):
        """重建低延迟编码器，使0x11码率命令实际作用于输出流。"""
        bitrate_kbps = int(bitrate_kbps)
        launch, pipeline, appsrc = self._new_pipeline(bitrate_kbps)
        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            pipeline.set_state(Gst.State.NULL)
            raise HardwareVideoError("新H.264码率管线启动失败")
        with self.pipeline_lock:
            old = self.pipeline
            self.launch, self.pipeline, self.appsrc = launch, pipeline, appsrc
            self.bitrate_kbps = bitrate_kbps
            self.base_pts = None
        old.set_state(Gst.State.NULL)

    def set_processing(self, *, enabled: bool | None = None, enhance: bool | None = None,
                       osd_mode: str | None = None):
        if osd_mode is not None and osd_mode not in ("full", "half", "off"):
            raise ValueError("OSD模式须为full、half或off")
        if enabled is not None:
            self.enabled = bool(enabled)
        if enhance is not None:
            self.enhance = bool(enhance)
        if osd_mode is not None:
            self.osd_mode = osd_mode

    def set_overlay(self, *, crosshair=None, tracking_box=None, detection_boxes=(),
                    status_text: str = ""):
        """更新OSD几何；矩形均为原始1920×1080像素的(x,y,w,h)。"""
        if crosshair is not None:
            self.crosshair = (int(crosshair[0]), int(crosshair[1]))
        self.tracking_box = tracking_box
        self.detection_boxes = tuple(detection_boxes)[:4]
        self.status_text = str(status_text)[:120]

    def offer_rgb(self, data: bytes, *, width: int, height: int, timestamp: float | None = None):
        if (width, height) != (WIDTH, HEIGHT):
            raise ValueError(f"A2416A视频帧必须为{WIDTH}x{HEIGHT}")
        if len(data) != WIDTH * HEIGHT * 3:
            raise ValueError("A2416A RGB视频帧字节数不正确")
        if not self.enabled:
            return False
        item = (bytes(data), time.monotonic() if timestamp is None else float(timestamp))
        try:
            self.mailbox.put_nowait(item)
        except queue.Full:
            self.dropped += 1
            try:
                self.mailbox.get_nowait()
            except queue.Empty:
                pass
            self.mailbox.put_nowait(item)
        return True

    def _process(self, data: bytes) -> bytes:
        if not self.enhance and self.osd_mode == "off":
            return data
        rgb = np.frombuffer(data, np.uint8).reshape(HEIGHT, WIDTH, 3)
        output = rgb.copy()
        if self.enhance:
            yuv = cv2.cvtColor(output, cv2.COLOR_RGB2YUV)
            yuv[:, :, 0] = cv2.createCLAHE(
                clipLimit=2.0, tileGridSize=(8, 8)).apply(yuv[:, :, 0])
            output = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB)
        if self.osd_mode != "off":
            cx, cy = self.crosshair
            cv2.drawMarker(output, (cx, cy), (255, 255, 0), cv2.MARKER_CROSS, 32, 2)
            if self.tracking_box is not None:
                x, y, width, height = map(int, self.tracking_box)
                cv2.rectangle(output, (x, y), (x + width, y + height), (0, 255, 0), 2)
            if self.osd_mode == "full":
                for box in self.detection_boxes:
                    x, y, width, height = map(int, box)
                    cv2.rectangle(output, (x, y), (x + width, y + height), (255, 255, 0), 1)
                if self.status_text:
                    cv2.putText(output, self.status_text, (20, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
                                cv2.LINE_AA)
        return output.tobytes()

    def _push(self):
        while not self.closing.is_set():
            try:
                data, stamp = self.mailbox.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                data = self._process(data)
            except HardwareVideoError:
                self.enabled = False
                continue
            buffer = Gst.Buffer.new_wrapped(data)
            if self.base_pts is None:
                self.base_pts = stamp
            buffer.pts = int(max(0.0, stamp - self.base_pts) * Gst.SECOND)
            buffer.duration = Gst.SECOND // FPS
            with self.pipeline_lock:
                result = self.appsrc.emit("push-buffer", buffer)
            if result == Gst.FlowReturn.OK:
                self.frames += 1
            else:
                self.dropped += 1

    def close(self):
        self.closing.set()
        with self.pipeline_lock:
            self.pipeline.set_state(Gst.State.NULL)
        self.thread.join(timeout=1.0)
