"""Gazebo 相机帧 -> NVENC H.264 -> RTP over UDP。"""
from __future__ import annotations

import queue
import threading
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

from gz.msgs10.image_pb2 import Image  # noqa: E402

Gst.init(None)

ENCODE_LAUNCH = (
    "appsrc name=vsrc is-live=true format=time do-timestamp=false "
    "caps=video/x-raw,format=RGB,width={w},height={h},framerate={fps}/1 "
    "max-buffers=3 leaky-type=downstream block=false "
    "! queue leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
    "! videoconvert "
    "! nvcudah264enc preset=low-latency-hq tune=ultra-low-latency rate-control=cbr "
    "bitrate={bitrate} gop-size={fps} zero-reorder-delay=true b-frames=0 "
    "! rtph264pay pt=96 config-interval=1 mtu=1400 "
    "! udpsink host={host} port={port} sync=false async=false"
)


class VideoSender:
    def __init__(self, node, topic: str, host: str, port: int,
                 width: int = 1920, height: int = 1080, fps: int = 30,
                 bitrate_kbps: int = 8000):
        self.node = node
        self.topic = topic
        self.width = width
        self.height = height
        self.fps = fps
        self.port = port
        self._mailbox: queue.Queue = queue.Queue(maxsize=1)
        self._closing = False
        self._seq = 0
        self._last_push_done = time.monotonic()
        self._last_frame = 0.0
        self._base_pts = None
        self._warned_size = False
        self._pushed_once = False
        self.pipeline = Gst.parse_launch(ENCODE_LAUNCH.format(
            w=width, h=height, fps=fps, bitrate=bitrate_kbps,
            host=host, port=port))
        self._appsrc = self.pipeline.get_by_name("vsrc")
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(f"视频管线启动失败：{topic}")
        self.node.subscribe(Image, topic, self._on_image)
        threading.Thread(target=self._push_loop, daemon=True).start()
        threading.Thread(target=self._watchdog_loop, daemon=True).start()

    def _on_bus_message(self, _bus, message):
        if message.type == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            print(f"[video:{self.port}] 管线错误: {err.message} ({dbg})", flush=True)
        elif message.type == Gst.MessageType.WARNING:
            print(f"[video:{self.port}] 警告: {message.parse_warning()[0].message}",
                  flush=True)

    def _on_image(self, msg: Image) -> None:
        self._last_frame = time.monotonic()
        if self._seq == 0:
            print(f"[video:{self.port}] 首帧 {msg.width}x{msg.height} "
                  f"bytes={len(msg.data)}", flush=True)
        if not self._warned_size and (msg.width != self.width or msg.height != self.height):
            self._warned_size = True
            print(f"[video:{self.port}] 图像尺寸 {msg.width}x{msg.height} "
                  f"与管线 {self.width}x{self.height} 不一致", flush=True)
        self._seq += 1
        item = (self._seq, msg.data, self._last_frame)
        try:
            self._mailbox.put_nowait(item)
        except queue.Full:
            try:
                self._mailbox.get_nowait()
            except queue.Empty:
                pass
            try:
                self._mailbox.put_nowait(item)
            except queue.Full:
                pass

    def _push_loop(self) -> None:
        while not self._closing:
            try:
                _seq, data, t0 = self._mailbox.get(timeout=0.2)
            except queue.Empty:
                continue
            buf = Gst.Buffer.new_wrapped(bytes(data))
            if self._base_pts is None:
                self._base_pts = t0
            buf.pts = int(max(0.0, t0 - self._base_pts) * Gst.SECOND)
            buf.duration = Gst.SECOND // self.fps
            t_before = time.monotonic()
            ret = self._appsrc.emit("push-buffer", buf)
            dt = time.monotonic() - t_before
            self._last_push_done = time.monotonic()
            if not self._pushed_once:
                self._pushed_once = True
                print(f"[video:{self.port}] 首次 push ret={ret}", flush=True)
            if dt > 1.0:
                print(f"[video:{self.port}] 编码推送阻塞 {dt:.1f}s", flush=True)
            if ret != Gst.FlowReturn.OK:
                print(f"[video:{self.port}] push-buffer 返回 {ret}", flush=True)

    def _watchdog_loop(self) -> None:
        while not self._closing:
            time.sleep(1.0)
            now = time.monotonic()
            if self._last_frame and now - self._last_frame > 5.0:
                self._last_frame = 0.0
                print(f"[video:{self.port}] 5 s 无新图像", flush=True)

    def close(self) -> None:
        self._closing = True
        self.pipeline.set_state(Gst.State.NULL)
