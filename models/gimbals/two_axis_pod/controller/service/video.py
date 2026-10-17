"""视频服务：Gazebo 相机帧 -> NVENC 硬件 H.264 编码 -> RTP/UDP -> RTSP 中继。

结构（两段式，避免 appsrc 与 GstRtspServer media 生命周期耦合）：
  [编码管线] appsrc -> videoconvert -> nvcudah264enc -> rtph264pay -> udpsink（本机独立 RTP 端口）
  [共享 RTSP 中继] 每路 udpsrc -> rtph264depay -> rtph264pay(pay0) -> 对应飞机路径
- 每帧左下角烧录 32bit 二进制序号（8bit 魔数 0xA5 + 24bit 帧序号），
  客户端解码该序号并按服务端发送的 (序号 -> 采样时刻) 表计算端到端时延。
- RTSP 挂载点默认 rtsp://<host>:8555/two_axis_pod_<aircraft_id>
"""
from __future__ import annotations

import queue
import socket
import threading
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst  # noqa: E402

Gst.init(None)

from rtsp_relay import RelayRegistration

from stamp import STAMP_BLOCK_H, STAMP_BLOCK_W, seq_to_bits  # noqa: E402

ENCODE_LAUNCH = (
    "appsrc name=vsrc is-live=true format=time do-timestamp=false "
    "caps=video/x-raw,format=RGB,width={w},height={h},framerate={fps}/1 "
    "max-buffers=3 leaky-type=downstream block=false "
    "! queue leaky=downstream max-size-buffers=2 max-size-time=0 max-size-bytes=0 "
    "! videoconvert "
    "! nvcudah264enc preset=low-latency-hq tune=ultra-low-latency rate-control=cbr "
    "bitrate={bitrate} gop-size={fps} zero-reorder-delay=true b-frames=0 "
    "! rtph264pay pt=96 config-interval=1 "
    "! udpsink host=127.0.0.1 port={udp_port} sync=false async=false"
)



class VideoStreamer:
    def __init__(self, width: int, height: int, fps: int = 30,
                 rtsp_port: int = 8555, mount: str = "/two_axis_pod_1",
                 bitrate_kbps: int = 8000, udp_port: int | None = None,
                 on_error=None):
        self.width = width
        self.height = height
        self.fps = fps
        self.rtsp_port = rtsp_port
        self.mount = mount
        self._on_error = on_error
        # 未指定时由系统分配内部 RTP 端口；显式端口先检查占用。
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(("127.0.0.1", udp_port or 0))
            udp_port = reservation.getsockname()[1]
        self.udp_port = udp_port
        self._relay = RelayRegistration(rtsp_port, mount, udp_port)
        self._last_seq = None
        self._t_base = None
        self.frames_pushed = 0
        self.frames_dropped = 0
        self._mailbox: queue.Queue = queue.Queue(maxsize=1)
        self._closing = False
        self._last_push_done = time.monotonic()
        self._restart_lock = threading.Lock()

        try:
            self.pipeline = Gst.parse_launch(ENCODE_LAUNCH.format(
                w=width, h=height, fps=fps, bitrate=bitrate_kbps, udp_port=udp_port))
            self._appsrc = self.pipeline.get_by_name("vsrc")
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self._on_bus_message)
            if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("编码管线启动失败")
        except Exception:
            if hasattr(self, "pipeline"):
                self.pipeline.set_state(Gst.State.NULL)
            self._relay.close()
            raise
        self._loop = GLib.MainLoop()
        self._glib_thread = threading.Thread(target=self._loop.run, daemon=True)
        self._glib_thread.start()
        threading.Thread(target=self._push_loop, daemon=True).start()
        threading.Thread(target=self._watchdog_loop, daemon=True).start()

    @property
    def clients(self):
        return self._relay.clients

    def _on_bus_message(self, _bus, message):
        if message.type == Gst.MessageType.ERROR:
            err, dbg = message.parse_error()
            print(f"[video] 编码管线错误: {err.message} ({dbg})", flush=True)
            if self._on_error is not None:
                self._on_error(err.message)
        elif message.type == Gst.MessageType.WARNING:
            print(f"[video] 编码管线警告: {message.parse_warning()[0].message}", flush=True)

    def push(self, seq: int, data: bytes, t0: float) -> bool:
        """只投递最新帧到邮箱，绝不阻塞视频线程。"""
        if seq == self._last_seq:
            return False
        self._last_seq = seq
        item = (seq, data, t0)
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
        return True

    def _push_loop(self):
        while not self._closing:
            try:
                seq, data, t0 = self._mailbox.get(timeout=0.2)
            except queue.Empty:
                continue
            stamped = bytearray(data)
            self._paint_stamp(stamped, seq)
            buf = Gst.Buffer.new_wrapped(bytes(stamped))
            if self._t_base is None:
                self._t_base = t0
            buf.pts = int(max(0.0, t0 - self._t_base) * Gst.SECOND)
            buf.duration = Gst.SECOND // self.fps
            t_before = time.monotonic()
            ret = self._appsrc.emit("push-buffer", buf)
            dt = time.monotonic() - t_before
            self._last_push_done = time.monotonic()
            if dt > 1.0:
                print(f"[video] 编码推送阻塞 {dt:.1f}s，重建编码管线", flush=True)
                self._restart_pipeline()
            if ret == Gst.FlowReturn.OK:
                self.frames_pushed += 1
            else:
                self.frames_dropped += 1

    def _watchdog_loop(self):
        while not self._closing:
            time.sleep(1.0)
            if time.monotonic() - self._last_push_done > 5.0:
                print("[video] 看门狗：编码管线超时未推送，重建", flush=True)
                self._restart_pipeline()

    def _restart_pipeline(self):
        if not self._restart_lock.acquire(blocking=False):
            return
        try:
            if self._closing:
                return
            self.pipeline.set_state(Gst.State.NULL)
            time.sleep(0.3)
            self.pipeline.set_state(Gst.State.PLAYING)
            self._t_base = None
            self._last_push_done = time.monotonic()
        finally:
            self._restart_lock.release()

    def _paint_stamp(self, data: bytearray, seq: int):
        bits = seq_to_bits(seq)
        row_bytes = self.width * 3
        for i, bit in enumerate(bits):
            val = 255 if bit else 0
            x0 = i * STAMP_BLOCK_W
            for y in range(self.height - STAMP_BLOCK_H, self.height):
                base = y * row_bytes + x0 * 3
                for off in range(base, base + STAMP_BLOCK_W * 3):
                    data[off] = val

    def close(self):
        self._closing = True
        with self._restart_lock:
            self.pipeline.set_state(Gst.State.NULL)
        self._loop.quit()
        self._relay.close()
