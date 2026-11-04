#!/usr/bin/env python3
"""中科 D80 地面站：原 two_axis 视频/发送/摇杆布局，标准二进制 UDP。"""
from __future__ import annotations
import ctypes
import ctypes.util
import faulthandler
import math
import os
import signal
from pathlib import Path
import sys
import threading
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
# XInitThreads必须早于Qt和视频线程对Xlib的调用。
try:
    _X11 = ctypes.CDLL(ctypes.util.find_library('X11'))
    _X11.XInitThreads.argtypes = []
    _X11.XInitThreads.restype = ctypes.c_int
    _X11.XInitThreads()
except (OSError,TypeError,AttributeError):
    _X11 = None
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from ground_station.configuration import (parse_arguments, resolve_configuration,
    station_preferences, client_manifest_path)
from common.protocol import quantize
from ground_station.binary_client import BinaryClient
from ground_station.telemetry_forward import TelemetryForwarder, ForwardInputs, endpoint, save_destination
from ground_station.diagnostics import DiagnosticSubscriber, DisabledDiagnostics
from ground_station.stamp import decode_stamp_from_bgr
from ground_station.packet_rate import PacketRate
try:
    import gi
    gi.require_version('Gst', '1.0')
    gi.require_version('GstVideo', '1.0')
    from gi.repository import Gst, GstVideo
    Gst.init(None)
    _HAVE_GST, _GST_ERR = True, ''
except (ImportError, ValueError) as exc:
    _HAVE_GST, _GST_ERR = False, str(exc)
DECODER_ELEMENTS = {'nvdec': 'nvh264dec'}
STATE_CN = {'WAIT': '等待目标', 'TRACK': '跟踪中', 'TEMP_LOST': '暂时丢失',
            'LOST': '已丢失', 'IDLE': '手动', 'INIT': '初始化', 'FAULT': '故障'}
_FAULT_LOG_FD = None


def _session_fault_log(session):
    """保留进程生命周期的日志FD，使原生崩溃和退出清理也能留下线程栈。"""
    global _FAULT_LOG_FD
    path=Path(session['session_path']).parent/'ground_station.log'
    _FAULT_LOG_FD=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_APPEND,0o600)
    os.write(_FAULT_LOG_FD,f'中科地面站启动，PID {os.getpid()}，会话 {session["session_path"]}\n'.encode())
    faulthandler.enable(file=_FAULT_LOG_FD,all_threads=True)
    print(f'地面站故障日志：{path}',flush=True)


def rtsp_url(config):
    if config.get('_rtsp_url'):
        return config['_rtsp_url']
    video = config['video']
    host = video['rtsp_bind']
    if ':' in host:
        host = f'[{host}]'
    return f"rtsp://{host}:{video['rtsp_port']}{video['rtsp_mount']}"


class ManualController(threading.Thread):
    """按轴短租约以 10 Hz 刷新24帧；松手、租约到期和关闭发送零速。"""
    def __init__(self, client, max_rate):
        super().__init__(name='zhongke-d80-manual', daemon=True)
        self.client, self.max_rate = client, max_rate
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self.axes = {}
        self._active = False

    def hold(self, axis, value, duration=.3):
        if axis not in ('yaw', 'pitch') or not math.isfinite(value):
            raise ValueError('手动输入须为有限 yaw/pitch 速度')
        with self._lock:
            self.axes[axis] = (max(-self.max_rate, min(self.max_rate, value)), time.monotonic()+duration)

    def _send(self, yaw, pitch):
        self.client.send(0x24, x=quantize(yaw, '.1', -32768, 32767, '手动方位速度'),
                         y=quantize(pitch, '.1', -32768, 32767, '手动俯仰速度'))

    def cancel(self):
        with self._lock:
            active = bool(self.axes) or self._active
            self.axes.clear()
            self._active = False
            if active:
                self._send(0, 0)

    def run(self):
        while not self._stop_event.is_set():
            with self._lock:
                now = time.monotonic()
                self.axes = {axis: item for axis, item in self.axes.items() if item[1] > now}
                values = {axis: item[0] for axis, item in self.axes.items()}
                if values or self._active:
                    self._send(values.get('yaw', 0), values.get('pitch', 0))
                self._active = bool(values)
            self._stop_event.wait(.1)

    def stop(self):
        self._stop_event.set()
        self.cancel()
        if self.is_alive():
            self.join(timeout=1)


class PointerGuard:
    """X11 下检查窗口焦点和物理左键，捕获窗口外松手及切换窗口。

    其他窗口系统返回 None，由 GUI 使用短输入租约，避免永久保持角速度。
    """

    def __init__(self, title):
        self.title = title
        self.display = None
        self._lock = threading.RLock()
        try:
            self.lib = _X11
            self.lib.XOpenDisplay.argtypes = [ctypes.c_char_p]
            self.lib.XOpenDisplay.restype = ctypes.c_void_p
            self.lib.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
            self.lib.XDefaultRootWindow.restype = ctypes.c_ulong
            self.lib.XGetInputFocus.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int)]
            self.lib.XGetInputFocus.restype = ctypes.c_int
            self.lib.XFetchName.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_char_p)]
            self.lib.XFetchName.restype = ctypes.c_int
            self.lib.XQueryTree.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.POINTER(ctypes.c_ulong)), ctypes.POINTER(ctypes.c_uint)]
            self.lib.XQueryTree.restype = ctypes.c_int
            self.lib.XQueryPointer.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_uint)]
            self.lib.XQueryPointer.restype = ctypes.c_int
            self.lib.XInternAtom.argtypes = [ctypes.c_void_p,ctypes.c_char_p,ctypes.c_int]
            self.lib.XInternAtom.restype = ctypes.c_ulong
            self.lib.XGetWindowProperty.argtypes = [ctypes.c_void_p,ctypes.c_ulong,ctypes.c_ulong,
                ctypes.c_long,ctypes.c_long,ctypes.c_int,ctypes.c_ulong,
                ctypes.POINTER(ctypes.c_ulong),ctypes.POINTER(ctypes.c_int),
                ctypes.POINTER(ctypes.c_ulong),ctypes.POINTER(ctypes.c_ulong),ctypes.POINTER(ctypes.c_void_p)]
            self.lib.XGetWindowProperty.restype = ctypes.c_int
            self.lib.XFree.argtypes = [ctypes.c_void_p]
            self.lib.XFree.restype = ctypes.c_int
            self.lib.XCloseDisplay.argtypes = [ctypes.c_void_p]
            self.lib.XCloseDisplay.restype = ctypes.c_int
            self.display = self.lib.XOpenDisplay(None)
            self._name_atom = self.lib.XInternAtom(self.display,b'_NET_WM_NAME',1) if self.display else 0
        except (OSError, TypeError, AttributeError):
            self.display = None

    def focused(self):
        with self._lock:
            return self._focused_locked()

    def _window_title(self,window):
        data=ctypes.c_void_p()
        if self._name_atom:
            actual_type,actual_format=ctypes.c_ulong(),ctypes.c_int()
            count,remaining=ctypes.c_ulong(),ctypes.c_ulong()
            try:
                status=self.lib.XGetWindowProperty(self.display,window,self._name_atom,
                    0,1024,0,0,ctypes.byref(actual_type),ctypes.byref(actual_format),
                    ctypes.byref(count),ctypes.byref(remaining),ctypes.byref(data))
                if status==0 and actual_format.value==8 and data.value:
                    return ctypes.string_at(data,min(count.value,4096)).decode('utf-8',errors='replace')
            finally:
                if data.value:self.lib.XFree(data)
        name=ctypes.c_char_p()
        try:
            if self.lib.XFetchName(self.display,window,ctypes.byref(name)) and name.value:
                return name.value.decode('utf-8',errors='replace')
        finally:
            if ctypes.cast(name,ctypes.c_void_p).value:self.lib.XFree(ctypes.cast(name,ctypes.c_void_p))
        return None

    def _focused_locked(self):
        if not self.display:
            return None
        focus, revert = ctypes.c_ulong(), ctypes.c_int()
        self.lib.XGetInputFocus(self.display, ctypes.byref(focus), ctypes.byref(revert))
        window = focus.value
        matched = False
        for _ in range(16):
            if window <= 1:
                break
            matched = self._window_title(window) == self.title
            if matched:
                break
            root, parent, children, count = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.POINTER(ctypes.c_ulong)(), ctypes.c_uint()
            if not self.lib.XQueryTree(self.display, window, ctypes.byref(root), ctypes.byref(parent), ctypes.byref(children), ctypes.byref(count)):
                break
            if children:
                self.lib.XFree(children)
            if parent.value == window:
                break
            window = parent.value
        return matched

    def held_in_window(self):
        with self._lock:
            focused = self._focused_locked()
            if focused is None:
                return None
            root, child, mask = ctypes.c_ulong(), ctypes.c_ulong(), ctypes.c_uint()
            rx, ry, wx, wy = (ctypes.c_int() for _ in range(4))
            ok = self.lib.XQueryPointer(self.display, self.lib.XDefaultRootWindow(self.display), ctypes.byref(root), ctypes.byref(child), ctypes.byref(rx), ctypes.byref(ry), ctypes.byref(wx), ctypes.byref(wy), ctypes.byref(mask))
            return bool(ok and focused and mask.value & (1 << 8))

    def close(self):
        with self._lock:
            if self.display:
                self.lib.XCloseDisplay(self.display)
                self.display = None


# ---------------------------------------------------------------------------
# RTSP 解码管线
# ---------------------------------------------------------------------------
def build_pipeline(rtsp: str, decoder: str = "nvdec", verbose: bool = True):
    """构造 RTSP -> BGR appsink 管线；只允许 NVIDIA ``nvh264dec``。"""
    if not _HAVE_GST:
        raise RuntimeError(f"GStreamer/PyGObject 不可用: {_GST_ERR}")
    if decoder not in DECODER_ELEMENTS:
        raise RuntimeError(f"拒绝非硬件解码配置 {decoder!r}；地面站仅允许 nvh264dec")
    elem = DECODER_ELEMENTS[decoder]
    if Gst.ElementFactory.find(elem) is None:
        raise RuntimeError("缺少 NVIDIA 硬解元件 nvh264dec；请安装并启用 GStreamer NVIDIA 插件")
    desc = (
        f"rtspsrc location={rtsp} latency=0 drop-on-latency=true protocols=udp "
        f"! rtph264depay ! h264parse ! {elem} ! videoconvert "
        f"! video/x-raw,format=BGR ! appsink name=sink max-buffers=1 drop=true"
    )
    try:
        pipeline = Gst.parse_launch(desc)
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f"NVDEC RTSP 管线创建失败: {exc}") from exc
    sink = pipeline.get_by_name("sink")
    if sink is None:
        raise RuntimeError("NVDEC RTSP 管线创建失败：appsink 缺失")
    if verbose:
        print(f"[video] RTSP 管线就绪: {rtsp} (强制硬解 {elem})")
    return pipeline, sink, "nvdec"


class GroundStation:
    def __init__(self, cfg, preferences, args):
        self.cfg, self.preferences, self.args = cfg, preferences, args
        # HighGUI的Qt后端用ASCII标识定位窗口；中文只作为显示标题。
        self.title = preferences['window_title']
        self.win = f'zhongke_d80_station_{os.getpid()}'
        self._window_created = False
        self.scale = preferences['display_scale']
        self.max_rate = cfg['mechanics']['max_rate_dps']
        self.tele = BinaryClient(cfg['network']['udp_bind'], cfg['network']['udp_port'],
                                 max_age_s=cfg['network']['telemetry_max_age_s'], on_log=print,
                                 local_bind=preferences.get('local_bind'),local_port=preferences.get('local_port',0))
        self.manual = ManualController(self.tele, self.max_rate)
        self.forwarder = TelemetryForwarder(max_age_s=cfg['network']['telemetry_max_age_s'])
        self.tele.on_telemetry = self.forwarder.offer
        session=getattr(args,'resolved_session',None)
        self.diagnostics=(DiagnosticSubscriber(cfg['config_hash'],
            topic=session['diagnostics_topic'],model_name=session['model'])
            if session else DisabledDiagnostics())
        print(f"中科连接：UDP远端 {self.tele.peer[0]}:{self.tele.peer[1]}；"
              f"本地遥测接收 {self.tele.local_endpoint[0]}:{self.tele.local_endpoint[1]}；"
              f"RTSP {rtsp_url(cfg)}",flush=True)
        self.forward_inputs = ForwardInputs(preferences['forward_ip'], preferences['forward_port'])
        self.forward_note = '未发送'
        self._forward_pointer = self._forward_pressed = False
        self.pipeline = self.sink = self.decoder = None
        self.lock = threading.Lock()
        self.latest_frame = None
        self.frame_size = (cfg['optics']['width_px'], cfg['optics']['height_px'])
        self.display_size = tuple(int(round(v*self.scale)) for v in self.frame_size)
        self.last_frame_mono = self._t_start = time.monotonic()
        self._stop = False
        self.fatal_video_error = ''
        self.frame_times, self.delays = [], []
        self.video_rate = PacketRate()
        self.decoded_frames = 0
        self.last_seq = None
        self.drops = self.expected = 0
        self.last_cmd = ''
        self.last_disp = None
        self.buttons, self.pressed, self.hover = [], None, None
        self.joystick = None
        self.joystick_values = (0.0, 0.0)
        self.joystick_event_at = 0.0
        self.pointer_guard = PointerGuard(self.title)
        self.font = None
        for font in (preferences['font_path'], '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'):
            if Path(font).is_file():
                self.font = ImageFont.truetype(font, 24)
                qt_fonts = os.environ.get('QT_QPA_FONTDIR')
                if not qt_fonts or not Path(qt_fonts).is_dir():
                    os.environ['QT_QPA_FONTDIR'] = str(Path(font).parent)
                break

    def start_video(self):
        self.pipeline, self.sink, self.decoder = build_pipeline(
            rtsp_url(self.cfg), self.preferences["decoder"])
        result = self.pipeline.set_state(Gst.State.PLAYING)
        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("NVDEC RTSP 管线启动失败；请检查 NVIDIA 驱动、权限和视频源")
        threading.Thread(target=self.pull_loop, daemon=True).start()
        threading.Thread(target=self.video_watchdog, daemon=True).start()

    def pull_loop(self):
        while not self._stop:
            sink = self.sink
            if sink is None:
                time.sleep(0.05)
                continue
            sample = sink.emit("try-pull-sample", 100_000_000)
            if sample is None:
                time.sleep(0.002)
                continue
            buf = sample.get_buffer()
            caps = sample.get_caps().get_structure(0)
            w = int(caps.get_value("width"))
            h = int(caps.get_value("height"))
            ok, mapinfo = buf.map(Gst.MapFlags.READ)
            now = time.monotonic()
            if ok:
                stride = GstVideo.VideoInfo.new_from_caps(sample.get_caps()).stride[0]
                arr = np.ndarray(shape=(h, w, 3), dtype=np.uint8,
                                 buffer=mapinfo.data, strides=(stride, 3, 1))
                seq = decode_stamp_from_bgr(arr, w, h) if w >= 512 and h >= 16 else None
                new_disp = (int(round(w * self.scale)), int(round(h * self.scale)))
                frame = cv2.resize(arr, new_disp, interpolation=cv2.INTER_AREA)
                # 遮盖帧序号条（仅影响显示，保留解码结果）
                fh, fw = frame.shape[:2]
                strip_h = max(1, int(16 * fh / h))
                strip_w = min(fw, int(512 * fw / w))
                if strip_h + 1 < fh:
                    frame[fh - strip_h:fh, 0:strip_w] = frame[fh - strip_h - 1, 0:strip_w]
                buf.unmap(mapinfo)
                with self.lock:
                    self.frame_size = (w, h)
                    self.decoded_frames += 1
                    self.video_rate.record(now)
                    self.latest_frame = frame
                    self.display_size = new_disp
                    self.last_frame_mono = now
                    if seq is not None:
                        if self.last_seq is not None:
                            gap = (seq - self.last_seq) & 0xFFFFFF
                            if gap == 0:
                                seq = None  # 重复帧
                            elif gap > 100:
                                pass  # 疑似误码，重新对齐
                            else:
                                self.drops += gap - 1
                                self.expected += gap
                        if seq is not None:
                            self.last_seq = seq
                            self.frame_times.append((now, seq))
                    # 限制内存
                    if len(self.delays) > 4000:
                        del self.delays[:2000]
                    if len(self.frame_times) > 4000:
                        del self.frame_times[:2000]

    def _recreate_pipeline(self):
        try:
            if self.pipeline is not None:
                self.pipeline.set_state(Gst.State.NULL)
        except Exception:  # noqa: BLE001
            pass
        try:
            pipeline, sink, dec = build_pipeline(
                rtsp_url(self.cfg), self.preferences["decoder"], verbose=False)
            self.pipeline = pipeline
            self.sink = sink
            self.decoder = dec
            result = pipeline.set_state(Gst.State.PLAYING)
            if result == Gst.StateChangeReturn.FAILURE:
                raise RuntimeError("重建后的 NVDEC 管线无法进入 PLAYING")
            print(f"[video] 已重建 RTSP 管线（解码 {dec}）", flush=True)
        except Exception as exc:  # noqa: BLE001
            self._fail_video(f"重建 NVDEC RTSP 管线失败: {exc}")

    def _fail_video(self, message):
        if not self.fatal_video_error:
            self.fatal_video_error = str(message)
            print(f"[video] 致命错误：{self.fatal_video_error}；地面站退出，不回退软件解码。",
                  file=sys.stderr, flush=True)
        self._stop = True

    def _poll_video_error(self):
        pipeline = self.pipeline
        if pipeline is None:
            return
        bus = pipeline.get_bus()
        message = bus.pop_filtered(Gst.MessageType.ERROR)
        if message is not None:
            error, debug = message.parse_error()
            detail = error.message
            if debug:
                detail += f" ({debug})"
            self._fail_video(f"NVDEC RTSP 管线运行失败: {detail}")

    def video_watchdog(self):
        """视频长时间无新帧时尝试 set NULL/PLAYING，仍无则重建管线。"""
        while not self._stop:
            time.sleep(1.0)
            self._poll_video_error()
            if self._stop:
                return
            if time.monotonic() - self._t_start < 6.0:
                continue
            with self.lock:
                last = self.last_frame_mono
            age = time.monotonic() - last
            if age <= 3.0:
                continue
            print(f"[video] {age:.1f}s 无新帧，尝试重启管线 ...", flush=True)
            try:
                if self.pipeline is not None:
                    self.pipeline.set_state(Gst.State.NULL)
                    time.sleep(0.3)
                    self.pipeline.set_state(Gst.State.PLAYING)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(2.5)
            with self.lock:
                if self.last_frame_mono == last:
                    self._recreate_pipeline()
                    time.sleep(2.5)

    # ------------------------------------------------------------- mouse
    def _button_at(self, x, y):
        return next((b for b in self.buttons if self._inside(b[1], x, y)), None)

    def on_mouse(self, event, x, y, flags, param):
        if self._forward_mouse(event, x, y):
            return
        if event == cv2.EVENT_LBUTTONDBLCLK:
            if y >= self.display_size[1] or self._button_at(x, y):
                return
            if not self.tele.stats()['connected'] or time.monotonic()-self.last_frame_mono > 1:
                self.last_cmd = '画面或中科遥测已过期，不能指点'
                return
            self._release_joystick()
            w, h = self.frame_size
            dw, dh = self.display_size
            u, v = x*w/dw, y*h/dh
            if not (0 <= u < w and 0 <= v < h):
                return
            px = quantize(u-w/2, '1', -32768, 32767, '指点X')
            py = quantize(v-h/2, '1', -32768, 32767, '指点Y')
            self.tele.send(0x0D, x=px, y=py)
            self.last_cmd = f'已发送双击指点 ({px}, {py})，等待状态更新'
        elif event == cv2.EVENT_LBUTTONDOWN:
            hit = self._joystick_at(x, y)
            if hit:
                if not self.tele.stats()['connected']:
                    self.last_cmd = '中科遥测失效，手动输入已禁用'
                    return
                self.tele.send(0x0E)
                self.manual.cancel()
                self.joystick = hit
                self._update_joystick(x, y)
            else:
                self.pressed = self._button_at(x, y)
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.joystick:
                if flags & cv2.EVENT_FLAG_LBUTTON:
                    self._update_joystick(x, y)
                else:
                    self._release_joystick()
            button = self._button_at(x, y)
            self.hover = button[0] if button else None
        elif event == cv2.EVENT_LBUTTONUP:
            if self.joystick:
                self._release_joystick()
            if self.pressed:
                button = self.pressed
                self.pressed = None
                if self._button_at(x, y) == button:
                    self._action(button[2])

    def _action(self, action):
        self._release_joystick()
        if action == 'point_hint':
            self.last_cmd = '请双击视频中的目标，空白处将按最近候选回退'
            return
        if action == 'stop':
            self.tele.send(0x0E)
            self.tele.send(0x24)
        elif action == 'home':
            self.tele.send(0x2B)
        elif action == 'auto':
            diag = self.diagnostics.snapshot()
            self.tele.send(0x0E if diag and str(diag.get('selection_mode')).upper() == 'AUTO' else 0x0F)
        elif action in ('yaw_zero', 'pitch_zero'):
            sample = self.tele.snapshot()
            if not sample or not self.tele.stats()['connected']:
                self.last_cmd = '姿态遥测过期，单轴回零已禁用'
                return
            yaw = 0 if action == 'yaw_zero' else sample.fields['frame_yaw']
            pitch = 0 if action == 'pitch_zero' else sample.fields['frame_pitch']
            self.tele.send(0x26, x=quantize(yaw, '.01', -32768, 32767, '方位角'),
                           y=quantize(pitch, '.01', -32768, 32767, '俯仰角'))
        self.last_cmd = '已发送操作，执行结果以新鲜遥测及诊断为准'

    def _manual(self, axis, sign):
        if not self.tele.stats()['connected']:
            self.last_cmd = '中科遥测失效，手动输入已禁用'
            return
        self._release_joystick()
        self.tele.send(0x0E)
        self.manual.hold(axis, sign*self.max_rate, .3)
        self.last_cmd = f'已发送手动 {axis}，最大 {self.max_rate:g}°/s'

    def _joystick_geometry(self):
        dw, dh = self.display_size
        radius = min(56, max(30, dw // 10))
        return ((dw // 2, dh + 105, radius),)

    def _joystick_at(self, x, y):
        cx, cy, radius = self._joystick_geometry()[0]
        return "aim" if math.hypot(x - cx, y - cy) <= radius else None

    def _update_joystick(self, x, y):
        cx, cy, radius = self._joystick_geometry()[0]
        dx, dy = (x - cx) / radius, (cy - y) / radius
        length = math.hypot(dx, dy)
        if length > 1.0:
            dx, dy = dx / length, dy / length
        def response(value):
            return math.copysign(max(0.0, (abs(value) - 0.08) / 0.92), value)
        self.joystick_values = (response(dx), response(dy))
        self.joystick_event_at = time.monotonic()
        self._refresh_joystick()

    def _refresh_joystick(self):
        if self.joystick:
            x, y = self.joystick_values
            self.manual.hold("yaw", self.max_rate * x, 0.3)
            self.manual.hold("pitch", self.max_rate * y, 0.3)

    def _release_joystick(self):
        self.joystick = None
        self.joystick_values = (0.0, 0.0)
        self.manual.cancel()

    def _check_joystick(self):
        if self.pointer_guard.focused() is False:
            self._release_joystick()
            return
        if not self.joystick:
            return
        if not self.tele.stats()["connected"]:
            self._release_joystick()
            self.last_cmd = "机械姿态协议失效，已停止手动输入"
            return
        held = self.pointer_guard.held_in_window()
        if held is False or (held is None and time.monotonic() - self.joystick_event_at > 0.3):
            self._release_joystick()
        else:
            self._refresh_joystick()

    def _forward_geometry(self):
        width = min(416, max(300, self.display_size[0] - 16))
        x = max(8, self.display_size[0] - width - 12)
        ip_width = width - 232
        return {"ip": (x, 92, ip_width, 32),
                "port": (x + ip_width + 6, 92, 64, 32),
                "toggle": (x + ip_width + 76, 92, 156, 32)}

    @staticmethod
    def _inside(rect, x, y):
        return rect[0] <= x < rect[0] + rect[2] and rect[1] <= y < rect[1] + rect[3]

    def _forward_mouse(self, event, x, y):
        if not hasattr(self, "forward_inputs"):
            return False
        geometry = self._forward_geometry()
        if event == cv2.EVENT_LBUTTONDOWN:
            hit = next((name for name, rect in geometry.items() if self._inside(rect, x, y)), None)
            self.forward_inputs.focus = None
            if hit is None:
                return False
            self._release_joystick()
            self._forward_pointer = True
            self._forward_pressed = hit == "toggle"
            if hit in ("ip", "port"):
                self._pause_forward_for_edit()
                self.forward_inputs.activate(hit)
            return True
        if event == cv2.EVENT_LBUTTONUP and self._forward_pointer:
            self._forward_pointer = False
            pressed, self._forward_pressed = self._forward_pressed, False
            if pressed and self._inside(geometry["toggle"], x, y):
                self._toggle_forward()
            return True
        return event == cv2.EVENT_MOUSEMOVE and self._forward_pointer

    def _pause_forward_for_edit(self):
        if self.forwarder.snapshot()["active"]:
            self.forwarder.stop()
            self.forward_note = "已暂停，修改后点击发送"

    def _handle_forward_key(self, key):
        inputs = getattr(self, "forward_inputs", None)
        if inputs is not None and inputs.focus is not None and key not in (-1, 0):
            self._pause_forward_for_edit()
        return inputs.key(key) if inputs is not None else False

    def _toggle_forward(self):
        self.forward_inputs.focus = None
        if self.forwarder.snapshot()["active"]:
            self.forwarder.stop()
            self.forward_note = "已停止发送"
            return
        try:
            if not self.tele.stats()["connected"]:
                raise ValueError("请等待有效中科二进制遥测")
            ip, port = endpoint(self.forward_inputs.values["ip"], self.forward_inputs.values["port"])
            if (ip, port) == self.tele.peer:
                raise ValueError("接收端不能是吊舱网关控制端口")
            self.forwarder.start(ip, port)
            self.forward_inputs.values.update(ip=ip, port=str(port))
            self.preferences.update(forward_ip=ip, forward_port=port)
            self.forward_note = "等待新遥测"
            try:
                if getattr(self.args,'save_preferences',True):
                    save_destination(self.args.station_config, ip, port)
            except (OSError, ValueError) as exc:
                self.forward_note = "已发送；地址保存失败"
                print(f"[forward] 地址保存失败: {exc}", flush=True)
        except (ValueError, OSError) as exc:
            self.forward_note = str(exc)

    def _draw_forward_panel(self, disp):
        if not hasattr(self, "forward_inputs"):
            return
        geometry = self._forward_geometry()
        status = self.forwarder.snapshot()
        for name, rect in geometry.items():
            x, y, width, height = rect
            active = self.forward_inputs.focus == name
            bg = (0, 110, 0) if name == "toggle" and status["active"] else (35, 35, 35)
            cv2.rectangle(disp, (x, y), (x + width, y + height), bg, -1)
            cv2.rectangle(disp, (x, y), (x + width, y + height),
                          (0, 230, 255) if active else (190, 190, 190), 1)
            if name != "toggle":
                value = self.forward_inputs.values[name]
                visible = value[-max(5, (width - 12) // 10):]
                color = (0, 230, 255) if active and self.forward_inputs.selected else (255, 255, 255)
                cv2.putText(disp, visible + ("|" if active and not self.forward_inputs.selected else ""),
                            (x + 5, y + 22), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
        x = geometry["ip"][0]
        note = status["error"] or (f"发送中 {status['sent']} 包" if status["active"] and status["sent"] else self.forward_note)
        if status["active"] and self.forward_note == "已发送；地址保存失败":
            note = self.forward_note
        if status["active"] and not self.tele.stats()["connected"]:
            note = "等待有效新遥测"
        labels = [(geometry["toggle"][0] + 8, 94,
                   "停止发送" if status["active"] else "发送中科报文"),
                  (x, 130, note)]
        # 只绘制小区域，避免覆盖左侧姿态和脱靶量文字。
        if self.font is not None:
            pil = Image.fromarray(cv2.cvtColor(disp[64:159, x:], cv2.COLOR_BGR2RGB))
            painter = ImageDraw.Draw(pil)
            small_font = self.font.font_variant(size=18)
            for px, py, label in labels:
                painter.text((px - x, py - 64), label, font=small_font,
                             fill=(255, 150, 60) if status["error"] else (245, 245, 245))
            disp[64:159, x:] = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
        else:
            rect = geometry["toggle"]
            cv2.putText(disp, "STOP" if status["active"] else "SEND", (rect[0]+12, 114), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
        self._draw_rate_panel(disp, status)

    def _text(self, image, text, x, y, *, size=18, color=(230, 230, 230), max_width=None):
        if self.font is None:
            cv2.putText(image, text, (x, y+size), cv2.FONT_HERSHEY_SIMPLEX,
                        size/32, (12,12,12), 3, cv2.LINE_AA)
            cv2.putText(image, text, (x, y+size), cv2.FONT_HERSHEY_SIMPLEX,
                        size/32, color, 1, cv2.LINE_AA)
            return
        pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        painter = ImageDraw.Draw(pil)
        font = self.font.font_variant(size=size)
        limit = image.shape[1]-x-6 if max_width is None else max_width
        while text and painter.textbbox((0, 0), text, font=font)[2] > limit:
            text = text[:-1]
        painter.text((x, y), text, font=font, fill=color[::-1],
                     stroke_width=1, stroke_fill=(12,12,12))
        image[:] = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)

    def _draw_rate_panel(self, image, forward):
        stats = self.tele.stats()
        x = self._forward_geometry()['ip'][0]
        target = self.cfg['network']['telemetry_hz']
        receive = '预热' if stats['warming_up'] else f"{stats['receive_hz']:.1f}"
        send = '预热' if forward['warming_up'] else f"{forward['send_hz']:.1f}"
        self._text(image, f'目标 {target:g} Hz | 二进制接收 {receive} Hz', x, 161, size=15)
        self._text(image, f'中科 JSON 发送 {send} Hz | 丢弃 {forward["dropped"]}', x, 183, size=15)

    def draw(self, frame, sample):
        dw, dh = self.display_size
        image = np.vstack((frame.copy(), np.full((280, dw, 3), 28, dtype=np.uint8)))
        cv2.drawMarker(image, (dw//2, dh//2), (255,255,255), cv2.MARKER_CROSS, 24, 1)
        stats = self.tele.stats()
        fresh = sample is not None and stats['connected']
        diag = self.diagnostics.snapshot()
        fields = sample.fields if fresh else None
        top = [('停止', 'stop'), ('回零', 'home')]
        self.buttons = [(label, (dw-(len(top)-i)*94-12, 10, 86, 34), action, {})
                        for i, (label, action) in enumerate(top)]
        labels = [('双击选取', 'point_hint'), ('自动锁定', 'auto'), ('停止', 'stop')]
        width = min(130, max(24, (dw-40)//len(labels)))
        left = max(4, (dw-(width*len(labels)+8*(len(labels)-1)))//2)
        self.buttons += [(label, (left+i*(width+8), dh+2, width, 28), action, {})
                         for i, (label, action) in enumerate(labels)]
        width = min(140, (dw-36)//2)
        left = (dw-(width*2+12))//2
        self.buttons += [('Yaw 回零', (left, dh+182, width, 32), 'yaw_zero', {}),
                         ('Pitch 回零', (left+width+12, dh+182, width, 32), 'pitch_zero', {})]
        for label, (x,y,w,h), action, _ in self.buttons:
            disabled = action.endswith('_zero') and not fresh
            color = (40,40,40) if disabled else ((90,90,90) if self.hover == label else (65,65,65))
            if action == 'auto' and diag and str(diag.get('selection_mode')).upper() == 'AUTO':
                color = (0,110,0)
            cv2.rectangle(image, (x,y), (x+w,y+h), color, -1)
            cv2.rectangle(image, (x,y), (x+w,y+h), (180,180,180), 1)
            self._text(image, label, x+7, y+4, size=18, max_width=w-12)
        cx,cy,radius = self._joystick_geometry()[0]
        cv2.circle(image,(cx,cy),radius,(150,150,150),1,cv2.LINE_AA)
        cv2.line(image,(cx-radius,cy),(cx+radius,cy),(80,80,80),1)
        cv2.line(image,(cx,cy-radius),(cx,cy+radius),(80,80,80),1)
        x,y = self.joystick_values if self.joystick else (0,0)
        cv2.circle(image,(int(cx+x*radius),int(cy-y*radius)),11,(0,200,255),-1)
        self._text(image, 'YAW / PITCH', cx-radius,dh+36,size=16)
        self._text(image, '左负右正 / 上正下负', max(4,cx-92),dh+158,size=15)
        self._text(image, '双击视频指点 | 方向键手动 | R 回零 | 空格停止 | Esc 退出',12,dh+235,size=16)
        lines = [f'中科 D80  | 连接：{"有效" if fresh else "等待有效二进制遥测"}',
                 f'限速 {self.max_rate:g}°/s  | 发现距离 {self.cfg["tracking"]["discovery_range_m"]:g} m']
        optics = self.cfg['optics']
        lines += [f'标定 {optics["reference_focal_length_mm"]:g} mm：{optics["reference_hfov_deg"]:g}°×{optics["reference_vfov_deg"]:g}°']
        if fields:
            lines += [f'框架角 yaw {fields["frame_yaw"]:.2f}° / pitch {fields["frame_pitch"]:.2f}°',
                      f'目标偏差 X {fields["offset_x"]:.2f}° / Y {fields["offset_y"]:.2f}°',
                      f'有效跟踪标志：{"是" if fields["status1"] & 4 else "否"} | 自检 0x{fields["self_check"]:02X}']
        if diag:
            state = diag.get('tracking_state', '未知')
            target = diag.get('target_name') if diag.get('target_valid') else '无有效目标'
            lines += [f'状态：{STATE_CN.get(state,state)} | {diag.get("selection_mode", "未知")}',
                      f'目标：{target or "无有效目标"}', str(diag.get('reason') or '')]
            if diag.get('singular'):
                lines.append('视线退化，机械关节控制仍可用')
            limits = diag.get('limits') or {}
            if any(limits.values()):
                lines.append(f'限位 / 饱和：{limits}')
            result = diag.get('command_result') or {}
            if isinstance(result, dict) and result.get('message'):
                lines.append(str(result['message']))
            box = diag.get('target_box_px')
            if fresh and diag.get('target_valid') and isinstance(box,(list,tuple)) and len(box)==4:
                bw,bh = self.frame_size
                bx,by,ww,hh = box
                cv2.rectangle(image,(int(bx*dw/bw),int(by*dh/bh)),
                              (int((bx+ww)*dw/bw),int((by+hh)*dh/bh)),(0,255,0),1)
        else:
            lines.append('目标身份 / 原因：诊断未知或已过期')
        if self.last_cmd:
            lines.append(self.last_cmd)
        for index,line in enumerate(lines):
            y=8+index*24
            if y >= dh-20:
                break
            # 视频叠加文字避让右侧原发送面板，测量在下方使用整行。
            right = self._forward_geometry()['ip'][0]-20 if y<210 else dw-24
            if y<48:
                right=min(right,dw-210)
            self._text(image,line,12,y,size=17,max_width=max(60,right))
        self._draw_forward_panel(image)
        return image

    def run(self):
        try:
            if not _HAVE_GST:
                raise RuntimeError(f'GStreamer 不可用：{_GST_ERR}')
            self.tele.start()
            self.manual.start()
            cv2.namedWindow(self.win,cv2.WINDOW_AUTOSIZE)
            self._window_created = True
            cv2.setWindowTitle(self.win,self.title)
            cv2.setMouseCallback(self.win,self.on_mouse)
            self.start_video()
            print('中科地面站已启动：双击指点、自动锁定、方向键手动、R 回零、空格停止。',flush=True)
            while not self._stop:
                self._check_joystick()
                with self.lock:
                    frame=self.latest_frame.copy() if self.latest_frame is not None else None
                if frame is None:
                    frame=np.zeros((self.display_size[1],self.display_size[0],3),dtype=np.uint8)
                self.last_disp=self.draw(frame,self.tele.snapshot())
                cv2.imshow(self.win,self.last_disp)
                key=cv2.waitKeyEx(20)
                if self._handle_forward_key(key):
                    continue
                if key in (27,ord('q')) or cv2.getWindowProperty(self.win,cv2.WND_PROP_VISIBLE)<1:
                    break
                if key in (ord('r'),ord('h')):
                    self._action('home')
                elif key==ord('o'):
                    self._action('auto')
                elif key in (32,ord('c')):
                    self._action('stop')
                elif key in (65361,2424832,81):
                    self._manual('yaw',-1)
                elif key in (65363,2555904,83):
                    self._manual('yaw',1)
                elif key in (65362,2490368,82):
                    self._manual('pitch',1)
                elif key in (65364,2621440,84):
                    self._manual('pitch',-1)
                if self.args.duration and time.monotonic()-self._t_start>=self.args.duration:
                    break
        finally:
            self.shutdown()
        if self.fatal_video_error:
            raise RuntimeError(self.fatal_video_error)

    def check_stream(self, *, decode_only=False):
        if not decode_only:
            self.tele.start()
        self.start_video()
        duration = self.args.duration or 8
        deadline = time.monotonic()+duration
        try:
            while time.monotonic()<deadline and not self._stop:
                time.sleep(.05)
            with self.lock:
                frame=self.latest_frame.copy() if self.latest_frame is not None else None
            stats=self.tele.stats()
            if frame is None or time.monotonic()-self.last_frame_mono>2:
                raise RuntimeError(self.fatal_video_error or '自检未收到新鲜 NVDEC 视频')
            if not decode_only and not stats['connected']:
                raise RuntimeError('自检未收到新鲜中科二进制遥测')
            self.last_disp=self.draw(frame,self.tele.snapshot())
            print(f'中科地面站自检通过：二进制遥测 {stats["received"]} 包，接收 {stats["receive_hz"]:.1f} Hz，视频 {self.frame_size[0]}×{self.frame_size[1]}、解码 {self.decoded_frames} 帧。',flush=True)
        finally:
            self.shutdown()

    def shutdown(self):
        self._stop=True
        self.manual.stop()
        self.tele.stop()
        self.forwarder.close()
        self.diagnostics.close()
        self.pointer_guard.close()
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)
        if getattr(self,'_window_created',False):
            cv2.destroyWindow(self.win)
            cv2.waitKey(1)
            self._window_created=False


def main(argv=None):
    args=parse_arguments(argv)
    station=None
    client_manifest=None
    try:
        cfg,preferences,session=resolve_configuration(args)
        args.resolved_session=session
        if session:
            os.environ['GZ_PARTITION']=session['gz_partition']
            _session_fault_log(session)
        elif args.runtime_connection:
            faulthandler.enable(file=sys.stderr,all_threads=True)
        else:
            client_manifest=client_manifest_path(args.station_config)
            client_manifest.parent.mkdir(parents=True,exist_ok=True)
            _session_fault_log({'session_path':str(client_manifest)})
        station=GroundStation(cfg,preferences,args)
        signal.signal(signal.SIGTERM,lambda *_: setattr(station,'_stop',True))
        signal.signal(signal.SIGINT,lambda *_: setattr(station,'_stop',True))
        if args.selftest or args.decode_test:
            station.check_stream(decode_only=args.decode_test)
        else:
            station.run()
        if args.screenshot and station.last_disp is not None:
            destination=Path(args.screenshot)
            if not destination.is_absolute():
                raise ValueError('截图路径须为绝对路径')
            if not cv2.imwrite(str(destination),station.last_disp):
                raise RuntimeError('无法保存地面站截图')
        return 0
    except (ValueError,OSError,RuntimeError,cv2.error) as exc:
        print(f'中科地面站错误：{exc}',file=sys.stderr,flush=True)
        if _FAULT_LOG_FD is not None:
            os.write(_FAULT_LOG_FD,('中科地面站异常：\n'+traceback.format_exc()).encode())
        if station:
            station.shutdown()
        return 1
    finally:
        if client_manifest is not None and client_manifest.exists():
            from scripts.session import edit_session
            with edit_session(client_manifest) as record:
                section='process ground_station'
                if (record.has_section(section) and
                        record[section].get('pid')==str(os.getpid())):
                    record['session']['state']='stopped'


if __name__=='__main__':
    raise SystemExit(main())
