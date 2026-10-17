#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两轴吊舱地面站的 Windows 原生入口。

控制、遥测和界面复用 ``ground_station.py``，视频使用 OpenCV wheel 自带的
FFmpeg 后端读取 RTSP，并强制、校验 Windows D3D11 硬件解码。入口不依赖
PyGObject、GStreamer 或系统级 FFmpeg。
"""
from __future__ import annotations

import ctypes
from ctypes import wintypes
import os
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import ground_station as common  # noqa: E402
from stamp import decode_stamp_from_bgr  # noqa: E402


WINDOWS_CONFIG = str(HERE / "config.windows.json")


def d3d11_capture_params():
    """返回严格 D3D11 硬解的 VideoCapture 打开参数。"""
    required = (
        "CAP_PROP_HW_ACCELERATION",
        "VIDEO_ACCELERATION_D3D11",
        "CAP_PROP_OPEN_TIMEOUT_MSEC",
        "CAP_PROP_READ_TIMEOUT_MSEC",
    )
    missing = [name for name in required if getattr(cv2, name, None) is None]
    if missing:
        raise RuntimeError(
            "当前 OpenCV 缺少 D3D11 硬解接口: " + ", ".join(missing))
    return [
        cv2.CAP_PROP_HW_ACCELERATION, cv2.VIDEO_ACCELERATION_D3D11,
        cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
        cv2.CAP_PROP_READ_TIMEOUT_MSEC, 3000,
    ]


def verify_d3d11_capture(capture):
    """拒绝后端未启用 D3D11 的流，禁止静默回退到 CPU 解码。"""
    actual = int(round(capture.get(cv2.CAP_PROP_HW_ACCELERATION)))
    expected = int(cv2.VIDEO_ACCELERATION_D3D11)
    if actual != expected:
        raise RuntimeError(
            "RTSP 已打开，但 OpenCV/FFmpeg 未启用 D3D11 硬件解码"
            f"（实际加速类型={actual}，期望={expected}）；已拒绝软件解码回退")


class WindowsPointerGuard:
    """检查前台窗口是否属于当前进程且物理左键仍按下。

    OpenCV HighGUI 在部分 Windows 区域设置下会把中文标题显示成乱码，不能用
    窗口标题判断焦点；进程 ID 不受标题编码影响。
    """

    VK_LBUTTON = 0x01

    def __init__(self, title: str):
        self.title = title
        try:
            self.user32 = ctypes.windll.user32
            self.user32.GetForegroundWindow.restype = wintypes.HWND
            self.user32.GetWindowThreadProcessId.argtypes = (
                wintypes.HWND, ctypes.POINTER(wintypes.DWORD))
            self.user32.GetAsyncKeyState.argtypes = (ctypes.c_int,)
            self.user32.GetAsyncKeyState.restype = ctypes.c_short
        except (AttributeError, OSError):
            self.user32 = None

    def held_in_window(self):
        if self.user32 is None:
            return None
        hwnd = self.user32.GetForegroundWindow()
        if not hwnd:
            return False
        process_id = wintypes.DWORD()
        self.user32.GetWindowThreadProcessId(hwnd, ctypes.byref(process_id))
        held = bool(self.user32.GetAsyncKeyState(self.VK_LBUTTON) & 0x8000)
        return bool(held and process_id.value == os.getpid())

    def close(self):
        return None


class WindowsGroundStation(common.GroundStation):
    """使用 OpenCV/FFmpeg D3D11 硬解拉取 RTSP 的 Windows 地面站。"""

    def __init__(self, cfg: dict, args):
        super().__init__(cfg, args)
        self.pointer_guard.close()
        self.pointer_guard = WindowsPointerGuard(self.win)
        self.capture = None
        self.capture_lock = threading.Lock()
        self.video_thread = None
        self.rtsp_transport = str(cfg.get("rtsp_transport", "tcp")).lower()
        if self.rtsp_transport not in ("tcp", "udp"):
            self.rtsp_transport = "tcp"

    def _open_capture(self):
        # OpenCV wheel 内置 FFmpeg；TCP 在 Windows 网络和防火墙环境中更稳定。
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            f"rtsp_transport;{self.rtsp_transport}|stimeout;5000000"
        )
        params = d3d11_capture_params()
        url = common.rtsp_url(self.cfg)
        try:
            capture = cv2.VideoCapture(url, cv2.CAP_FFMPEG, params)
        except (TypeError, cv2.error) as exc:
            raise RuntimeError(
                "当前 OpenCV/FFmpeg 无法请求 D3D11 硬件解码；"
                "请更新 Windows 显卡驱动并重新运行 setup_windows.ps1") from exc
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(
                f"无法使用 D3D11 硬件解码打开 RTSP 视频: {url}；"
                "请检查显卡驱动、OpenCV FFmpeg 后端和网络")
        try:
            verify_d3d11_capture(capture)
        except Exception:
            capture.release()
            raise
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def start_video(self):
        self.video_thread = threading.Thread(
            target=self._video_loop, name="windows-rtsp", daemon=True)
        self.video_thread.start()

    def _replace_capture(self, capture):
        with self.capture_lock:
            old = self.capture
            self.capture = capture
        if old is not None and old is not capture:
            old.release()

    def _video_loop(self):
        failures = 0
        while not self._stop:
            with self.capture_lock:
                capture = self.capture
            if capture is None or not capture.isOpened():
                try:
                    capture = self._open_capture()
                    self._replace_capture(capture)
                    failures = 0
                    print(
                        "[video] RTSP 已连接"
                        f"（OpenCV/FFmpeg D3D11 硬解，{self.rtsp_transport.upper()}）",
                        flush=True)
                except Exception as exc:  # noqa: BLE001
                    failures += 1
                    if failures == 1 or failures % 5 == 0:
                        print(f"[video] 连接失败，1 秒后重试: {exc}", flush=True)
                    time.sleep(1.0)
                    continue

            try:
                ok, frame = capture.read()
            except cv2.error as exc:
                if self._stop:
                    break
                print(f"[video] 解码器读取失败，正在重连: {exc}", flush=True)
                ok, frame = False, None
            if not ok or frame is None:
                failures += 1
                if failures == 1 or failures % 5 == 0:
                    print("[video] 未收到有效视频帧，正在重连 ...", flush=True)
                self._replace_capture(None)
                time.sleep(0.3)
                continue
            failures = 0
            self._publish_frame(frame)

    def _publish_frame(self, frame):
        height, width = frame.shape[:2]
        now = time.monotonic()
        seq = decode_stamp_from_bgr(frame, width, height)
        new_disp = (int(round(width * self.scale)), int(round(height * self.scale)))
        display = cv2.resize(frame, new_disp, interpolation=cv2.INTER_AREA)

        # 遮盖生产视频中的帧序号条，仅影响本地显示。
        display_height, display_width = display.shape[:2]
        strip_h = max(1, int(16 * display_height / height))
        strip_w = min(display_width, int(512 * display_width / width))
        if strip_h + 1 < display_height:
            display[display_height - strip_h:display_height, 0:strip_w] = (
                display[display_height - strip_h - 1, 0:strip_w]
            )

        with self.lock:
            self.latest_frame = display
            self.display_size = new_disp
            self.last_frame_mono = now
            if seq is not None:
                if self.last_seq is not None:
                    gap = (seq - self.last_seq) & 0xFFFFFF
                    if gap == 0:
                        seq = None
                    elif gap <= 100:
                        self.drops += gap - 1
                        self.expected += gap
                if seq is not None:
                    self.last_seq = seq
                    delay = self.tele.frame_delay_ms(seq, now)
                    if delay is not None:
                        self.delays.append((now, delay))
                    self.frame_times.append((now, seq))
            if len(self.delays) > 4000:
                del self.delays[:2000]
            if len(self.frame_times) > 4000:
                del self.frame_times[:2000]

    def run(self):
        self.tele.start()
        self.manual.start()
        self.start_video()
        cv2.namedWindow(self.win, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(self.win, self.on_mouse)
        print("Windows 地面站已启动：视频使用 OpenCV/FFmpeg D3D11 硬解；"
              "四种视觉模式互斥，底部摇杆控制 yaw/pitch，ESC 退出。")
        self.last_draw_t = 0.0
        while not self._stop:
            if self.args.duration and time.monotonic() - self._t_start > self.args.duration:
                break
            with self.lock:
                new_frame = self.latest_frame is not None
                if new_frame:
                    self.last_frame = self.latest_frame
                    self.latest_frame = None
            telemetry = self.tele.snapshot()
            now = time.monotonic()
            self._check_joystick()
            if (getattr(self, "last_frame", None) is not None
                    and (new_frame or now - self.last_draw_t > 0.1)):
                display = self.draw(self.last_frame.copy(), telemetry)
                self.last_disp = display
                cv2.imshow(self.win, display)
                self.last_draw_t = now
                time.sleep(0.005)
            elif getattr(self, "last_frame", None) is None and now - self.last_draw_t > 0.1:
                blank = np.zeros((self.display_size[1], self.display_size[0], 3),
                                 dtype=np.uint8)
                cv2.putText(blank, "Waiting video...",
                            (30, self.display_size[1] // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                self.last_disp = self.draw(blank, telemetry)
                cv2.imshow(self.win, self.last_disp)
                self.last_draw_t = now
                time.sleep(0.02)

            key = cv2.waitKeyEx(1)
            try:
                visible = cv2.getWindowProperty(self.win, cv2.WND_PROP_VISIBLE)
            except cv2.error:
                visible = 0
            if visible < 1 or key == 27:
                break
            if key in (ord("s"), ord("S")):
                self._send_ctrl("stop", "已停止跟踪")
            elif key in (ord("h"), ord("H")):
                self._send_ctrl("home", "返回朝天")
            elif key in (ord("m"), ord("M")):
                self._send_ctrl("reset_fault", "清除故障")
            elif key in (ord("r"), ord("R")):
                self._send_ctrl("reset", "复位：停止跟踪并朝天")
            elif key in (0, -1):
                pass
            elif key in (2424832, 65361):
                self._manual("yaw", -30, "手动: yaw -30°/s")
            elif key in (2555904, 65363):
                self._manual("yaw", 30, "手动: yaw +30°/s")
            elif key in (2490368, 65362):
                self._manual("pitch", 20, "手动: pitch +20°/s")
            elif key in (2621440, 65364):
                self._manual("pitch", -20, "手动: pitch -20°/s")

        if self.args.gui_snapshot and self.last_disp is not None:
            cv2.imwrite(self.args.gui_snapshot, self.last_disp)
            print("界面截图:", self.args.gui_snapshot)
        return 0

    def shutdown(self):
        self._stop = True
        thread = self.video_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=4.0)
        self._replace_capture(None)
        super().shutdown()


def run_decode_test(cfg: dict, args) -> int:
    """Windows 无 GUI 视频检查。"""
    station = WindowsGroundStation.__new__(WindowsGroundStation)
    station.cfg = cfg
    station.rtsp_transport = cfg.get("rtsp_transport", "tcp")
    capture = None
    deadline = time.monotonic() + max(0.1, args.decode_test)
    frames = 0
    latest = None
    try:
        capture = station._open_capture()
        while time.monotonic() < deadline:
            ok, frame = capture.read()
            if not ok or frame is None:
                break
            latest = frame
            frames += 1
    except Exception as exc:  # noqa: BLE001
        print(f"[decode-test] 失败: {exc}", file=sys.stderr)
        return 3
    finally:
        if capture is not None:
            capture.release()
    if frames == 0 or latest is None:
        print("[decode-test] 未解码到视频帧", file=sys.stderr)
        return 4
    print("[decode-test] D3D11 硬件解码成功 "
          f"{frames} 帧，尺寸 {latest.shape[1]}x{latest.shape[0]}")
    if args.snapshot:
        output = Path(args.snapshot).resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(output), latest)
        print(f"[decode-test] 截图: {output}")
    return 0


def main(argv=None) -> int:
    parser = common.build_parser()
    parser.set_defaults(config=WINDOWS_CONFIG)
    parser.add_argument("--rtsp-transport", choices=("tcp", "udp"), default=None,
                        help="Windows RTSP 传输方式")
    args = parser.parse_args(argv)
    cfg = common.apply_cli_overrides(common.load_config(args.config), args)
    if args.ws_port is not None and args.udp_port is None:
        cfg["udp_port"] = args.ws_port
    if args.rtsp_transport is not None:
        cfg["rtsp_transport"] = args.rtsp_transport

    if args.selftest is not None:
        return common.run_selftest(cfg, args)
    if args.decode_test is not None:
        return run_decode_test(cfg, args)

    print(f"[ground_station_windows] UDP  {common.udp_endpoint(cfg)}")
    print(f"[ground_station_windows] RTSP {common.rtsp_url(cfg)}")
    station = WindowsGroundStation(cfg, args)
    try:
        return station.run()
    finally:
        station.shutdown()


def acquire_single_instance():
    """使用 Windows 命名互斥量阻止重复地面站窗口。"""
    kernel32 = ctypes.windll.kernel32
    kernel32.CreateMutexW.argtypes = (ctypes.c_void_p, wintypes.BOOL,
                                      wintypes.LPCWSTR)
    kernel32.CreateMutexW.restype = wintypes.HANDLE
    kernel32.ReleaseMutex.argtypes = (wintypes.HANDLE,)
    kernel32.CloseHandle.argtypes = (wintypes.HANDLE,)
    handle = kernel32.CreateMutexW(None, True, "Local\\Px4TwoAxisPodGroundStation")
    if not handle:
        raise ctypes.WinError()
    if kernel32.GetLastError() == 183:  # ERROR_ALREADY_EXISTS
        kernel32.CloseHandle(handle)
        return None
    return handle


def release_single_instance(handle):
    if handle:
        kernel32 = ctypes.windll.kernel32
        kernel32.ReleaseMutex(handle)
        kernel32.CloseHandle(handle)


if __name__ == "__main__":
    instance_handle = acquire_single_instance()
    if instance_handle is None:
        ctypes.windll.user32.MessageBoxW(
            None, "两轴吊舱地面站已经在运行。", "无法重复启动", 0x30)
        raise SystemExit(6)
    try:
        exit_code = main()
    except Exception as exc:  # noqa: BLE001 - pythonw 无控制台时显示可诊断错误
        ctypes.windll.user32.MessageBoxW(
            None, f"地面站启动失败：\n\n{exc}", "两轴吊舱地面站", 0x10)
        exit_code = 1
    finally:
        release_single_instance(instance_handle)
    raise SystemExit(exit_code)
