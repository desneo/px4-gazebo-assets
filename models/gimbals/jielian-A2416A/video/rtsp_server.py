#!/usr/bin/env python3
"""把本地H.264 RTP流发布为A2416A固定RTSP地址。"""
from __future__ import annotations

import argparse
from pathlib import Path
import signal
import sys

try:
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstRtspServer", "1.0")
    from gi.repository import GLib, Gst, GstRtspServer
except (ImportError, ValueError) as exc:
    print(f"GStreamer RTSP组件不可用：{exc}", file=sys.stderr)
    raise SystemExit(2)

Gst.init(None)

INGRESS_LAUNCH = (
    "udpsrc address=127.0.0.1 port={rtp_port} "
    "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 "
    "! rtph264depay ! h264parse config-interval=-1 "
    "! video/x-h264,stream-format=byte-stream,alignment=au "
    "! shmsink socket-path={shm_path} wait-for-connection=false sync=false "
    "shm-size=8388608"
)

LAUNCH = (
    "( shmsrc socket-path={shm_path} is-live=true do-timestamp=true "
    "! video/x-h264,stream-format=byte-stream,alignment=au "
    "! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rtp-port", type=int, default=7001)
    parser.add_argument("--rtsp-port", type=int, default=8554)
    parser.add_argument("--mount", default="/jielian_a2416a")
    args = parser.parse_args()
    if not args.mount.startswith("/"):
        parser.error("--mount必须以/开头")
    shm_path = Path(f"/tmp/jielian-a2416a-h264-{args.rtp_port}.sock")
    shm_path.unlink(missing_ok=True)
    try:
        ingress = Gst.parse_launch(INGRESS_LAUNCH.format(
            rtp_port=args.rtp_port, shm_path=shm_path))
    except Exception as exc:  # noqa: BLE001 - 需把GStreamer诊断交给操作者
        print(f"A2416A H.264 RTP接收管线创建失败：{exc}", file=sys.stderr)
        return 1
    if ingress.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
        ingress.set_state(Gst.State.NULL)
        print(f"A2416A RTP端口{args.rtp_port}无法监听", file=sys.stderr)
        return 1
    server = GstRtspServer.RTSPServer()
    server.set_service(str(args.rtsp_port))
    if not server.attach(None):
        print(f"RTSP端口{args.rtsp_port}无法监听", file=sys.stderr)
        return 1
    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_launch(LAUNCH.format(shm_path=shm_path))
    factory.set_shared(True)
    factory.set_stop_on_disconnect(False)
    factory.set_suspend_mode(GstRtspServer.RTSPSuspendMode.NONE)
    server.get_mount_points().add_factory(args.mount, factory)
    print(f"A2416A RTSP服务：rtsp://0.0.0.0:{args.rtsp_port}{args.mount}", flush=True)
    loop = GLib.MainLoop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_args: loop.quit())
    loop.run()
    ingress.set_state(Gst.State.NULL)
    shm_path.unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
