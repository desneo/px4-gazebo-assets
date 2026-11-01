#!/usr/bin/env python3
"""把本地单路 H.264 RTP 流发布为固定 RTSP URL。

网关只编码当前激活镜头到本地 RTP 端口；本进程把同一端口的内容发布到
固定挂载点，镜头切换时 URL 不变，RTSP 客户端重新连接即可看到新镜头。
"""
from __future__ import annotations

import argparse
import signal
import sys

import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import GLib, Gst, GstRtspServer  # noqa: E402

Gst.init(None)

LAUNCH = (
    "( udpsrc address=127.0.0.1 port={rtp_port} "
    "caps=application/x-rtp,media=video,clock-rate=90000,"
    "encoding-name=H264,payload=96 "
    "! queue max-size-buffers=1000 max-size-time=0 max-size-bytes=0 "
    "! rtph264depay "
    "! queue leaky=downstream max-size-buffers=3 max-size-time=0 max-size-bytes=0 "
    "! rtph264pay name=pay0 pt=96 config-interval=1 )"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rtp-port", type=int, default=7001)
    parser.add_argument("--rtsp-port", type=int, default=8554)
    parser.add_argument("--mount", default="/aisheng_seeker")
    args = parser.parse_args()
    if not args.mount.startswith("/"):
        parser.error("--mount 必须以 / 开头")
    server = GstRtspServer.RTSPServer()
    server.set_service(str(args.rtsp_port))
    source_id = server.attach(None)
    if not source_id:
        print(f"RTSP 端口 {args.rtsp_port} 无法监听", file=sys.stderr, flush=True)
        return 1
    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_launch(LAUNCH.format(rtp_port=args.rtp_port))
    factory.set_shared(True)
    server.get_mount_points().add_factory(args.mount, factory)
    print(f"RTSP 服务已启动 rtsp://0.0.0.0:{args.rtsp_port}{args.mount} "
          f"<- RTP {args.rtp_port}", flush=True)
    loop = GLib.MainLoop()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_args: loop.quit())
    try:
        loop.run()
    finally:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
