#!/usr/bin/env python3
"""检查本组件依赖和硬件视频能力，不加载目标检测网络。"""
import argparse
import importlib
import sys


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--skip-video', action='store_true', help='仅供离线协议/配置测试')
    a = p.parse_args()
    try:
        for module in ('numpy', 'cv2', 'PIL', 'jsonschema', 'gi',
                       'gz.transport13', 'gz.msgs10.image_pb2', 'gz.msgs10.stringmsg_pb2'):
            importlib.import_module(module)
        if not a.skip_video:
            import gi
            gi.require_version('Gst', '1.0')
            gi.require_version('GstRtspServer', '1.0')
            from gi.repository import Gst, GstRtspServer
            Gst.init(None)
            for plugin in ('nvcudah264enc', 'nvh264dec'):
                if Gst.ElementFactory.find(plugin) is None:
                    raise RuntimeError(f'缺少硬件视频插件：{plugin}')
    except Exception as exc:
        print(f'中科运行环境检查失败：{exc}', file=sys.stderr)
        return 1
    print('中科运行环境就绪，未加载其他模型或检测网络')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
