#!/usr/bin/env python3
"""保存一帧 Gazebo 相机图像，用于外观预览。"""
import argparse
import threading

import numpy as np
from PIL import Image as PILImage
from gz.msgs10 import image_pb2
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/aisheng_seeker/overview')
    parser.add_argument('--output', required=True)
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()

    node = Node()
    done = threading.Event()
    saved = {'ok': False}

    def callback(msg):
        if done.is_set():
            return
        try:
            width, height, step = msg.width, msg.height, msg.step
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.pixel_format_type == image_pb2.RGB_INT8:
                rows = raw.reshape(height, step)[:, :width * 3]
                image = rows.reshape(height, width, 3)
            elif msg.pixel_format_type == image_pb2.RGBA_INT8:
                rows = raw.reshape(height, step)[:, :width * 4]
                image = rows.reshape(height, width, 4)[:, :, :3]
            elif msg.pixel_format_type == image_pb2.BGRA_INT8:
                rows = raw.reshape(height, step)[:, :width * 4]
                image = rows.reshape(height, width, 4)[:, :, [2, 1, 0]]
            elif msg.pixel_format_type == image_pb2.BGR_INT8:
                rows = raw.reshape(height, step)[:, :width * 3]
                image = rows.reshape(height, width, 3)[:, :, ::-1]
            else:
                raise ValueError(f'不支持的像素格式 {msg.pixel_format_type}')
            PILImage.fromarray(image, 'RGB').save(args.output)
            saved['ok'] = True
            print(f'已保存 {args.output} ({width}x{height}, format={msg.pixel_format_type})')
        except Exception as exc:  # noqa: BLE001 - 预览工具需要打印具体原因
            print(f'保存失败：{exc}')
        finally:
            done.set()

    node.subscribe(Image, args.topic, callback)
    if not done.wait(args.timeout):
        raise SystemExit(f'等待图像超时：{args.topic}')
    if not saved['ok']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
