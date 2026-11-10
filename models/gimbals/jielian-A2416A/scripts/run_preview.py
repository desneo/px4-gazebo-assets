#!/usr/bin/env python3
"""构建插件、启动近景世界并设置 Gazebo 浏览相机灵敏度。"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

from generate_model import load_config

ROOT = Path(__file__).resolve().parents[1]


def prepend(env, key, value):
    current = env.get(key, "")
    env[key] = str(value) + (os.pathsep + current if current else "")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--build-dir", type=Path, default=ROOT / "build")
    parser.add_argument("--no-build", action="store_true")
    args = parser.parse_args()
    cfg = load_config()
    if not args.no_build:
        subprocess.run(["cmake", "-S", str(ROOT), "-B", str(args.build_dir),
                        "-DBUILD_TESTING=ON"], check=True)
        subprocess.run(["cmake", "--build", str(args.build_dir), "-j2"], check=True)

    env = os.environ.copy()
    prepend(env, "SDF_PATH", ROOT.parent)
    prepend(env, "GZ_SIM_RESOURCE_PATH", ROOT.parent)
    prepend(env, "GZ_SIM_SYSTEM_PLUGIN_PATH", args.build_dir / "lib")
    process = subprocess.Popen(
        ["gz", "sim", "-r", str(ROOT / "worlds/preview.sdf")], env=env)
    request = [
        "gz", "service", "-s", "/gui/camera/view_control/sensitivity",
        "--reqtype", "gz.msgs.Double", "--reptype", "gz.msgs.Boolean",
        "--timeout", "500", "--req", f"data: {cfg['sensitivity']}",
    ]
    applied = False
    try:
        for _ in range(30):
            if process.poll() is not None:
                return process.returncode
            result = subprocess.run(request, env=env, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL, check=False)
            if result.returncode == 0:
                applied = True
                break
            time.sleep(.2)
        if not applied:
            print("警告：未能设置浏览相机灵敏度，Gazebo GUI 服务未就绪", file=sys.stderr)
        return process.wait()
    except KeyboardInterrupt:
        process.terminate()
        try:
            return process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            return process.wait()


if __name__ == "__main__":
    raise SystemExit(main())
