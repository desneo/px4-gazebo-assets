#!/usr/bin/env python3
"""只读记录和复核参考组件，保留用户已有工作区状态。"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[2]
PREFIXES = (
    "models/gimbals/two_axis_pod", "models/gimbals/aisheng-ASD-2-075-DD0",
    "models/gimbals/three_axis_pod", "assembledmodels", "scenarios",
)
PX4_MESSAGE = REPO.parent / "PX4-Autopilot/msg/DytZhongkeTelemetry.msg"
DEFAULT = ROOT / "runs/baseline/reference_hashes.json"


def digest(path: Path) -> dict:
    if path.is_symlink():
        return {"symlink": str(path.readlink())}
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return {"size": path.stat().st_size, "sha256": result.hexdigest()}


def capture() -> dict:
    listed = subprocess.check_output(
        ["git", "-C", str(REPO), "ls-files", "-z", "--cached", "--others",
         "--exclude-standard", "--", *PREFIXES])
    paths = sorted(set(x.decode() for x in listed.split(b"\0") if x))
    files = {name: digest(REPO / name) for name in paths if (REPO / name).exists()}
    if PX4_MESSAGE.is_file():
        files["@px4/msg/DytZhongkeTelemetry.msg"] = digest(PX4_MESSAGE)
    status = subprocess.check_output(
        ["git", "-C", str(REPO), "status", "--short", "--", *PREFIXES], text=True)
    return {"files": files, "initial_status": status}


def verify(baseline: dict) -> list[str]:
    changes = []
    for name, expected in baseline["files"].items():
        path = PX4_MESSAGE if name.startswith("@px4/") else REPO / name
        if not path.exists() or digest(path) != expected:
            changes.append(name)
    return changes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("capture", "verify"))
    parser.add_argument("--output", type=Path, default=DEFAULT)
    args = parser.parse_args()
    if args.action == "capture":
        if args.output.exists():
            parser.error("基线已存在，拒绝覆盖；请复核现有基线")
        data = capture()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n")
        print(f"已记录 {len(data['files'])} 个参考文件，未修改参考组件")
        return 0
    changes = verify(json.loads(args.output.read_text()))
    if changes:
        print("参考内容有变化，请核对其他会话或用户改动，禁止自动回退：")
        print("\n".join(changes))
        return 1
    print("参考文件内容与基线一致")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
