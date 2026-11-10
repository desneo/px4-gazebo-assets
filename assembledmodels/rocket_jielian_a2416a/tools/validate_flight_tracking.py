#!/usr/bin/env python3
"""验证完整飞行期间的 A2416A 捷联遥测和真值跟踪证据。"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def all_zero(value) -> bool:
    return isinstance(value, list) and all(isinstance(item, (int, float)) and item == 0 for item in value)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--telemetry", type=Path, required=True)
    parser.add_argument("--flight-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not args.flight_report.is_file():
        raise SystemExit(f"缺少飞行验收结果：{args.flight_report}")
    flight = json.loads(args.flight_report.read_text(encoding="utf-8"))
    if flight.get("passed") is not True:
        raise SystemExit("火箭飞行验收结果不是通过状态")
    if not args.telemetry.is_file():
        raise SystemExit(f"缺少捷联遥测记录：{args.telemetry}")
    records = [json.loads(line) for line in args.telemetry.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(records) < 50:
        raise SystemExit(f"捷联遥测不足50帧：{len(records)}")
    sequences = [int(record.get("measurement_seq", -1)) for record in records]
    if any(sequence < 0 for sequence in sequences) or sequences[-1] <= sequences[0]:
        raise SystemExit("捷联 measurement_seq 未递增")
    stamps = [float(record.get("capture_monotonic_s", math.nan)) for record in records]
    if not all(math.isfinite(stamp) for stamp in stamps) or stamps[-1] <= stamps[0]:
        raise SystemExit("捷联采集时间无效")
    observed_rate = (len(records) - 1) / (stamps[-1] - stamps[0])
    if not 40.0 <= observed_rate <= 60.0:
        raise SystemExit(f"捷联内部遥测频率不在50Hz容差内：{observed_rate:.3f}Hz")
    tracking = [record for record in records if record.get("tracking_state") == "TRACKING"]
    if not tracking:
        raise SystemExit("完整飞行期间未记录到真实目标 TRACKING 状态")
    for record in records:
        for field in ("platform_frame_deg", "platform_gyro_rate_dps", "los_rate_dps", "seeker_attitude_deg"):
            if not all_zero(record.get(field)):
                raise SystemExit(f"捷联平台专有字段必须为零：{field}")
    summary = {
        "flight_report": str(args.flight_report.resolve()),
        "flight_passed": True,
        "landing_confirmed": flight.get("landing_confirmed") is True,
        "flight_results": flight.get("results", {}),
        "telemetry_frames": len(records),
        "tracking_frames": len(tracking),
        "observed_telemetry_hz": observed_rate,
        "measurement_seq_first": sequences[0],
        "measurement_seq_last": sequences[-1],
        "platform_fields_zero": True,
        "passed": True,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
