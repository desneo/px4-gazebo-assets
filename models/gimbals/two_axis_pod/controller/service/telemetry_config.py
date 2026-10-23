"""插件和网关共用的姿态遥测频率配置。"""
import json
import math
from pathlib import Path

DEFAULT_TELEMETRY_CONFIG = Path(__file__).with_name("telemetry.json")


def load_telemetry_config(path=DEFAULT_TELEMETRY_CONFIG) -> float:
    path = Path(path).expanduser().resolve()
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ValueError(f"无法读取遥测频率配置 {path}: {exc}") from exc
    hz = data.get("hz") if isinstance(data, dict) else None
    if (isinstance(hz, bool) or not isinstance(hz, (int, float))
            or not 0 < hz <= 1000 or not math.isfinite(hz)):
        raise ValueError(f"遥测配置 {path} 的 hz 必须为 (0, 1000] 的有限数值")
    return float(hz)
