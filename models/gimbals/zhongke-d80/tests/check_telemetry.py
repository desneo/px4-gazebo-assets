#!/usr/bin/env python3
"""真实中科UDP至JSON验收入口。"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from tools.check_telemetry import main
if __name__ == '__main__':
    raise SystemExit(main())
