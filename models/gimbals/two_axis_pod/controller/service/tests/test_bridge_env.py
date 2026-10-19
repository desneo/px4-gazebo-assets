#!/usr/bin/env python3
"""网关自动环境检查中的视觉配置与实际 ONNX 加载测试。"""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[6]
CHECKER_PATH = ROOT / "models/gimbals/two_axis_pod/scripts/check_bridge_env.py"
VISION_CONFIG = ROOT / "models/gimbals/two_axis_pod/controller/service/vision.json"
SPEC = importlib.util.spec_from_file_location("two_axis_bridge_env", CHECKER_PATH)
checker = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(checker)


class BridgeEnvironmentTests(unittest.TestCase):
    def test_archived_model_loads_with_opencv_dnn(self):
        cv_version, numpy_version, model_path = checker.check_python_and_model(
            VISION_CONFIG)
        self.assertTrue(cv_version)
        self.assertTrue(numpy_version)
        self.assertEqual(model_path.name, "model.onnx")
        self.assertGreater(model_path.stat().st_size, 1_000_000)

    def test_missing_model_is_rejected_before_bridge_start(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / "vision.json"
            config.write_text(
                json.dumps({"model_path": "missing.onnx"}), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "ONNX 模型不存在"):
                checker.resolve_model_path(config)


if __name__ == "__main__":
    unittest.main(verbosity=2)
