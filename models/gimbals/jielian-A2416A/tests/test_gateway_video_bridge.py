#!/usr/bin/env python3
"""网关视频覆盖层映射不需要NVENC即可执行的测试。"""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gateway.main import _detection_boxes, _gate  # noqa: E402


class OverlayMappingTest(unittest.TestCase):
    def test_gate_accepts_list_and_mapping(self):
        self.assertEqual(_gate({"gate_px": [1, 2, 3, 4]}), (1, 2, 3, 4))
        self.assertEqual(_gate({"gate_px": {"x": 1, "y": 2, "w": 3, "h": 4}}),
                         (1, 2, 3, 4))
        self.assertIsNone(_gate({"gate_px": [1, 2]}))

    def test_detection_boxes_are_limited_to_four(self):
        payload = {"targets": [
            {"box_px": [index, 2, 3, 4]} for index in range(6)
        ]}
        self.assertEqual(_detection_boxes(payload), (
            (0, 2, 3, 4), (1, 2, 3, 4),
            (2, 2, 3, 4), (3, 2, 3, 4),
        ))


if __name__ == "__main__":
    unittest.main(verbosity=2)
