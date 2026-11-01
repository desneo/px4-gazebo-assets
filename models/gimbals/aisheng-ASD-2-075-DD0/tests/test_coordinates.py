#!/usr/bin/env python3
"""广角/长焦像素换算单元测试。"""
from __future__ import annotations

from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ground_station.main import (  # noqa: E402
    F_TELE, F_WIDE, HEIGHT, WIDTH, tele_to_wide, wide_to_tele)


class CoordinateTest(unittest.TestCase):
    def test_center(self):
        self.assertAlmostEqual(wide_to_tele(WIDTH / 2, HEIGHT / 2)[0], WIDTH / 2)
        self.assertAlmostEqual(tele_to_wide(WIDTH / 2, HEIGHT / 2)[1], HEIGHT / 2)

    def test_scale(self):
        u, v = wide_to_tele(WIDTH / 2 + 200, HEIGHT / 2 + 100)
        self.assertAlmostEqual(u - WIDTH / 2, 200 * F_TELE / F_WIDE, places=3)
        self.assertAlmostEqual(v - HEIGHT / 2, 100 * F_TELE / F_WIDE, places=3)

    def test_roundtrip(self):
        u, v = tele_to_wide(800, 500)
        u2, v2 = wide_to_tele(u, v)
        self.assertAlmostEqual(u2, 800, places=3)
        self.assertAlmostEqual(v2, 500, places=3)


if __name__ == "__main__":
    unittest.main()
