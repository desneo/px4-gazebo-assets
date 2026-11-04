"""独立双向标定与固定感光面公式的数值契约。"""
import math
from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from common.config import load_config, refresh_derived, to_model_parameters


class OpticalTests(unittest.TestCase):
    def test_fixed_sensor_and_focal_change(self):
        c = load_config()
        d = c['derived'].copy()
        self.assertAlmostEqual(d['sensor_width_mm'], 6.142540310851201, places=11)
        self.assertAlmostEqual(d['sensor_height_mm'], 3.448391171758914, places=11)
        self.assertAlmostEqual(d['hfov_deg'], 7.03, places=10)
        self.assertAlmostEqual(d['vfov_deg'], 3.95, places=10)
        self.assertNotAlmostEqual(d['fx'], d['fy'], places=1)
        c['optics']['focal_length_mm'] = 100
        refresh_derived(c)
        e = c['derived']
        self.assertEqual(e['sensor_width_mm'], d['sensor_width_mm'])
        self.assertEqual(e['sensor_height_mm'], d['sensor_height_mm'])
        self.assertAlmostEqual(e['fx'], 2*d['fx'])
        self.assertAlmostEqual(e['fy'], 2*d['fy'])
        self.assertAlmostEqual(e['hfov_deg'], 3.5183103922, places=9)
        self.assertAlmostEqual(e['vfov_deg'], 1.9755868472, places=9)
        self.assertEqual(to_model_parameters(c)['camera']['fy'], e['fy'])

    def test_resolution_recomputes_both_axes(self):
        c = load_config()
        fx, fy = c['derived']['fx'], c['derived']['fy']
        c['optics'].update(width_px=960, height_px=540)
        refresh_derived(c)
        self.assertAlmostEqual(c['derived']['fx'], fx/2)
        self.assertAlmostEqual(c['derived']['fy'], fy/2)
        self.assertEqual((c['derived']['cx'], c['derived']['cy']), (480,270))

    def test_large_finite_calibration_avoids_intermediate_overflow(self):
        c = load_config()
        c['optics'].update(focal_length_mm=1e308,reference_focal_length_mm=1e308)
        refresh_derived(c)
        self.assertTrue(all(math.isfinite(v) and v>0 for v in c['derived'].values() if isinstance(v,(float,int))))
        self.assertAlmostEqual(c['derived']['hfov_deg'],7.03)
        self.assertAlmostEqual(c['derived']['fx'],15628.713063617944,places=8)

    def test_underflow_and_nonfinite_derived_values_rejected(self):
        for changes in (
            dict(focal_length_mm=5e-324,reference_focal_length_mm=5e-324),
            dict(focal_length_mm=1e308,reference_focal_length_mm=1e308,reference_hfov_deg=179.99),
            dict(reference_hfov_deg=5e-324),
            dict(focal_length_mm=5e-324,reference_focal_length_mm=1e308)):
            c=load_config();c['optics'].update(changes)
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                refresh_derived(c)


if __name__ == '__main__':
    unittest.main()
