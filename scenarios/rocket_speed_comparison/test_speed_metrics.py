import math
import unittest
from speed_metrics import StableSpeed


class StableSpeedTest(unittest.TestCase):
    def feed(self, metric, speed=lambda t: 95., altitude=lambda t: 0., count=1001):
        for i in range(count):
            t = i/100
            metric.update(t, speed(t), altitude(t))
        return metric.summary()

    def test_full_simulation_duration_required(self):
        metric = StableSpeed()
        self.assertFalse(self.feed(metric, count=1000)['valid_stable_window'])
        metric.update(10., 95., 0.)
        result = metric.summary()
        self.assertTrue(result['valid_stable_window'])
        self.assertEqual(result['stable_window']['duration_s'], 10.)
        self.assertTrue(result['target_reached'])

    def test_transient_peak_does_not_replace_stable_speed(self):
        metric = StableSpeed()
        result = self.feed(metric, speed=lambda t: 110. if t == 5. else 95.)
        self.assertEqual(result['peak_speed_m_s'], 110.)
        self.assertFalse(result['valid_stable_window'])

    def test_missing_samples_break_continuity(self):
        metric = StableSpeed()
        self.feed(metric, count=600)
        for i in range(601, 1101):
            metric.update(i/100+.5, 95., 0.)
        self.assertFalse(metric.summary()['valid_stable_window'])
        self.assertEqual(metric.discontinuities, 1)

    def test_altitude_excursion_breaks_window(self):
        result = self.feed(StableSpeed(), altitude=lambda t: 2.1 if t == 5. else 0.)
        self.assertFalse(result['valid_stable_window'])
        self.assertEqual(result['max_altitude_error_m'], 2.1)

    def test_accelerating_flight_is_not_steady(self):
        result = self.feed(StableSpeed(speed_std=5., speed_range=5.), speed=lambda t: 90.+.3*t)
        self.assertFalse(result['valid_stable_window'])

    def test_unreachable_target_records_measured_stable_speed(self):
        result = self.feed(StableSpeed(), speed=lambda t: 83.+.1*math.sin(t))
        self.assertTrue(result['valid_stable_window'])
        self.assertFalse(result['target_reached'])
        self.assertGreater(result['stable_window']['mean_m_s'], 82.)

    def test_clock_regression_and_nonfinite_are_invalid(self):
        metric = StableSpeed()
        metric.update(1., 95., 0.)
        for stamp, speed in ((1., 95.), (.9, 95.), (2., math.nan)):
            with self.assertRaises(ValueError):
                metric.update(stamp, speed, 0.)


if __name__ == '__main__':
    unittest.main()
