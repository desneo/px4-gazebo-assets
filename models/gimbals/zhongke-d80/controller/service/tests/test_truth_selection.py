"""真值初选使用三维距离与稳定排序，不改变持续跟踪规则。"""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from common.config import load_config
from controller.service.truth_selection import choose_candidate


def candidate(name='target_a', x=960, y=540, distance=100, **extra):
    return dict(name=name, center_px=[x, y], range_m=distance, depth_m=99,
                entity_id=11, frame_seq=20, valid=True, **extra)


class TruthSelectionTest(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.config['tracking']['target_names'] = ['target_a', 'target_b', 'target_c']

    def select(self, candidates, point=None):
        return choose_candidate({'truth_candidates': candidates}, self.config, point)

    def test_point_and_auto_use_different_nearest_locations(self):
        values = [candidate('target_a', x=500), candidate('target_b', x=950)]
        self.assertEqual(self.select(values)[0]['name'], 'target_b')
        self.assertEqual(self.select(values, (490, 540))[0]['name'], 'target_a')

    def test_fallback_is_reported_but_still_selects_nearest_on_screen(self):
        selected, fallback = self.select([candidate(x=1100)], (10, 540))
        self.assertEqual(selected['name'], 'target_a')
        self.assertTrue(fallback)

    def test_inclusive_discovery_boundaries_use_range_not_depth(self):
        for distance, expected in ((299, True), (300, True), (301, False)):
            with self.subTest(distance=distance):
                value = candidate(distance=distance)
                value['depth_m'] = 5
                self.assertEqual(self.select([value])[0] is not None, expected)

    def test_registration_visibility_validity_and_finite_values_are_required(self):
        original = candidate()
        changes = ({'name': 'unknown'}, {'valid': False}, {'center_px': [-1, 540]},
                   {'center_px': [1920, 540]}, {'depth_m': 0}, {'range_m': -1},
                   {'range_m': float('nan')}, {'entity_id': True}, {'frame_seq': -1})
        for change in changes:
            with self.subTest(change=change):
                value = deepcopy(original)
                value.update(change)
                self.assertIsNone(self.select([value])[0])

    def test_equal_distances_use_stable_entity_name_order(self):
        values = [candidate('target_b', x=970), candidate('target_a', x=950)]
        self.assertEqual(self.select(values)[0]['name'], 'target_a')
        self.assertEqual(self.select(list(reversed(values)))[0]['name'], 'target_a')

    def test_no_candidate_returns_clear_absence(self):
        self.assertEqual(self.select([]), (None, False))


if __name__ == '__main__':
    unittest.main()
