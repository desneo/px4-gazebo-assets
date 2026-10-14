"""对照报告不能把不同环境、错误机型或缺失证据判为成功。"""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

from report_comparison import compare, load_trial


class ReportComparisonTest(unittest.TestCase):
    def setUp(self):
        self.trials, self.manifests = [], []
        for variant, model in (('standalone', 'rocket_racing_quad'),
                               ('assembled', 'rocket_two_axis_pod')):
            self.trials.append(dict(model_name=model, settings={'target_speed_m_s': 95.},
                control_speed_cap_m_s=95., flight_succeeded=True, landing_confirmed=True,
                valid_stable_window=True, stable_window={'mean_m_s': 94.9},
                target_reached=True, limit_reason='control_limited'))
            self.manifests.append(dict(variant=variant, model_name=model,
                environment_sha256='same-environment', source_sha256={
                    'scenarios/rocket_speed_comparison/flight_trial.py': 'same-script'}))

    def result(self, **kwargs):
        return compare(*self.trials, manifests=self.manifests, **kwargs)

    def test_same_conditions_reach_cap(self):
        self.assertEqual(self.result()['conclusion'], 'both_reach_current_control_cap')

    def test_environment_mismatch_is_inconclusive(self):
        self.manifests[1]['environment_sha256'] = 'different-launch-surface'
        result = self.result()
        self.assertFalse(result['comparable'])
        self.assertIsNone(result['assembled_reaches_baseline'])

    def test_old_manifest_without_environment_is_inconclusive(self):
        for manifest in self.manifests:
            del manifest['environment_sha256']
        self.assertFalse(self.result()['comparable'])

    def test_same_model_twice_does_not_make_a_comparison(self):
        self.trials[0] = copy.deepcopy(self.trials[1])
        self.manifests[0] = copy.deepcopy(self.manifests[1])
        self.assertFalse(self.result()['correct_model_roles'])
        self.assertFalse(self.result()['comparable'])

    def test_scene_code_changes_are_not_comparable(self):
        self.manifests[1]['source_sha256']['scenarios/rocket_speed_comparison/flight_trial.py'] = 'new-script'
        self.assertFalse(self.result()['same_scene_sources'])
        self.assertFalse(self.result()['comparable'])

    def test_launcher_failure_overrides_existing_success(self):
        self.assertFalse(self.result(launcher_results={
            'standalone': {'returncode': 1}, 'assembled': {'returncode': 0}})['comparable'])

    def test_missing_trial_produces_structured_inconclusive_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            trial, manifest, checks = load_trial(directory / 'missing')
            self.assertTrue(trial['evidence_errors'])
            self.assertFalse(manifest)
            self.assertFalse(checks)
            output = directory / 'comparison.json'
            process = subprocess.run([sys.executable,
                str(Path(__file__).with_name('report_comparison.py')),
                '--standalone', str(directory / 'missing-standalone'),
                '--assembled', str(directory / 'missing-assembled'), '--output', str(output)],
                capture_output=True, text=True)
            self.assertEqual(process.returncode, 1)
            report = json.loads(output.read_text())
            self.assertEqual(report['conclusion'], 'inconclusive')
            self.assertFalse(report['evidence_complete'])
            self.assertNotIn('Traceback', process.stderr)


if __name__ == '__main__':
    unittest.main()
