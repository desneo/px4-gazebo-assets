"""诊断仅接受当前模型/性能配置，失效后不能沿用旧目标身份。"""
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from ground_station.diagnostics import DiagnosticSubscriber


class DiagnosticsTest(unittest.TestCase):
    def test_config_model_and_freshness_guard(self):
        subscriber=DiagnosticSubscriber.__new__(DiagnosticSubscriber)
        subscriber.config_hash='expected'
        subscriber.model_name='zhongke_d80'
        subscriber.max_age_s=1
        subscriber._lock=threading.Lock()
        subscriber._latest=None
        subscriber._received_at=0
        message={'model_name':'zhongke_d80','config_hash':'wrong','target_name':'target_a'}
        subscriber._on_message(SimpleNamespace(data=json.dumps(message)))
        self.assertIsNone(subscriber.snapshot())
        message.update(config_hash='expected',model_name='other')
        subscriber._on_message(SimpleNamespace(data=json.dumps(message)))
        self.assertIsNone(subscriber.snapshot())
        message['model_name']='zhongke_d80'
        subscriber._on_message(SimpleNamespace(data=json.dumps(message)))
        self.assertEqual(subscriber.snapshot()['target_name'],'target_a')
        subscriber._received_at=time.monotonic()-2
        self.assertIsNone(subscriber.snapshot())


if __name__=='__main__':
    unittest.main()
