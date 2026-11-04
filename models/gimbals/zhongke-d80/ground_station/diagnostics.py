"""本机 Gazebo 只读状态；不可进入遥测外发链。"""
import json
import threading
import time


class DisabledDiagnostics:
    """独立连接未指定仿真分区时，丰富诊断显示未知。"""
    error = '独立连接未启用仿真诊断'

    def snapshot(self):
        return None

    def close(self):
        pass


class DiagnosticSubscriber:
    def __init__(self, config_hash, *, topic='/zhongke_d80/diagnostics',
                 model_name='zhongke_d80', max_age_s=1.0):
        self.config_hash, self.model_name = config_hash, model_name
        self.topic = topic
        self.max_age_s = max_age_s
        self._lock = threading.Lock()
        self._latest = None
        self._received_at = 0.0
        self.error = ''
        self.node = None
        try:
            from gz.transport13 import Node
            from gz.msgs10.stringmsg_pb2 import StringMsg
            self.node = Node()
            if not self.node.subscribe(StringMsg, topic, self._on_message):
                raise RuntimeError('无法订阅本机诊断话题')
        except (ImportError, RuntimeError) as exc:
            self.error = f'本机诊断不可用：{exc}'

    def _on_message(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            if (value.get('config_hash') != self.config_hash or
                    value.get('model_name') != self.model_name):
                return
        except (ValueError, TypeError, AttributeError):
            return
        with self._lock:
            self._latest = value
            self._received_at = time.monotonic()
            self.error = ''

    def snapshot(self):
        with self._lock:
            if self._latest is None or time.monotonic() - self._received_at > self.max_age_s:
                return None
            return dict(self._latest)

    def close(self):
        if self.node is not None:
            self.node.unsubscribe(self.topic)
            self.node = None
