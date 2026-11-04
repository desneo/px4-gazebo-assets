"""独立黄金极值帧与PX4/Schema契约，覆盖发送时刻和完整类型边界。"""
from dataclasses import replace
import json
import math
from pathlib import Path
import re
import socket
import struct
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from jsonschema import Draft202012Validator
from common.protocol import decode_telemetry
from common.runtime_types import ReceivedTelemetry
from ground_station.telemetry_forward import json_measurement,TelemetryForwarder

# 随测试保存外部字段契约，不依赖不入库的spec-kit目录。
SCHEMA={'type':'object','additionalProperties':False,'properties':{
    'timestamp':{'type':'integer','minimum':0,'maximum':2**64-1},
    **{key:{'type':'integer','minimum':0,'maximum':255}
       for key in ('status1','status2','status3','self_check')},
    'zoom':{'type':'number','minimum':0,'maximum':409.5},
    **{key:{'type':'number','minimum':-1638.4,'maximum':1638.35}
       for key in ('offset_x','offset_y')},
    **{key:{'type':'number','minimum':-327.68,'maximum':327.67}
       for key in ('frame_roll','frame_pitch','frame_yaw','roll_rate','pitch_rate','yaw_rate')},
    'laser_range':{'type':'number','minimum':0,'maximum':6553.5}}}
SCHEMA['required']=list(SCHEMA['properties'])


def golden_frame(*,yaw_raw=-23456):
    packet=bytearray(32)
    packet[:6]=bytes((0xEE,0x16,0xF4,0x8D,0xBC,0xDA))
    for offset,raw in ((6,-32768),(8,32767),(10,-1),(12,12345),(14,yaw_raw),
                        (20,-32768),(22,32767),(24,-4567)):
        struct.pack_into('<h',packet,offset,raw)
    packet[16:20]=bytes((1,2,3,4))
    struct.pack_into('<H',packet,26,65535)
    packet[28]=255
    packet[29:31]=bytes((5,6))
    packet[31]=sum(packet[:31])&255
    return bytes(packet)


def sample(sequence=1,*,yaw_raw=-23456):
    raw=golden_frame(yaw_raw=yaw_raw)
    return ReceivedTelemetry(rx_seq=sequence,received_monotonic=time.monotonic(),
                             fields=decode_telemetry(raw),source=('127.0.0.1',8870),raw=raw)


class JsonContractTest(unittest.TestCase):
    def test_schema_field_types_match_each_px4_field_and_uint64_boundaries(self):
        px4=ROOT.parents[2].parent/'PX4-Autopilot/msg/DytZhongkeTelemetry.msg'
        if not px4.is_file():
            self.skipTest('指定PX4消息文件不在本机，此项需在验收环境运行')
        fields=dict((name,kind) for kind,name in re.findall(
            r'^(uint64|uint8|float32)\s+(\w+)',px4.read_text(),re.MULTILINE))
        self.assertEqual(set(SCHEMA['properties']),set(fields))
        self.assertEqual(set(SCHEMA['required']),set(fields))
        self.assertFalse(SCHEMA['additionalProperties'])
        for name,kind in fields.items():
            prop=SCHEMA['properties'][name]
            self.assertEqual(prop['type'],'number' if kind=='float32' else 'integer')
            if kind in ('uint8','uint64'):
                self.assertEqual(prop['minimum'],0)
                self.assertEqual(prop['maximum'],255 if kind=='uint8' else 2**64-1)
        validator=Draft202012Validator(SCHEMA)
        payload=dict(json_measurement(sample()),timestamp=2**64-1)
        self.assertTrue(validator.is_valid(payload))
        for invalid in (-1,2**64,True,1.25):
            self.assertFalse(validator.is_valid(dict(payload,timestamp=invalid)))
        self.assertFalse(validator.is_valid(dict(payload,target_name='forbidden')))

    def test_independent_golden_extremes_status_u12_and_units(self):
        actual=json_measurement(sample())
        self.assertEqual(actual,dict(status1=0xF4,status2=0x8D,status3=0xDA,zoom=274.8,
            offset_x=-1638.4,offset_y=1638.35,frame_roll=-.01,frame_pitch=123.45,
            frame_yaw=-234.56,roll_rate=-327.68,pitch_rate=327.67,yaw_rate=-45.67,
            laser_range=6553.5,self_check=255))
        Draft202012Validator(SCHEMA).validate(dict(actual,timestamp=0))

    def test_every_field_rejects_bool_and_nonfinite_and_exact_key_set_required(self):
        original=sample()
        for name in original.fields:
            for invalid in (True,float('nan'),float('inf'),-float('inf')):
                with self.subTest(field=name,invalid=invalid):
                    fields=dict(original.fields,**{name:invalid})
                    with self.assertRaises(ValueError):
                        json_measurement(replace(original,fields=fields))
        for fields in (dict(original.fields,rx_seq=1),
                       {k:v for k,v in original.fields.items() if k!='zoom'}):
            with self.assertRaises(ValueError):
                json_measurement(replace(original,fields=fields))

    def test_timestamp_generated_at_send_after_waiting_in_queue(self):
        class Gated(TelemetryForwarder):
            def __init__(self):
                self.gate=threading.Event()
                super().__init__()
            def _run(self):
                self.gate.wait(2)
                super()._run()
        receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        receiver.bind(('127.0.0.1',0));receiver.settimeout(1)
        forward=Gated()
        try:
            forward.start(*receiver.getsockname())
            with patch('ground_station.telemetry_forward.time.time_ns',return_value=1_000_000) as clock:
                forward.offer(sample())
                clock.return_value=9_876_543_210_000_000
                forward.gate.set()
                payload=json.loads(receiver.recvfrom(4096)[0])
            self.assertEqual(payload['timestamp'],9_876_543_210)
            Draft202012Validator(SCHEMA).validate(payload)
        finally:
            forward.gate.set();forward.close();receiver.close()


if __name__=='__main__':
    unittest.main()
