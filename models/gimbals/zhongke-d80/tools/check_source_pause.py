#!/usr/bin/env python3
"""真实暂停仿真和网关，验证标准遥测与JSON不重放缓存。"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from scripts.session import load_session,process_matches
from ground_station.binary_client import BinaryClient
from ground_station.telemetry_forward import TelemetryForwarder
from tools.check_telemetry import Collector,wait_until,assert_silent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',default='validation')
    args=parser.parse_args()
    session=load_session(args.session)
    os.environ['GZ_PARTITION']=session['gz_partition']
    from gz.transport13 import Node
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.msgs10.boolean_pb2 import Boolean
    node=Node()
    collector=Collector()
    forward=TelemetryForwarder()
    client=BinaryClient(session['udp_host'],session['udp_port'],on_telemetry=forward.offer)
    gateway=session['processes']['gateway']
    paused=False
    gateway_paused=False
    result={'passed':False}
    def pause(value):
        success,response=node.request('/world/'+session['world']+'/control',
            WorldControl(pause=value),WorldControl,Boolean,2000)
        if not success or not response.data:
            raise AssertionError('Gazebo暂停/恢复服务失败')
    def check_silence(label):
        time.sleep(.3)
        before=client.stats()['received']
        assert_silent(collector,1.1)
        if client.stats()['received']!=before:
            raise AssertionError(label+'仍收到重复标准遥测')
        if client.stats()['connected']:
            raise AssertionError(label+'后仍标记机械测量新鲜')
        return {'binary_count_unchanged':True,'json_count_unchanged':True,'stale_marked':True}
    try:
        client.start()
        wait_until(lambda:client.stats()['connected'],timeout=6)
        forward.start(*collector.address)
        wait_until(lambda:len(collector.snapshot())>20)
        pause(True);paused=True
        result['world_pause']=check_silence('仿真暂停')
        pause(False);paused=False
        previous=client.stats()['received']
        wait_until(lambda:client.stats()['received']>previous+20)
        if not process_matches(gateway):
            raise AssertionError('网关身份失配，拒绝发送进程信号')
        os.kill(int(gateway['pid']),signal.SIGSTOP);gateway_paused=True
        result['gateway_disconnect']=check_silence('网关断源')
        os.kill(int(gateway['pid']),signal.SIGCONT);gateway_paused=False
        previous=client.stats()['received']
        wait_until(lambda:client.stats()['received']>previous+20)
        forward.stop()
        assert_silent(collector,.4)
        result.update(passed=True,config_hash=session['config_hash'],resumed_new_samples=True,
                      stop_silent=True,source='实际Gazebo暂停和本会话网关SIGSTOP/SIGCONT')
    except Exception as exc:
        result['error']=str(exc)
    finally:
        if gateway_paused and process_matches(gateway):
            os.kill(int(gateway['pid']),signal.SIGCONT)
        if paused:
            pause(False)
        client.stop();forward.close();collector.close()
    path=Path(session['session_path']).parent/'source_pause.json'
    path.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
