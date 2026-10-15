#!/usr/bin/env python3
"""只读验证网关飞机身份、RTSP路径与视频帧/遥测表对应关系。"""
import argparse
import hashlib
import json
from pathlib import Path
import socket
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def validate_identity(message, aircraft_id, path):
    if type(message.get('aircraft_id')) is not int or message['aircraft_id'] != aircraft_id:
        raise ValueError('网关上报飞机 ID 与期望不一致')
    if message.get('video_rtsp_path') != path:
        raise ValueError('网关上报视频路径与期望不一致')


def describe(host, port, path):
    with socket.create_connection((host, port), timeout=3) as sock:
        sock.sendall((f'DESCRIBE rtsp://{host}:{port}{path} RTSP/1.0\r\n'
                      'CSeq: 1\r\nAccept: application/sdp\r\n\r\n').encode())
        data = b''
        while b'\r\n' not in data:
            chunk = sock.recv(4096)
            if not chunk:
                raise RuntimeError('RTSP服务未返回状态')
            data += chunk
        return int(data.split(b'\r\n', 1)[0].split()[1])


def verify_station_guard(station, args):
    """用真实网关hello验证身份；误配控制只到本地记录socket，不发往飞机。"""
    clients = [station.UdpClient(args.host, args.udp_port, aircraft_id=identity)
               for identity in (args.aircraft_id, args.aircraft_id+1)]
    try:
        for client in clients:
            client.start()
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            if all(c.hello is not None and c.snapshot() for c in clients):
                break
            time.sleep(.02)
        correct, wrong = clients
        if not correct.identity_ready or not correct.public_angles_ready or wrong.identity_ready:
            raise RuntimeError('地面站匹配/错误ID身份门禁状态不正确')
        evidence = dict(correct_identity_ready=True, correct_public_angles_ready=True,
                        mismatched_identity_ready=False, mismatched_error=wrong.identity_error,
                        correct_hello=correct.hello, mismatched_hello=wrong.hello)
    finally:
        for client in clients:
            client.stop()
    attempts = []
    class RecordingSocket:
        def sendto(self, data, address):
            attempts.append(json.loads(data))
            return len(data)
    old = wrong.sock
    wrong.sock = RecordingSocket()
    try:
        results = [wrong.send_reliable('home'), wrong.send_reliable('track', box=[.4,.4,.1,.1]),
                   wrong.send_manual(yaw=10.), wrong.send_once('reset_fault')]
    finally:
        wrong.sock = old
    if any(value != -1 for value in results) or attempts:
        raise RuntimeError('误配ID仍尝试发送控制')
    evidence.update(blocked_commands=['home','track','manual','reset_fault'],
                    command_api_results=results, attempted_control_datagrams=len(attempts))
    return evidence


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--aircraft-id', type=int, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--udp-port', type=int, default=8770)
    parser.add_argument('--rtsp-port', type=int, default=8555)
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--absent-id', type=int, help='明确约定未注册的ID，应返回404')
    parser.add_argument('--check-station-identity', action='store_true', help='验证最新地面站的匹配与误配ID控制门禁')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.aircraft_id < 0 or not 2 <= args.duration <= 60:
        parser.error('飞机ID必须非负，采样时长须在2至60秒之间')
    sys.path.insert(0, str(ROOT / 'models/gimbals/two_axis_pod/ground_station'))
    import ground_station as station
    import numpy as np
    import cv2
    path = f'/two_axis_pod_{args.aircraft_id}'
    url = station.rtsp_url(dict(gimbal_host=args.host, rtsp_port=args.rtsp_port,
                               aircraft_id=args.aircraft_id))
    assert url == f'rtsp://{args.host}:{args.rtsp_port}{path}'
    args.output.mkdir(parents=True, exist_ok=True)
    source_names = ['models/gimbals/two_axis_pod/controller/service/'+name
                    for name in ('gimbal_bridge.py', 'video.py', 'rtsp_relay.py')]
    source_names += ['models/gimbals/two_axis_pod/ground_station/ground_station.py',
                     str(Path(__file__).relative_to(ROOT))]
    hashes = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in source_names}
    report = dict(aircraft_id=args.aircraft_id, udp_port=args.udp_port, rtsp_url=url,
                  expected_path=path, source_sha256=hashes, result='failed',
                  frames=0, telemetry_count=0, hello_count=0, identity_errors=[])
    address = (socket.gethostbyname(args.host), args.udp_port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(('0.0.0.0', 0))
    sock.setblocking(False)
    pipeline = None
    video_sequences, table_sequences, clock_domains = set(), set(), set()
    decoded, latest, next_ping = 0, None, 0.
    started = time.monotonic()
    try:
        if args.check_station_identity:
            report['station_identity_guard'] = verify_station_guard(station, args)
        report['describe_status'] = describe(args.host, args.rtsp_port, path)
        if report['describe_status'] != 200:
            raise RuntimeError('目标RTSP路径未成功注册')
        if args.absent_id is not None:
            if args.absent_id == args.aircraft_id:
                raise ValueError('不存在ID不能等于待测ID')
            report['absent_id'] = args.absent_id
            report['absent_describe_status'] = describe(args.host, args.rtsp_port,
                                                       f'/two_axis_pod_{args.absent_id}')
            if report['absent_describe_status'] != 404:
                raise RuntimeError('约定未注册路径没有返回404')
        pipeline, sink, decoder = station.build_pipeline(url, 'avdec')
        report['decoder'] = decoder
        pipeline.set_state(station.Gst.State.PLAYING)
        with (args.output/'binding_samples.jsonl').open('w') as log:
            while time.monotonic()-started < args.duration+1:
                now = time.monotonic()
                if now >= next_ping:
                    sock.sendto(b'{"cmd":"ping"}', address)
                    next_ping = now+.5
                for _ in range(300):
                    try:
                        data, sender = sock.recvfrom(65535)
                    except BlockingIOError:
                        break
                    if sender != address:
                        continue
                    message = json.loads(data)
                    kind = message.get('type')
                    if kind not in ('hello', 'telemetry'):
                        continue
                    try:
                        validate_identity(message, args.aircraft_id, path)
                    except ValueError as error:
                        report['identity_errors'].append(str(error))
                    report[kind+'_count'] += 1
                    if message.get('clock_domain'):
                        clock_domains.add(message['clock_domain'])
                    for row in message.get('frame_table', []):
                        table_sequences.add(int(row[0]) & 0xffffff)
                    log.write(json.dumps(dict(kind=kind, wall=time.time(), payload=message))+'\n')
                if now-started < args.duration:
                    sample = sink.emit('try-pull-sample', 20*station.Gst.MSECOND)
                    if sample is not None:
                        buf, caps = sample.get_buffer(), sample.get_caps().get_structure(0)
                        width, height = int(caps.get_value('width')), int(caps.get_value('height'))
                        ok, info = buf.map(station.Gst.MapFlags.READ)
                        if ok:
                            frame = np.ndarray((height, width, 3), np.uint8, buffer=info.data)
                            seq = station.decode_stamp_from_bgr(frame, width, height)
                            latest = frame.copy()
                            buf.unmap(info)
                            report['frames'] += 1
                            report['resolution'] = [width, height]
                            if seq is not None:
                                decoded += 1
                                video_sequences.add(seq)
                            log.write(json.dumps(dict(kind='video', wall=time.time(), seq=seq))+'\n')
                else:
                    time.sleep(.01)
        report.update(decoded_frames=decoded, unique_video_sequences=len(video_sequences),
                      decoded_ratio=decoded/max(1, report['frames']),
                      matched_frame_table_sequences=len(video_sequences & table_sequences),
                      frame_table_match_ratio=len(video_sequences & table_sequences)/max(1, len(video_sequences)),
                      clock_domains=sorted(clock_domains))
        if (report['hello_count'] < 1 or report['telemetry_count'] < 20 or
                report['frames'] < 20 or report['decoded_ratio'] < .95 or
                report['frame_table_match_ratio'] < .95 or report['identity_errors'] or len(clock_domains) != 1):
            raise RuntimeError('身份、连续视频或帧表对应关系未满足验收条件')
        if latest is not None:
            cv2.imwrite(str(args.output/'rtsp_snapshot.png'), latest)
        report['result'] = 'passed'
    except Exception as error:
        report['error'] = str(error)
    finally:
        if pipeline is not None:
            pipeline.set_state(station.Gst.State.NULL)
        sock.close()
        report['source_fingerprints_current'] = all(
            hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == value for name,value in hashes.items())
        if not report['source_fingerprints_current']:
            report['result'] = 'failed'
        (args.output/'binding_summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['result'] == 'passed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
