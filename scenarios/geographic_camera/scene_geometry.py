"""地理场景固定安装与目标视角参数。"""
import math
import time


def target_x(mount_mode):
    return 0.0 if mount_mode == 'sideward' else 30.0


def panel_position(mount_mode):
    return (target_x(mount_mode) / 2, 0, 40)


def initialize_camera(send, collect, mount_mode):
    # 公共 FRD 俯仰相对水平面；目标位于平台顶面附近。
    target_pitch = {'sideward': -90.0,
                    'skyward': -math.degrees(math.atan2(19.3, 30)),
                    'downward': -math.degrees(math.atan2(19.3, 30))}[mount_mode]
    send('reset')
    def public_zero(data):
        return (data.get('mode') == 'IDLE' and data.get('angle_convention') == 'px4_frd'
                and all(abs(data['attitude_deg'][axis]['pos']) < 0.8
                        for axis in ('yaw', 'pitch', 'roll')))
    last = collect(30, public_zero)
    if not last:
        raise AssertionError('初始化没有收到遥测')
    if not public_zero(last[-1]):
        raise AssertionError('公共三轴零位复位未到位，不能开始目标初始化')
    deadline = time.monotonic() + 40
    while time.monotonic() < deadline:
        assert last[-1].get('angle_convention') == 'px4_frd', '需要公共角度协议 v2'
        attitude = last[-1]['attitude_deg']
        pitch = attitude['pitch']['pos']
        yaw = attitude['yaw']['pos']
        roll = attitude['roll']['pos']
        pitch_error = target_pitch - pitch
        roll_error = -roll
        yaw_error = (-yaw + 180) % 360 - 180
        if abs(pitch_error) < 0.3 and abs(yaw_error) < 0.3 and abs(roll_error) < 0.3:
            send('stop')
            settled = collect(0.3)
            if settled:
                actual = settled[-1]['attitude_deg']
                if (abs(actual['pitch']['pos'] - target_pitch) < 0.5 and
                        abs((actual['yaw']['pos'] + 180) % 360 - 180) < 0.5 and
                        abs(actual['roll']['pos']) < 0.5):
                    return
                last = settled
                continue
            raise AssertionError('相机停止到位确认时遥测中断')
        send('manual', pitch=max(-20, min(20, pitch_error * 4)),
             yaw=max(-20, min(20, yaw_error * 4)),
             roll=max(-20, min(20, roll_error * 4)))
        last = collect(0.1)
        if not last:
            raise AssertionError('初始化遥测中断')
    raise AssertionError(f'相机未对准目标，目标pitch={target_pitch:.2f}°')
