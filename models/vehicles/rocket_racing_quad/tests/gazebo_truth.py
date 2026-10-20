"""订阅 Gazebo 世界位姿，提供独立于 PX4 估计器的米制运动真值。"""
from collections import deque
import math
import threading
import time


def finite_vector(values, length, label):
    if isinstance(values, str):
        values = values.split()
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError):
        raise ValueError(f'{label}必须为{length}个有限数值') from None
    if len(result) != length or any(isinstance(value, bool) for value in values) or not all(map(math.isfinite, result)):
        raise ValueError(f'{label}必须为{length}个有限数值')
    return result


def nominal_cg_reference(snapshot):
    """直接模型/link位姿下，计算零关节位姿的固定机体参考点。"""
    links = snapshot.get('model', {}).get('links')
    if not isinstance(links, dict) or not links:
        raise ValueError('重心真值测量需要运行快照中完整的link质量和直接位姿')
    mass, moment = 0., [0., 0., 0.]
    for link in links.values():
        if not isinstance(link, dict):
            raise ValueError('重心计算需要完整link对象')
        value = link.get('mass_kg')
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('重心计算需要有限正数link质量')
        x, y, z, roll, pitch, yaw = finite_vector(link.get('pose'), 6, 'link位姿')
        ix, iy, iz, *_ = finite_vector(link.get('inertial_pose'), 6, '惯性位姿')
        # SDF欧拉角使用弧度，主动旋转为Rz(yaw) Ry(pitch) Rx(roll)。
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                                 math.sin(pitch), math.cos(yaw), math.sin(yaw))
        iy, iz = cr * iy - sr * iz, sr * iy + cr * iz
        ix, iz = cp * ix + sp * iz, -sp * ix + cp * iz
        ix, iy = cy * ix - sy * iy, sy * ix + cy * iy
        for axis, position in enumerate((x + ix, y + iy, z + iz)):
            moment[axis] += value * position
        mass += value
    if not math.isfinite(mass) or not all(map(math.isfinite, moment)):
        raise ValueError('重心质量或一阶矩溢出')
    return dict(kind='nominal_cg', body_offset_flu_m=[value / mass for value in moment],
                reference_pose='zero_joint_positions')


def validate_measurement_reference(snapshot, reference):
    expected = nominal_cg_reference(snapshot)
    if not isinstance(reference, dict) or set(reference) != set(expected):
        raise ValueError('报告缺少完整重心测量参考点')
    offset = finite_vector(reference['body_offset_flu_m'], 3, '测量偏移')
    if (reference['kind'] != expected['kind'] or reference['reference_pose'] != expected['reference_pose']
            or any(not math.isclose(a, b, rel_tol=0, abs_tol=1e-12)
                   for a, b in zip(offset, expected['body_offset_flu_m']))):
        raise ValueError('报告重心测量参考点与运行快照不一致')
    return expected


def world_reference_point(pose, offset):
    position = finite_vector((pose.position.x, pose.position.y, pose.position.z), 3, 'Gazebo位置')
    if not any(offset):
        return position
    q = pose.orientation
    w, x, y, z = finite_vector((q.w, q.x, q.y, q.z), 4, 'Gazebo四元数')
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if not math.isfinite(norm) or abs(norm - 1) > .001:
        raise ValueError('Gazebo四元数必须为单位四元数')
    w, x, y, z = (value / norm for value in (w, x, y, z))
    # 同条消息的model-FLU→world-ENU主动旋转；之后才做ENU→NED。
    vx, vy, vz = offset
    tx, ty, tz = 2*(y*vz-z*vy), 2*(z*vx-x*vz), 2*(x*vy-y*vx)
    rotated = (vx+w*tx+y*tz-z*ty, vy+w*ty+z*tx-x*tz, vz+w*tz+x*ty-y*tx)
    return finite_vector([a + b for a, b in zip(position, rotated)], 3, 'Gazebo参考点位置')


class GazeboTruth:
    def __init__(self, world_name='rocket_racing_quad_test', model_name='rocket_racing_quad',
                 body_offset_flu_m=(0., 0., 0.)):
        if not all(isinstance(value, str) and value.strip() for value in (world_name, model_name)):
            raise ValueError('Gazebo 真值世界名和实体名不能为空')
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.model_name = model_name
        self.body_offset_flu_m = finite_vector(body_offset_flu_m, 3, '测量偏移')
        self.error = None
        self.lock = threading.Lock()
        self.rows = deque()
        self.latest = None
        self.previous = None
        self.origin = None
        self.last_wall = 0
        self.node = Node()
        self.topic = f'/world/{world_name}/dynamic_pose/info'
        self.node.subscribe(Pose_V, self.topic, self.receive)

    def receive(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        for pose in message.pose:
            if pose.name != self.model_name:
                continue
            try:
                if not math.isfinite(stamp):
                    raise ValueError('Gazebo真值时间必须有限')
                row = (stamp, *world_reference_point(pose, self.body_offset_flu_m))
            except (ValueError, AttributeError) as error:
                with self.lock:
                    self.error = str(error)
                return
            with self.lock:
                if self.latest:
                    if stamp < self.latest[0]:
                        self.error = 'Gazebo 真值时钟倒退'
                        return
                    if stamp == self.latest[0]:
                        return
                self.latest = row
                self.last_wall = time.monotonic()
                if self.origin is not None:
                    self.rows.append(row)
            return

    def calibrate(self):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            if self.latest is None:
                return False
            self.origin = self.latest[1:]
            self.previous = self.latest
            self.rows.clear()
        return True

    def drain(self, home_z):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            rows = list(self.rows)
            self.rows.clear()
        samples = []
        ox, oy, oz = self.origin
        for row in rows:
            old = self.previous
            self.previous = row
            dt = row[0] - old[0]
            if dt <= 0:
                raise RuntimeError('Gazebo 真值时钟倒退')
            position = [row[2] - oy, row[1] - ox, home_z + oz - row[3]]
            velocity = [(row[2] - old[2]) / dt, (row[1] - old[1]) / dt, (old[3] - row[3]) / dt]
            if not all(math.isfinite(value) for value in position + velocity):
                raise RuntimeError('Gazebo 真值包含非有限数')
            samples.append((row[0], position, velocity))
        return samples

    def position(self, home_z):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            row = self.latest
        ox, oy, oz = self.origin
        return [row[2] - oy, row[1] - ox, home_z + oz - row[3]]

    def close(self):
        self.node.unsubscribe(self.topic)
