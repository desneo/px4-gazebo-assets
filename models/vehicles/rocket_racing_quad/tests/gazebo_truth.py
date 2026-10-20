"""订阅 Gazebo 世界位姿，提供独立于 PX4 估计器的米制运动真值。"""
from collections import deque
import math
import threading
import time


class GazeboTruth:
    def __init__(self, world_name='rocket_racing_quad_test', model_name='rocket_racing_quad'):
        if not all(isinstance(value, str) and value.strip() for value in (world_name, model_name)):
            raise ValueError('Gazebo 真值世界名和实体名不能为空')
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.model_name = model_name
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
            row = (stamp, pose.position.x, pose.position.y, pose.position.z)
            with self.lock:
                if self.latest and stamp <= self.latest[0]:
                    return
                self.latest = row
                self.last_wall = time.monotonic()
                if self.origin is not None:
                    self.rows.append(row)
            return

    def calibrate(self):
        with self.lock:
            if self.latest is None:
                return False
            self.origin = self.latest[1:]
            self.previous = self.latest
            self.rows.clear()
        return True

    def drain(self, home_z):
        with self.lock:
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
            row = self.latest
        ox, oy, oz = self.origin
        return [row[2] - oy, row[1] - ox, home_z + oz - row[3]]

    def close(self):
        self.node.unsubscribe(self.topic)
