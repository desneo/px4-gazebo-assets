"""Gazebo (gz-transport) 数据源。

提供：
- 相机最新帧（RGB，1920x1080）
- 吊舱关节状态（位置/速度）
- 载机（安装系）姿态：mount_imu（直接姿态或自滤波姿态），含仿真时钟->单调时钟映射与角速度外推
- 仿真时钟 -> 单调时钟映射（用于时延统计）
"""
from __future__ import annotations

import threading
import time
from collections import deque

import numpy as np
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.image_pb2 import Image
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.model_pb2 import Model
from gz.transport13 import Node

from attitude import AttitudeEstimator
from kinematics import quat_mul, quat_normalize, quat_to_R

GIMBAL_JOINTS = (
    "cgo3_vertical_arm_joint",   # yaw
    "cgo3_horizontal_arm_joint", # roll
    "cgo3_camera_joint",         # pitch
)


class Frame:
    __slots__ = ("seq", "data", "width", "height", "sim_stamp", "t_arrive", "t0")

    def __init__(self, seq, data, width, height, sim_stamp, t_arrive, t0):
        self.seq = seq
        self.data = data
        self.width = width
        self.height = height
        self.sim_stamp = sim_stamp     # 仿真时间（秒）
        self.t_arrive = t_arrive       # 服务端收到帧的单调时间（秒）
        self.t0 = t0                   # 估计的帧生成单调时间（秒）


class GzSource:
    """订阅 Gazebo 话题并提供线程安全的最新数据。"""

    def __init__(self, world: str, gimbal_model: str, attitude_source: str = "none"):
        self.node = Node()
        self.world = world
        self.gimbal_model = gimbal_model
        self.attitude_source = attitude_source
        self._lock = threading.Lock()

        self._latest_frame: Frame | None = None
        self._frame_count = 0
        self._last_seq = None

        self._joints: dict[str, tuple[float, float]] = {}
        self._joint_t: dict[str, tuple[float, float]] = {}  # name -> (t, stamp)

        # 时钟偏移： 单调时间 ≈ 仿真时间 + offset
        self._clock_offsets: deque[float] = deque(maxlen=201)
        self._clock_offset: float = 0.0

        # 安装系 IMU（载机姿态）：(t_mono估计, 到达时刻, q(wxyz), 机体系角速度)
        self._imu_samples: deque[tuple[float, float, np.ndarray, np.ndarray]] = \
            deque(maxlen=4000)
        self._imu_seen = False
        self._estimator = AttitudeEstimator() if attitude_source == "filter" else None

        img_topic = (f"/world/{world}/model/{gimbal_model}/link/camera_link/"
                     f"sensor/camera/image")
        self.node.subscribe(Image, img_topic, self._on_image)
        self.node.subscribe(Model, "/gimbal/joint_state", self._on_joint_state)
        self.node.subscribe(Clock, f"/world/{world}/clock", self._on_clock)
        if attitude_source in ("imu_direct", "filter"):
            imu_topic = (f"/world/{world}/model/{gimbal_model}/link/cgo3_mount_link/"
                         f"sensor/mount_imu/imu")
            self.node.subscribe(IMU, imu_topic, self._on_imu)

    # ---------------- callbacks ----------------
    def _on_clock(self, msg):
        sim_t = msg.sim.sec + msg.sim.nsec * 1e-9
        if sim_t <= 0:
            return
        off = time.monotonic() - sim_t
        with self._lock:
            self._clock_offsets.append(off)
            if len(self._clock_offsets) >= 5:
                s = sorted(self._clock_offsets)
                self._clock_offset = s[len(s) // 2]

    def _on_image(self, msg):
        t_arrive = time.monotonic()
        seq = None
        for entry in msg.header.data:
            if entry.key == "seq" and len(entry.value) > 0:
                try:
                    seq = int(entry.value[0])
                except ValueError:
                    pass
        if seq is None:
            self._frame_count += 1
            seq = self._frame_count
        sim_stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        data = bytes(msg.data)
        with self._lock:
            off = self._clock_offset
            t0 = (sim_stamp + off) if off > 0 else t_arrive
            self._latest_frame = Frame(seq, data, msg.width, msg.height,
                                       sim_stamp, t_arrive, t0)
            self._last_seq = seq

    def _on_joint_state(self, msg):
        t = time.monotonic()
        with self._lock:
            for j in msg.joint:
                self._joints[j.name] = (j.axis1.position, j.axis1.velocity)
                self._joint_t[j.name] = (t, t)

    def _on_imu(self, msg):
        t_arrive = time.monotonic()
        sim_t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        w = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z])
        if self._estimator is not None:
            a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
            q = self._estimator.update(t_arrive, w, a)
        else:
            q = np.array([msg.orientation.w, msg.orientation.x,
                          msg.orientation.y, msg.orientation.z])
        with self._lock:
            off = self._clock_offset
            t0 = (sim_t + off) if off > 0 else t_arrive
            self._imu_samples.append((t0, t_arrive, q, w))
            self._imu_seen = True

    # ---------------- accessors ----------------
    def latest_frame(self) -> Frame | None:
        with self._lock:
            return self._latest_frame

    def joints(self) -> dict[str, tuple[float, float]]:
        with self._lock:
            return dict(self._joints)

    @property
    def clock_offset(self) -> float:
        with self._lock:
            return self._clock_offset

    def imu_age(self) -> float:
        """最近一次 IMU 数据的年龄（秒）；从未收到返回 inf。"""
        with self._lock:
            if not self._imu_samples:
                return float("inf")
            return time.monotonic() - self._imu_samples[-1][1]

    def base_attitude_at(self, t_mono: float | None = None) -> dict | None:
        """查询 t_mono（默认当前）时刻的安装系姿态。

        返回 {"R": 3x3, "w": (wx,wy,wz) rad/s, "age": 秒, "t": 样本时刻}；
        无数据返回 None。用样本角速度外推到查询时刻。
        """
        now = time.monotonic()
        tq = now if t_mono is None else t_mono
        with self._lock:
            if not self._imu_samples:
                return None
            sample = None
            for s in reversed(self._imu_samples):
                if s[0] <= tq:
                    sample = s
                    break
            if sample is None:
                sample = self._imu_samples[0]
            t0, t_arr, q, w = sample
        dt = float(np.clip(tq - t0, -0.05, 0.1))
        q_pred = quat_normalize(q + quat_mul(q, np.array([0.0, *w])) * (0.5 * dt))
        return {"R": quat_to_R(q_pred), "w": w, "age": now - t_arr, "t": t0}
