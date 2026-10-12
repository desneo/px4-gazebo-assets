#!/usr/bin/env python3
"""载机姿态估计（自滤波姿态——鲁棒性验证项）。

四元数互补滤波（Mahony 风格 P 校正；可选 I 估计陀螺零偏）：
- 陀螺积分作为预测；加速度计重力方向对 roll/pitch 做慢校正
- 加速度门限：|‖a‖ - g| 超限时暂停加速度校正（机动加速度不污染水平姿态）
- yaw 无绝对参考，由陀螺积分（仿真零均值噪声下分钟级漂移可忽略）

约定（与 Gazebo IMU / gz ENU 世界一致）：
- q = (w, x, y, z)：机体（安装系）-> 世界（ENU，Z 上），与 kinematics.quat_to_R 一致
- gyro：机体系角速度（rad/s）
- accel：机体系比力（m/s²）。静止水平时读数约为 (0,0,+g)（重力反作用）。

用法：
  est = AttitudeEstimator()
  q = est.update(t_mono, gyro, accel)

离线自检：python3 models/gimbals/three_axis_pod/controller/service/attitude.py
"""
from __future__ import annotations

import math
import time

import numpy as np

import kinematics as km

G = 9.80665


class AttitudeEstimator:
    def __init__(self, kp: float = 1.0, ki: float = 0.02,
                 accel_gate: float = 2.0, max_dt: float = 0.05):
        self.kp = kp
        self.ki = ki
        self.accel_gate = accel_gate
        self.max_dt = max_dt
        self.q: np.ndarray | None = None
        self.bias = np.zeros(3)
        self._t: float | None = None

    # ------------------------------------------------------------------
    @staticmethod
    def _init_from_accel(accel: np.ndarray) -> np.ndarray:
        """由重力方向估计初始 roll/pitch（yaw=0）。"""
        a = accel / max(1e-9, float(np.linalg.norm(accel)))
        up = a                                   # 静止时比力指向天（机体的世界+Z 表示）
        z = np.array([0.0, 0.0, 1.0])
        v = np.cross(up, z)
        c = float(np.clip(np.dot(up, z), -1.0, 1.0))
        s = math.sqrt(max(0.0, 2.0 * (1.0 + c)))
        if s < 1e-6:
            return np.array([0.0, 1.0, 0.0, 0.0])   # 完全倒置
        return km.quat_normalize(np.array([0.5 * s, v[0] / s, v[1] / s, v[2] / s]))

    # ------------------------------------------------------------------
    def update(self, t: float, gyro, accel) -> np.ndarray:
        gyro = np.asarray(gyro, dtype=float)
        accel = np.asarray(accel, dtype=float)
        if self.q is None:
            self._t = t
            self.q = self._init_from_accel(accel)
            return self.q.copy()

        dt = t - self._t
        self._t = t
        if dt <= 0:
            return self.q.copy()
        dt = min(dt, self.max_dt)

        # 校正项（加速度计重力方向）
        corr = np.zeros(3)
        an = float(np.linalg.norm(accel))
        if abs(an - G) < self.accel_gate and an > 1e-3:
            up_meas = accel / an                      # 机体上方向（测量）
            up_est = km.quat_to_R(self.q).T @ np.array([0.0, 0.0, 1.0])  # 估计的机体上方向
            e = np.cross(up_meas, up_est)
            self.bias += self.ki * e * dt
            corr = self.kp * e + self.bias

        # 预测 + 校正（同一积分步）
        w = gyro + corr
        dq = km.quat_mul(self.q, np.array([0.0, w[0], w[1], w[2]])) * (0.5 * dt)
        self.q = km.quat_normalize(self.q + dq)
        return self.q.copy()


# ----------------------------------------------------------------------
def _selftest():
    """离线自检：模拟已知姿态轨迹，验证估计器收敛误差 < 2°。"""
    rng = np.random.default_rng(7)
    est = AttitudeEstimator()
    dt = 0.004
    n = 2500
    max_err = 0.0
    for k in range(n):
        t = k * dt
        roll = math.radians(25.0 * math.sin(2 * math.pi * 0.25 * t))
        pitch = math.radians(15.0 * math.sin(2 * math.pi * 0.18 * t + 0.7))
        yaw = math.radians(40.0 * math.sin(2 * math.pi * 0.1 * t))
        R = km.Rz(yaw) @ km.Ry(pitch) @ km.Rx(roll)          # body->world
        q_true = np.array(km.quat_from_R(R))
        gyro = np.array([
            math.cos(pitch) * (-math.sin(roll)) * 0 + 0.0,   # 由数值微分得到精确机体角速度
            0.0, 0.0])
        # 数值微分求真实机体角速度（避免解析求导出错）
        if k < n - 1:
            t2 = t + dt
            roll2 = math.radians(25.0 * math.sin(2 * math.pi * 0.25 * t2))
            pitch2 = math.radians(15.0 * math.sin(2 * math.pi * 0.18 * t2 + 0.7))
            yaw2 = math.radians(40.0 * math.sin(2 * math.pi * 0.1 * t2))
            R2 = km.Rz(yaw2) @ km.Ry(pitch2) @ km.Rx(roll2)
            dR = R2 @ R.T
            gyro = R.T @ _log_so3(dR) / dt   # 世界系增量 -> 机体系角速度
        # 加速度计：重力在机体的表示（静止比力）+ 少量机动加速度
        up_body = R.T @ np.array([0.0, 0.0, 1.0])
        accel = up_body * G + rng.normal(0, 0.05, 3)
        if k == 0:
            est.update(t, gyro, accel)
            continue
        q_est = est.update(t, gyro, accel)
        Re = km.quat_to_R(q_est)
        # 姿态误差角（忽略 yaw 不可观测：比较机体上方向）
        up_err = math.degrees(math.acos(float(np.clip(Re.T @ np.array([0, 0, 1.0]) @ up_body, -1, 1))))
        max_err = max(max_err, up_err)
    up_est = km.quat_to_R(est.q).T @ np.array([0.0, 0.0, 1.0])
    up_true = R.T @ np.array([0.0, 0.0, 1.0])
    final_err = math.degrees(math.acos(float(np.clip(np.dot(up_est, up_true), -1, 1))))
    print(f"[attitude self-test] 最大水平误差 {max_err:.2f}°，最终 {final_err:.2f}°")
    return max_err < 2.0


def _log_so3(R: np.ndarray) -> np.ndarray:
    """SO(3) 对数映射（接近单位阵）。"""
    tr = float(np.trace(R))
    c = max(-1.0, min(1.0, (tr - 1.0) / 2.0))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v * (th / (2.0 * math.sin(th)))


if __name__ == "__main__":
    import sys
    sys.exit(0 if _selftest() else 1)
