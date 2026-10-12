"""吊舱几何与运动学工具（与 models/gimbals/three_axis_pod/model.sdf 保持一致）。

坐标与关节链：
  R_M->C = Rz(-yaw) @ Rx(-roll) @ Ry(pitch)
  - yaw 关节轴 (0,0,-1)，roll 轴 (-1,0,0)，pitch 轴 (0,1,0)
相机传感器重装（rpy = 180,-90,0 度）：
  传感器 x（视线）= camera_link +Z；传感器 y = +Y；传感器 z（图像上）= +X

IK（朝天分支）：
  俯仰角 e（相对水平面）、方位角 a： pitch = e - 90°, yaw = 180° - a
"""
from __future__ import annotations

import math

import numpy as np

# 传感器相对 camera_link 的旋转: 把传感器坐标变换到 camera_link
R_CAM_SENSOR = np.array([
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
])

# 相机水平视场角 60°
HFOV = math.radians(60.0)


def Rz(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def Rx(t: float) -> np.ndarray:
    c, s = math.cos(t), math.sin(t)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def R_mount_camera(yaw: float, roll: float, pitch: float) -> np.ndarray:
    """camera_link 在 mount 系中的姿态。"""
    return Rz(-yaw) @ Rx(-roll) @ Ry(pitch)


def R_mount_sensor(yaw: float, roll: float, pitch: float) -> np.ndarray:
    return R_mount_camera(yaw, roll, pitch) @ R_CAM_SENSOR


def los_mount(yaw: float, roll: float, pitch: float) -> np.ndarray:
    """当前视线方向（mount 系）。"""
    return R_mount_camera(yaw, roll, pitch) @ np.array([0.0, 0.0, 1.0])


def az_el_from_dir(d: np.ndarray) -> tuple[float, float]:
    """方向 -> (方位角, 俯仰角)，弧度。方位角自 +X 逆时针。"""
    az = math.atan2(d[1], d[0])
    el = math.asin(max(-1.0, min(1.0, d[2])))
    return az, el


def ik_up(az: float, el: float) -> tuple[float, float]:
    """朝天安装分支 IK：返回 (yaw, pitch)（弧度）。"""
    return math.pi - az, el - math.pi / 2.0


def pixel_to_sensor_ray(px: float, py: float, width: int, height: int,
                        hfov: float = HFOV) -> np.ndarray:
    """像素坐标（含 0.5 像素中心）-> 传感器系单位方向向量。

    与 Gazebo 相机渲染一致（图像 x 向右对应传感器 +y）。
    """
    f = (width / 2.0) / math.tan(hfov / 2.0)
    ax = math.atan((px - width / 2.0) / f)   # 目标在右侧为正
    ay = math.atan((py - height / 2.0) / f)  # 目标在下方为正
    d = np.array([1.0, math.tan(ax), -math.tan(ay)])
    return d / np.linalg.norm(d)


def sensor_ray_to_az_el(d_sensor: np.ndarray, yaw: float, roll: float,
                        pitch: float, base_R: np.ndarray | None = None,
                        ) -> tuple[float, float]:
    """传感器系方向 -> mount 系（或世界系，给 base_R）方位/俯仰角。"""
    d = R_mount_sensor(yaw, roll, pitch) @ d_sensor
    if base_R is not None:
        d = base_R @ d
    return az_el_from_dir(d)


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def quat_from_R(R: np.ndarray) -> tuple[float, float, float, float]:
    """旋转矩阵 -> 四元数 (w, x, y, z)。"""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return w, x, y, z


def R_to_rpy_deg(R: np.ndarray) -> tuple[float, float, float]:
    """旋转矩阵 -> (roll, pitch, yaw) 角度制（ZYX 内旋）。"""
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 0.999999:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quat_mul(a, b) -> np.ndarray:
    """四元数乘法 a ⊗ b（w, x, y, z）。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_normalize(q) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_to_R(q) -> np.ndarray:
    """四元数 (w, x, y, z) -> 旋转矩阵。"""
    w, x, y, z = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
