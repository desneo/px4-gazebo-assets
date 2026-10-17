#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Two-axis pod aircraft detection and multi-object tracking.

The module deliberately keeps the real-time scheduling policy out of the vision
code.  ``VisionEngine.process_rgb`` processes exactly the frame supplied by the
bridge; the bridge owns the one-element/latest-frame mailbox.

Runtime dependencies are NumPy and OpenCV (DNN).  Tracking, assignment and NMS
do not depend on SciPy, Torch or an external tracker package.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import math
import time
from typing import Iterable, Sequence

import numpy as np


def _cv2():
    """Import OpenCV lazily so pure tracker tests do not require it."""
    try:
        import cv2  # type: ignore
    except ImportError as exc:
        raise RuntimeError("目标检测需要安装带 DNN 的 OpenCV Python 运行库") from exc
    return cv2


def _finite_number(name: str, value: float, low: float, high: float,
                   *, low_inclusive: bool = True) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} 必须是数值")
    value = float(value)
    valid_low = value >= low if low_inclusive else value > low
    if not math.isfinite(value) or not valid_low or value > high:
        left = "[" if low_inclusive else "("
        raise ValueError(f"{name} 必须位于 {left}{low}, {high}] 范围")
    return value


def _positive_int(name: str, value: int, maximum: int = 1_000_000) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not (1 <= value <= maximum):
        raise ValueError(f"{name} 必须是 1 到 {maximum} 的整数")
    return value


@dataclass(frozen=True)
class DetectorConfig:
    """YOLO11 ONNX detector configuration."""

    model_path: str | Path
    input_size: int = 640
    frame_width: int = 1920
    frame_height: int = 1080
    confidence_threshold: float = 0.50
    low_confidence_threshold: float = 0.10
    nms_iou_threshold: float = 0.45
    opencv_threads: int = 2

    def __post_init__(self):
        path = Path(self.model_path).expanduser()
        if not path.is_file():
            raise ValueError(f"ONNX 模型不存在或不是文件: {path}")
        if path.suffix.lower() != ".onnx":
            raise ValueError(f"目标检测模型必须是 .onnx 文件: {path}")
        object.__setattr__(self, "model_path", path)
        _positive_int("input_size", self.input_size, 4096)
        _positive_int("frame_width", self.frame_width, 16384)
        _positive_int("frame_height", self.frame_height, 16384)
        high = _finite_number("confidence_threshold", self.confidence_threshold, 0.0, 1.0,
                              low_inclusive=False)
        low = _finite_number("low_confidence_threshold", self.low_confidence_threshold,
                             0.0, 1.0, low_inclusive=True)
        if low >= high:
            raise ValueError("low_confidence_threshold 必须小于 confidence_threshold")
        _finite_number("nms_iou_threshold", self.nms_iou_threshold, 0.0, 1.0,
                       low_inclusive=False)
        _positive_int("opencv_threads", self.opencv_threads, 256)


@dataclass(frozen=True)
class TrackerConfig:
    """ByteTrack-style two-stage tracker configuration."""

    confirmation_hits: int = 5
    max_missed: int = 15
    high_confidence_threshold: float = 0.50
    low_confidence_threshold: float = 0.10
    high_match_iou: float = 0.30
    low_match_iou: float = 0.20

    def __post_init__(self):
        _positive_int("confirmation_hits", self.confirmation_hits, 10_000)
        _positive_int("max_missed", self.max_missed, 100_000)
        high = _finite_number("high_confidence_threshold", self.high_confidence_threshold,
                              0.0, 1.0, low_inclusive=False)
        low = _finite_number("low_confidence_threshold", self.low_confidence_threshold,
                             0.0, 1.0, low_inclusive=True)
        if low >= high:
            raise ValueError("low_confidence_threshold 必须小于 high_confidence_threshold")
        _finite_number("high_match_iou", self.high_match_iou, 0.0, 1.0,
                       low_inclusive=False)
        _finite_number("low_match_iou", self.low_match_iou, 0.0, 1.0,
                       low_inclusive=False)


@dataclass(frozen=True)
class Detection:
    xyxy: tuple[float, float, float, float]
    confidence: float
    class_id: int = 0


@dataclass(frozen=True)
class Track:
    track_id: int
    xyxy: tuple[float, float, float, float]
    confidence: float
    hits: int
    missed: int
    confirmed: bool


@dataclass(frozen=True)
class VisionResult:
    seq: int
    tracks: tuple[Track, ...]
    inference_ms: float


@dataclass(frozen=True)
class LetterboxMeta:
    source_width: int
    source_height: int
    scale: float
    pad_x: float
    pad_y: float


def letterbox_rgb(frame: np.ndarray, size: int = 640) -> tuple[np.ndarray, LetterboxMeta]:
    """Resize an RGB image without distortion and pad it to ``size`` square."""
    if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8:
        raise ValueError("frame 必须是 uint8 NumPy 数组")
    if frame.ndim != 3 or frame.shape[2] != 3 or frame.shape[0] < 1 or frame.shape[1] < 1:
        raise ValueError("frame 必须是非空 HxWx3 RGB 图像")
    _positive_int("size", size, 4096)
    height, width = frame.shape[:2]
    scale = min(size / width, size / height)
    new_width = max(1, min(size, int(round(width * scale))))
    new_height = max(1, min(size, int(round(height * scale))))
    cv2 = _cv2()
    resized = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    pad_left = (size - new_width) // 2
    pad_top = (size - new_height) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[pad_top:pad_top + new_height, pad_left:pad_left + new_width] = resized
    return canvas, LetterboxMeta(width, height, scale, float(pad_left), float(pad_top))


def _box_iou(boxes_a: np.ndarray, boxes_b: np.ndarray) -> np.ndarray:
    if len(boxes_a) == 0 or len(boxes_b) == 0:
        return np.zeros((len(boxes_a), len(boxes_b)), dtype=np.float64)
    a = np.asarray(boxes_a, dtype=np.float64)
    b = np.asarray(boxes_b, dtype=np.float64)
    tl = np.maximum(a[:, None, :2], b[None, :, :2])
    br = np.minimum(a[:, None, 2:], b[None, :, 2:])
    wh = np.maximum(0.0, br - tl)
    inter = wh[..., 0] * wh[..., 1]
    area_a = np.maximum(0.0, a[:, 2] - a[:, 0]) * np.maximum(0.0, a[:, 3] - a[:, 1])
    area_b = np.maximum(0.0, b[:, 2] - b[:, 0]) * np.maximum(0.0, b[:, 3] - b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    return np.divide(inter, union, out=np.zeros_like(inter), where=union > 0.0)


def nms(detections: Sequence[Detection], iou_threshold: float) -> list[Detection]:
    """Class-agnostic NMS for the single-class aircraft model."""
    _finite_number("iou_threshold", iou_threshold, 0.0, 1.0, low_inclusive=False)
    if not detections:
        return []
    order = sorted(range(len(detections)), key=lambda i: (-detections[i].confidence, i))
    boxes = np.asarray([d.xyxy for d in detections], dtype=np.float64)
    kept: list[Detection] = []
    while order:
        current = order.pop(0)
        kept.append(detections[current])
        if order:
            overlaps = _box_iou(boxes[[current]], boxes[order])[0]
            order = [index for index, overlap in zip(order, overlaps) if overlap <= iou_threshold]
    return kept


def _prediction_rows(output: np.ndarray) -> np.ndarray:
    array = np.asarray(output)
    if array.ndim == 3:
        if array.shape[0] != 1:
            raise ValueError(f"YOLO 输出 batch 必须为 1，实际形状 {array.shape}")
        array = array[0]
    if array.ndim != 2:
        raise ValueError(f"YOLO 输出必须是 [1,5,N] 或 [1,N,5]，实际形状 {array.shape}")
    if array.shape[1] == 5:
        rows = array
    elif array.shape[0] == 5:
        rows = array.T
    else:
        raise ValueError(f"单类别 YOLO 输出最后一维必须为 5，实际形状 {array.shape}")
    return np.asarray(rows, dtype=np.float64)


def parse_yolo11_output(output: np.ndarray, meta: LetterboxMeta,
                        confidence_threshold: float = 0.10,
                        nms_iou_threshold: float = 0.45) -> list[Detection]:
    """Parse a raw single-class Ultralytics YOLO11 tensor.

    Supported layouts are ``[1, 5, N]`` and ``[1, N, 5]``.  Each prediction is
    ``(center_x, center_y, width, height, aircraft_confidence)`` in letterboxed
    input pixels.  Boxes are converted back to source-frame coordinates.
    """
    threshold = _finite_number("confidence_threshold", confidence_threshold,
                               0.0, 1.0, low_inclusive=True)
    _finite_number("nms_iou_threshold", nms_iou_threshold, 0.0, 1.0,
                   low_inclusive=False)
    rows = _prediction_rows(output)
    # A 640px YOLO output has 8,400 proposals.  Filter and transform them in
    # NumPy rather than iterating through every proposal in Python; this keeps
    # preprocessing/postprocessing from consuming the 30 fps frame budget.
    valid = (np.all(np.isfinite(rows), axis=1)
             & (rows[:, 4] >= threshold)
             & (rows[:, 2] > 0.0)
             & (rows[:, 3] > 0.0))
    selected = rows[valid]
    if len(selected) == 0:
        return []
    centres = selected[:, :2]
    half_sizes = selected[:, 2:4] * 0.5
    top_left = (centres - half_sizes - np.array([meta.pad_x, meta.pad_y])) / meta.scale
    bottom_right = (centres + half_sizes - np.array([meta.pad_x, meta.pad_y])) / meta.scale
    boxes = np.concatenate((top_left, bottom_right), axis=1)
    boxes[:, (0, 2)] = np.clip(boxes[:, (0, 2)], 0.0, meta.source_width - 1.0)
    boxes[:, (1, 3)] = np.clip(boxes[:, (1, 3)], 0.0, meta.source_height - 1.0)
    nonempty = ((boxes[:, 2] > boxes[:, 0]) & (boxes[:, 3] > boxes[:, 1]))
    detections = [
        Detection(tuple(map(float, box)), float(confidence), 0)
        for box, confidence in zip(boxes[nonempty], selected[nonempty, 4])
    ]
    return nms(detections, nms_iou_threshold)


class Yolo11Detector:
    """OpenCV-DNN inference for an Ultralytics single-class YOLO11 ONNX."""

    def __init__(self, config: DetectorConfig):
        self.config = config
        cv2 = _cv2()
        cv2.setNumThreads(config.opencv_threads)
        try:
            self._net = cv2.dnn.readNetFromONNX(str(config.model_path))
        except Exception as exc:  # OpenCV exception type varies by build
            raise RuntimeError(f"无法加载 YOLO11 ONNX 模型 {config.model_path}: {exc}") from exc
        self._net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self._net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def detect_rgb(self, frame: np.ndarray) -> tuple[list[Detection], float]:
        if frame.shape != (self.config.frame_height, self.config.frame_width, 3):
            raise ValueError(
                f"RGB 图像尺寸必须为 {self.config.frame_width}x{self.config.frame_height}，"
                f"实际为 {frame.shape[1] if frame.ndim > 1 else '?'}x"
                f"{frame.shape[0] if frame.ndim > 0 else '?'}")
        cv2 = _cv2()
        letterboxed, meta = letterbox_rgb(frame, self.config.input_size)
        blob = cv2.dnn.blobFromImage(letterboxed, scalefactor=1.0 / 255.0,
                                    size=(self.config.input_size, self.config.input_size),
                                    mean=(0.0, 0.0, 0.0), swapRB=False, crop=False)
        started = time.perf_counter()
        self._net.setInput(blob)
        output = self._net.forward()
        inference_ms = (time.perf_counter() - started) * 1000.0
        detections = parse_yolo11_output(
            output, meta, self.config.low_confidence_threshold,
            self.config.nms_iou_threshold)
        return detections, inference_ms


def _xyxy_to_measurement(box: Sequence[float]) -> np.ndarray:
    x1, y1, x2, y2 = map(float, box)
    return np.array([(x1 + x2) * 0.5, (y1 + y2) * 0.5, x2 - x1, y2 - y1],
                    dtype=np.float64)


def _measurement_to_xyxy(value: np.ndarray) -> tuple[float, float, float, float]:
    cx, cy, width, height = value[:4]
    width = max(1e-3, float(width))
    height = max(1e-3, float(height))
    return (float(cx - width * 0.5), float(cy - height * 0.5),
            float(cx + width * 0.5), float(cy + height * 0.5))


class _KalmanBox:
    """Eight-state constant-velocity Kalman filter for (cx, cy, w, h)."""

    def __init__(self, box: Sequence[float]):
        self.state = np.zeros(8, dtype=np.float64)
        self.state[:4] = _xyxy_to_measurement(box)
        self.covariance = np.diag([10.0] * 4 + [100.0] * 4)
        self.transition = np.eye(8, dtype=np.float64)
        self.transition[:4, 4:] = np.eye(4, dtype=np.float64)
        self.observation = np.zeros((4, 8), dtype=np.float64)
        self.observation[:, :4] = np.eye(4, dtype=np.float64)
        self.process_noise = np.diag([1.0] * 4 + [4.0] * 4)
        self.measurement_noise = np.diag([4.0] * 4)

    def predict(self) -> tuple[float, float, float, float]:
        self.state = self.transition @ self.state
        self.covariance = (self.transition @ self.covariance @ self.transition.T
                           + self.process_noise)
        self.state[2:4] = np.maximum(self.state[2:4], 1e-3)
        return _measurement_to_xyxy(self.state)

    def update(self, box: Sequence[float]) -> tuple[float, float, float, float]:
        measurement = _xyxy_to_measurement(box)
        innovation = measurement - self.observation @ self.state
        innovation_cov = (self.observation @ self.covariance @ self.observation.T
                          + self.measurement_noise)
        gain = np.linalg.solve(innovation_cov.T,
                               (self.covariance @ self.observation.T).T).T
        self.state += gain @ innovation
        identity = np.eye(8, dtype=np.float64)
        kh = gain @ self.observation
        # Joseph form preserves a positive-semidefinite covariance better.
        self.covariance = ((identity - kh) @ self.covariance @ (identity - kh).T
                           + gain @ self.measurement_noise @ gain.T)
        self.state[2:4] = np.maximum(self.state[2:4], 1e-3)
        return _measurement_to_xyxy(self.state)


class _TrackState:
    def __init__(self, track_id: int, detection: Detection, confirmation_hits: int):
        self.track_id = track_id
        self.filter = _KalmanBox(detection.xyxy)
        self.xyxy = detection.xyxy
        self.confidence = detection.confidence
        self.hits = 1
        self.hit_streak = 1
        self.missed = 0
        self.confirmed = confirmation_hits <= 1

    def predict(self):
        self.xyxy = self.filter.predict()

    def update(self, detection: Detection, confirmation_hits: int):
        self.xyxy = self.filter.update(detection.xyxy)
        self.confidence = detection.confidence
        self.hits += 1
        self.hit_streak += 1
        self.missed = 0
        if self.hit_streak >= confirmation_hits:
            self.confirmed = True

    def miss(self):
        self.missed += 1
        self.hit_streak = 0

    def view(self) -> Track:
        return Track(self.track_id, tuple(map(float, self.xyxy)), float(self.confidence),
                     self.hits, self.missed, self.confirmed)


def _hungarian(cost: np.ndarray) -> list[tuple[int, int]]:
    """Return a globally minimal rectangular assignment using NumPy only."""
    matrix = np.asarray(cost, dtype=np.float64)
    if matrix.ndim != 2:
        raise ValueError("cost 必须是二维矩阵")
    rows, columns = matrix.shape
    if rows == 0 or columns == 0:
        return []
    transposed = rows > columns
    if transposed:
        matrix = matrix.T
        rows, columns = matrix.shape
    # Shortest augmenting path form of the Hungarian algorithm (rows <= cols).
    u = np.zeros(rows + 1, dtype=np.float64)
    v = np.zeros(columns + 1, dtype=np.float64)
    p = np.zeros(columns + 1, dtype=np.int64)
    way = np.zeros(columns + 1, dtype=np.int64)
    for row in range(1, rows + 1):
        p[0] = row
        min_value = np.full(columns + 1, np.inf, dtype=np.float64)
        used = np.zeros(columns + 1, dtype=bool)
        column0 = 0
        while True:
            used[column0] = True
            row0 = p[column0]
            delta = np.inf
            column1 = 0
            for column in range(1, columns + 1):
                if not used[column]:
                    current = matrix[row0 - 1, column - 1] - u[row0] - v[column]
                    if current < min_value[column]:
                        min_value[column] = current
                        way[column] = column0
                    if min_value[column] < delta:
                        delta = min_value[column]
                        column1 = column
            for column in range(columns + 1):
                if used[column]:
                    u[p[column]] += delta
                    v[column] -= delta
                else:
                    min_value[column] -= delta
            column0 = column1
            if p[column0] == 0:
                break
        while True:
            column1 = way[column0]
            p[column0] = p[column1]
            column0 = column1
            if column0 == 0:
                break
    result: list[tuple[int, int]] = []
    for column in range(1, columns + 1):
        if p[column] != 0:
            pair = (int(p[column] - 1), column - 1)
            result.append((pair[1], pair[0]) if transposed else pair)
    return result


def _associate(states: Sequence[_TrackState], detections: Sequence[Detection],
               minimum_iou: float) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    if not states or not detections:
        return [], list(range(len(states))), list(range(len(detections)))
    state_boxes = np.asarray([state.xyxy for state in states], dtype=np.float64)
    detection_boxes = np.asarray([detection.xyxy for detection in detections], dtype=np.float64)
    overlaps = _box_iou(state_boxes, detection_boxes)
    assignments = _hungarian(1.0 - overlaps)
    matches = [(row, column) for row, column in assignments
               if overlaps[row, column] >= minimum_iou]
    matched_states = {row for row, _ in matches}
    matched_detections = {column for _, column in matches}
    return (matches,
            [i for i in range(len(states)) if i not in matched_states],
            [i for i in range(len(detections)) if i not in matched_detections])


class ByteTrackTracker:
    """ByteTrack-style high/low-score association with stable track IDs."""

    def __init__(self, config: TrackerConfig = TrackerConfig()):
        self.config = config
        self._states: list[_TrackState] = []
        self._next_id = 1

    def reset(self):
        self._states.clear()
        self._next_id = 1

    def update(self, detections: Iterable[Detection]) -> tuple[Track, ...]:
        candidates = list(detections)
        for detection in candidates:
            if detection.class_id != 0:
                raise ValueError("两轴吊舱跟踪器只接受 aircraft 类别 0")
            if (not math.isfinite(detection.confidence)
                    or not 0.0 <= detection.confidence <= 1.0):
                raise ValueError("检测置信度必须是 [0,1] 的有限数")
            box = detection.xyxy
            if (len(box) != 4 or not np.all(np.isfinite(box))
                    or box[2] <= box[0] or box[3] <= box[1]):
                raise ValueError("检测框必须是有限且有效的 (x1,y1,x2,y2)")

        high = [d for d in candidates
                if d.confidence >= self.config.high_confidence_threshold]
        low = [d for d in candidates
               if self.config.low_confidence_threshold <= d.confidence
               < self.config.high_confidence_threshold]
        for state in self._states:
            state.predict()

        matches_high, unmatched_state_indices, unmatched_high = _associate(
            self._states, high, self.config.high_match_iou)
        for state_index, detection_index in matches_high:
            self._states[state_index].update(high[detection_index],
                                             self.config.confirmation_hits)

        remaining_states = [self._states[i] for i in unmatched_state_indices]
        matches_low, unmatched_remaining, _ = _associate(
            remaining_states, low, self.config.low_match_iou)
        for remaining_index, detection_index in matches_low:
            state_index = unmatched_state_indices[remaining_index]
            self._states[state_index].update(low[detection_index],
                                             self.config.confirmation_hits)

        unmatched_after_both = {
            unmatched_state_indices[index] for index in unmatched_remaining
        }
        for state_index in unmatched_after_both:
            self._states[state_index].miss()

        for detection_index in unmatched_high:
            self._states.append(_TrackState(self._next_id, high[detection_index],
                                            self.config.confirmation_hits))
            self._next_id += 1

        self._states = [state for state in self._states
                        if state.missed <= self.config.max_missed]
        # Unconfirmed states are deliberately neither returned nor selectable.
        return tuple(state.view() for state in self._states if state.confirmed)


class VisionEngine:
    """Convenient synchronous bridge-facing detector + tracker API."""

    def __init__(self, detector_config: DetectorConfig,
                 tracker_config: TrackerConfig | None = None):
        if tracker_config is None:
            tracker_config = TrackerConfig(
                high_confidence_threshold=detector_config.confidence_threshold,
                low_confidence_threshold=detector_config.low_confidence_threshold)
        elif (tracker_config.high_confidence_threshold
              != detector_config.confidence_threshold
              or tracker_config.low_confidence_threshold
              != detector_config.low_confidence_threshold):
            raise ValueError("检测器与跟踪器的高低置信度阈值必须一致")
        self.detector = Yolo11Detector(detector_config)
        self.tracker = ByteTrackTracker(tracker_config)

    def reset(self):
        """Drop all tracks when the mutually exclusive vision mode changes."""
        self.tracker.reset()

    def process_rgb(self, frame: np.ndarray, seq: int) -> VisionResult:
        if isinstance(seq, bool) or not isinstance(seq, (int, np.integer)) or int(seq) < 0:
            raise ValueError("seq 必须是非负整数")
        detections, inference_ms = self.detector.detect_rgb(frame)
        tracks = self.tracker.update(detections)
        return VisionResult(int(seq), tracks, float(inference_ms))


def nearest_confirmed_track(tracks: Iterable[Track], frame_width: int,
                            frame_height: int, *, visible_only: bool = True) -> Track | None:
    """Choose the confirmed target whose box centre is nearest the image centre."""
    _positive_int("frame_width", frame_width, 16384)
    _positive_int("frame_height", frame_height, 16384)
    cx = frame_width * 0.5
    cy = frame_height * 0.5
    eligible = [track for track in tracks
                if track.confirmed and (not visible_only or track.missed == 0)]
    if not eligible:
        return None
    return min(eligible, key=lambda track: (
        ((track.xyxy[0] + track.xyxy[2]) * 0.5 - cx) ** 2
        + ((track.xyxy[1] + track.xyxy[3]) * 0.5 - cy) ** 2,
        track.track_id))


def draw_tracks_rgb(frame: np.ndarray, tracks: Iterable[Track], *,
                    line_width: int = 3) -> np.ndarray:
    """Return an RGB copy with confirmed targets drawn in red and labelled ``#ID``."""
    if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8:
        raise ValueError("frame 必须是 uint8 NumPy 数组")
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame 必须是 HxWx3 RGB 图像")
    _positive_int("line_width", line_width, 64)
    cv2 = _cv2()
    output = frame.copy()
    height, width = output.shape[:2]
    for track in tracks:
        if not track.confirmed:
            continue
        x1, y1, x2, y2 = track.xyxy
        pt1 = (int(np.clip(round(x1), 0, width - 1)),
               int(np.clip(round(y1), 0, height - 1)))
        pt2 = (int(np.clip(round(x2), 0, width - 1)),
               int(np.clip(round(y2), 0, height - 1)))
        # The image is RGB, therefore red is (255, 0, 0), not OpenCV's usual BGR red.
        cv2.rectangle(output, pt1, pt2, (255, 0, 0), line_width)
        label = f"#{track.track_id}"
        text_y = max(16, pt1[1] - 7)
        cv2.putText(output, label, (pt1[0], text_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 0, 0), max(1, line_width - 1), cv2.LINE_AA)
    return output


__all__ = [
    "ByteTrackTracker", "Detection", "DetectorConfig", "LetterboxMeta", "Track",
    "TrackerConfig", "VisionEngine", "VisionResult", "Yolo11Detector",
    "draw_tracks_rgb", "letterbox_rgb", "nearest_confirmed_track", "nms",
    "parse_yolo11_output",
]
