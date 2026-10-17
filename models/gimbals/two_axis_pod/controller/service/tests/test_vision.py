#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Synthetic tests for two-axis pod detection parsing and tracking."""
from __future__ import annotations

from pathlib import Path
import importlib.util
import sys
import tempfile
import unittest

import numpy as np

SERVICE_DIR = Path(__file__).resolve().parents[1]
if str(SERVICE_DIR) not in sys.path:
    sys.path.insert(0, str(SERVICE_DIR))

from vision import (  # noqa: E402
    ByteTrackTracker,
    Detection,
    DetectorConfig,
    LetterboxMeta,
    Track,
    TrackerConfig,
    draw_tracks_rgb,
    nearest_confirmed_track,
    nms,
    parse_yolo11_output,
)


def detection(x1, y1, x2, y2, confidence=0.9):
    return Detection((float(x1), float(y1), float(x2), float(y2)), confidence, 0)


class TrackerTests(unittest.TestCase):
    def test_requires_five_consecutive_hits_by_default(self):
        tracker = ByteTrackTracker()
        for _ in range(4):
            self.assertEqual(tracker.update([detection(100, 100, 180, 160)]), ())
        tracks = tracker.update([detection(100, 100, 180, 160)])
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].track_id, 1)
        self.assertTrue(tracks[0].confirmed)
        self.assertEqual(tracks[0].hits, 5)

    def test_one_frame_false_positive_is_never_output(self):
        tracker = ByteTrackTracker()
        self.assertEqual(tracker.update([detection(20, 30, 60, 80)]), ())
        for _ in range(20):
            self.assertEqual(tracker.update([]), ())

    def test_multiple_targets_keep_stable_ids(self):
        tracker = ByteTrackTracker(TrackerConfig(confirmation_hits=2))
        self.assertEqual(tracker.update([
            detection(100, 100, 160, 150), detection(700, 400, 770, 460)
        ]), ())
        tracks = tracker.update([
            detection(108, 101, 168, 151), detection(692, 402, 762, 462)
        ])
        self.assertEqual([track.track_id for track in tracks], [1, 2])
        for step in range(2, 8):
            # Deliberately reverse detector order: assignment must not depend on list order.
            tracks = tracker.update([
                detection(700 - 8 * step, 400 + step, 770 - 8 * step, 460 + step),
                detection(100 + 8 * step, 100 + step, 160 + 8 * step, 150 + step),
            ])
            left_to_right = sorted(tracks,
                                   key=lambda track: (track.xyxy[0] + track.xyxy[2]) * 0.5)
            self.assertEqual([track.track_id for track in left_to_right], [1, 2])

    def test_confirmed_track_survives_occlusion_buffer(self):
        tracker = ByteTrackTracker(TrackerConfig(confirmation_hits=2, max_missed=3))
        tracker.update([detection(200, 200, 260, 250)])
        original = tracker.update([detection(202, 200, 262, 250)])[0]
        for missed in range(1, 4):
            predicted = tracker.update([])
            self.assertEqual(len(predicted), 1)
            self.assertEqual(predicted[0].track_id, original.track_id)
            self.assertEqual(predicted[0].missed, missed)
        recovered = tracker.update([detection(210, 200, 270, 250)])
        self.assertEqual(len(recovered), 1)
        self.assertEqual(recovered[0].track_id, original.track_id)
        self.assertEqual(recovered[0].missed, 0)

    def test_track_is_removed_after_occlusion_limit(self):
        tracker = ByteTrackTracker(TrackerConfig(confirmation_hits=1, max_missed=2))
        self.assertEqual(tracker.update([detection(10, 10, 50, 50)])[0].track_id, 1)
        self.assertEqual(len(tracker.update([])), 1)
        self.assertEqual(len(tracker.update([])), 1)
        self.assertEqual(tracker.update([]), ())
        self.assertEqual(tracker.update([detection(10, 10, 50, 50)])[0].track_id, 2)

    def test_low_confidence_stage_updates_but_does_not_create(self):
        tracker = ByteTrackTracker(TrackerConfig(confirmation_hits=2))
        tracker.update([detection(100, 100, 150, 150, 0.9)])
        # A low-score continuation contributes to confirmation through stage two.
        tracks = tracker.update([detection(101, 100, 151, 150, 0.3)])
        self.assertEqual(len(tracks), 1)
        self.assertEqual(tracks[0].track_id, 1)
        empty_tracker = ByteTrackTracker(TrackerConfig(confirmation_hits=1))
        self.assertEqual(empty_tracker.update([detection(20, 20, 40, 40, 0.3)]), ())

    def test_nearest_confirmed_track_uses_box_centre(self):
        tracks = [
            Track(1, (0, 0, 40, 40), 0.9, 5, 0, True),
            Track(2, (80, 40, 100, 60), 0.8, 5, 0, True),
            Track(3, (98, 48, 102, 52), 0.99, 1, 0, False),
            Track(4, (99, 49, 101, 51), 0.99, 5, 1, True),
        ]
        self.assertEqual(nearest_confirmed_track(tracks, 200, 100).track_id, 2)
        self.assertEqual(nearest_confirmed_track(tracks, 200, 100,
                                                 visible_only=False).track_id, 4)
        self.assertIsNone(nearest_confirmed_track([], 200, 100))


class OutputParsingTests(unittest.TestCase):
    def setUp(self):
        self.meta = LetterboxMeta(640, 640, 1.0, 0.0, 0.0)

    def test_parses_1_5_n_and_applies_nms(self):
        # Two strongly overlapping boxes plus one low-confidence proposal.
        output = np.array([[[100.0, 102.0, 400.0],
                            [100.0, 102.0, 400.0],
                            [40.0, 40.0, 50.0],
                            [40.0, 40.0, 50.0],
                            [0.90, 0.80, 0.05]]], dtype=np.float32)
        parsed = parse_yolo11_output(output, self.meta, 0.1, 0.45)
        self.assertEqual(len(parsed), 1)
        self.assertAlmostEqual(parsed[0].confidence, 0.9, places=5)
        self.assertEqual(parsed[0].class_id, 0)
        self.assertEqual(parsed[0].xyxy, (80.0, 80.0, 120.0, 120.0))

    def test_parses_1_n_5_and_reverses_letterbox(self):
        # 1920x1080 -> 640 uses scale=1/3 and 140 px vertical padding.
        meta = LetterboxMeta(1920, 1080, 1.0 / 3.0, 0.0, 140.0)
        output = np.array([[[320.0, 320.0, 64.0, 40.0, 0.75],
                            [20.0, 20.0, -1.0, 10.0, 0.99]]], dtype=np.float32)
        parsed = parse_yolo11_output(output, meta, 0.1, 0.45)
        self.assertEqual(len(parsed), 1)
        x1, y1, x2, y2 = parsed[0].xyxy
        self.assertAlmostEqual(x1, 864.0)
        self.assertAlmostEqual(y1, 480.0)
        self.assertAlmostEqual(x2, 1056.0)
        self.assertAlmostEqual(y2, 600.0)

    def test_nms_keeps_separate_targets(self):
        candidates = [
            detection(0, 0, 100, 100, 0.8),
            detection(5, 5, 95, 95, 0.9),
            detection(300, 300, 350, 350, 0.7),
        ]
        kept = nms(candidates, 0.45)
        self.assertEqual([item.confidence for item in kept], [0.9, 0.7])

    def test_rejects_wrong_output_shape(self):
        with self.assertRaisesRegex(ValueError, "最后一维必须为 5"):
            parse_yolo11_output(np.zeros((1, 6, 20), dtype=np.float32), self.meta)

    @unittest.skipUnless(importlib.util.find_spec("cv2"), "需要 OpenCV")
    def test_overlay_is_red_in_rgb_order_and_labels_confirmed_only(self):
        frame = np.zeros((40, 60, 3), dtype=np.uint8)
        tracks = [
            Track(7, (5, 20, 30, 35), 0.9, 5, 0, True),
            Track(8, (35, 20, 55, 35), 0.9, 1, 0, False),
        ]
        overlay = draw_tracks_rgb(frame, tracks, line_width=1)
        self.assertEqual(tuple(overlay[20, 5]), (255, 0, 0))
        self.assertEqual(tuple(overlay[20, 35]), (0, 0, 0))
        self.assertFalse(np.shares_memory(frame, overlay))


class ConfigurationTests(unittest.TestCase):
    def test_detector_configuration_is_strict(self):
        with self.assertRaisesRegex(ValueError, "不存在"):
            DetectorConfig("missing.onnx")
        with tempfile.TemporaryDirectory() as directory:
            model = Path(directory) / "model.onnx"
            model.write_bytes(b"placeholder")
            config = DetectorConfig(model, confidence_threshold=0.5,
                                    low_confidence_threshold=0.1, opencv_threads=1)
            self.assertEqual(config.model_path, model)
            with self.assertRaisesRegex(ValueError, "必须小于"):
                DetectorConfig(model, confidence_threshold=0.5,
                               low_confidence_threshold=0.5)

    def test_tracker_configuration_is_strict(self):
        with self.assertRaisesRegex(ValueError, "confirmation_hits"):
            TrackerConfig(confirmation_hits=0)
        with self.assertRaisesRegex(ValueError, "必须小于"):
            TrackerConfig(high_confidence_threshold=0.2,
                          low_confidence_threshold=0.3)


if __name__ == "__main__":
    unittest.main()
