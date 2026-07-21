import math
import os
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from vision.circle_dector import CircleDetector


SAMPLE_DIR = ROOT / "scripts" / "圆形检测" / "v2" / "circle"
SAMPLE_IMAGE = SAMPLE_DIR / "(108, 71).jpg"


class CircleDetectorTest(unittest.TestCase):
    def setUp(self):
        self.detector = CircleDetector()

    def test_detects_sample_with_subpixel_center(self):
        result = self.detector.detect(image_path=str(SAMPLE_IMAGE))

        self.assertTrue(result.found)
        self.assertIsNotNone(result.center)
        self.assertIsInstance(result.center[0], float)
        self.assertLess(result.error_px, 1.0)
        self.assertIsNone(result.time_ms)

    def test_perspective_correction_improves_known_sample(self):
        result, debug = self.detector.detect(image_path=str(SAMPLE_IMAGE), return_debug=True)
        truth = self.detector.parse_center_from_filename(str(SAMPLE_IMAGE))

        self.assertTrue(result.found)
        self.assertGreaterEqual(len(debug["rings"]), 2)
        self.assertLessEqual(math.dist(result.center, truth), math.dist(debug["rough_center"], truth))

    def test_blank_frame_is_not_detected(self):
        frame = np.full((360, 640, 3), 255, dtype=np.uint8)

        result = self.detector.detect(frame=frame)

        self.assertFalse(result.found)
        self.assertIsNone(result.center)
        self.assertEqual(self.detector.detect_center(frame=frame), None)

    @unittest.skipUnless(
        os.getenv("CIRCLE_DEBUG_OUTPUT_DIR"),
        "设置 CIRCLE_DEBUG_OUTPUT_DIR 后导出可视化结果",
    )
    def test_exports_visual_debug_images(self):
        result, debug = self.detector.detect(image_path=str(SAMPLE_IMAGE), return_debug=True)
        output_dir = Path(os.environ["CIRCLE_DEBUG_OUTPUT_DIR"])
        output_dir.mkdir(parents=True, exist_ok=True)
        source = self.detector._read_image(str(SAMPLE_IMAGE))
        truth = self.detector.parse_center_from_filename(str(SAMPLE_IMAGE))
        overlay = self.detector.draw_debug_result(source, result, debug, gt=truth)
        threshold = cv2.cvtColor(debug["threshold_img"], cv2.COLOR_GRAY2BGR)

        self.assertTrue(cv2.imwrite(str(output_dir / "circle_overlay.png"), overlay))
        self.assertTrue(cv2.imwrite(str(output_dir / "circle_threshold.png"), threshold))


if __name__ == "__main__":
    unittest.main()
