import math
import sys
import unittest
from pathlib import Path

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


if __name__ == "__main__":
    unittest.main()
