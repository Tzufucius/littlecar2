import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from vision import detect_colored_materials, detect_numbered_marker, detect_qr
from vision.yolo import detect_yolo


def read_image(path: Path) -> np.ndarray:
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise AssertionError(f"无法读取测试图片: {path}")
    return image


class VisionTest(unittest.TestCase):
    def test_numbered_marker_returns_stable_result_fields(self):
        image = read_image(next((ROOT / "assets" / "circle_with_number").glob("*.jpg")))

        result = detect_numbered_marker(image)

        self.assertIsNotNone(result)
        assert result is not None
        self.assertIn(result["digit"], {"1", "2", "3"})
        self.assertGreaterEqual(result["ring_count"], 2)
        self.assertGreater(result["confidence"], 0.0)
        self.assertGreaterEqual(result["scale_factor"], 0.0)

    def test_blank_frame_has_no_marker_or_material(self):
        frame = np.full((360, 640, 3), 255, dtype=np.uint8)

        self.assertIsNone(detect_numbered_marker(frame))
        self.assertEqual(detect_colored_materials(frame), [])

    def test_colored_material_result_shape(self):
        image = read_image(next((ROOT / "assets" / "彩色物料").glob("*.jpg")))

        results = detect_colored_materials(image)

        self.assertLessEqual(len(results), 3)
        for result in results:
            self.assertIn(result["color_id"], range(7))
            self.assertIn("center_x", result)
            self.assertIn("center_y", result)
            self.assertIn("radius", result)

    def test_qr_without_code_returns_none(self):
        frame = np.full((360, 640, 3), 127, dtype=np.uint8)

        self.assertIsNone(detect_qr(frame))

    def test_yolo_validates_arguments_without_loading_model(self):
        frame = np.zeros((10, 10, 3), dtype=np.uint8)

        with self.assertRaises(ValueError):
            detect_yolo(frame, object(), conf_thres=1.1)


if __name__ == "__main__":
    unittest.main()
