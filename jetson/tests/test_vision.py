import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from vision import (
    detect_colored_materials,
    detect_numbered_marker,
    detect_qr,
    estimate_disk_center,
    load_yolo_model,
)
from vision.yolo import detect_yolo


def read_image(path: Path) -> np.ndarray:
    image = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise AssertionError(f"无法读取测试图片: {path}")
    return image


class VisionTest(unittest.TestCase):
    @staticmethod
    def empty_model():
        return SimpleNamespace(predict=lambda **kwargs: [SimpleNamespace(boxes=None)])

    @staticmethod
    def detection(class_id, center_x, center_y, confidence=0.9):
        return {
            "class_id": class_id,
            "center_x": center_x,
            "center_y": center_y,
            "confidence": confidence,
        }

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
        self.assertEqual(detect_colored_materials(frame, self.empty_model()), [])

    def test_colored_material_result_shape(self):
        image = read_image(next((ROOT / "assets" / "物料盘").glob("*.jpg")))
        model = load_yolo_model(ROOT / "assets" / "models" / "6color-circle-v3.pt")

        results = detect_colored_materials(image, model)

        self.assertGreater(len(results), 0)
        for result in results:
            self.assertIn(result["class_id"], range(7))
            self.assertIn("class_name", result)
            self.assertIn("center_x", result)
            self.assertIn("center_y", result)

    def test_disk_center_uses_three_points(self):
        detections = [
            self.detection(0, 100, 100),
            self.detection(1, 200, 100),
            self.detection(6, 150, 200),
        ]

        result = estimate_disk_center(detections, (400, 640, 3))

        self.assertEqual((result["center_x"], result["center_y"]), (150, 133))
        self.assertEqual(result["method"], "three_point_mean")

    def test_disk_center_uses_two_point_geometry(self):
        result = estimate_disk_center(
            [self.detection(0, 200, 200), self.detection(1, 400, 200)],
            (400, 640, 3),
        )

        self.assertEqual(result["method"], "two_point_geometry")
        self.assertEqual((result["center_x"], result["center_y"]), (300, 142))

    def test_disk_center_falls_back_for_one_or_zero_points(self):
        single = estimate_disk_center([self.detection(6, 123, 234)], (400, 640, 3))
        empty = estimate_disk_center([], (400, 640, 3))

        self.assertEqual((single["center_x"], single["center_y"]), (123, 234))
        self.assertEqual(single["method"], "single_point_fallback")
        self.assertEqual((empty["center_x"], empty["center_y"]), (320, 200))
        self.assertEqual(empty["method"], "image_center_fallback")

    def test_qr_without_code_returns_none(self):
        frame = np.full((360, 640, 3), 127, dtype=np.uint8)

        self.assertIsNone(detect_qr(frame))

    def test_qr_returns_task_code(self):
        image = read_image(ROOT / "assets" / "二维码" / "145+634+312+132.png")

        self.assertEqual(detect_qr(image), "145+634+312+132")

    def test_yolo_validates_arguments_without_loading_model(self):
        frame = np.zeros((10, 10, 3), dtype=np.uint8)

        with self.assertRaises(ValueError):
            detect_yolo(frame, object(), conf_thres=1.1)


if __name__ == "__main__":
    unittest.main()
