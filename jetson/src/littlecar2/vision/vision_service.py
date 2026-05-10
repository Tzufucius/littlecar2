from pathlib import Path
from typing import Optional, Union

import cv2
import numpy as np

from littlecar2.vision.qr_detector import QRChangeFilter, QRDetector
from littlecar2.vision.types import VisionResult
from littlecar2.vision.yolo_detector import YoloDetector


class VisionService:
    def __init__(
        self,
        qr_detector: Optional[QRDetector] = None,
        yolo_detector: Optional[YoloDetector] = None,
        qr_change_filter: Optional[QRChangeFilter] = None,
    ) -> None:
        self.qr_detector = qr_detector
        self.yolo_detector = yolo_detector
        self.qr_change_filter = qr_change_filter

    def process_image(self, image: Union[str, Path, np.ndarray]) -> VisionResult:
        frame = self._load_image(image)
        qr_result = None
        if self.qr_detector is not None:
            qr_result = self.qr_detector.detect_raw(frame)
            if self.qr_change_filter is not None:
                qr_result = self.qr_change_filter.update(qr_result)

        detections = []
        if self.yolo_detector is not None:
            detections = self.yolo_detector.detect(frame)

        return VisionResult(qr=qr_result, detections=detections)

    @staticmethod
    def _load_image(image: Union[str, Path, np.ndarray]) -> np.ndarray:
        if isinstance(image, np.ndarray):
            return image
        frame = cv2.imread(str(image))
        if frame is None:
            raise ValueError(f"无法读取图像: {image}")
        return frame
