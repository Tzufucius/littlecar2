import time
from typing import Optional

import cv2
import numpy as np

from littlecar2.vision.types import QRResult


class QRDetector:
    """只负责二维码识别，不负责去重和任务状态判断。"""

    def __init__(self) -> None:
        self.detector = cv2.QRCodeDetector()

    def detect_raw(self, frame: np.ndarray) -> Optional[QRResult]:
        data, points, _ = self.detector.detectAndDecode(frame)
        if not data or points is None:
            return None

        points = points[0].astype(int)
        point_list = [(int(x), int(y)) for x, y in points]
        cx = int(np.mean(points[:, 0]))
        cy = int(np.mean(points[:, 1]))
        return QRResult(
            data=data,
            points=point_list,
            cx=cx,
            cy=cy,
            timestamp=time.time(),
        )

    @staticmethod
    def draw_result(frame: np.ndarray, result: QRResult) -> np.ndarray:
        img = frame.copy()
        points = result.points
        for index in range(4):
            pt1 = points[index]
            pt2 = points[(index + 1) % 4]
            cv2.line(img, pt1, pt2, (0, 255, 0), 2)

        cv2.circle(img, (result.cx, result.cy), 5, (0, 0, 255), -1)
        cv2.putText(
            img,
            result.data,
            (result.cx, result.cy - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )
        return img


class QRChangeFilter:
    """二维码内容变化过滤器。"""

    def __init__(self) -> None:
        self.last_data: Optional[str] = None
        self.last_result: Optional[QRResult] = None

    def update(self, result: Optional[QRResult]) -> Optional[QRResult]:
        if result is None:
            return None

        if self.last_data is None or result.data != self.last_data:
            self.last_data = result.data
            self.last_result = result
            return result

        return None

    def reset(self) -> None:
        self.last_data = None
        self.last_result = None
