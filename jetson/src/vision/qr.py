"""二维码任务码识别函数。"""

from __future__ import annotations

import cv2
import numpy as np


_DETECTOR = cv2.QRCodeDetector()


def detect_qr(frame_bgr: np.ndarray) -> str | None:
    """从 BGR 图像中识别二维码并直接返回任务码。"""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise TypeError("frame_bgr must be a BGR numpy.ndarray")

    data, points, _ = _DETECTOR.detectAndDecode(frame_bgr)
    if not data or points is None:
        return None

    return data
