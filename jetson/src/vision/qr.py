"""二维码识别函数。"""

from __future__ import annotations

import time
from typing import Any

import cv2
import numpy as np


_DETECTOR = cv2.QRCodeDetector()


def detect_qr(frame_bgr: np.ndarray) -> dict[str, Any] | None:
    """从 BGR 图像中识别一个二维码；未识别到时返回 ``None``。"""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise TypeError("frame_bgr must be a BGR numpy.ndarray")

    data, points, _ = _DETECTOR.detectAndDecode(frame_bgr)
    if not data or points is None:
        return None

    corners = points[0].astype(int)
    return {
        "data": data,
        "points": [(int(x), int(y)) for x, y in corners],
        "center_x": int(np.mean(corners[:, 0])),
        "center_y": int(np.mean(corners[:, 1])),
        "timestamp": time.time(),
    }
