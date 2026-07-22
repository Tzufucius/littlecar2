"""传统彩色物料检测函数。"""

from __future__ import annotations

from typing import Any

import cv2
import numpy as np


_COLOR_NAMES = {
    0: "Abnormal/Defective",
    1: "Red",
    2: "Yellow",
    3: "Blue",
    4: "Green",
    5: "Black",
    6: "LightBlue",
}


def classify_material_color(roi_bgr: np.ndarray) -> tuple[int | None, str]:
    """使用传统 HSV 阈值规则识别物料 ROI 的颜色。"""
    if roi_bgr is None or roi_bgr.size == 0:
        return 0, _COLOR_NAMES[0]

    median_bgr = np.median(roi_bgr.reshape(-1, 3), axis=0).astype(np.uint8)
    hue, saturation, value = cv2.cvtColor(median_bgr.reshape(1, 1, 3), cv2.COLOR_BGR2HSV)[0, 0]

    if saturation < 25 and value > 200:
        return None, "Background"
    if value < 70:
        return 5, _COLOR_NAMES[5]
    if saturation < 45:
        return 0, _COLOR_NAMES[0]
    if hue < 8 or hue >= 165:
        return 1, _COLOR_NAMES[1]
    if hue < 34:
        return 2, _COLOR_NAMES[2]
    if hue < 85:
        return 4, _COLOR_NAMES[4]
    if hue < 103:
        return 6, _COLOR_NAMES[6]
    if hue < 140:
        return 3, _COLOR_NAMES[3]
    return 0, _COLOR_NAMES[0]


def detect_colored_materials(frame_bgr: np.ndarray) -> list[dict[str, Any]]:
    """在 BGR 图像中检测最多三个圆形彩色物料。"""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise TypeError("frame_bgr must be a BGR numpy.ndarray")

    height, width = frame_bgr.shape[:2]
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    denoised = cv2.medianBlur(gray, 7)
    blurred = cv2.GaussianBlur(denoised, (9, 9), 2)
    image_radius = 0.5 * (height + width)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=85,
        param1=50,
        param2=32,
        minRadius=int(image_radius * 0.05),
        maxRadius=int(image_radius * 0.35),
    )
    if circles is None:
        return []

    detections: list[dict[str, Any]] = []
    for center_x, center_y, radius in np.round(circles[0]).astype(int):
        center_x = int(np.clip(center_x, 0, width - 1))
        center_y = int(np.clip(center_y, 0, height - 1))
        radius = int(radius)
        roi_radius = max(5, int(radius * 0.6))
        roi = frame_bgr[
            max(0, center_y - roi_radius):min(height, center_y + roi_radius),
            max(0, center_x - roi_radius):min(width, center_x + roi_radius),
        ]
        color_id, color_name = classify_material_color(roi)
        if color_id is None:
            continue
        detections.append(
            {
                "center_x": center_x,
                "center_y": center_y,
                "radius": radius,
                "color_id": color_id,
                "color_name": color_name,
            }
        )

    return sorted(detections, key=lambda item: item["radius"], reverse=True)[:3]
