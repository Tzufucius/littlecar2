"""YOLO material detection and disk center estimation."""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

from .yolo import detect_yolo


def detect_colored_materials(
    frame_bgr: np.ndarray,
    model: Any,
    conf_thres: float = 0.5,
    iou_thres: float = 0.45,
    device: str | None = None,
) -> list[dict[str, Any]]:
    """Detect materials and empty slots with an already loaded YOLO model."""
    return detect_yolo(
        frame_bgr,
        model,
        conf_thres=conf_thres,
        iou_thres=iou_thres,
        device=device,
    )


def estimate_disk_center(
    detections: Sequence[dict[str, Any]],
    frame_shape: Sequence[int],
) -> dict[str, Any]:
    """Estimate the material disk center and report the fallback strategy."""
    if len(frame_shape) < 2 or frame_shape[0] <= 0 or frame_shape[1] <= 0:
        raise ValueError("frame_shape must contain positive height and width")

    img_h, img_w = int(frame_shape[0]), int(frame_shape[1])
    colored = sorted(
        (det for det in detections if det.get("class_id") != 6),
        key=lambda det: det.get("confidence", 0.0),
        reverse=True,
    )
    empty = sorted(
        (det for det in detections if det.get("class_id") == 6),
        key=lambda det: det.get("confidence", 0.0),
        reverse=True,
    )
    selected = (colored[:3] + empty)[:3]
    points = [(int(det["center_x"]), int(det["center_y"])) for det in selected]

    if len(points) == 3:
        center = np.mean(points, axis=0)
        method = "three_point_mean"
    elif len(points) == 2:
        p1, p2 = np.asarray(points, dtype=float)
        distance = float(np.linalg.norm(p2 - p1))
        midpoint = (p1 + p2) / 2.0
        height = distance / (2.0 * np.sqrt(3.0))
        normal = np.array([-(p2 - p1)[1], (p2 - p1)[0]]) / (distance or 1.0)
        image_center = np.array([img_w / 2.0, img_h / 2.0])
        candidate_a = midpoint + height * normal
        candidate_b = midpoint - height * normal
        center = min((candidate_a, candidate_b), key=lambda item: np.linalg.norm(item - image_center))
        method = "two_point_geometry"
    elif len(points) == 1:
        center = np.asarray(points[0], dtype=float)
        method = "single_point_fallback"
    else:
        center = np.array([img_w / 2.0, img_h / 2.0])
        method = "image_center_fallback"

    return {
        "center_x": int(round(center[0])),
        "center_y": int(round(center[1])),
        "method": method,
        "support_points": points,
    }
