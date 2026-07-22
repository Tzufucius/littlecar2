"""可选的 Ultralytics YOLO 推理函数。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np


def load_yolo_model(model_path: str | Path):
    """仅在显式调用时加载并返回 Ultralytics YOLO 模型。"""
    from ultralytics import YOLO

    return YOLO(str(model_path))


def detect_yolo(
    frame_bgr: np.ndarray,
    model: Any,
    conf_thres: float = 0.5,
    iou_thres: float = 0.45,
    device: str | None = None,
) -> list[dict[str, Any]]:
    """使用已经加载的 YOLO 模型处理一帧 BGR 图像。"""
    if not isinstance(frame_bgr, np.ndarray) or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
        raise TypeError("frame_bgr must be a BGR numpy.ndarray")
    if not 0.0 <= conf_thres <= 1.0 or not 0.0 <= iou_thres <= 1.0:
        raise ValueError("conf_thres and iou_thres must be in [0, 1]")

    result = model.predict(source=frame_bgr, conf=conf_thres, iou=iou_thres, device=device, verbose=False)[0]
    if result.boxes is None:
        return []

    names = model.names
    detections: list[dict[str, Any]] = []
    for box in result.boxes:
        class_id = int(box.cls[0])
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        detections.append(
            {
                "class_id": class_id,
                "class_name": names.get(class_id, str(class_id)),
                "confidence": float(box.conf[0]),
                "x1": int(x1),
                "y1": int(y1),
                "x2": int(x2),
                "y2": int(y2),
                "center_x": int((x1 + x2) / 2),
                "center_y": int((y1 + y2) / 2),
            }
        )
    return detections
