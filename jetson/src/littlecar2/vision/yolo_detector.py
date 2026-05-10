from pathlib import Path
from typing import List, Optional, Union

import cv2
import numpy as np
from ultralytics import YOLO

from littlecar2.vision.types import Detection


class YoloDetector:
    def __init__(
        self,
        model_path: Union[str, Path],
        conf_thres: float = 0.5,
        iou_thres: float = 0.45,
        device: Optional[str] = None,
    ) -> None:
        self.model = YOLO(str(model_path))
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.device = device
        self.names = self.model.names

    def detect(
        self,
        image: Union[str, Path, np.ndarray],
        target_classes: Optional[List[int]] = None,
    ) -> List[Detection]:
        img = self._load_image(image)
        results = self.model.predict(
            source=img,
            conf=self.conf_thres,
            iou=self.iou_thres,
            device=self.device,
            verbose=False,
        )

        detections: List[Detection] = []
        result = results[0]
        if result.boxes is None:
            return detections

        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            if target_classes is not None and class_id not in target_classes:
                continue

            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            class_name = self.names.get(class_id, str(class_id))
            detections.append(
                Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidence,
                    x1=int(x1),
                    y1=int(y1),
                    x2=int(x2),
                    y2=int(y2),
                    cx=cx,
                    cy=cy,
                    width=int(x2 - x1),
                    height=int(y2 - y1),
                )
            )
        return detections

    def get_best_detection(
        self,
        image: Union[str, Path, np.ndarray],
        target_classes: Optional[List[int]] = None,
    ) -> Optional[Detection]:
        detections = self.detect(image, target_classes)
        if not detections:
            return None
        return max(detections, key=lambda det: det.confidence)

    def get_detections_threshold(
        self,
        image: Union[str, Path, np.ndarray],
        target_classes: Optional[List[int]] = None,
        threshold: float = 0.5,
    ) -> List[Detection]:
        detections = self.detect(image, target_classes)
        return [det for det in detections if det.confidence >= threshold]

    def draw_detections(
        self,
        image: np.ndarray,
        detections: List[Detection],
    ) -> np.ndarray:
        img = image.copy()
        for det in detections:
            cv2.rectangle(img, (det.x1, det.y1), (det.x2, det.y2), (0, 255, 0), 2)
            label = f"{det.class_name} {det.confidence:.2f}"
            cv2.putText(
                img,
                label,
                (det.x1, det.y1 - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.circle(img, (det.cx, det.cy), 4, (0, 0, 255), -1)
        return img

    @staticmethod
    def _load_image(image: Union[str, Path, np.ndarray]) -> np.ndarray:
        if isinstance(image, np.ndarray):
            return image

        img = cv2.imread(str(image))
        if img is None:
            raise ValueError(f"无法读取图像: {image}")
        return img
