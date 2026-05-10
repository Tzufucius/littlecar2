from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraConfig:
    index: int = 0
    width: Optional[int] = None
    height: Optional[int] = None


class OpenCVCamera:
    def __init__(self, camera_options: CameraConfig) -> None:
        self.camera_options = camera_options
        self.capture: Optional[cv2.VideoCapture] = None

    def open(self) -> None:
        self.capture = cv2.VideoCapture(self.camera_options.index)
        if self.camera_options.width is not None:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_options.width)
        if self.camera_options.height is not None:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_options.height)
        if not self.capture.isOpened():
            raise RuntimeError(f"无法打开摄像头: {self.camera_options.index}")

    def read(self) -> np.ndarray:
        if self.capture is None:
            self.open()
        assert self.capture is not None
        ok, frame = self.capture.read()
        if not ok or frame is None:
            raise RuntimeError("摄像头读取失败")
        return frame

    def close(self) -> None:
        if self.capture is not None:
            self.capture.release()
            self.capture = None

    def __enter__(self) -> "OpenCVCamera":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
