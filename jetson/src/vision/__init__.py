"""littlecar2 的直接调用视觉算法。"""

from .marker import detect_numbered_marker
from .materials import detect_colored_materials
from .qr import detect_qr
from .yolo import detect_yolo, load_yolo_model

__all__ = [
    "detect_colored_materials",
    "detect_numbered_marker",
    "detect_qr",
    "detect_yolo",
    "load_yolo_model",
]
