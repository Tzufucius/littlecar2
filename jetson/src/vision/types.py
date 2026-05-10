from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class Detection:
    class_id: int
    class_name: str
    confidence: float
    x1: int
    y1: int
    x2: int
    y2: int
    cx: int
    cy: int
    width: int
    height: int


@dataclass(frozen=True)
class QRResult:
    data: str
    points: List[Tuple[int, int]]
    cx: int
    cy: int
    timestamp: float


@dataclass(frozen=True)
class VisionResult:
    qr: Optional[QRResult]
    detections: List[Detection]
