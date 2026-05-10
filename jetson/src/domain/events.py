from dataclasses import dataclass
from typing import Optional

from vision.types import Detection, QRResult


@dataclass(frozen=True)
class VisionEvent:
    qr: Optional[QRResult]
    best_detection: Optional[Detection]
