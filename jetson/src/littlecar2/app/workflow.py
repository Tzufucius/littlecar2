from pathlib import Path
from typing import Optional, Union

from littlecar2.domain.events import VisionEvent
from littlecar2.vision.vision_service import VisionService


class LittleCarWorkflow:
    def __init__(self, vision_service: VisionService) -> None:
        self.vision_service = vision_service

    def process_once(self, image: Union[str, Path]) -> VisionEvent:
        result = self.vision_service.process_image(image)
        best_detection = None
        if result.detections:
            best_detection = max(result.detections, key=lambda det: det.confidence)
        return VisionEvent(qr=result.qr, best_detection=best_detection)
