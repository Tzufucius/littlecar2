import time
from pathlib import Path
from typing import Optional, Union

from domain.events import VisionEvent
from vision.camera import CameraConfig, OpenCVCamera
from vision.qr_detector import QRChangeFilter, QRDetector
from vision.vision_service import VisionService
from vision.yolo_detector import YoloDetector


PROJECT_ROOT = Path(__file__).resolve().parents[2]

YOLO_MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "RGB_circle.pt"
YOLO_CONF_THRES = 0.5
YOLO_IOU_THRES = 0.45
YOLO_DEVICE = "cuda:0"

QR_CHANGE_FILTER = True

CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

LOOP_INTERVAL_S = 0.05


def build_vision_service() -> VisionService:
    qr_filter = QRChangeFilter() if QR_CHANGE_FILTER else None
    return VisionService(
        qr_detector=QRDetector(),
        yolo_detector=YoloDetector(
            model_path=YOLO_MODEL_PATH,
            conf_thres=YOLO_CONF_THRES,
            iou_thres=YOLO_IOU_THRES,
            device=YOLO_DEVICE,
        ),
        qr_change_filter=qr_filter,
    )


def build_camera() -> OpenCVCamera:
    return OpenCVCamera(
        CameraConfig(
            index=CAMERA_INDEX,
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
        )
    )


class RobotMain:
    """Jetson 侧视觉主流程。

    当前只组织二维码识别、YOLO 推理和结果打印，不包含通信协议。
    """

    def __init__(
        self,
        vision_service: VisionService,
        loop_interval_s: float = LOOP_INTERVAL_S,
    ) -> None:
        self.vision_service = vision_service
        self.loop_interval_s = loop_interval_s

    def run_once_image(self, image: Union[str, Path]) -> VisionEvent:
        event = self._process_frame(image)
        self.print_event(event)
        return event

    def run_camera_loop(
        self,
        camera: OpenCVCamera,
        max_frames: Optional[int] = None,
    ) -> None:
        frame_count = 0
        with camera:
            while True:
                event = self._process_frame(camera.read())
                self.print_event(event)

                frame_count += 1
                if max_frames is not None and frame_count >= max_frames:
                    break
                time.sleep(self.loop_interval_s)

    def _process_frame(self, image) -> VisionEvent:
        result = self.vision_service.process_image(image)
        best_detection = None
        if result.detections:
            best_detection = max(result.detections, key=lambda det: det.confidence)
        return VisionEvent(qr=result.qr, best_detection=best_detection)

    @staticmethod
    def print_event(event: VisionEvent) -> None:
        if event.qr is not None:
            print(f"QR data={event.qr.data} center=({event.qr.cx},{event.qr.cy})")

        if event.best_detection is not None:
            det = event.best_detection
            print(
                "YOLO "
                f"class={det.class_name} conf={det.confidence:.2f} "
                f"center=({det.cx},{det.cy})"
            )

        if event.qr is None and event.best_detection is None:
            print("未识别到二维码或 YOLO 目标。")


def build_robot_main() -> RobotMain:
    return RobotMain(
        vision_service=build_vision_service(),
        loop_interval_s=LOOP_INTERVAL_S,
    )
