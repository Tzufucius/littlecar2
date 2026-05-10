import time
from pathlib import Path
from typing import Optional, Union

from comm.interface import Stm32Client
from comm.mock_client import MockStm32Client
from domain.events import VisionEvent
from vision.camera import CameraConfig, OpenCVCamera
from vision.qr_detector import QRChangeFilter, QRDetector
from vision.vision_service import VisionService
from vision.yolo_detector import YoloDetector


PROJECT_ROOT = Path(__file__).resolve().parents[2]

YOLO_MODEL_PATH = PROJECT_ROOT / "models" / "RGB_circle.pt"
YOLO_CONF_THRES = 0.5
YOLO_IOU_THRES = 0.45
YOLO_DEVICE = "cuda:0"

QR_CHANGE_FILTER = True

CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

SERIAL_PORT = "/dev/ttyTHS1"
SERIAL_BAUDRATE = 115200
SERIAL_TIMEOUT = 0.1

LOOP_INTERVAL_S = 0.05


def build_vision_service() -> VisionService:
    qr_filter = None
    if QR_CHANGE_FILTER:
        qr_filter = QRChangeFilter()

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


def build_stm32_client(use_mock: bool = False) -> Stm32Client:
    if use_mock:
        return MockStm32Client()

    from comm.serial_client import SerialConfig, SerialStm32Client

    return SerialStm32Client(
        SerialConfig(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUDRATE,
            timeout=SERIAL_TIMEOUT,
        )
    )


class RobotMain:
    """整车主流程编排入口。

    本类只负责任务顺序和模块调用，不在这里写 YOLO、二维码、串口协议细节。
    """

    def __init__(
        self,
        vision_service: VisionService,
        stm32_client: Optional[Stm32Client] = None,
        loop_interval_s: float = 0.05,
    ) -> None:
        self.vision_service = vision_service
        self.stm32_client = stm32_client
        self.loop_interval_s = loop_interval_s

    def run_once_image(self, image: Union[str, Path]) -> VisionEvent:
        result = self.vision_service.process_image(image)
        event = VisionEvent(
            qr=result.qr,
            best_detection=max(result.detections, key=lambda det: det.confidence)
            if result.detections
            else None,
        )
        self.handle_event(event)
        return event

    def run_camera_loop(
        self,
        camera: OpenCVCamera,
        max_frames: Optional[int] = None,
    ) -> None:
        frame_count = 0
        if self.stm32_client is not None:
            self.stm32_client.open()

        try:
            with camera:
                while True:
                    frame = camera.read()
                    result = self.vision_service.process_image(frame)
                    event = VisionEvent(
                        qr=result.qr,
                        best_detection=max(result.detections, key=lambda det: det.confidence)
                        if result.detections
                        else None,
                    )
                    self.handle_event(event)

                    frame_count += 1
                    if max_frames is not None and frame_count >= max_frames:
                        break
                    time.sleep(self.loop_interval_s)
        finally:
            if self.stm32_client is not None:
                self.stm32_client.close()

    def handle_event(self, event: VisionEvent) -> None:
        if event.qr is not None:
            print(f"QR data={event.qr.data} center=({event.qr.cx},{event.qr.cy})")

        if event.best_detection is not None:
            det = event.best_detection
            print(
                "YOLO "
                f"class={det.class_name} conf={det.confidence:.2f} "
                f"center=({det.cx},{det.cy})"
            )

        # 后续在这里把识别结果转换成 STM32 协议帧并调用 self.stm32_client.send(...)。


def build_robot_main(
    use_mock_comm: bool = False,
) -> RobotMain:
    return RobotMain(
        vision_service=build_vision_service(),
        stm32_client=build_stm32_client(use_mock=use_mock_comm),
        loop_interval_s=LOOP_INTERVAL_S,
    )
