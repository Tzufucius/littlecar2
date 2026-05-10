import time
from pathlib import Path
from typing import Any, Dict, Optional, Union

import yaml

from comm.interface import Stm32Client
from comm.mock_client import MockStm32Client
from domain.events import VisionEvent
from vision.camera import CameraConfig, OpenCVCamera
from vision.qr_detector import QRChangeFilter, QRDetector
from vision.vision_service import VisionService


def load_config(path: Union[str, Path]) -> Dict[str, Any]:
    config_path = Path(path)
    with config_path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def build_vision_service(config: Dict[str, Any], project_root: Path) -> VisionService:
    from vision.yolo_detector import YoloDetector

    vision_config = config.get("vision", {})
    yolo_config = vision_config.get("yolo", {})
    model_path = project_root / yolo_config.get("model_path", "models/RGB_circle.pt")
    yolo_detector = YoloDetector(
        model_path=model_path,
        conf_thres=float(yolo_config.get("conf_thres", 0.5)),
        iou_thres=float(yolo_config.get("iou_thres", 0.45)),
        device=yolo_config.get("device"),
    )

    qr_filter = None
    if vision_config.get("qr", {}).get("change_filter", True):
        qr_filter = QRChangeFilter()

    return VisionService(
        qr_detector=QRDetector(),
        yolo_detector=yolo_detector,
        qr_change_filter=qr_filter,
    )


def build_camera(config: Dict[str, Any]) -> OpenCVCamera:
    camera_config = config.get("vision", {}).get("camera", {})
    return OpenCVCamera(
        CameraConfig(
            index=int(camera_config.get("index", 0)),
            width=camera_config.get("width"),
            height=camera_config.get("height"),
        )
    )


def build_stm32_client(config: Dict[str, Any], use_mock: bool = False) -> Stm32Client:
    if use_mock:
        return MockStm32Client()

    from comm.serial_client import SerialConfig, SerialStm32Client

    serial_config = config.get("comm", {}).get("serial", {})
    return SerialStm32Client(
        SerialConfig(
            port=serial_config.get("port", "/dev/ttyTHS1"),
            baudrate=int(serial_config.get("baudrate", 115200)),
            timeout=float(serial_config.get("timeout", 0.1)),
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
    config: Dict[str, Any],
    project_root: Path,
    use_mock_comm: bool = False,
) -> RobotMain:
    app_config = config.get("app", {})
    return RobotMain(
        vision_service=build_vision_service(config, project_root),
        stm32_client=build_stm32_client(config, use_mock=use_mock_comm),
        loop_interval_s=float(app_config.get("loop_interval_s", 0.05)),
    )
