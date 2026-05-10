import argparse
from pathlib import Path
from typing import Any, Dict

import yaml

from littlecar2.app.workflow import LittleCarWorkflow
from littlecar2.vision.qr_detector import QRChangeFilter, QRDetector
from littlecar2.vision.vision_service import VisionService
from littlecar2.vision.yolo_detector import YoloDetector


def load_config(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


def build_vision_service(config: Dict[str, Any], project_root: Path) -> VisionService:
    vision_config = config.get("vision", {})
    yolo_config = vision_config.get("yolo", {})
    model_path = project_root / yolo_config.get("model_path", "models/RGB_circle.pt")
    yolo_detector = YoloDetector(
        model_path=model_path,
        conf_thres=float(yolo_config.get("conf_thres", 0.5)),
        iou_thres=float(yolo_config.get("iou_thres", 0.45)),
        device=yolo_config.get("device"),
    )

    qr_detector = QRDetector()
    qr_filter = None
    if vision_config.get("qr", {}).get("change_filter", True):
        qr_filter = QRChangeFilter()

    return VisionService(
        qr_detector=qr_detector,
        yolo_detector=yolo_detector,
        qr_change_filter=qr_filter,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="littlecar2 Jetson 侧入口")
    parser.add_argument("--config", default="configs/default.yaml", help="配置文件路径")
    parser.add_argument("--once", help="处理一张图片并输出识别结果")
    args = parser.parse_args()

    project_root = Path.cwd()
    config = load_config(project_root / args.config)
    workflow = LittleCarWorkflow(build_vision_service(config, project_root))

    if args.once:
        event = workflow.process_once(project_root / args.once)
        print(event)
        return

    parser.print_help()
