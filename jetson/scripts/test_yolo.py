import argparse
from pathlib import Path

from littlecar2.vision.yolo_detector import YoloDetector


def main() -> None:
    parser = argparse.ArgumentParser(description="测试 YOLO 推理")
    parser.add_argument("--image", default="assets/sim_train_00025.jpg", help="测试图片路径")
    parser.add_argument("--model", default="models/RGB_circle.pt", help="模型路径")
    parser.add_argument("--device", default=None, help="推理设备，例如 cuda:0 或 cpu")
    parser.add_argument("--conf", type=float, default=0.5, help="置信度阈值")
    args = parser.parse_args()

    detector = YoloDetector(
        model_path=Path(args.model),
        conf_thres=args.conf,
        device=args.device,
    )
    detections = detector.detect(Path(args.image))
    for det in detections:
        print(det)


if __name__ == "__main__":
    main()
