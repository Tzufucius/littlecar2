from pathlib import Path
import sys

import cv2


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision.yolo_detector import YoloDetector


IMAGE_PATH = PROJECT_ROOT / "assets" / "sim_train_00025.jpg"
MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "RGB_circle.pt"
OUTPUT_PATH = PROJECT_ROOT / "outputs" / "yolo_infer_example.jpg"

DEVICE = "cuda:0"
CONF_THRES = 0.5
IOU_THRES = 0.45


def main() -> None:
    detector = YoloDetector(
        model_path=MODEL_PATH,
        conf_thres=CONF_THRES,
        iou_thres=IOU_THRES,
        device=DEVICE,
    )

    frame = cv2.imread(str(IMAGE_PATH))
    if frame is None:
        raise SystemExit(f"无法读取图像: {IMAGE_PATH}")

    detections = detector.detect(frame)
    annotated = detector.draw_detections(frame, detections)

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(OUTPUT_PATH), annotated)

    print(f"检测数量: {len(detections)}")
    for det in detections:
        print(det)
    print(f"结果图已保存: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
