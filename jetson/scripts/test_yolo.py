from pathlib import Path
import sys
import numpy as np
import cv2

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision import load_yolo_model, detect_yolo


IMAGE_PATH = PROJECT_ROOT / "assets" / "sim_train_00025.jpg"
MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "RGB_circle.pt"
DEVICE = "cuda:0"
CONF_THRES = 0.5


def main() -> None:
    detector = load_yolo_model(model_path=MODEL_PATH)
    np_img = cv2.imread(str(IMAGE_PATH))
    detections = detect_yolo(np_img, detector, device=DEVICE, conf_thres=CONF_THRES)
    for det in detections:
        print(det)


if __name__ == "__main__":
    main()
