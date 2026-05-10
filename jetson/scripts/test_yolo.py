from pathlib import Path

from vision.yolo_detector import YoloDetector

IMAGE_PATH = Path("assets/sim_train_00025.jpg")
MODEL_PATH = Path("models/RGB_circle.pt")
DEVICE = "cuda:0"
CONF_THRES = 0.5


def main() -> None:
    detector = YoloDetector(
        model_path=MODEL_PATH,
        conf_thres=CONF_THRES,
        device=DEVICE,
    )
    detections = detector.detect(IMAGE_PATH)
    for det in detections:
        print(det)


if __name__ == "__main__":
    main()
