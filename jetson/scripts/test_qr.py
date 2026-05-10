from pathlib import Path

import cv2

from vision.qr_detector import QRDetector

IMAGE_PATH = Path("assets/同色.png")


def main() -> None:
    frame = cv2.imread(str(IMAGE_PATH))
    if frame is None:
        raise SystemExit(f"无法读取图像: {IMAGE_PATH}")

    result = QRDetector().detect_raw(frame)
    if result is None:
        print("未识别到二维码")
        return

    print("识别成功")
    print("内容:", result.data)
    print("中心点:", result.cx, result.cy)
    print("角点:", result.points)


if __name__ == "__main__":
    main()
