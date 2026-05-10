import argparse
from pathlib import Path

import cv2

from littlecar2.vision.qr_detector import QRDetector


def main() -> None:
    parser = argparse.ArgumentParser(description="测试二维码识别")
    parser.add_argument("--image", default="assets/同色.png", help="测试图片路径")
    args = parser.parse_args()

    image_path = Path(args.image)
    frame = cv2.imread(str(image_path))
    if frame is None:
        raise SystemExit(f"无法读取图像: {image_path}")

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
