from pathlib import Path
import sys

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision.qr_detector import QRDetector


IMAGE_PATH = PROJECT_ROOT / "assets" / "同色.png"


def read_image(path: Path):
    data = np.fromfile(str(path), dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def main() -> None:
    frame = read_image(IMAGE_PATH)
    if frame is None:
        raise SystemExit(f"无法读取图像: {IMAGE_PATH}")

    result = QRDetector().detect_raw(frame)
    if result is None:
        print("未识别到二维码。")
        return

    print("识别成功")
    print("内容:", result.data)
    print("中心点:", result.cx, result.cy)
    print("角点:", result.points)


if __name__ == "__main__":
    main()
