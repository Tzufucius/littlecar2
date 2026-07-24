"""Run YOLO material detection and material-disk center evaluation."""

from __future__ import annotations

import os
import random
import re
import sys
import time
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import torch


PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from vision import detect_colored_materials, estimate_disk_center, load_yolo_model


MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "6color-circle-v3.pt"
DATASET_DIR = PROJECT_ROOT / "assets" / "物料盘"
NUM_TEST_IMAGES = 9
CONF_THRES = 0.5
IOU_THRES = 0.45
DEVICE = "cuda:0" if torch.cuda.is_available() else "cpu"

COLOR_CLASSES = {
    0: {"name": "Red", "rgb": (230, 20, 30)},
    1: {"name": "Yellow", "rgb": (255, 215, 0)},
    2: {"name": "Blue", "rgb": (10, 60, 210)},
    3: {"name": "Green", "rgb": (0, 170, 50)},
    4: {"name": "Black", "rgb": (50, 50, 50)},
    5: {"name": "LightBlue", "rgb": (0, 191, 255)},
    6: {"name": "EmptySlot", "rgb": (120, 120, 120)},
}


def parse_center_gt_from_filename(filename: str) -> tuple[int, int] | None:
    match = re.search(r"\((\d+),\s*(\d+)\)", filename)
    return (int(match.group(1)), int(match.group(2))) if match else None


def main() -> None:
    if not MODEL_PATH.exists():
        raise SystemExit(f"Model not found: {MODEL_PATH}")
    if not DATASET_DIR.exists():
        raise SystemExit(f"Dataset not found: {DATASET_DIR}")

    image_files = [
        path for path in DATASET_DIR.iterdir()
        if path.suffix.lower() in {".jpg", ".jpeg", ".png"}
    ]
    if not image_files:
        raise SystemExit(f"No images found in {DATASET_DIR}")

    model = load_yolo_model(MODEL_PATH)
    selected_files = random.sample(image_files, min(NUM_TEST_IMAGES, len(image_files)))
    fig, axes = plt.subplots(3, 3, figsize=(15, 15))
    axes = np.asarray(axes).reshape(-1)
    latencies: list[float] = []
    errors: list[float] = []

    for index, image_path in enumerate(selected_files):
        image = cv2.imdecode(np.fromfile(str(image_path), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            continue

        started = time.perf_counter()
        detections = detect_colored_materials(
            image,
            model,
            conf_thres=CONF_THRES,
            iou_thres=IOU_THRES,
            device=DEVICE,
        )
        latency_ms = (time.perf_counter() - started) * 1000
        latencies.append(latency_ms)

        center = estimate_disk_center(detections, image.shape)
        estimated = (center["center_x"], center["center_y"])
        ground_truth = parse_center_gt_from_filename(image_path.name)
        error = None
        if ground_truth is not None:
            error = float(np.linalg.norm(np.asarray(estimated) - np.asarray(ground_truth)))
            errors.append(error)

        axis = axes[index]
        axis.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        for detection in detections:
            class_id = detection["class_id"]
            color = np.asarray(COLOR_CLASSES.get(class_id, {"rgb": (128, 128, 128)})["rgb"]) / 255
            axis.add_patch(plt.Rectangle(
                (detection["x1"], detection["y1"]),
                detection["x2"] - detection["x1"],
                detection["y2"] - detection["y1"],
                fill=False,
                edgecolor=color,
                linewidth=2,
            ))
            axis.plot(detection["center_x"], detection["center_y"], "o", color=color, markersize=5)

        if len(center["support_points"]) == 3:
            axis.add_patch(plt.Polygon(center["support_points"], closed=True, fill=False, color="cyan", linestyle="--"))
        axis.plot(*estimated, marker="x", color="red", markersize=14, markeredgewidth=3)
        if ground_truth is not None:
            axis.plot(*ground_truth, marker="+", color="lime", markersize=14, markeredgewidth=2)
        axis.axis("off")
        error_text = f"{error:.1f}px" if error is not None else "N/A"
        axis.set_title(f"{image_path.name}\n{latency_ms:.1f} ms | error {error_text}\n{center['method']}")

    for axis in axes[len(selected_files):]:
        axis.axis("off")

    average_latency = np.mean(latencies) if latencies else 0.0
    average_error = np.mean(errors) if errors else 0.0
    plt.suptitle(f"YOLO disk center evaluation | {average_latency:.1f} ms | {average_error:.1f}px")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
