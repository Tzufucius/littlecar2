import os
import sys
import glob
import time
import random
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np
import matplotlib.pyplot as plt

# 建立路径引用
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

try:
    from vision import load_yolo_model, detect_yolo
except ImportError:
    from ultralytics import YOLO
    
    def load_yolo_model(model_path: str | Path):
        return YOLO(str(model_path))

    def detect_yolo(
        frame_bgr: np.ndarray,
        model: Any,
        conf_thres: float = 0.5,
        iou_thres: float = 0.45,
        device: str | None = None,
    ) -> list[dict[str, Any]]:
        result = model.predict(source=frame_bgr, conf=conf_thres, iou=iou_thres, device=device, verbose=False)[0]
        if result.boxes is None:
            return []
        names = model.names
        detections: list[dict[str, Any]] = []
        for box in result.boxes:
            class_id = int(box.cls[0])
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            detections.append({
                "class_id": class_id,
                "class_name": names.get(class_id, str(class_id)),
                "confidence": float(box.conf[0]),
                "x1": int(x1),
                "y1": int(y1),
                "x2": int(x2),
                "y2": int(y2),
                "center_x": int((x1 + x2) / 2),
                "center_y": int((y1 + y2) / 2),
            })
        return detections


# ==================== 配置区域 ====================
MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "6color-circle-v2.pt"
DATASET_DIR = PROJECT_ROOT / "assets" / "彩色物料数据集"
DEVICE = "cuda:0" if cv2.ocl.haveOpenCL() else "cpu"
CONF_THRES = 0.5
IOU_THRES = 0.45
NUM_TEST_IMAGES = 200  # 压测样本数

# 映射类别
CLASS_NAMES = ["Red", "Yellow", "Blue", "Green", "Black", "LightBlue"]
# 用于图表美观的近似物理颜色对应
CHART_COLORS = ["crimson", "gold", "royalblue", "forestgreen", "black", "skyblue"]
# ==================================================


def load_ground_truths(image_path: Path, img_shape: tuple) -> List[Dict[str, Any]]:
    """
    还原 YOLO 归一化标注为绝对像素坐标
    """
    height, width = img_shape[:2]
    label_path = Path(str(image_path).replace("images", "labels").replace(".jpg", ".txt"))
    
    gts = []
    if not label_path.exists():
        return gts

    with open(label_path, "r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) >= 5:
                class_id = int(parts[0])
                x_c = float(parts[1]) * width
                y_c = float(parts[2]) * height
                bw = float(parts[3]) * width
                bh = float(parts[4]) * height
                
                gts.append({
                    "class_id": class_id,
                    "class_name": CLASS_NAMES[class_id] if class_id < len(CLASS_NAMES) else str(class_id),
                    "center_x": int(x_c),
                    "center_y": int(y_c),
                    "x1": int(x_c - bw / 2),
                    "y1": int(y_c - bh / 2),
                    "x2": int(x_c + bw / 2),
                    "y2": int(y_c + bh / 2)
                })
    return gts


def match_predictions_and_gts(predictions: List[Dict[str, Any]], ground_truths: List[Dict[str, Any]], max_dist_thresh: float = 120.0):
    """
    使用距离排序匹配算法，对预测框与真值框进行空间配对 [1]
    """
    used_gts = set()
    used_preds = set()
    matches = []

    all_pairs = []
    for gt in ground_truths:
        for pred in predictions:
            dist = np.hypot(gt["center_x"] - pred["center_x"], gt["center_y"] - pred["center_y"])
            all_pairs.append((dist, gt, pred))

    all_pairs.sort(key=lambda x: x[0])

    for dist, gt, pred in all_pairs:
        gt_id = id(gt)
        pred_id = id(pred)
        if gt_id not in used_gts and pred_id not in used_preds:
            if dist < max_dist_thresh:
                used_gts.add(gt_id)
                used_preds.add(pred_id)
                matches.append({
                    "distance": dist,
                    "gt": gt,
                    "pred": pred,
                    "class_correct": gt["class_id"] == pred["class_id"]
                })

    unmatched_gts = [gt for gt in ground_truths if id(gt) not in used_gts]
    unmatched_preds = [pred for pred in predictions if id(pred) not in used_preds]

    return matches, unmatched_gts, unmatched_preds


def draw_visuals(img: np.ndarray, matches: list, unmatched_gts: list, unmatched_preds: list) -> np.ndarray:
    """
    在图像上叠加绘制真实值、预测值以及它们之间的偏差标识 [1]
    """
    canvas = img.copy()

    # 1. 绘制匹配对
    for match in matches:
        gt = match["gt"]
        pred = match["pred"]
        dist = match["distance"]
        correct = match["class_correct"]

        # 预测框 (蓝色)
        cv2.rectangle(canvas, (pred["x1"], pred["y1"]), (pred["x2"], pred["y2"]), (255, 100, 0), 2)
        cv2.circle(canvas, (pred["center_x"], pred["center_y"]), 4, (255, 100, 0), -1)

        # 真实中心点 (绿色)
        cv2.circle(canvas, (gt["center_x"], gt["center_y"]), 4, (0, 255, 0), -1)

        # 连线反映偏差
        cv2.line(canvas, (gt["center_x"], gt["center_y"]), (pred["center_x"], pred["center_y"]), (0, 255, 255), 1, cv2.LINE_AA)

        if correct:
            label_text = f"{pred['class_name']}: {pred['confidence']:.2f} ({dist:.1f}px)"
            text_color = (0, 255, 0)
        else:
            label_text = f"ERR: {pred['class_name']}->GT:{gt['class_name']}"
            text_color = (0, 0, 255)

        cv2.putText(canvas, label_text, (pred["x1"], pred["y1"] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1, cv2.LINE_AA)

    # 2. 绘制漏检真物 (橙黄色虚圆代表未检测到)
    for gt in unmatched_gts:
        cv2.circle(canvas, (gt["center_x"], gt["center_y"]), 15, (0, 165, 255), 2)
        cv2.putText(canvas, f"Missed: {gt['class_name']}", (gt["center_x"] - 30, gt["center_y"] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 165, 255), 1, cv2.LINE_AA)

    # 3. 绘制误检多余物 (粉色框表示背景误判)
    for pred in unmatched_preds:
        cv2.rectangle(canvas, (pred["x1"], pred["y1"]), (pred["x2"], pred["y2"]), (180, 105, 255), 2)
        cv2.putText(canvas, f"FP: {pred['class_name']}", (pred["x1"], pred["y1"] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 105, 255), 1, cv2.LINE_AA)

    return canvas


def main() -> None:
    print("正在加载 YOLO 模型...")
    detector = load_yolo_model(model_path=MODEL_PATH)

    search_pattern = str(DATASET_DIR / "images" / "**" / "*.jpg")
    all_images = glob.glob(search_pattern, recursive=True)

    if not all_images:
        print(f"在路径 '{DATASET_DIR / 'images'}' 下未发现任何图像。")
        return

    # 确定测试数量
    test_count = min(NUM_TEST_IMAGES, len(all_images))
    selected_img_paths = random.sample(all_images, test_count)
    print(f"开始批量压测，共抽样 {test_count} 张图像...\n")

    # 初始化统计数据
    num_classes = len(CLASS_NAMES)
    total_counts = [0] * num_classes    # 每种颜色真实出现的总次数
    class_errors = [0] * num_classes    # 每种颜色定位到了但分类判断错误的次数
    missed_counts = [0] * num_classes   # 每种颜色完全漏检未检出的次数
    
    wrong_examples = []  # 存放包含“错误/漏检”的绘图结果和路径名
    infer_times = []

    # 循环推理并收集指标
    for i, path_str in enumerate(selected_img_paths, 1):
        img_path = Path(path_str)
        np_img = cv2.imread(str(img_path))
        if np_img is None:
            continue

        # 推理并记录用时
        t_start = time.perf_counter()
        detections = detect_yolo(np_img, detector, device=DEVICE, conf_thres=CONF_THRES, iou_thres=IOU_THRES)
        t_end = time.perf_counter()
        infer_times.append((t_end - t_start) * 1000.0)

        # 真值载入与空间匹配
        ground_truths = load_ground_truths(img_path, np_img.shape)
        matches, unmatched_gts, unmatched_preds = match_predictions_and_gts(detections, ground_truths)

        # --- 状态统计分析 ---
        # 统计真实物料中各颜色出现的频次
        for gt in ground_truths:
            total_counts[gt["class_id"]] += 1

        # 统计分类判断错误
        has_local_error = False
        for m in matches:
            if not m["class_correct"]:
                class_errors[m["gt"]["class_id"]] += 1
                has_local_error = True

        # 统计漏检频次
        for gt in unmatched_gts:
            missed_counts[gt["class_id"]] += 1
            has_local_error = True

        # 如果存在多余物检测（误检），也归为错误用例
        if unmatched_preds:
            has_local_error = True

        # 绘制并保留异常图像用于之后的展示
        annotated_img = draw_visuals(np_img, matches, unmatched_gts, unmatched_preds)
        if has_local_error:
            wrong_examples.append((annotated_img, img_path.name))

        if i % 10 == 0 or i == test_count:
            print(f"进度进度: 已推理并处理 {i}/{test_count} 张图像...")

    print(f"\n==================================================")
    print(f"压测结束！共分析 {test_count} 张图 | 累计平均推理耗时: {sum(infer_times)/len(infer_times):.2f} ms")
    print(f"有异常（含分类错/漏检/虚警）的图像数: {len(wrong_examples)} / {test_count}")
    print(f"==================================================\n")

    # --- 1. 使用 Matplotlib 绘制统计树状柱状图 ---
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))

    # 子图 A: 样本中颜色的总分布柱状图
    ax1.bar(CLASS_NAMES, total_counts, color=CHART_COLORS, edgecolor='grey', alpha=0.85)
    ax1.set_title("Ground Truth Color Distribution", fontsize=11, fontweight='bold')
    ax1.set_xlabel("Material Color")
    ax1.set_ylabel("Total Occurrences")
    ax1.grid(axis='y', linestyle='--', alpha=0.5)

    # 子图 B: 判断错误与完全漏检的频次统计对比
    x = np.arange(num_classes)
    bar_width = 0.35
    ax2.bar(x - bar_width/2, class_errors, bar_width, label="Classification Errors", color="tomato", edgecolor='grey')
    ax2.bar(x + bar_width/2, missed_counts, bar_width, label="Missed Detections (FN)", color="orange", edgecolor='grey')
    ax2.set_title("Failures Distribution per Color", fontsize=11, fontweight='bold')
    ax2.set_xlabel("Material Color")
    ax2.set_ylabel("Failures Count")
    ax2.set_xticks(x)
    ax2.set_xticklabels(CLASS_NAMES)
    ax2.legend()
    ax2.grid(axis='y', linestyle='--', alpha=0.5)

    plt.suptitle("Batch Performance & Failures Diagnostics", fontsize=14, y=0.98)
    plt.tight_layout()
    plt.show()  # 弹出统计图窗口

    # --- 2. 抽取并展示最多 4 张错误或漏检的示例图像 ---
    if wrong_examples:
        # 打乱后选择最多 4 张失败样本
        random.shuffle(wrong_examples)
        display_num = min(4, len(wrong_examples))
        
        fig, axes = plt.subplots(2, 2, figsize=(11, 11))
        axes = axes.flatten()

        for idx in range(4):
            ax = axes[idx]
            if idx < display_num:
                err_img, name_str = wrong_examples[idx]
                img_rgb = cv2.cvtColor(err_img, cv2.COLOR_BGR2RGB)
                ax.imshow(img_rgb)
                ax.set_title(f"Example {idx+1}: {name_str}", fontsize=9)
            else:
                # 凑不满 4 张时显示空白格子
                ax.text(0.5, 0.5, "No More Failures", ha="center", va="center", color="gray", fontsize=12)
            ax.axis("off")

        plt.suptitle(f"Selected Error & Missed Examples ({len(wrong_examples)} failed images found in total)", fontsize=13, y=0.98)
        plt.tight_layout()
        plt.show()  # 弹出失败样例窗口
    else:
        print("🎉 检测分析完毕：50张抽样样本表现极其优异，未发现任何分类错误或漏检目标！")


if __name__ == "__main__":
    main()