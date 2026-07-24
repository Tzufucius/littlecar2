import os
import re
import cv2
import numpy as np
import random
import time
from pathlib import Path
from typing import Any, List, Dict
import matplotlib.pyplot as plt

# 支持中文显示
plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

# ==================== 自动路径解析与配置区域 ====================
# PROJECT_ROOT 自动指向 F:\Project\littleCar2\zhengdian\4-29\jetson
PROJECT_ROOT = Path(__file__).resolve().parents[2]

# 模型与数据集的绝对路径引用
MODEL_PATH = PROJECT_ROOT / "assets" / "models" / "6color-circle-v3.pt"
DATASET_DIR = PROJECT_ROOT / "assets" / "物料盘"
NUM_TEST_IMAGES = 9  # 压测展示样本数 (九宫格形式)

CONF_THRES = 0.5
IOU_THRES = 0.45
DEVICE = "cuda:0" if cv2.ocl.haveOpenCL() else "cpu"

# 7类颜色配置表 (RGB格式，用于 matplotlib 绘图)
COLOR_CLASSES = {
    0: {"name": "Red", "rgb": (230, 20, 30)},
    1: {"name": "Yellow", "rgb": (255, 215, 0)},
    2: {"name": "Blue", "rgb": (10, 60, 210)},
    3: {"name": "Green", "rgb": (0, 170, 50)},
    4: {"name": "Black", "rgb": (50, 50, 50)},
    5: {"name": "LightBlue", "rgb": (0, 191, 255)},
    6: {"name": "EmptySlot", "rgb": (120, 120, 120)}, # 第7类：物料空位 (索引为6)
}
# ==================================================

# 尝试载入 YOLO
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


def parse_center_gt_from_filename(filename: str):
    """
    通过解析文件名提取物料盘中心点的真实标注 (Ground Truth)
    文件名示例: (84, 225).jpg
    """
    match = re.search(r"\((\d+),\s*(\d+)\)", filename)
    if match:
        return int(match.group(1)), int(match.group(2))
    return None


def estimate_disk_center_yolo(detections: List[Dict[str, Any]], img_w: int, img_h: int):
    """
    基于 YOLO 检测结果的物料盘中心解算算法
    
    逻辑：
      1. 优先选择有颜色物料（Class 0-5）的中心点，最多选择3个。
      2. 若有颜色的物料不足3个，则使用空孔位（Class 6）的中心点补齐。
      3. 优化：如果只检测到1个目标（例如整个圆盘被误识别为唯一空槽，或仅发现单个空位），
         直接将其框的几何中心作为物料盘原点，避免硬用画布中心保底。
    """
    colored_pts = []
    empty_pts = []
    
    for det in detections:
        pt = (det["center_x"], det["center_y"])
        if det["class_id"] == 6:  # 空槽
            empty_pts.append(pt)
        else:  # 有色物料 (0~5)
            colored_pts.append(pt)
            
    # 优先选取有颜色的，不足3个时从空槽中补齐
    selected_pts = []
    selected_pts.extend(colored_pts[:3])
    needed = 3 - len(selected_pts)
    if needed > 0:
        selected_pts.extend(empty_pts[:needed])
        
    num_pts = len(selected_pts)
    
    # --- 策略 A：成功凑齐 3 个定位中心点 ---
    if num_pts == 3:
        est_x = int(round(np.mean([p[0] for p in selected_pts])))
        est_y = int(round(np.mean([p[1] for p in selected_pts])))
        method_name = "YOLO 三点形心解算"
        
    # --- 策略 B：降级方案，仅有 2 个定位点 ---
    elif num_pts == 2:
        p1 = np.array(selected_pts[0])
        p2 = np.array(selected_pts[1])
        d = np.linalg.norm(p1 - p2)
        midpoint = (p1 + p2) / 2.0
        
        # 弦心距计算（基于3等分圆盘的三角几何关系）
        h_dist = d / (2.0 * np.sqrt(3.0))
        v = p2 - p1
        n = np.array([-v[1], v[0]]) / (d if d > 0 else 1.0)
        
        # 以图像几何中心作为全局方位参考，剔除背向中心的一侧
        ref_center = np.array([img_w / 2.0, img_h / 2.0])
        c1 = midpoint + h_dist * n
        c2 = midpoint - h_dist * n
        
        best_c = c1 if np.linalg.norm(c1 - ref_center) < np.linalg.norm(c2 - ref_center) else c2
        est_x = int(round(best_c[0]))
        est_y = int(round(best_c[1]))
        method_name = "YOLO 双点几何解算 (降级)"
        
    # --- 策略 C：新增/重构方案：仅检出 1 个目标 (针对图3情况进行物理级优化) ---
    elif num_pts == 1:
        # 当模型将整个大圆盘误检测为了一个巨大的 EmptySlot(空位)，
        # 或者在画面偏置极度严重、只检出了单个空位时，直接将该唯一目标的中心作为物料盘原点
        est_x = int(round(selected_pts[0][0]))
        est_y = int(round(selected_pts[0][1]))
        method_name = "单目标中心定位 (保底)"
        
    # --- 策略 D：极端降级方案，定位点为0个 ---
    else:
        est_x = int(round(img_w / 2.0))
        est_y = int(round(img_h / 2.0))
        method_name = "画布中点保底 (失效)"
        
    return est_x, est_y, method_name, selected_pts


def main():
    print(f"当前项目根目录识别为: '{PROJECT_ROOT}'")
    print("正在加载 YOLO 目标检测模型...")
    if not MODEL_PATH.exists():
        print(f"错误: 未找到模型权重文件 '{MODEL_PATH}'，请确认路径是否正确。")
        return
        
    detector = load_yolo_model(model_path=MODEL_PATH)
    
    if not DATASET_DIR.exists() or len(os.listdir(DATASET_DIR)) == 0:
        print(f"提示: 数据集路径 '{DATASET_DIR}' 不存在或没有图像文件。")
        return

    all_files = [f for f in os.listdir(DATASET_DIR) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    if not all_files:
        print("未在数据集中找到有效的图像文件。")
        return

    # 随机抽取 9 张图像进行压力测试
    test_count = min(NUM_TEST_IMAGES, len(all_files))
    selected_files = random.sample(all_files, test_count)
    
    print(f"成功选择 {test_count} 张图像开始压力测试...\n")
    
    fig, axes = plt.subplots(3, 3, figsize=(15, 15))
    axes = axes.flatten()
    
    latencies = []
    errors = []
    
    for idx, filename in enumerate(selected_files):
        img_path = DATASET_DIR / filename
        img_bgr = cv2.imread(str(img_path))
        if img_bgr is None:
            continue
            
        h, w, _ = img_bgr.shape
        gt_center = parse_center_gt_from_filename(filename)
        
        # 1. 运行检测并统计耗时
        start_time = time.perf_counter()
        detections = detect_yolo(img_bgr, detector, device=DEVICE, conf_thres=CONF_THRES, iou_thres=IOU_THRES)
        end_time = time.perf_counter()
        
        latency_ms = (end_time - start_time) * 1000.0
        latencies.append(latency_ms)
        
        # 2. 解算转盘中心点
        est_x, est_y, method, selected_pts = estimate_disk_center_yolo(detections, w, h)
        
        # 3. 计算绝对定位误差
        if gt_center:
            err_px = np.sqrt((est_x - gt_center[0])**2 + (est_y - gt_center[1])**2)
            errors.append(err_px)
        else:
            err_px = None
            
        # 4. 可视化绘制结果
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        
        # 在子图画布上绘制
        ax = axes[idx]
        ax.imshow(img_rgb)
        
        # 4.1 绘制检测到的所有 YOLO 框
        for det in detections:
            cid = det["class_id"]
            color_info = COLOR_CLASSES.get(cid, {"rgb": (128, 128, 128)})
            color = [c / 255.0 for c in color_info["rgb"]]
            
            # 画目标框
            rect = plt.Rectangle(
                (det["x1"], det["y1"]), det["x2"] - det["x1"], det["y2"] - det["y1"],
                fill=False, edgecolor=color, linewidth=2, alpha=0.8
            )
            ax.add_patch(rect)
            # 标记目标框中心
            ax.plot(det["center_x"], det["center_y"], marker='o', color=color, markersize=5)
            
        # 4.2 绘制解算采用的3点构成的三角形（如果足3个点）
        if len(selected_pts) == 3:
            polygon = plt.Polygon(selected_pts, closed=True, fill=False, edgecolor='cyan', linestyle='--', linewidth=1.5, alpha=0.7)
            ax.add_patch(polygon)
            
        # 4.3 标注估算中心与真实中心
        # 红色斜十字代表 YOLO 估算中心
        ax.plot(est_x, est_y, marker='x', color='red', markersize=14, markeredgewidth=3, label="Est Center")
        ax.plot(est_x, est_y, marker='o', color='red', markersize=6)
        
        # 绿色十字代表文件名解析出的 GT 真实中心
        if gt_center:
            ax.plot(gt_center[0], gt_center[1], marker='+', color='lime', markersize=14, markeredgewidth=2, label="GT Center")
            # 绘制指示偏差的虚黄线
            ax.plot([est_x, gt_center[0]], [est_y, gt_center[1]], color='yellow', linestyle=':', linewidth=1.5)
            
        # 4.4 设置图像标题与轴信息
        ax.axis("off")
        err_str = f"Error: {err_px:.1f}px" if err_px is not None else "Error: N/A"
        ax.set_title(
            f"图{idx+1}: {filename[:12]}...\n"
            f"耗时: {latency_ms:.1f}ms | {err_str}\n"
            f"方法: {method}",
            fontsize=9, color="white", backgroundcolor="black", alpha=0.8
        )
        
    # 如果抽样数不满9张，则将多余的子图置空
    for idx in range(test_count, 9):
        axes[idx].axis("off")
        axes[idx].text(0.5, 0.5, "No Image Data", ha="center", va="center", color="gray", fontsize=12)
        
    # 计算并汇总压测指标
    avg_latency = np.mean(latencies) if latencies else 0.0
    avg_error = np.mean(errors) if errors else 0.0
    
    summary_text = (
        f"📊 旋转物料盘 YOLO 定位压力测试结果汇总:\n"
        f"  >> 样本总数: {test_count} 张图像\n"
        f"  >> YOLO 平均推理延时: {avg_latency:.2f} ms\n"
        f"  >> 绝对像素级定位平均误差: {avg_error:.2f} pixels"
    )
    print("\n" + "="*50)
    print(summary_text)
    print("="*50 + "\n")
    
    plt.suptitle(
        f"旋转物料盘 YOLO 综合定位压测 (3x3 九宫格)\n平均耗时: {avg_latency:.1f}ms | 平均绝对误差: {avg_error:.1f}px", 
        fontsize=16, fontweight="bold", y=0.98
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()