import os
import re
import cv2
import numpy as np
import random
import time
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

# ==================== 配置区域 ====================
DATASET_DIR = os.path.join("assets", "物料盘")
# ==================================================

# 颜色映射定义
COLOR_CLASSES = {
    1: {"name": "Red", "bgr": (30, 20, 230)},
    2: {"name": "Yellow", "bgr": (0, 215, 255)},
    3: {"name": "Blue", "bgr": (210, 60, 10)},
    4: {"name": "Green", "bgr": (50, 170, 0)},
    5: {"name": "Black", "bgr": (50, 50, 50)},
    6: {"name": "LightBlue", "bgr": (255, 191, 0)},
    0: {"name": "Abnormal/Defective", "bgr": (128, 0, 128)}
}


def parse_center_gt_from_filename(filename):
    """
    通过解析文件名提取物料盘中心点的真实标注 (Ground Truth)
    文件名示例: (84, 225).jpg
    """
    match = re.search(r"\((\d+),\s*(\d+)\)", filename)
    if match:
        return int(match.group(1)), int(match.group(2))
    return None


def robust_classify_color(roi):
    """
    基于中值滤波的颜色分类算法
    """
    if roi is None or roi.size == 0:
        return 0, "Abnormal/Defective"

    median_b = np.median(roi[:, :, 0])
    median_g = np.median(roi[:, :, 1])
    median_r = np.median(roi[:, :, 2])

    hsv_pixel = cv2.cvtColor(np.uint8([[[median_b, median_g, median_r]]]), cv2.COLOR_BGR2HSV)[0][0]
    h, s, v = hsv_pixel[0], hsv_pixel[1], hsv_pixel[2]

    if s < 25 and v > 200:
        return None, "Background"
    if v < 70:
        return 5, "Black"
    if s < 45:
        return 0, "Abnormal/Defective"

    if (h >= 0 and h < 8) or (h >= 165 and h <= 180):
        return 1, "Red"
    elif h >= 10 and h < 34:
        return 2, "Yellow"
    elif h >= 35 and h < 85:
        return 4, "Green"
    elif h >= 86 and h < 103:
        return 6, "LightBlue"
    elif h >= 104 and h < 140:
        return 3, "Blue"
    else:
        return 0, "Abnormal/Defective"


def detect_materials(img_bgr):
    """
    物料定位检测管线 (沿用您微调后的参数)
    """
    h, w, _ = img_bgr.shape
    detected_list = []
    r = 0.5 * (h + w)
    
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    denoised = cv2.medianBlur(gray, 7)
    blurred = cv2.GaussianBlur(denoised, (9, 9), 2)

    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=85,
        param1=50,
        param2=32,  
        minRadius=int(r * 0.05),
        maxRadius=int(r * 0.35)
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (cx, cy, radius) in circles:
            cx = np.clip(cx, 0, w - 1)
            cy = np.clip(cy, 0, h - 1)

            r_roi = int(radius * 0.6)
            if r_roi < 5:
                r_roi = 5
            roi = img_bgr[max(0, cy - r_roi):min(h, cy + r_roi), max(0, cx - r_roi):min(w, cx + r_roi)]

            res = robust_classify_color(roi)
            if res is None:
                continue
            
            color_code, color_name = res
            detected_list.append({
                "x": int(cx),
                "y": int(cy),
                "r": int(radius),
                "color_code": color_code,
                "color_name": color_name
            })

    detected_list = sorted(detected_list, key=lambda x: x['r'], reverse=True)[:3]
    return detected_list, denoised


def detect_outer_circle(img_bgr):
    """
    大圆(物料盘体)霍夫探测器：用于低空位时的降级定位及两点解算时的方向参考
    物料盘半径由于缩放(0.4-1.6)，像素半径范围约在 75 到 340 之间
    """
    h, w, _ = img_bgr.shape
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    denoised = cv2.medianBlur(gray, 7)
    blurred = cv2.GaussianBlur(denoised, (9, 9), 2)
    
    r_avg = 0.5 * (h + w)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.5,
        minDist=200,
        param1=50,
        param2=40,
        minRadius=int(r_avg * 0.12),  # 对应约 72 像素
        maxRadius=int(r_avg * 0.60)   # 对应约 360 像素
    )
    if circles is not None:
        return circles[0][0]  # 返回最强的一个大圆拟合结果 (cx, cy, r)
    return None


def estimate_disk_center(detected, img_bgr):
    """
    多级降备转盘中心解算核心管线
    """
    h, w, _ = img_bgr.shape
    num_detected = len(detected)
    
    # 1. 探测大圆盘体作为全局参考点（辅助消除歧义）
    c_hough = detect_outer_circle(img_bgr)
    if c_hough is not None:
        ref_center = (c_hough[0], c_hough[1])
    else:
        # 若大圆也未检出，以图像物理中心作为默认参考基准
        ref_center = (w / 2.0, h / 2.0)
        
    # --- 策略 A：检出 3 个物料（0个空位） ---
    if num_detected == 3:
        xs = [d["x"] for d in detected]
        ys = [d["y"] for d in detected]
        # 直接求等边三角形形心
        est_x = int(round(np.mean(xs)))
        est_y = int(round(np.mean(ys)))
        return est_x, est_y, "等边三角形形心解算 (3点)", c_hough
        
    # --- 策略 B：检出 2 个物料（1个空位） ---
    elif num_detected == 2:
        p1 = np.array([detected[0]["x"], detected[0]["y"]])
        p2 = np.array([detected[1]["x"], detected[1]["y"]])
        
        # 计算弦长 d
        d = np.linalg.norm(p1 - p2)
        midpoint = (p1 + p2) / 2.0
        
        # 弦心距 h_dist = d / (2 * sqrt(3))
        h_dist = d / (2.0 * np.sqrt(3.0))
        
        # 计算弦向量与单位法向量
        v = p2 - p1
        n = np.array([-v[1], v[0]]) / d
        
        # 计算两个镜像对称的候选中心点
        c1 = midpoint + h_dist * n
        c2 = midpoint - h_dist * n
        
        # 利用参考中心，剔除背离圆盘体的一侧，保留真正朝向圆内的一侧
        dist1 = np.linalg.norm(c1 - ref_center)
        dist2 = np.linalg.norm(c2 - ref_center)
        
        best_c = c1 if dist1 < dist2 else c2
        return int(round(best_c[0])), int(round(best_c[1])), "双点弦心距几何解算 (2点)", c_hough
        
    # --- 策略 C：检出少于 2 个物料（2-3个空位） ---
    elif c_hough is not None:
        # 直接降级使用大圆探测器圆心
        return int(round(c_hough[0])), int(round(c_hough[1])), "大圆盘体霍夫探测器 (降级)", c_hough
    else:
        # 终极极端保底：使用图像几何中心
        return int(round(w / 2.0)), int(round(h / 2.0)), "画布中点保底 (失效)", None


def evaluate_and_visualize_disk(img_path, detected, est_center, gt_center, c_hough, denoised_gray, latency_ms, method_name):
    """
    可视化绘制模块：集成转盘中心定位与真值比对
    """
    img_bgr = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img_bgr is None:
        return

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    vis_img = img_rgb.copy()

    # 1. 绘制检测到的彩色物料
    for det in detected:
        color_info = COLOR_CLASSES.get(det["color_code"], {"bgr": (128, 128, 128)})
        rgb_color = (color_info["bgr"][2], color_info["bgr"][1], color_info["bgr"][0])
        # 绘制物料轮廓
        cv2.circle(vis_img, (det["x"], det["y"]), det["r"], rgb_color, 2)
        # 标记物料中心点
        cv2.drawMarker(vis_img, (det["x"], det["y"]), rgb_color, markerType=cv2.MARKER_CROSS, markerSize=12, thickness=2)

    # 2. 绘制大圆探测器拟合轮廓（如果存在）
    if c_hough is not None:
        cv2.circle(vis_img, (int(c_hough[0]), int(c_hough[1])), int(c_hough[2]), (200, 200, 200), 1, lineType=cv2.LINE_AA)

    # 3. 绘制估算的转盘中心（红色十字）
    cv2.drawMarker(vis_img, (est_center[0], est_center[1]), (255, 0, 0), markerType=cv2.MARKER_TILTED_CROSS, markerSize=25, thickness=3)
    cv2.circle(vis_img, (est_center[0], est_center[1]), 8, (255, 0, 0), -1)

    # 4. 绘制真实转盘中心 Ground Truth（绿色十字）
    if gt_center is not None:
        cv2.drawMarker(vis_img, (gt_center[0], gt_center[1]), (0, 255, 0), markerType=cv2.MARKER_CROSS, markerSize=25, thickness=2)
        cv2.circle(vis_img, (gt_center[0], gt_center[1]), 5, (0, 255, 0), -1)

    # 计算像素级定位误差 (Euclidean Distance)
    if gt_center is not None:
        error_px = np.sqrt((est_center[0] - gt_center[0])**2 + (est_center[1] - gt_center[1])**2)
    else:
        error_px = None

    # 创建可视化大图
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    # 子图1：物料盘整体定位图
    axes[0].imshow(vis_img)
    axes[0].set_title("1. Disk Center Localization")
    axes[0].axis("off")

    # 子图2：详细定位指标看板
    axes[1].axis("off")
    stats_text = (
        f"===== Disk Localization Metrics =====\n\n"
        f"Image Name: {os.path.basename(img_path)}\n\n"
        f"Inference Latency: {latency_ms:.2f} ms\n"
        f"Detected Materials: {len(detected)} / 3\n"
        f"Localization Method: {method_name}\n\n"
        f"Estimated Center: ({est_center[0]}, {est_center[1]})\n"
        f"Ground Truth Center: {gt_center if gt_center else 'N/A'}\n\n"
        f"-------------------------------------\n"
        f"Pixel-level Absolute Error:\n"
        f" => {f'{error_px:.2f} pixels' if error_px is not None else 'N/A'}\n"
        f"-------------------------------------\n\n"
        f"Visualization Legend:\n"
        f" - Red Large Cross: Estimated Disk Center\n"
        f" - Green Small Cross: Ground Truth Center\n"
        f" - Colored Small Circles: Detected Materials\n"
        f" - Thin Gray Circle: Detected Outer Disk Contour\n"
    )
    axes[1].text(0.05, 0.95, stats_text, fontsize=12, family='monospace', verticalalignment='top')
    axes[1].set_title("2. Localization Report")

    plt.tight_layout()
    plt.show()


def run_disk_localization_pipeline():
    """
    定位管线主入口：随机读取一张物料盘图像并实现中心解算与评估
    """
    if not os.path.exists(DATASET_DIR) or len(os.listdir(DATASET_DIR)) == 0:
        print(f"提示: 未在路径 '{DATASET_DIR}' 下检测到生成的图像。")
        print("请确保已运行批量生成脚本将图像保存至 assets\\物料盘")
        return

    all_files = [f for f in os.listdir(DATASET_DIR) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    if not all_files:
        print("未找到有效的图像文件。")
        return

    selected_file = random.choice(all_files)
    img_path = os.path.join(DATASET_DIR, selected_file)

    # 1. 提取中心点真值
    gt_center = parse_center_gt_from_filename(selected_file)
    print(f"选定测试文件: {selected_file}")
    print(f"解析到 Ground Truth 中心点: {gt_center}")

    # 2. 读取图像
    img_bgr = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img_bgr is None:
        print("错误: 无法读取图像。")
        return

    # 3. 运行管线解算中心
    start_time = time.perf_counter()
    
    # 3.1 提取物料圆
    detected, denoised_gray = detect_materials(img_bgr)
    # 3.2 多级降备解算中心
    est_x, est_y, method_name, c_hough = estimate_disk_center(detected, img_bgr)
    
    end_time = time.perf_counter()
    latency_ms = (end_time - start_time) * 1000

    print(f"解算所用方法: {method_name}")
    print(f"估算中心点位置: ({est_x}, {est_y})")
    if gt_center:
        err = np.sqrt((est_x - gt_center[0])**2 + (est_y - gt_center[1])**2)
        print(f"像素定位绝对误差: {err:.2f} pixels")
    print(f"总计算耗时: {latency_ms:.2f} 毫秒 (ms)\n")

    # 4. 可视化报告展示
    evaluate_and_visualize_disk(
        img_path, 
        detected, 
        (est_x, est_y), 
        gt_center, 
        c_hough, 
        denoised_gray, 
        latency_ms, 
        method_name
    )


if __name__ == "__main__":
    run_disk_localization_pipeline()