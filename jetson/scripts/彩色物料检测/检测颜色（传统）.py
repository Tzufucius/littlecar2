import os
import re
import cv2
import numpy as np
import random
import time  # 引入高精度计时库
import matplotlib.pyplot as plt

# ==================== 配置区域 ====================
DATASET_DIR = os.path.join("assets", "彩色物料")
# ==================================================

# 颜色映射定义
COLOR_CLASSES = {
    1: {"name": "Red", "bgr": (30, 20, 230)},
    2: {"name": "Yellow", "bgr": (0, 215, 255)},
    3: {"name": "Blue", "bgr": (210, 60, 10)},
    4: {"name": "Green", "bgr": (50, 170, 0)},
    5: {"name": "Black", "bgr": (50, 50, 50)},
    6: {"name": "LightBlue", "bgr": (255, 191, 0)},
    0: {"name": "Abnormal/Defective", "bgr": (128, 0, 128)}  # 异常/缺损
}


def parse_gt_from_filename(filename):
    """
    通过解析文件名提取真实标注 (Ground Truth)
    文件名示例: 1(195, 308)2(100, 500)5(76, 200).jpg
    """
    pattern = r"([1-6])\((\d+),\s*(\d+)\)"
    matches = re.findall(pattern, filename)
    gt_list = []
    for color_code, x, y in matches:
        gt_list.append({
            "color_code": int(color_code),
            "x": int(x),
            "y": int(y)
        })
    return gt_list


def robust_classify_color(roi):
    """
    基于中值滤波的颜色分类算法，对高感光噪点具有极强的鲁棒性
    """
    if roi is None or roi.size == 0:
        return 0, "Abnormal/Defective"

    # 计算该区域的 BGR 中值
    median_b = np.median(roi[:, :, 0])
    median_g = np.median(roi[:, :, 1])
    median_r = np.median(roi[:, :, 2])

    # 转换为 HSV 空间进行阈值判定
    hsv_pixel = cv2.cvtColor(np.uint8([[[median_b, median_g, median_r]]]), cv2.COLOR_BGR2HSV)[0][0]
    h, s, v = hsv_pixel[0], hsv_pixel[1], hsv_pixel[2]

    # 1. 过滤白色背景误检 (高亮度，极低饱和度)
    if s < 25 and v > 200:
        return None, "Background"

    # 2. 识别黑色 (低亮度)
    if v < 70:
        return 5, "Black"

    # 3. 识别异常/缺损 (如饱和度过低，或颜色无法匹配)
    if s < 45:
        return 0, "Abnormal/Defective"

    # 4. 正常颜色 Hue 区间划分 (OpenCV H: 0-180)
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
    抗噪预处理与物料定位检测管线
    """
    h, w, _ = img_bgr.shape
    detected_list = []

    r = 0.5 * (h + w)
    
    # 1. 预处理：灰度化 -> 强中值滤波（抹除椒盐噪声） -> 高斯模糊（平滑干扰线）
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    denoised = cv2.medianBlur(gray, 7)
    blurred = cv2.GaussianBlur(denoised, (9, 9), 2)

    # 2. 霍夫圆检测
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=85,
        param1=50,
        param2=32,  # 稍微调低以适应阴影条带穿过的圆
        minRadius=int(r*0.05),
        maxRadius=int(r*0.35)
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (cx, cy, r) in circles:
            cx = np.clip(cx, 0, w - 1)
            cy = np.clip(cy, 0, h - 1)

            # 裁剪圆心处 60% 范围的 ROI，避开边缘干涉和投影
            r_roi = int(r * 0.6)
            if r_roi < 5:
                r_roi = 5
            roi = img_bgr[max(0, cy - r_roi):min(h, cy + r_roi), max(0, cx - r_roi):min(w, cx + r_roi)]

            # 颜色分类
            res = robust_classify_color(roi)
            if res is None:  # 被判定为背景误检
                continue
            
            color_code, color_name = res
            detected_list.append({
                "x": int(cx),
                "y": int(cy),
                "r": int(r),
                "color_code": color_code,
                "color_name": color_name
            })

    # 限制一个画面最多检测3个物料
    detected_list = sorted(detected_list, key=lambda x: x['r'], reverse=True)[:3]
    return detected_list, denoised


def evaluate_and_visualize(img_path, detected, gt_list, denoised_gray, latency_ms):
    """
    匹配检测结果与真实标注，并使用 Matplotlib 绘制结果图表（已支持中文路径与耗时统计）
    """
    img_bgr = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img_bgr is None:
        print(f"错误: 无法载入图像: {img_path}")
        return

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    
    # 拷贝图像用于绘制视觉圈
    vis_img = img_rgb.copy()

    matched_gt = set()
    tp_count = 0  # 检出且颜色正确的数量
    fp_count = 0  # 误检数量
    wrong_class_count = 0  # 检出但颜色错误的数量

    # 匹配检测圆和真实标注
    for det in detected:
        best_match_idx = -1
        min_dist = 99999
        
        for i, gt in enumerate(gt_list):
            if i in matched_gt:
                continue
            dist = np.sqrt((det["x"] - gt["x"])**2 + (det["y"] - gt["y"])**2)
            if dist < 50 and dist < min_dist:  # 50像素内视为同一物体
                min_dist = dist
                best_match_idx = i
                
        if best_match_idx != -1:
            matched_gt.add(best_match_idx)
            gt_obj = gt_list[best_match_idx]
            
            if det["color_code"] == gt_obj["color_code"]:
                tp_count += 1
                # 绘制绿色圈表示：成功检测且颜色正确
                cv2.circle(vis_img, (det["x"], det["y"]), det["r"], (0, 255, 0), 3)
                cv2.putText(vis_img, f"Class {det['color_code']}", (det["x"] - 30, det["y"] - det["r"] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                wrong_class_count += 1
                # 绘制黄色圈表示：检测到物体，但颜色识别错误/异常
                cv2.circle(vis_img, (det["x"], det["y"]), det["r"], (255, 255, 0), 3)
                cv2.putText(vis_img, f"Wrong: {det['color_code']} (GT:{gt_obj['color_code']})", 
                            (det["x"] - 50, det["y"] - det["r"] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        else:
            fp_count += 1
            # 绘制红色圈表示：纯误检
            cv2.circle(vis_img, (det["x"], det["y"]), det["r"], (255, 0, 0), 3)
            cv2.putText(vis_img, "FP / Dust", (det["x"] - 30, det["y"] - det["r"] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # 漏检计算 (Ground Truth 中未被匹配的部分)
    fn_count = len(gt_list) - len(matched_gt)
    for i, gt in enumerate(gt_list):
        if i not in matched_gt:
            # 红色圈表示漏检物料的位置
            cv2.circle(vis_img, (gt["x"], gt["y"]), 45, (255, 0, 0), 2, lineType=cv2.LINE_AA)
            cv2.putText(vis_img, f"Missed: {gt['color_code']}", (gt["x"] - 40, gt["y"] - 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    # 创建 Matplotlib 观察图表
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # 子图1：中值滤波去噪效果（观察算法预处理阶段如何抹除噪点和划痕）
    axes[0].imshow(denoised_gray, cmap='gray')
    axes[0].set_title("1. Denoised Grayscale Image")
    axes[0].axis("off")

    # 子图2：最终检测视觉图
    axes[1].imshow(vis_img)
    axes[1].set_title("2. Detection Visualization")
    axes[1].axis("off")

    # 子图3：数据统计和参数微调建议
    axes[2].axis("off")
    stats_text = (
        f"--- Validation Statistics ---\n\n"
        f"Image: {os.path.basename(img_path)[:35]}...\n\n"
        f"Inference Latency: {latency_ms:.2f} ms\n"  # 整合推理耗时
        f"Ground Truth Count: {len(gt_list)}\n"
        f"Detected Count: {len(detected)}\n\n"
        f"True Positives (Correct Class): {tp_count}\n"
        f"Wrong Classification: {wrong_class_count}\n"
        f"False Positives (False Alarm): {fp_count}\n"
        f"False Negatives (Missed): {fn_count}\n\n"
        f"---------------------------------\n"
        f"Tuning Recommendations:\n"
        f"- If Latency is High: Try lowering resolution or reducing kernel size.\n"
        f"- If Missed (FN > 0): Decrease 'param2' in HoughCircles.\n"
        f"- If Over-detected (FP > 0): Increase 'param2' or refine min/max radius.\n"
        f"- If Wrong Class > 0: Adjust BGR/HSV thresholds in 'robust_classify_color'.\n"
    )
    axes[2].text(0.05, 0.95, stats_text, fontsize=11, family='monospace', verticalalignment='top')
    axes[2].set_title("3. Metrics & Tuning Guide")

    plt.tight_layout()
    plt.show()


def run_validation_pipeline():
    """
    随机读取一张生成图像并执行检测与验证
    """
    # 检查数据集目录
    if not os.path.exists(DATASET_DIR) or len(os.listdir(DATASET_DIR)) == 0:
        print(f"提示: 未在路径 '{DATASET_DIR}' 下检测到生成的图像。")
        print("请先运行前一步骤的图像生成脚本。")
        return

    # 过滤并随机选择一张图片
    all_files = [f for f in os.listdir(DATASET_DIR) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    if not all_files:
        print("未找到有效的图像文件。")
        return

    selected_file = random.choice(all_files)
    img_path = os.path.join(DATASET_DIR, selected_file)

    # 1. 提取真值标注
    gt_list = parse_gt_from_filename(selected_file)
    print(f"选定测试图片: {selected_file}")
    print(f"解析到 Ground Truth 标注: {gt_list}")

    # 2. 读取图像并执行物料检测
    img_bgr = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img_bgr is None:
        print("无法载入图像。")
        return

    # 高精度性能耗时统计
    start_time = time.perf_counter()
    detected, denoised_gray = detect_materials(img_bgr)
    end_time = time.perf_counter()
    
    # 换算为毫秒
    latency_ms = (end_time - start_time) * 1000
    
    print(f"检测到的物料结果: {detected}")
    print(f"检测耗时: {latency_ms:.2f} 毫秒 (ms)")

    # 3. 结果对齐并绘制调参图表
    evaluate_and_visualize(img_path, detected, gt_list, denoised_gray, latency_ms)


if __name__ == "__main__":
    run_validation_pipeline()