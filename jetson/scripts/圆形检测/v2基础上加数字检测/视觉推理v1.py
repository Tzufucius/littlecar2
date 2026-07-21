import os
import re
import time
import random
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
import itertools  # 用于多圈组合匹配

# ==========================================
# OpenCV 全局底层优化配置
# ==========================================
cv2.setUseOptimized(True)
cv2.setNumThreads(4)

# ==========================================
# 1. 路径与参数配置
# ==========================================
# 查找带数字图像文件夹
IMAGE_DIR = os.path.join("assets", "circle_with_number")
if not os.path.exists(IMAGE_DIR):
    IMAGE_DIR = os.path.join(os.getcwd(), "circle_with_number")
if not os.path.exists(IMAGE_DIR):
    IMAGE_DIR = os.path.join(os.getcwd(), "circle")
    print(f"提示：未找到指定数字目录，切换至通用目录：{IMAGE_DIR}")

# 6圈同心圆设计的名义外径（Nominal Radii）
R_NOMINAL = np.array([28.0, 30.5, 35.5, 40.2, 44.9, 49.6])

# ==========================================
# 2. 定位与数学计算基础函数
# ==========================================
def parse_gt_center(filename):
    """从文件名解析真实中心点坐标 (真值)"""
    match = re.search(r"\((\d+),\s*(\d+)\)", filename)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None

def parse_gt_digit(filename):
    """从文件名解析真实的数字标签"""
    match = re.match(r"^(\d)", filename)
    if match:
        return match.group(1)
    return None

def fit_ellipse_model(pts):
    """极速代数椭圆拟合"""
    if len(pts) < 5:
        return None
    try:
        return cv2.fitEllipse(pts.astype(np.float32))
    except Exception:
        return None

def ransac_ellipse_fit(pts, d_min, n_iter=30, threshold=1.2):
    """
    轻量化、高确定性的 RANSAC 椭圆精拟合 (消除超越函数并对点集等间隔下采样)
    """
    if len(pts) < 5:
        return None, []
    
    step = max(1, len(pts) // 150)
    pts_eval = pts[::step]
    
    best_inliers = []
    best_model = None
    
    for _ in range(n_iter):
        sample_idx = np.random.choice(len(pts_eval), 5, replace=False)
        sample = pts_eval[sample_idx]
        
        model = fit_ellipse_model(sample)
        if model is None:
            continue
        (cx, cy), (w, h), angle_deg = model
        
        if w <= 0 or h <= 0 or np.isnan(cx) or np.isnan(cy):
            continue
        if max(w, h) > d_min * 0.95 or min(w, h) < 10.0:
            continue
        if min(w, h) / max(w, h) < 0.50:
            continue
            
        angle_rad = np.radians(angle_deg)
        cos_a = np.cos(-angle_rad)
        sin_a = np.sin(-angle_rad)
        
        dx = pts_eval[:, 0] - cx
        dy = pts_eval[:, 1] - cy
        x_local = dx * cos_a - dy * sin_a
        y_local = dx * sin_a + dy * cos_a
        
        a = w / 2.0
        b = h / 2.0
        b_val = b if b > 1e-6 else 1e-6
        
        r_pts = np.sqrt(x_local**2 + y_local**2)
        Q = np.sqrt((x_local / a)**2 + (y_local / b_val)**2)
        Q = np.where(Q < 1e-6, 1e-6, Q)
        
        dists = r_pts * np.abs(1.0 - 1.0 / Q)
        inliers = np.where(dists < threshold)[0]
        
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_model = model
            
    if len(best_inliers) >= 5:
        (cx, cy), (w, h), angle_deg = best_model
        angle_rad = np.radians(angle_deg)
        cos_a = np.cos(-angle_rad)
        sin_a = np.sin(-angle_rad)
        
        dx = pts[:, 0] - cx
        dy = pts[:, 1] - cy
        x_local = dx * cos_a - dy * sin_a
        y_local = dx * sin_a + dy * cos_a
        
        a = w / 2.0
        b = h / 2.0
        b_val = b if b > 1e-6 else 1e-6
        
        r_pts = np.sqrt(x_local**2 + y_local**2)
        Q = np.sqrt((x_local / a)**2 + (y_local / b_val)**2)
        Q = np.where(Q < 1e-6, 1e-6, Q)
        
        full_dists = r_pts * np.abs(1.0 - 1.0 / Q)
        full_inliers = np.where(full_dists < threshold)[0]
        
        if len(full_inliers) >= 5:
            final_model = fit_ellipse_model(pts[full_inliers])
            return final_model, full_inliers
        else:
            eval_inliers_mapped = np.arange(len(pts))[::step][best_inliers]
            final_model = fit_ellipse_model(pts[eval_inliers_mapped])
            return final_model, eval_inliers_mapped
    
    return best_model, []

def match_rings_to_nominal(measured_radii):
    """自动精准匹配名义半径"""
    K = len(measured_radii)
    if K == 0:
        return [], 1.0
        
    best_err = float('inf')
    best_indices = []
    best_scale = 1.0
    
    all_combinations = list(itertools.combinations(range(len(R_NOMINAL)), K))
    
    for comb in all_combinations:
        candidate_nominal = R_NOMINAL[list(comb)]
        scales = candidate_nominal / measured_radii
        s_cand = np.mean(scales)
        pred_nominal = s_cand * measured_radii
        err = np.sum(((pred_nominal - candidate_nominal) / candidate_nominal) ** 2)
        
        if err < best_err:
            best_err = err
            best_indices = list(comb)
            best_scale = s_cand
            
    return best_indices, best_scale

def homography_residual(h_params, points_list, ring_indices):
    """单应性残差函数"""
    H = np.array([
        [h_params[0], h_params[1], h_params[2]],
        [h_params[3], h_params[4], h_params[5]],
        [h_params[6], h_params[7], 1.0]
    ])
    
    residuals = []
    for pts, r_idx in zip(points_list, ring_indices):
        r_target = R_NOMINAL[r_idx]
        pts_homo = np.column_stack((pts, np.ones(len(pts))))
        projected = pts_homo @ H.T
        w = projected[:, 2]
        w = np.where(np.abs(w) < 1e-6, 1e-6, w)
        u = projected[:, 0] / w
        v = projected[:, 1] / w
        
        dist = np.sqrt(u**2 + v**2)
        residuals.extend(dist - r_target)
        
    return np.array(residuals)

def correct_perspective_center(points_list, ring_indices, rough_center, s_init):
    """非线性单应性变换校正"""
    cx, cy = rough_center
    s = s_init
    H_init = np.array([
        [s, 0, -s*cx],
        [0, s, -s*cy],
        [0, 0, 1.0]
    ])
    h0 = H_init.flatten()[:8]
    
    sampled_points = []
    for pts in points_list:
        step = max(1, len(pts) // 10)
        sampled_points.append(pts[::step])
        
    res = least_squares(homography_residual, h0, args=(sampled_points, ring_indices), method='lm')
    
    h_opt = res.x
    H_opt = np.array([
        [h_opt[0], h_opt[1], h_opt[2]],
        [h_opt[3], h_opt[4], h_opt[5]],
        [h_opt[6], h_opt[7], 1.0]
    ])
    
    try:
        H_inv = np.linalg.inv(H_opt)
        true_center_homo = H_inv @ np.array([0.0, 0.0, 1.0])
        true_cx = true_center_homo[0] / true_center_homo[2]
        true_cy = true_center_homo[1] / true_center_homo[2]
        return true_cx, true_cy, H_opt
    except np.linalg.LinAlgError:
        return rough_center[0], rough_center[1], H_opt

# ==========================================
# 3. 自适应中心检测主函数
# ==========================================
def process_frame_adaptive_fast(gray_img):
    """极速自适应检测核心算法 (返回精确尺度因子的 output)"""
    h_img, w_img = gray_img.shape[:2]
    d_min = min(h_img, w_img)
    total_pixels = h_img * w_img
    
    adaptive_block_size = int(d_min * 0.16) | 1  
    adaptive_min_area = int(total_pixels * 0.0012)
    adaptive_ransac_thresh = max(1.0, d_min * 0.00375)
    
    thresh = cv2.adaptiveThreshold(
        gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY_INV, adaptive_block_size, 15
    )
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    detected_candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < adaptive_min_area:
            continue
        x_b, y_b, w_b, h_b = cv2.boundingRect(cnt)
        aspect_ratio = float(w_b) / h_b
        if 0.55 < aspect_ratio < 1.85:
            pts = cnt.squeeze(axis=1).astype(np.float32)
            quick_model = fit_ellipse_model(pts)
            if quick_model is not None:
                (cx, cy), (w, h), angle_deg = quick_model
                if w <= 0 or h <= 0 or max(w, h) > d_min * 0.95 or min(w, h) < 10.0:
                    continue
                if min(w, h) / max(w, h) < 0.50:
                    continue
                if not (-d_min*0.1 <= cx <= d_min*1.1 and -d_min*0.1 <= cy <= d_min*1.1):
                    continue
                
                detected_candidates.append({
                    "model": quick_model,
                    "pts": pts
                })
                
    if len(detected_candidates) < 2:
        return None, "有效轮廓不足"
        
    clusters = []
    distance_tolerance = d_min * 0.05
    for cand in detected_candidates:
        cx, cy = cand["model"][0]
        placed = False
        for cluster in clusters:
            ref_cx, ref_cy = cluster[0]["model"][0]
            if np.sqrt((cx - ref_cx)**2 + (cy - ref_cy)**2) < distance_tolerance:
                cluster.append(cand)
                placed = True
                break
        if not placed:
            clusters.append([cand])
            
    best_cluster = max(clusters, key=len)
    if len(best_cluster) < 2:
        return None, "未找到有效的共轴结构"
        
    best_cluster = sorted(best_cluster, key=lambda x: max(x["model"][1]))  
    deduped_cluster = []
    for el in best_cluster:
        if not deduped_cluster:
            deduped_cluster.append(el)
        else:
            prev_size = max(deduped_cluster[-1]["model"][1])
            curr_size = max(el["model"][1])
            if (curr_size - prev_size) > (d_min * 0.025):
                deduped_cluster.append(el)

    if len(deduped_cluster) > 3:
        best_cluster = deduped_cluster[-3:]  
    else:
        best_cluster = deduped_cluster
    
    final_verified_rings = []
    for el in best_cluster:
        fit_res, inliers = ransac_ellipse_fit(el["pts"], d_min, n_iter=30, threshold=adaptive_ransac_thresh)
        if fit_res is not None and len(inliers) > 0:
            final_verified_rings.append({
                "model": fit_res,
                "inliers": el["pts"][inliers]
            })
            
    if len(final_verified_rings) < 2:
        return None, "拓扑提纯后有效同轴环不足"
        
    measured_radii = np.array([max(el["model"][1])/2.0 for el in final_verified_rings])
    ring_indices, s_init = match_rings_to_nominal(measured_radii)
        
    rough_center = final_verified_rings[0]["model"][0]
    points_to_fit = [el["inliers"] for el in final_verified_rings]
    true_cx, true_cy, H_opt = correct_perspective_center(points_to_fit, ring_indices, rough_center, s_init)
    
    output = {
        "img_shape": (w_img, h_img),
        "rough_center": rough_center,
        "true_center": (true_cx, true_cy),
        "ring_count": len(final_verified_rings),
        "matched_indices": ring_indices,
        "cluster_data": final_verified_rings,
        "thresh_img": thresh,
        "scale_factor": s_init  
    }
    return output, None

# ==========================================
# 4. 传统数字标准化与多角度模板匹配算法 (抗大角度旋转)
# ==========================================
def standardize_digit(roi_bin):
    """通过查找最大外接矩形对数字完成缩放和完美质心对齐"""
    contours, _ = cv2.findContours(roi_bin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    standard_canvas = np.zeros((64, 64), dtype=np.uint8)
    
    if not contours:
        return cv2.resize(roi_bin, (64, 64))
        
    largest_cnt = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_cnt)
    
    digit_crop = roi_bin[y:y+h, x:x+w]
    
    max_dim = max(w, h)
    if max_dim == 0:
        return standard_canvas
        
    scale = 46.0 / max_dim
    new_w = int(w * scale)
    new_h = int(h * scale)
    
    if new_w > 0 and new_h > 0:
        digit_resized = cv2.resize(digit_crop, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        start_x = (64 - new_w) // 2
        start_y = (64 - new_h) // 2
        standard_canvas[start_y:start_y+new_h, start_x:start_x+new_w] = digit_resized
        
    return standard_canvas

def generate_multi_angle_templates(angles=np.arange(-60, 61, 5)):
    """
    【升级优化点】：为 1, 2, 3 在多角度区间下（-60度到60度，步长5度）预生成全角度标准化模板库。
    使算法具备强大的大角度抗旋转特性，同时由于预先载入，推理时不会产生额外运算开销。
    """
    templates_bank = {}
    
    for char in ['1', '2', '3']:
        templates_bank[char] = {}
        
        # 1. 渲染无畸变的原始大字图样
        try:
            from PIL import Image, ImageDraw, ImageFont
            img = Image.new("L", (128, 128), 0)
            draw = ImageDraw.Draw(img)
            
            font_loaded = False
            for font_name in ["simhei.ttf", "msyh.ttf", "arial.ttf"]:
                try:
                    font = ImageFont.truetype(font_name, 80)
                    font_loaded = True
                    break
                except IOError:
                    continue
            if not font_loaded:
                font = ImageFont.load_default()
                
            draw.text((20, 20), char, fill=255, font=font)
            arr = np.array(img)
        except ImportError:
            arr = np.zeros((128, 128), dtype=np.uint8)
            cv2.putText(arr, char, (30, 95), cv2.FONT_HERSHEY_SIMPLEX, 3.2, 255, 9, cv2.LINE_AA)
            
        # 2. 旋转并进行几何标准化处理（确保旋转后的包围盒也能被放大居中）
        h, w = arr.shape[:2]
        center = (w // 2, h // 2)
        
        for angle in angles:
            M = cv2.getRotationMatrix2D(center, angle, 1.0)
            rotated_arr = cv2.warpAffine(arr, M, (w, h), flags=cv2.INTER_NEAREST)
            # 送入几何标准化函数
            templates_bank[char][angle] = standardize_digit(rotated_arr)
            
    return templates_bank

def classify_digit_via_multi_angle(std_roi, templates_bank):
    """
    【升级优化点】：遍历多角度模板库。
    在 75 种状态组合中通过极速逻辑 And/Or 运算寻找最大 IoU 分数，输出分类、匹配角与相似度。
    """
    best_char = None
    best_angle = None
    best_score = -1.0
    all_char_best_scores = {'1': -1.0, '2': -1.0, '3': -1.0}
    
    for char, angle_dict in templates_bank.items():
        for angle, temp in angle_dict.items():
            intersection = np.logical_and(std_roi, temp).sum()
            union = np.logical_or(std_roi, temp).sum()
            iou = intersection / union if union > 0 else 0.0
            
            # 记录当前类别的最高分数（用于雷达评分呈现）
            if iou > all_char_best_scores[char]:
                all_char_best_scores[char] = iou
                
            # 全局最优解更新
            if iou > best_score:
                best_score = iou
                best_char = char
                best_angle = angle
                
    return best_char, best_angle, best_score, all_char_best_scores

# ==========================================
# 5. 主流程控制：随机选图推理、耗时统计与可视化
# ==========================================
def main():
    if not os.path.exists(IMAGE_DIR):
        print(f"错误：未找到测试文件夹 {IMAGE_DIR}，请确保前一节的代码已运行并成功生成了数据集")
        return
        
    img_files = [f for f in os.listdir(IMAGE_DIR) if f.lower().endswith(('.jpg', '.png'))]
    if not img_files:
        print(f"错误：文件夹 {IMAGE_DIR} 中没有测试图像")
        return
        
    # 随机挑选一张图片
    selected_file = random.choice(img_files)
    img_path = os.path.join(IMAGE_DIR, selected_file)
    print(f"随机选取的测试图像: {selected_file}")
    
    # 自动解析图像的真值标签
    gt_digit = parse_gt_digit(selected_file)
    gt_center = parse_gt_center(selected_file)
    
    # 加载灰度图
    img_bgr = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img_bgr is None:
        print("错误：无法成功加载图像")
        return
    gray_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    H, W = gray_img.shape[:2]
    
    # 预先生成多角度模板库（作为系统静态资源库，不计入推理延时）
    print("系统初始化：正在为您预生成 -60°~60° 全角度数字模板库...")
    angles_list = np.arange(-60, 61, 5)  # 以 5 度为步长
    templates_bank = generate_multi_angle_templates(angles_list)
    
    # ========================================================
    # ⏱️ 高精度单帧推理时延评测区 (开始)
    # ========================================================
    t_start = time.perf_counter()
    
    # 步骤 A：同心圆自适应极速定位中心
    metrics, err = process_frame_adaptive_fast(gray_img)
    if metrics is None:
        print(f"中心定位算法失败: {err}")
        return
    t_localized = time.perf_counter()
    
    # 步骤 B：基于定位出的尺度因子裁切数字 ROI
    cx_true, cy_true = metrics["true_center"]
    s_init = metrics["scale_factor"]
    
    inner_r_px = 26.5 / s_init
    crop_half_w = int(inner_r_px * 0.82)
    
    ymin = max(0, int(cy_true - crop_half_w))
    ymax = min(H, int(cy_true + crop_half_w))
    xmin = max(0, int(cx_true - crop_half_w))
    xmax = min(W, int(cx_true + crop_half_w))
    
    roi_gray = gray_img[ymin:ymax, xmin:xmax]
    
    # 对 ROI 自适应二值化 (反色处理) 
    _, roi_bin = cv2.threshold(roi_gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    
    # 对提取的数字二值图像进行标准化居中对齐
    std_roi = standardize_digit(roi_bin)
    t_preprocessed = time.perf_counter()
    
    # 步骤 C：多角度多类别 IoU 模板联合检索
    predicted_digit, pred_angle, matching_iou, match_scores = classify_digit_via_multi_angle(std_roi, templates_bank)
    t_classified = time.perf_counter()
    # ========================================================
    # ⏱️ 高精度单帧推理时延评测区 (结束)
    # ========================================================
    
    # 时延数据统计与换算
    dt_localize = (t_localized - t_start) * 1000
    dt_preprocess = (t_preprocessed - t_localized) * 1000
    dt_classify = (t_classified - t_preprocessed) * 1000
    dt_total = (t_classified - t_start) * 1000
    
    print("\n" + "="*50)
    print("      🚀 支持大角度旋转的同心圆与数字多级推理分析报告      ")
    print("="*50)
    print(f"【阶段时延解析 (Stage Latencies)】:")
    print(f" 1. 亚像素同心圆定位耗时 : {dt_localize:.2f} ms")
    print(f" 2. ROI 裁剪与几何标准化  : {dt_preprocess:.2f} ms")
    print(f" 3. 旋转自适应 IoU 匹配耗时 : {dt_classify:.2f} ms")  # 即使检索了75种组合，耗时依然极低
    print(f" ------------------------------------")
    print(f" 💡 【单帧端到端总时延】 : {dt_total:.2f} ms")
    print("-" * 50)
    print(f"【推理与分类结论 (Inference Results)】:")
    print(f" 真值标签 (Ground Truth)  : {gt_digit}")
    print(f" 算法预测 (Prediction)    : {predicted_digit}  (置信度 IoU: {matching_iou:.4f})")
    print(f" 估算旋转偏角 (Orientation): {pred_angle}°")
    print(f" 各数字的最优匹配 IoU 列表: '1': {match_scores['1']:.4f}, '2': {match_scores['2']:.4f}, '3': {match_scores['3']:.4f}")
    if gt_digit == predicted_digit:
        print(" 🎉 识别匹配成功！")
    else:
        print(" ❌ 识别匹配出现偏差。")
    print("="*50)
    
    # ==========================================
    # 可视化三窗联合呈现
    # ==========================================
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    img_show_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    axes[0].imshow(img_show_rgb)
    axes[0].plot(cx_true, cy_true, 'r+', markersize=14, label='Corrected Center')
    rect = plt.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, edgecolor='lime', facecolor='none', linewidth=2, label='Digit ROI')
    axes[0].add_patch(rect)
    axes[0].set_title(f"Original Image & ROI Cutout\n(Localization: {dt_localize:.1f}ms)")
    axes[0].legend()
    axes[0].axis('off')
    
    axes[1].imshow(std_roi, cmap='gray')
    axes[1].set_title(f"Standardized ROI (64x64)\n(Binarization & Centering: {dt_preprocess:.1f}ms)")
    axes[1].axis('off')
    
    # 提取预测角度对应的数字模板
    matched_template = templates_bank[predicted_digit][pred_angle]
    axes[2].imshow(matched_template, cmap='gray')
    axes[2].set_title(
        f"Matched: '{predicted_digit}' at {pred_angle}° (Classification: {dt_classify:.1f}ms)\n"
        f"IoU Scores:\n"
        f"'1': {match_scores['1']:.3f}, '2': {match_scores['2']:.3f}, '3': {match_scores['3']:.3f}"
    )
    axes[2].axis('off')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()