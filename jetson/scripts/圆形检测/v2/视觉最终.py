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
IMAGE_DIR = r"F:\Project\littleCar2\zhengdian\4-29\jetson\scripts\圆形检测\v2\circle"

if not os.path.exists(IMAGE_DIR):
    IMAGE_DIR = os.path.join(os.getcwd(), "circle")
    print(f"提示：未找到指定绝对路径，切换至当前目录：{IMAGE_DIR}")

# 6圈同心圆设计的名义外径（Nominal Radii）
R_NOMINAL = np.array([28.0, 30.5, 35.5, 40.2, 44.9, 49.6])

# ==========================================
# 2. 优化后的拟合与单应性计算
# ==========================================
def parse_gt_center(filename):
    """从文件名解析真实中心点坐标 (真值)"""
    match = re.search(r"\((\d+),\s*(\d+)\)", filename)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None

def fit_ellipse_model(pts):
    """极速代数椭圆拟合 (用于初筛)"""
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
    
    # RANSAC 下采样。
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
        
        # 基础几何过滤
        if w <= 0 or h <= 0 or np.isnan(cx) or np.isnan(cy):
            continue
        if max(w, h) > d_min * 0.95 or min(w, h) < 10.0:
            continue
        if min(w, h) / max(w, h) < 0.50:
            continue
            
        # 局部极坐标投影变换
        angle_rad = np.radians(angle_deg)
        cos_a = np.cos(-angle_rad)
        sin_a = np.sin(-angle_rad)
        
        dx = pts_eval[:, 0] - cx
        dy = pts_eval[:, 1] - cy
        x_local = dx * cos_a - dy * sin_a
        y_local = dx * sin_a + dy * cos_a
        
        # 极速极坐标距离计算 (无三角函数)
        a = w / 2.0
        b = h / 2.0
        a_val = a if a > 1e-6 else 1e-6
        b_val = b if b > 1e-6 else 1e-6
        
        r_pts = np.sqrt(x_local**2 + y_local**2)
        Q = np.sqrt((x_local / a_val)**2 + (y_local / b_val)**2)
        Q = np.where(Q < 1e-6, 1e-6, Q)
        
        dists = r_pts * np.abs(1.0 - 1.0 / Q)
        inliers = np.where(dists < threshold)[0]
        
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_model = model
            
    if len(best_inliers) >= 5:
        # 完整映射确保精度
        (cx, cy), (w, h), angle_deg = best_model
        angle_rad = np.radians(angle_deg)
        cos_a = np.cos(-angle_rad)
        sin_a = np.sin(-angle_rad)
        
        dx = pts[:, 0] - cx
        dy = pts[:, 1] - cy
        x_local = dx * cos_a - dy * sin_a
        y_local = dx * sin_a + dy * cos_a
        
        a, b = w / 2.0, h / 2.0
        a_val = a if a > 1e-6 else 1e-6
        b_val = b if b > 1e-6 else 1e-6
        
        r_pts = np.sqrt(x_local**2 + y_local**2)
        Q = np.sqrt((x_local / a_val)**2 + (y_local / b_val)**2)
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
    """
    【新增优化点】：通过组合数检索，将测量半径精准匹配到 R_NOMINAL 的对应子集上。
    即使存在圈数截断（如只取最大的3圈）或中间漏检，也能实现完全精准的识别并输出正确的初始尺度。
    """
    K = len(measured_radii)
    if K == 0:
        return [], 1.0
        
    best_err = float('inf')
    best_indices = []
    best_scale = 1.0
    
    # 穷举 0~5 的所有长度为 K 的组合（通常 6 选 3 只有 20 种组合，计算时间 < 0.1ms）
    all_combinations = list(itertools.combinations(range(len(R_NOMINAL)), K))
    
    for comb in all_combinations:
        candidate_nominal = R_NOMINAL[list(comb)]
        
        # 计算当前组合对应的最佳比例尺度 s (物理单位 mm / 像素 px)
        scales = candidate_nominal / measured_radii
        s_cand = np.mean(scales)
        
        # 评估相对投影误差
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
    """
    非线性单应性变换校正
    【修复优化点】：传入匹配出来的真实初值 s_init，让单应性优化稳定在零均值初始残差下工作，杜绝发散。
    """
    cx, cy = rough_center
    s = s_init  # 替换原有固定的 1.0，使用准确估计出的尺度
    H_init = np.array([
        [s, 0, -s*cx],
        [0, s, -s*cy],
        [0, 0, 1.0]
    ])
    h0 = H_init.flatten()[:8]
    
    # 极致下采样加速
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
# 3. 极速自适应推理流水线
# ==========================================
def process_frame_adaptive_fast(gray_img):
    """极速自适应检测核心算法 (漏洞修复版)"""
    h_img, w_img = gray_img.shape[:2]
    d_min = min(h_img, w_img)
    total_pixels = h_img * w_img
    
    # 动态参数计算
    adaptive_block_size = int(d_min * 0.16) | 1  
    adaptive_min_area = int(total_pixels * 0.0012)
    adaptive_ransac_thresh = max(1.0, d_min * 0.00375)
    
    # 1. 局部自适应阈值化
    thresh = cv2.adaptiveThreshold(
        gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
        cv2.THRESH_BINARY_INV, adaptive_block_size, 15
    )
    
    # 2. 提取拓扑轮廓
    contours, _ = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    
    # 3. 极速初筛选
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
                # 基本物理约束过滤
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
        
    # 4. 空间拓扑中心聚类
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
        
    # 5. 去除同轴环冗余边缘并截断
    best_cluster = sorted(best_cluster, key=lambda x: max(x["model"][1]))  # 按长轴升序排序
    deduped_cluster = []
    for el in best_cluster:
        if not deduped_cluster:
            deduped_cluster.append(el)
        else:
            prev_size = max(deduped_cluster[-1]["model"][1])
            curr_size = max(el["model"][1])
            if (curr_size - prev_size) > (d_min * 0.025):
                deduped_cluster.append(el)

    # 提速截断：如果圈数多于 3 圈，我们只保留最大的 3 圈进行 RANSAC 和单应性校正
    if len(deduped_cluster) > 3:
        best_cluster = deduped_cluster[-3:]  # 取最大的3个外圈
    else:
        best_cluster = deduped_cluster
    
    # 6. 延迟 RANSAC 精拟合
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
        
    # 7. 【修复核心点】：高精确度模板索引组合匹配，同时估算最优初值尺度
    measured_radii = np.array([max(el["model"][1])/2.0 for el in final_verified_rings])
    ring_indices, s_init = match_rings_to_nominal(measured_radii)
        
    # 8. 【修复核心点】：使用合理的 s_init 引导非线性校正，输出真实的透视中心投影
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
        "thresh_img": thresh
    }
    return output, None

# ==========================================
# 5. 主执行与规范化 Benchmark 评估
# ==========================================
def main():
    if not os.path.exists(IMAGE_DIR):
        print(f"错误：未找到路径 {IMAGE_DIR}")
        return
        
    img_files = [f for f in os.listdir(IMAGE_DIR) if f.lower().endswith(('.jpg', '.png'))]
    if not img_files:
        print(f"错误：未找到测试图像")
        return
        
    test_file = random.choice(img_files)
    img_path = os.path.join(IMAGE_DIR, test_file)
    gt_center = parse_gt_center(test_file)
    
    # 图像加载
    img_array = np.fromfile(img_path, dtype=np.uint8)
    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
    if img is None:
        print("错误：图像加载失败")
        return
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 1. 预热运行 (Warmup)
    print("系统正在进行 10 次预热运行...")
    for _ in range(10):
        _, _ = process_frame_adaptive_fast(gray_img)
        
    # 2. 规范化时延 Benchmark
    print("正在进行 100 次基准测试推理...")
    latencies = []
    for _ in range(100):
        t0 = time.perf_counter()
        res, err = process_frame_adaptive_fast(gray_img)
        t1 = time.perf_counter()
        if err is None:
            latencies.append((t1 - t0) * 1000)
            
    if not latencies:
        print("基准测试期间检测全部失败！")
        return
        
    avg_latency = np.mean(latencies)
    p95_latency = np.percentile(latencies, 95)
    max_latency = np.max(latencies)
    
    metrics, err = process_frame_adaptive_fast(gray_img)
    if metrics is None:
        print(f"检测失败: {err}")
        return
        
    W, H = metrics["img_shape"]
    d_min = min(W, H)
    cx_rough, cy_rough = metrics["rough_center"]
    cx_true, cy_true = metrics["true_center"]
    
    print("\n" + "="*50)
    print("       🚀 修复与极速优化版：性能与精度评估报告       ")
    print("="*50)
    print(f"图像尺寸: {W} x {H}")
    print(f"去重过滤后通过验证的同轴环数: {metrics['ring_count']} 圈 (自动精准匹配的名义索引: {metrics['matched_indices']})")
    print("-" * 50)
    print(f"平均算法延时 (Mean Latency): {avg_latency:.2f} ms")
    print(f"【P95 算法延时】(P95 Latency) : {p95_latency:.2f} ms")
    print(f"最大单次延时 (Max Latency)  : {max_latency:.2f} ms")
    print("-" * 50)
    print(f"椭圆几何中心 (带有透视偏差) : X={cx_rough:.4f}, Y={cy_rough:.4f}")
    print(f"单应性校正圆心 (理论真投影) : X={cx_true:.4f}, Y={cy_true:.4f}")
    
    perspective_shift = np.sqrt((cx_rough - cx_true)**2 + (cy_rough - cy_true)**2)
    print(f"透视引起的中心漂移修正量   : {perspective_shift:.4f} 像素")
    
    if gt_center:
        err_rough = np.sqrt((cx_rough - gt_center[0])**2 + (cy_rough - gt_center[1])**2)
        err_true = np.sqrt((cx_true - gt_center[0])**2 + (cy_true - gt_center[1])**2)
        print(f"未校正几何中心对真值误差   : {err_rough:.4f} 像素")
        print(f"单应性校正后圆心对真值误差 : {err_true:.4f} 像素")
    print("="*50)

    # 可视化绘图
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    axes[0].imshow(img_rgb)
    if gt_center:
        axes[0].plot(gt_center[0], gt_center[1], 'go', markersize=8, label='Ground Truth')
    axes[0].plot(cx_rough, cy_rough, 'bx', markersize=8, label='Ellipse Center (Biased)')
    axes[0].plot(cx_true, cy_true, 'r+', markersize=10, label='True Center (Corrected)')
    axes[0].set_title("Original Image")
    axes[0].legend()
    axes[0].axis('off')
    
    axes[1].imshow(metrics["thresh_img"], cmap='gray')
    axes[1].set_title("Adaptive Thresh")
    axes[1].axis('off')
    
    contour_img = np.zeros((H, W, 3), dtype=np.uint8) + 240
    for el in metrics["cluster_data"]:
        for pt in el["inliers"]:
            cv2.circle(contour_img, (int(pt[0]), int(pt[1])), max(1, int(d_min*0.003)), (100, 200, 100), -1)
        cv2.ellipse(contour_img, el["model"], (180, 50, 50), max(1, int(d_min*0.003)))
        
    axes[2].imshow(contour_img)
    axes[2].plot(cx_true, cy_true, 'r+', markersize=12, label='Corrected Center')
    axes[2].set_title(f"Concentric Structure ({metrics['ring_count']} Rings)")
    axes[2].legend()
    axes[2].axis('off')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()