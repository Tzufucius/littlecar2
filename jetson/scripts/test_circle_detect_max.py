import cv2
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont
import os
import re
from scipy.signal import find_peaks
from scipy.ndimage import gaussian_filter1d

plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'sans-serif']
plt.rcParams['axes.unicode_minus'] = False

def draw_txt(img, text, pos, color=(0,0,255), size=20):
    img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    try: font = ImageFont.truetype("simhei.ttf", size)
    except: font = ImageFont.load_default()
    ImageDraw.Draw(img_pil).text(pos, text, font=font, fill=color)
    return cv2.cvtColor(np.array(img_pil), cv2.COLOR_RGB2BGR)

def process_subpixel_lsq(image_path, p):
    # --- 0. 解析真值 ---
    filename = os.path.basename(image_path)
    match = re.search(r'\((\d+),\s*(\d+)\)', filename)
    gt_x, gt_y = (int(match.group(1)), int(match.group(2))) if match else (0, 0)
    
    raw_img = cv2.imread(image_path)
    if raw_img is None: return
    gray = cv2.cvtColor(raw_img, cv2.COLOR_BGR2GRAY)
    plots = {}

    # ================= 阶段 1：粗略定位 (获取可靠 ROI) =================
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    img_norm = clahe.apply(gray)
    img_denoise = cv2.bilateralFilter(img_norm, d=9, sigmaColor=50, sigmaSpace=50)
    edges = cv2.Canny(img_denoise, p['canny_low'], p['canny_high'])
    
    # 清理噪点
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(edges)
    edges_clean = np.zeros_like(edges)
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] >= p['min_noise_area']:
            edges_clean[labels == i] = 255
            
    # 大核粗略重心
    kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (60, 60))
    blob = cv2.dilate(edges_clean, kernel_large)
    contours, _ = cv2.findContours(blob, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    rough_cx, rough_cy = 0, 0
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            rough_cx, rough_cy = int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])
            
    # 如果粗定位失败，提前退出
    if rough_cx == 0: return print("粗定位失败")
    plots['1_edges'] = (edges_clean, f"1. 纯净边缘与粗定位 ({rough_cx},{rough_cy})")

    # ================= 阶段 2：径向距离分组 (解剖圆环) =================
    # 获取所有纯净边缘点坐标 (N, 2)
    y_coords, x_coords = np.where(edges_clean > 0)
    edge_pts = np.column_stack((x_coords, y_coords))
    
    # 计算所有边缘点到【粗中心】的距离
    distances = np.linalg.norm(edge_pts - [rough_cx, rough_cy], axis=1)
    
    # 建立距离直方图 (横坐标是半径距离，纵坐标是像素数量)
    hist, bin_edges = np.histogram(distances, bins=np.arange(0, p['max_radius'], 1))
    
    # 高斯平滑直方图，防止一个环产生多个锯齿假波峰
    smooth_hist = gaussian_filter1d(hist, sigma=2)
    
    # 寻找波峰 (这些波峰就是每一个圆环的粗略半径)
    peaks, _ = find_peaks(smooth_hist, height=p['peak_height'], distance=15)
    ring_radii = bin_edges[peaks]
    
    # 绘制直方图可视化
    fig_hist, ax_hist = plt.subplots(figsize=(6, 4))
    ax_hist.plot(bin_edges[:-1], smooth_hist, color='blue', label="距离分布")
    ax_hist.plot(ring_radii, smooth_hist[peaks], "x", color='red', markersize=10, label="识别出的圆环")
    ax_hist.set_title(f"2. 径向分组解剖: 找到 {len(ring_radii)} 个环")
    ax_hist.legend()
    fig_hist.canvas.draw()
    img_hist = np.frombuffer(fig_hist.canvas.tostring_rgb(), dtype=np.uint8)
    img_hist = img_hist.reshape(fig_hist.canvas.get_width_height()[::-1] + (3,))
    plt.close(fig_hist)
    plots['2_hist'] = (img_hist, "2. 边缘点的径向距离分布直方图")

    # ================= 阶段 3：亚像素最小二乘拟合 (LSQ) =================
    vis_fit = raw_img.copy()
    refined_centers =[]
    
    # 遍历每一个被发现的环
    for i, r in enumerate(ring_radii):
        # 提取属于这个环的边缘点：距离半径 r 误差在 fit_margin 范围内的点
        margin = p['fit_margin']
        mask = (distances >= r - margin) & (distances <= r + margin)
        ring_pts = edge_pts[mask]
        
        # 只要点足够多 (比如超过30个点)，就进行椭圆/圆拟合
        if len(ring_pts) > 30:
            # 转换形状满足 cv2.fitEllipse 需求 (N, 1, 2)
            pts_for_fit = ring_pts.reshape(-1, 1, 2).astype(np.float32)
            
            # 【核心数学函数】最小二乘法拟合，返回 ((cx, cy), (a, b), angle)
            ellipse = cv2.fitEllipse(pts_for_fit)
            (cx, cy) = ellipse[0]
            refined_centers.append([cx, cy])
            
            # 可视化：用不同颜色画出分离出来的环，并画出拟合结果
            color = tuple(np.random.randint(50, 255, 3).tolist())
            for pt in ring_pts:
                cv2.circle(vis_fit, (pt[0], pt[1]), 1, color, -1)
            cv2.ellipse(vis_fit, ellipse, (0, 255, 0), 1)

    plots['3_fit'] = (vis_fit, f"3. 单独环最小二乘拟合 ({len(refined_centers)}个有效环)")

    # ================= 阶段 4：同心聚合与终极评估 =================
    final_img = raw_img.copy()
    if len(refined_centers) > 0:
        # 去除极端的离群值 (如果在拟合过程中某个环被噪点带偏)
        centers_arr = np.array(refined_centers)
        median_c = np.median(centers_arr, axis=0)
        valid_c =[]
        for c in centers_arr:
            if np.linalg.norm(c - median_c) < p['outlier_tol']:
                valid_c.append(c)
                
        if len(valid_c) > 0:
            # 最终的亚像素圆心！
            final_cx = np.mean([c[0] for c in valid_c])
            final_cy = np.mean([c[1] for c in valid_c])
            
            # 计算误差
            error = np.sqrt((final_cx - gt_x)**2 + (final_cy - gt_y)**2)
            
            # 绘制极细的准星
            cv2.drawMarker(final_img, (int(final_cx), int(final_cy)), (0,0,255), cv2.MARKER_CROSS, 40, 2)
            cv2.drawMarker(final_img, (gt_x, gt_y), (255,0,0), cv2.MARKER_SQUARE, 30, 2)
            
            info_sub = f"高精定位:({final_cx:.2f}, {final_cy:.2f})"
            info_err = f"误差: {error:.2f} px"
            final_img = draw_txt(final_img, info_sub, (int(final_cx) + 20, int(final_cy) - 50), size=25)
            final_img = draw_txt(final_img, info_err, (int(final_cx) + 20, int(final_cy) - 20), size=25)
            print(f"【高精拟合完成】 坐标: ({final_cx:.2f}, {final_cy:.2f}) | 误差: {error:.2f} 像素")
        else:
            final_img = draw_txt(final_img, "圆心离群过滤失败", (50, 50))
    else:
        final_img = draw_txt(final_img, "无有效环被拟合", (50, 50))

    plots['4_final'] = (final_img, "4. 最终高精同心聚合")

    # --- 统一展示 ---
    plt.figure(figsize=(18, 10))
    keys = ['1_edges', '2_hist', '3_fit', '4_final']
    for idx, key in enumerate(keys):
        plt.subplot(2, 2, idx+1)
        img, title = plots[key]
        if len(img.shape) == 2: plt.imshow(img, cmap='gray')
        else: plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        plt.title(title, fontsize=14, fontweight='bold')
        plt.axis('off')
    plt.tight_layout()
    plt.show()

def main():
    params = {
        "canny_low": 40,
        "canny_high": 120,
        "min_noise_area": 50,    # 清除孤立的小噪点
        
        # --- 径向解剖参数 ---
        "max_radius": 500,       # 相机视野内的最大可能半径
        "peak_height": 100,      # 直方图波峰最小高度（一个环至少得有 100 个边缘像素）
        
        # --- 拟合参数 ---
        "fit_margin": 6,         # 容差范围：在识别出的半径 ±6 像素内的点，归属于同一个环
        "outlier_tol": 10        # 聚合时，如果某个环拟合的中心偏离大部队超过 10 像素，剔除它
    }

    img_addr = r"F:\Project\littleCar2\zhengdian\4-29\jetson\scripts\circle\(567, 685).jpg"
    process_subpixel_lsq(img_addr, params)

if __name__ == "__main__":
    main()