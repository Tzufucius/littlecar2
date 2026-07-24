import os
import cv2
import numpy as np
import random

# ==================== 配置区域 ====================
NUM_IMAGES = 20  # 您可以自由修改需要生成的图片数量
OUTPUT_DIR = os.path.join("assets", "彩色物料")
IMAGE_SIZE = (640, 640)  # 生成图像的分辨率 (宽, 高)
# ==================================================

# 颜色映射表 (BGR格式)
# 1 红色、2 黄色、3 蓝色、4 绿色、5 黑色、6 浅蓝
COLOR_MAP = {
    1: {"name": "Red", "main": (25, 20, 220), "skirt": (10, 5, 120)},
    2: {"name": "Yellow", "main": (0, 215, 245), "skirt": (0, 130, 150)},
    3: {"name": "Blue", "main": (210, 45, 10), "skirt": (100, 20, 5)},
    4: {"name": "Green", "main": (35, 180, 20), "skirt": (15, 90, 10)},
    5: {"name": "Black", "main": (35, 35, 35), "skirt": (15, 15, 15)},
    6: {"name": "LightBlue", "main": (245, 175, 10), "skirt": (150, 100, 5)}
}


def save_image_robust(img, file_path):
    """
    支持中文路径的安全图像保存函数
    """
    try:
        dir_name = os.path.dirname(file_path)
        if not os.path.exists(dir_name):
            os.makedirs(dir_name, exist_ok=True)
        is_success, im_buf_arr = cv2.imencode(".jpg", img)
        if is_success:
            im_buf_arr.tofile(file_path)
        else:
            print(f"内存编码失败: {file_path}")
    except Exception as e:
        print(f"保存文件时出错 {file_path}: {e}")


def draw_irregular_shadow_set(mask, width, height, angle_rad):
    """
    在指定的单通道掩膜上，沿特定角度方向绘制一组宽度高度不一、间距随机的平行条带
    """
    cx, cy = width // 2, height // 2
    perp_angle = angle_rad + np.pi / 2  # 垂直于条带的步进轴
    dx = np.cos(perp_angle)
    dy = np.sin(perp_angle)
    
    start_dist = -int(max(width, height) * 1.0)
    end_dist = int(max(width, height) * 1.0)
    
    curr_dist = start_dist
    lx = np.cos(angle_rad)
    ly = np.sin(angle_rad)
    length = max(width, height) * 3.5
    
    while curr_dist < end_dist:
        # 宽度大跨度随机化：混合极细线、中等条带与极宽阴影板
        curr_band_width = random.choice([
            random.randint(15, 35),    # 细条干扰
            random.randint(40, 80),    # 中等条带
            random.randint(95, 165)    # 宽大投影
        ])
        
        px = cx + dx * curr_dist
        py = cy + dy * curr_dist
        
        p1 = (int(px - lx * length), int(py - ly * length))
        p2 = (int(px + lx * length), int(py + ly * length))
        
        # 绘制阴影，这里带有一些随机的亮度强弱变化，模拟光线散射不均
        intensity = random.randint(160, 255)
        cv2.line(mask, p1, p2, intensity, curr_band_width)
        
        # 间距（Gap）大幅随机化，打破均匀分布
        gap = random.randint(30, 240)
        curr_dist += curr_band_width + gap


def generate_white_harsh_background(width, height):
    """
    生成带有工业感光噪点、暗角以及多角度交错、宽窄不一斜向阴影条带的白色背景
    """
    # 1. 初始化高亮度的亮白色背景
    base_white = random.randint(242, 248)
    bg = np.ones((height, width, 3), dtype=np.uint8) * base_white

    # 2. 生成第一组阴影
    mask1 = np.zeros((height, width), dtype=np.uint8)
    angle1 = np.radians(random.uniform(-55, 55))
    draw_irregular_shadow_set(mask1, width, height, angle1)

    # 3. 生成第二组阴影 (强制与第一组产生至少 40 度夹角，实现明显的交错效果)
    mask2 = np.zeros((height, width), dtype=np.uint8)
    angle_offset = random.choice([random.uniform(40, 85), random.uniform(-85, -40)])
    angle2 = angle1 + np.radians(angle_offset)
    draw_irregular_shadow_set(mask2, width, height, angle2)

    # 4. 混合两组阴影，使其在交叉处产生更深的重叠影
    shadow_mask = cv2.addWeighted(mask1, 0.65, mask2, 0.65, 0)

    # 5. 超大核高斯模糊，让条带边界极为柔和
    blur_k = random.choice([85, 105, 125])
    shadow_mask_blurred = cv2.GaussianBlur(shadow_mask, (blur_k, blur_k), 0)
    
    # 6. 将合并后的不均匀交错阴影应用到白底板上
    shadow_opacity = random.uniform(0.18, 0.28)
    alpha = (shadow_mask_blurred / 255.0) * shadow_opacity
    
    for c in range(3):
        target_shadow_color = 45  # 阴影处的基色
        bg[:, :, c] = (bg[:, :, c] * (1.0 - alpha) + target_shadow_color * alpha).astype(np.uint8)

    # 7. 模拟工业镜头边缘暗角
    X, Y = np.meshgrid(np.arange(width), np.arange(height))
    dist_from_center = np.sqrt((X - width / 2) ** 2 + (Y - height / 2) ** 2)
    max_dist = np.sqrt((width / 2) ** 2 + (height / 2) ** 2)
    vignette = 1.0 - random.uniform(0.05, 0.11) * (dist_from_center / max_dist)
    for c in range(3):
        bg[:, :, c] = np.clip(bg[:, :, c] * vignette, 0, 255).astype(np.uint8)

    return bg


def draw_geometric_interference(img, width, height, draw_on_top=False):
    """
    绘制穿插于画面上下层级的随机标定线和辅助检测框
    """
    temp_layer = img.copy()

    # 彩色干涉长线 (如激光、乱线)
    num_lines = random.randint(3, 5) if draw_on_top else random.randint(4, 7)
    for _ in range(num_lines):
        p1 = (random.randint(-50, width + 50), random.randint(-50, height + 50))
        p2 = (random.randint(-50, width + 50), random.randint(-50, height + 50))
        color = (random.randint(60, 220), random.randint(60, 220), random.randint(60, 220))
        thickness = random.randint(1, 3)
        cv2.line(temp_layer, p1, p2, color, thickness, cv2.LINE_AA)

    # 干扰几何线框 (模拟其他零件轮廓或辅助框)
    num_rects = random.randint(2, 4) if draw_on_top else random.randint(3, 6)
    for _ in range(num_rects):
        x = random.randint(20, width - 150)
        y = random.randint(20, height - 150)
        w = random.randint(30, 130)
        h = random.randint(20, 130)
        color = (random.randint(50, 210), random.randint(50, 210), random.randint(50, 210))
        thickness = random.randint(1, 2)
        cv2.rectangle(temp_layer, (x, y), (x + w, y + h), color, thickness, cv2.LINE_AA)

    alpha = random.uniform(0.30, 0.55)
    cv2.addWeighted(temp_layer, alpha, img, 1 - alpha, 0, dst=img)


def draw_cone_with_effects(img, center, color_code, scale):
    """
    在图像上绘制带有立体裙边、椭圆扭曲和表面细节的物料
    """
    cx, cy = center
    col_info = COLOR_MAP[color_code]

    R_base = int(58 * scale)
    R_top = int(37 * scale)

    axis_ratio_x = random.uniform(0.91, 0.99)
    axis_ratio_y = random.uniform(0.91, 0.99)
    rot_angle = random.uniform(0, 360)

    shift_dx = int(random.uniform(-8, 8) * scale)
    shift_dy = int(random.uniform(-8, 8) * scale)
    top_center = (cx + shift_dx, cy + shift_dy)

    # --- 1. 投影绘制 ---
    shadow_mask = np.zeros_like(img, dtype=np.uint8)
    shadow_center = (cx + int(12 * scale), cy + int(15 * scale))
    shadow_axes = (int(R_base * axis_ratio_x * 1.05), int(R_base * axis_ratio_y * 0.95))
    cv2.ellipse(shadow_mask, shadow_center, shadow_axes, rot_angle, 0, 360, (20, 20, 20), -1)

    blur_k = int(35 * scale) | 1
    shadow_blur = cv2.GaussianBlur(shadow_mask, (blur_k, blur_k), 0)

    gray_shadow = cv2.cvtColor(shadow_blur, cv2.COLOR_BGR2GRAY)
    alpha_shadow = (gray_shadow / 255.0) * random.uniform(0.40, 0.55)
    for c in range(3):
        img[:, :, c] = (img[:, :, c] * (1.0 - alpha_shadow) + 10 * alpha_shadow).astype(np.uint8)

    # --- 2. 渐变裙边 ---
    steps = 25
    main_col = np.array(col_info["main"])
    skirt_col = np.array(col_info["skirt"])

    for i in range(steps):
        t = i / (steps - 1)
        r_curr = R_base - t * (R_base - R_top)
        curr_cx = int(cx + t * shift_dx)
        curr_cy = int(cy + t * shift_dy)

        curr_col = skirt_col * (1.0 - t) + main_col * t
        axes_curr = (int(r_curr * axis_ratio_x), int(r_curr * axis_ratio_y))

        cv2.ellipse(img, (curr_cx, curr_cy), axes_curr, rot_angle, 0, 360, tuple(map(int, curr_col)), -1)

    # --- 3. 顶面平面 ---
    top_axes = (int(R_top * axis_ratio_x), int(R_top * axis_ratio_y))
    cv2.ellipse(img, top_center, top_axes, rot_angle, 0, 360, col_info["main"], -1)
    cv2.ellipse(img, top_center, top_axes, rot_angle, 0, 360, col_info["skirt"], 1)

    # --- 4. 高光区 ---
    hl_mask = np.zeros_like(img, dtype=np.uint8)
    top_hl_center = (top_center[0] - int(7 * scale), top_center[1] - int(7 * scale))
    top_hl_axes = (int(12 * scale), int(6 * scale))
    cv2.ellipse(hl_mask, top_hl_center, top_hl_axes, rot_angle - 45, 0, 360, (255, 255, 255), -1)

    hl_blur_k = int(15 * scale) | 1
    hl_blur = cv2.GaussianBlur(hl_mask, (hl_blur_k, hl_blur_k), 0)

    gray_hl = cv2.cvtColor(hl_blur, cv2.COLOR_BGR2GRAY)
    alpha_hl = (gray_hl / 255.0) * random.uniform(0.15, 0.30)
    for c in range(3):
        img[:, :, c] = np.clip(img[:, :, c] * (1.0 - alpha_hl) + 255 * alpha_hl, 0, 255).astype(np.uint8)


def apply_heavy_camera_noise_on_white(img):
    """
    针对高亮/白色图像环境深度优化的感光粒子噪声与黑白噪点
    """
    height, width, _ = img.shape

    # 1. 强高斯彩色各通道独立噪声
    noise_sigma = random.uniform(34.0, 46.0)
    noise = np.random.normal(0, noise_sigma, img.shape).astype(np.int16)
    img_noisy = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    # 2. 强感光椒盐噪点 (分配 70% 的深色 Pepper 噪声与 30% 的亮色 Salt 噪声，提供完美的白色颗粒质感)
    prob = random.uniform(0.045, 0.08)  # 噪声系数
    rnd = np.random.rand(height, width)
    
    img_noisy[rnd < prob * 0.7] = [0, 0, 0]
    img_noisy[rnd > 1 - prob * 0.3] = [255, 255, 255]

    return img_noisy


def generate_dataset(num_images, output_dir):
    """
    数据集生成主流程
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    print(f"正在模拟交错光影条带的恶劣白底环境生成数据集，目标: {num_images} 张图片...")
    width, height = IMAGE_SIZE

    for idx in range(num_images):
        # 1. 产生带有极度斜向交错、宽窄不一阴影条带的白背景板
        img = generate_white_harsh_background(width, height)

        # 2. 绘制物料下层几何干扰 (线框、划痕)
        draw_geometric_interference(img, width, height, draw_on_top=False)

        # 3. 选取三种不重复的随机色块
        available_colors = list(COLOR_MAP.keys())
        selected_colors = random.sample(available_colors, 3)

        placed_objects = []

        for color_code in selected_colors:
            scale = random.uniform(0.70, 1.25)
            R_base = int(58 * scale)

            attempts = 0
            placed = False

            while attempts < 200:
                margin = int(R_base * 1.35)
                cx = random.randint(margin, width - margin)
                cy = random.randint(margin, height - margin)

                collision = False
                for prev_cx, prev_cy, prev_scale in [obj[:3] for obj in placed_objects]:
                    prev_R_base = int(58 * prev_scale)
                    distance = np.sqrt((cx - prev_cx) ** 2 + (cy - prev_cy) ** 2)
                    if distance < (R_base + prev_R_base) * 1.35:
                        collision = True
                        break

                if not collision:
                    placed_objects.append((cx, cy, scale, color_code))
                    placed = True
                    break

                attempts += 1

            if not placed:
                placed_objects = []
                break

        if len(placed_objects) < 3:
            continue

        # 4. 从上到下绘制圆台物料
        placed_objects.sort(key=lambda o: o[1])
        for cx, cy, scale, color_code in placed_objects:
            draw_cone_with_effects(img, (cx, cy), color_code, scale)

        # 5. 绘制覆盖在上层的标定辅助线 (部分线、框压在物体上方)
        draw_geometric_interference(img, width, height, draw_on_top=True)

        # 6. 后处理：施加重度数字各通道彩噪与颗粒噪点，将所有图像层级进行深度杂糅
        img = apply_heavy_camera_noise_on_white(img)

        # 7. 格式化文件名与安全写入本地路径
        filename_parts = []
        for cx, cy, _, color_code in placed_objects:
            filename_parts.append(f"{color_code}({cx}, {cy})")

        filename = "".join(filename_parts) + ".jpg"
        full_path = os.path.join(output_dir, filename)

        save_image_robust(img, full_path)

        if (idx + 1) % 10 == 0 or (idx + 1) == num_images:
            print(f"完成进度: {idx + 1}/{num_images} 张图片已合成并保存。")

    print(f"\n交错光影噪点白底数据集已全部生成，保存路径: '{output_dir}'")


if __name__ == "__main__":
    generate_dataset(NUM_IMAGES, OUTPUT_DIR)