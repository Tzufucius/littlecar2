import os
import cv2
import numpy as np
import random

# ==================== 配置区域 ====================
NUM_IMAGES = 500
OUTPUT_DIR = os.path.join("assets", "彩色物料数据集")  # 自定义导出路径
IMAGE_SIZE = (640, 640)  # 生成图像的分辨率 (宽, 高)
TRAIN_RATIO = 0.7  # 训练集与验证集的比例 (7:3)

IDEAL_MODE_RATIO = 0.15  # 极佳环境比例 (15% 概率生成纯白背景、无高光纯色物料、0噪点干扰)
HIGHLIGHT_PROB = 0.45  # 高光开启概率 (45% 几率开启高光增亮处理，55% 几率关闭并仅保留底色)

# 缩放倍率自主调整
MIN_SCALE = 0.70  # 物料的最小缩放倍率
MAX_SCALE = 2.50  # 物料的最大缩放倍率 (若设得过大可能会因空间拥挤导致生成速度变慢)

# 新增：局部高斯模糊相关配置
REGIONAL_BLUR_PROB = 0.65  # 引入局部模糊的概率 (例如 50% 的几率会对某个宫格区域进行虚化处理)
BLUR_GRID_DIV = 3  # 宫格的划分数量 (例如 3 代表 3x3 九宫格，4 代表 4x4 的 16 宫格)
MIN_BLUR_KERNEL = 35  # 局部模糊强度的下限 (高斯核大小，值越大越模糊，必须为正奇数)
MAX_BLUR_KERNEL = 75  # 局部模糊强度的上限 (高斯核大小，值越大越模糊，必须为正奇数)
# ==================================================

# 颜色映射表 (BGR格式)
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
    perp_angle = angle_rad + np.pi / 2
    dx = np.cos(perp_angle)
    dy = np.sin(perp_angle)
    
    start_dist = -int(max(width, height) * 1.0)
    end_dist = int(max(width, height) * 1.0)
    
    curr_dist = start_dist
    lx = np.cos(angle_rad)
    ly = np.sin(angle_rad)
    length = max(width, height) * 3.5
    
    while curr_dist < end_dist:
        curr_band_width = random.choice([
            random.randint(15, 35),
            random.randint(40, 80),
            random.randint(95, 165)
        ])
        
        px = cx + dx * curr_dist
        py = cy + dy * curr_dist
        
        p1 = (int(px - lx * length), int(py - ly * length))
        p2 = (int(px + lx * length), int(py + ly * length))
        
        intensity = random.randint(160, 255)
        cv2.line(mask, p1, p2, intensity, curr_band_width)
        
        gap = random.randint(30, 240)
        curr_dist += curr_band_width + gap


def generate_white_harsh_background(width, height, ideal_mode=False):
    """
    生成背景：理想模式下返回纯白背景，非理想模式下返回带有亮度渐变、阴影和暗角的工业背景
    """
    if ideal_mode:
        return np.ones((height, width, 3), dtype=np.uint8) * 255

    base_white = random.randint(238, 246)
    bg = np.ones((height, width, 3), dtype=np.uint8) * base_white

    # 线性渐变
    grad_map = np.ones((height, width), dtype=np.float32)
    gradient_direction = random.choice(["top_bottom", "left_right", "diagonal"])
    grad_intensity = random.uniform(0.20, 0.40)
    
    if gradient_direction == "top_bottom":
        for y in range(height):
            grad_map[y, :] = 1.0 - (y / height) * grad_intensity
    elif gradient_direction == "left_right":
        for x in range(width):
            grad_map[:, x] = 1.0 - (x / width) * grad_intensity
    else:
        for y in range(height):
            for x in range(width):
                grad_map[y, x] = 1.0 - ((x + y) / (width + height)) * grad_intensity

    for c in range(3):
        bg[:, :, c] = (bg[:, :, c] * grad_map).astype(np.uint8)

    # 投影阴影
    mask1 = np.zeros((height, width), dtype=np.uint8)
    angle1 = np.radians(random.uniform(-55, 55))
    draw_irregular_shadow_set(mask1, width, height, angle1)

    mask2 = np.zeros((height, width), dtype=np.uint8)
    angle_offset = random.choice([random.uniform(40, 85), random.uniform(-85, -40)])
    angle2 = angle1 + np.radians(angle_offset)
    draw_irregular_shadow_set(mask2, width, height, angle2)

    shadow_mask = cv2.addWeighted(mask1, 0.65, mask2, 0.65, 0)
    blur_k = random.choice([85, 105, 125])
    shadow_mask_blurred = cv2.GaussianBlur(shadow_mask, (blur_k, blur_k), 0)
    
    shadow_opacity = random.uniform(0.25, 0.45)
    alpha = (shadow_mask_blurred / 255.0) * shadow_opacity
    
    for c in range(3):
        target_shadow_color = 25
        bg[:, :, c] = (bg[:, :, c] * (1.0 - alpha) + target_shadow_color * alpha).astype(np.uint8)

    # 暗角
    X, Y = np.meshgrid(np.arange(width), np.arange(height))
    dist_from_center = np.sqrt((X - width / 2) ** 2 + (Y - height / 2) ** 2)
    max_dist = np.sqrt((width / 2) ** 2 + (height / 2) ** 2)
    
    vignette_drop = random.uniform(0.18, 0.35)
    vignette = 1.0 - vignette_drop * (dist_from_center / max_dist)
    for c in range(3):
        bg[:, :, c] = np.clip(bg[:, :, c] * vignette, 0, 255).astype(np.uint8)

    return bg


def draw_geometric_interference(img, width, height, draw_on_top=False):
    """
    绘制穿插于画面上下层级的随机标定线和辅助检测框
    """
    temp_layer = img.copy()

    num_lines = random.randint(3, 5) if draw_on_top else random.randint(4, 7)
    for _ in range(num_lines):
        p1 = (random.randint(-50, width + 50), random.randint(-50, height + 50))
        p2 = (random.randint(-50, width + 50), random.randint(-50, height + 50))
        color = (random.randint(60, 220), random.randint(60, 220), random.randint(60, 220))
        thickness = random.randint(1, 3)
        cv2.line(temp_layer, p1, p2, color, thickness, cv2.LINE_AA)

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


def draw_cone_with_effects(img, center, color_code, scale, ideal_mode=False, highlight_prob=0.35):
    """
    绘制物料。
    """
    cx, cy = center
    col_info = COLOR_MAP[color_code]

    # 色彩通道的随机微调
    shift_b = random.uniform(-25.0, 25.0)
    shift_g = random.uniform(-25.0, 25.0)
    shift_r = random.uniform(-25.0, 25.0)

    main_col = np.array([
        np.clip(col_info["main"][0] + shift_b, 0, 255),
        np.clip(col_info["main"][1] + shift_g, 0, 255),
        np.clip(col_info["main"][2] + shift_r, 0, 255)
    ])

    R_base = int(58 * scale)
    axis_ratio_x = random.uniform(0.91, 0.99)
    axis_ratio_y = random.uniform(0.91, 0.99)
    rot_angle = random.uniform(0, 360)

    # 1. 理想模式 (纯色平涂)
    if ideal_mode:
        axes = (int(R_base * axis_ratio_x), int(R_base * axis_ratio_y))
        cv2.ellipse(img, (cx, cy), axes, rot_angle, 0, 360, tuple(map(int, main_col)), -1)
        return

    # 2. 正常复杂模式
    skirt_col = np.array([
        np.clip(col_info["skirt"][0] + shift_b, 0, 255),
        np.clip(col_info["skirt"][1] + shift_g, 0, 255),
        np.clip(col_info["skirt"][2] + shift_r, 0, 255)
    ])

    R_top = int(37 * scale)
    shift_dx = int(random.uniform(-8, 8) * scale)
    shift_dy = int(random.uniform(-8, 8) * scale)
    top_center = (cx + shift_dx, cy + shift_dy)

    # 投影阴影
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

    # 渐变裙边
    steps = 25
    for i in range(steps):
        t = i / (steps - 1)
        r_curr = R_base - t * (R_base - R_top)
        curr_cx = int(cx + t * shift_dx)
        curr_cy = int(cy + t * shift_dy)

        curr_col = skirt_col * (1.0 - t) + main_col * t
        axes_curr = (int(r_curr * axis_ratio_x), int(r_curr * axis_ratio_y))

        cv2.ellipse(img, (curr_cx, curr_cy), axes_curr, rot_angle, 0, 360, tuple(map(int, curr_col)), -1)

    # 顶面平面
    top_axes = (int(R_top * axis_ratio_x), int(R_top * axis_ratio_y))
    cv2.ellipse(img, top_center, top_axes, rot_angle, 0, 360, tuple(map(int, main_col)), -1)
    cv2.ellipse(img, top_center, top_axes, rot_angle, 0, 360, tuple(map(int, skirt_col)), 1)

    # 3. 高光区渲染 (此处应用高光概率控制)
    has_highlight = random.random() < highlight_prob
    if has_highlight:
        hl_mask = np.zeros_like(img, dtype=np.uint8)
        hl_shift_x = int(random.uniform(-10, 10) * scale)
        hl_shift_y = int(random.uniform(-10, 10) * scale)
        top_hl_center = (top_center[0] + hl_shift_x, top_center[1] + hl_shift_y)
        
        hl_w = int(random.uniform(5, 18) * scale)
        hl_h = int(random.uniform(3, 10) * scale)
        top_hl_axes = (hl_w, hl_h)
        
        cv2.ellipse(hl_mask, top_hl_center, top_hl_axes, rot_angle - 45, 0, 360, (255, 255, 255), -1)

        hl_blur_k = int(random.choice([11, 15, 19]) * scale) | 1
        hl_blur = cv2.GaussianBlur(hl_mask, (hl_blur_k, hl_blur_k), 0)

        gray_hl = cv2.cvtColor(hl_blur, cv2.COLOR_BGR2GRAY)
        alpha_hl = (gray_hl / 255.0) * random.uniform(0.10, 0.65)
        for c in range(3):
            img[:, :, c] = np.clip(img[:, :, c] * (1.0 - alpha_hl) + 255 * alpha_hl, 0, 255).astype(np.uint8)


def apply_camera_noise_dynamic(img, noise_level):
    """
    根据给定的噪声级别动态附加噪点
    """
    if noise_level <= 0.01:
        return img

    height, width, _ = img.shape

    noise_sigma = noise_level * random.uniform(34.0, 46.0)
    noise = np.random.normal(0, noise_sigma, img.shape).astype(np.int16)
    img_noisy = np.clip(img.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    prob = noise_level * random.uniform(0.045, 0.08)
    rnd = np.random.rand(height, width)
    
    img_noisy[rnd < prob * 0.7] = [0, 0, 0]
    img_noisy[rnd > 1 - prob * 0.3] = [255, 255, 255]

    return img_noisy


def apply_regional_blur(img, grid_div, min_k, max_k):
    """
    在图像中随机选择一个格子区域，并对其进行局部高斯模糊处理以提升泛化能力
    """
    height, width, _ = img.shape
    
    # 计算每个宫格格子的宽和高
    cell_h = height // grid_div
    cell_w = width // grid_div

    # 随机选择一个宫格格子 (行，列)
    row = random.randint(0, grid_div - 1)
    col = random.randint(0, grid_div - 1)

    # 确定其切片边界 (注意最后一行/列采用图片实际尺寸，防止像素取整引起的边界残留问题)
    ymin = row * cell_h
    ymax = height if row == grid_div - 1 else (row + 1) * cell_h
    xmin = col * cell_w
    xmax = width if col == grid_div - 1 else (col + 1) * cell_w

    # 切片取出该区域
    roi = img[ymin:ymax, xmin:xmax]

    # 随机选择模糊核大小并保证其为正奇数
    k_size = random.randint(min(min_k, max_k), max(min_k, max_k))
    if k_size % 2 == 0:
        k_size = max(1, k_size + 1)
    else:
        k_size = max(1, k_size)

    # 实施高斯模糊并回写
    blurred_roi = cv2.GaussianBlur(roi, (k_size, k_size), 0)
    img[ymin:ymax, xmin:xmax] = blurred_roi
    
    return img


def write_yolo_yaml_relative(output_dir):
    """
    写出相对路径的 dataset.yaml 文件
    """
    yaml_path = os.path.join(output_dir, "dataset.yaml")
    names_str = "\n".join([f"  {k-1}: {v['name']}" for k, v in COLOR_MAP.items()])
    
    yaml_content = f"""# YOLOv5/v8 Dataset Configuration
path: .  # 相对路径
train: images/train
val: images/val

nc: {len(COLOR_MAP)}
names:
{names_str}
"""
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(yaml_content)
    print(f"配置文件已写入: '{yaml_path}'")


def generate_dataset(num_images, output_dir, train_ratio=0.7):
    """
    主生成流程
    """
    subdirs = [
        os.path.join("images", "train"),
        os.path.join("images", "val"),
        os.path.join("labels", "train"),
        os.path.join("labels", "val")
    ]
    for subdir in subdirs:
        os.makedirs(os.path.join(output_dir, subdir), exist_ok=True)

    print(f"开始生成数据集...")
    width, height = IMAGE_SIZE

    train_count = int(num_images * train_ratio)
    val_count = num_images - train_count
    splits = ["train"] * train_count + ["val"] * val_count
    random.shuffle(splits)

    generated_count = 0

    while generated_count < num_images:
        is_ideal_mode = (random.random() < IDEAL_MODE_RATIO)
        img = generate_white_harsh_background(width, height, ideal_mode=is_ideal_mode)

        if not is_ideal_mode:
            draw_geometric_interference(img, width, height, draw_on_top=False)

        available_colors = list(COLOR_MAP.keys())
        selected_colors = random.sample(available_colors, 3)

        placed_objects = []

        for color_code in selected_colors:
            scale = random.uniform(MIN_SCALE, MAX_SCALE)
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

        placed_objects.sort(key=lambda o: o[1])
        for cx, cy, scale, color_code in placed_objects:
            draw_cone_with_effects(
                img, (cx, cy), color_code, scale, 
                ideal_mode=is_ideal_mode, highlight_prob=HIGHLIGHT_PROB
            )

        if not is_ideal_mode:
            draw_geometric_interference(img, width, height, draw_on_top=True)

        # 局部模糊：在非理想模式下，根据设定的概率对指定划分的某个宫格区域实施高斯模糊
        if not is_ideal_mode and (random.random() < REGIONAL_BLUR_PROB):
            img = apply_regional_blur(img, BLUR_GRID_DIV, MIN_BLUR_KERNEL, MAX_BLUR_KERNEL)

        if is_ideal_mode:
            noise_level = 0.0
        else:
            noise_level = random.uniform(0.0, 1.0)
            
        img = apply_camera_noise_dynamic(img, noise_level)

        current_split = splits[generated_count]

        filename_parts = []
        for cx, cy, _, color_code in placed_objects:
            filename_parts.append(f"{color_code}({cx}, {cy})")
        ideal_flag = "ideal_" if is_ideal_mode else f"noise_{noise_level:.2f}_"
        base_name = f"{ideal_flag}" + "".join(filename_parts)

        img_filename = f"{base_name}.jpg"
        label_filename = f"{base_name}.txt"

        img_save_path = os.path.join(output_dir, "images", current_split, img_filename)
        label_save_path = os.path.join(output_dir, "labels", current_split, label_filename)

        save_image_robust(img, img_save_path)

        with open(label_save_path, "w", encoding="utf-8") as f_lbl:
            for cx, cy, scale, color_code in placed_objects:
                class_id = color_code - 1
                R_base = int(58 * scale)
                
                xmin = max(0, cx - R_base)
                ymin = max(0, cy - R_base)
                xmax = min(width, cx + R_base)
                ymax = min(height, cy + R_base)

                x_center = (xmin + xmax) / 2.0 / width
                y_center = (ymin + ymax) / 2.0 / height
                bbox_w = (xmax - xmin) / width
                bbox_h = (ymax - ymin) / height

                f_lbl.write(f"{class_id} {x_center:.6f} {y_center:.6f} {bbox_w:.6f} {bbox_h:.6f}\n")

        generated_count += 1
        print(f"进度: {generated_count}/{num_images} | 已保存在 [{current_split}]: {img_filename}")

    write_yolo_yaml_relative(output_dir)
    print(f"\n数据集成功构建在路径: '{output_dir}'")


if __name__ == "__main__":
    generate_dataset(NUM_IMAGES, OUTPUT_DIR, TRAIN_RATIO)