import os
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches

# ==========================================
# 全局参数与常量定义 (阴影范围已缩小至 0.6)
# ==========================================
CANVAS_SIZE = 600

# 噪点控制常量
NOISE_BASE = 5.0                        
NOISE_REGION_COUNT = 12                 
NOISE_SIGMA_MIN = 40.0
NOISE_SIGMA_MAX = 150.0
NOISE_REGION_STRENGTH_MIN = 2.5         
NOISE_REGION_STRENGTH_MAX = 7.5
BG_BASE = 232.0                         
SPOT_DENSITY = 0.07                     
SPOT_VALUES = np.array([-75.0, -45.0, -20.0, 20.0, 45.0]) 
SPOT_PROBS = np.array([0.3, 0.3, 0.2, 0.1, 0.1])
BG_CLIP_MIN = 0.0
BG_CLIP_MAX = 255.0

# 阴影条带范围缩小为原本的 0.6，强度（不透明度）保持不变
BAND_COUNT_MIN = 2
BAND_COUNT_MAX = 6
BAND_WIDTH_MIN = 20.0 * 0.6             # 缩减为 12.0
BAND_WIDTH_MAX = 90.0 * 0.6             # 缩减为 54.0
BAND_ALPHA_MIN = 0.15                   
BAND_ALPHA_MAX = 0.45
BAND_BLUR_SIGMA_MIN = 50.0 * 0.6         # 缩减为 30.0
BAND_BLUR_SIGMA_MAX = 160.0 * 0.6        # 缩减为 96.0

# 实物颜色配置表
COLOR_PROFILES = [
    {"name": "Red",      "top": np.array([0.92, 0.04, 0.12]), "base": np.array([0.48, 0.01, 0.03])},
    {"name": "Yellow",   "top": np.array([0.98, 0.88, 0.05]), "base": np.array([0.58, 0.45, 0.00])},
    {"name": "Green",    "top": np.array([0.00, 0.82, 0.25]), "base": np.array([0.00, 0.38, 0.10])},
    {"name": "DarkBlue", "top": np.array([0.05, 0.25, 0.85]), "base": np.array([0.01, 0.05, 0.38])},
    {"name": "Cyan",     "top": np.array([0.12, 0.65, 0.95]), "base": np.array([0.03, 0.25, 0.52])},
    {"name": "Black",    "top": np.array([0.18, 0.18, 0.20]), "base": np.array([0.06, 0.06, 0.08])}
]


# ==========================================
# 刚性变换（缩放、旋转、散布）与物理扭曲
# ==========================================
def transform(x, y, S, phi, X_c, Y_c, canvas_size):
    """
    将 CAD 局部坐标 x, y 进行缩放 S、旋转 phi，并散布平移到指定的像素中心点 (X_c, Y_c)，
    最后施加微小的物理扭曲。
    """
    # 1. 缩放 S
    xs = x * S
    ys = y * S

    # 2. 旋转 phi (弧度)
    xr = xs * np.cos(phi) - ys * np.sin(phi)
    yr = xs * np.sin(phi) + ys * np.cos(phi)

    # 3. 散布平移至目标中心点 (X_c, Y_c) 
    pixel_scale = 1.4  # CAD 空间到像素空间的基准转换比例 (600 * 0.7 / 300)
    xc = X_c + xr * pixel_scale
    yc = Y_c + yr * pixel_scale

    # 4. 纸张/扫描仪物理扭曲
    xc_s = xc + 0.04 * yc
    yc_s = yc - 0.02 * xc_s

    wave_x = 1.2 * np.sin(yc_s * 0.04 + 1.2)
    wave_y = 1.2 * np.cos(xc_s * 0.03 + 0.8)

    return xc_s + wave_x, yc_s + wave_y


# ==========================================
# 背景噪点与强化条带阴影
# ==========================================
def add_noise_background(ax, canvas_size):
    y, x = np.mgrid[0:canvas_size, 0:canvas_size]
    noise_strength = np.full((canvas_size, canvas_size), NOISE_BASE)

    for _ in range(NOISE_REGION_COUNT):
        noise_x = np.random.uniform(0, canvas_size)
        noise_y = np.random.uniform(0, canvas_size)
        sigma = np.random.uniform(NOISE_SIGMA_MIN, NOISE_SIGMA_MAX)
        strength = np.random.uniform(NOISE_REGION_STRENGTH_MIN, NOISE_REGION_STRENGTH_MAX)

        noise_strength += strength * np.exp(
            -((x - noise_x) ** 2 + (y - noise_y) ** 2) / (2 * sigma ** 2)
        )

    background = BG_BASE + np.random.normal(0, 1, (canvas_size, canvas_size)) * noise_strength
    density = noise_strength / noise_strength.max()
    spot_mask = np.random.random((canvas_size, canvas_size)) < density * SPOT_DENSITY
    spot_noise = np.random.choice(SPOT_VALUES, size=(canvas_size, canvas_size), p=SPOT_PROBS)
    background[spot_mask] += spot_noise[spot_mask]

    background = np.clip(background, BG_CLIP_MIN, BG_CLIP_MAX)
    ax.images[0].set_data(background)


def add_band_shadows(ax, canvas_size):
    y, x = np.mgrid[0:canvas_size, 0:canvas_size]
    band_shadow = np.zeros((canvas_size, canvas_size), dtype=float)

    band_count = np.random.randint(BAND_COUNT_MIN, BAND_COUNT_MAX + 1)

    for _ in range(band_count):
        theta = np.random.uniform(0, np.pi)
        width = np.random.uniform(BAND_WIDTH_MIN, BAND_WIDTH_MAX)
        alpha = np.random.uniform(BAND_ALPHA_MIN, BAND_ALPHA_MAX)
        blur_sigma = np.random.uniform(BAND_BLUR_SIGMA_MIN, BAND_BLUR_SIGMA_MAX)

        x0 = np.random.uniform(0, canvas_size)
        y0 = np.random.uniform(0, canvas_size)

        dist = (x - x0) * np.cos(theta) + (y - y0) * np.sin(theta)
        band = np.exp(-(dist ** 2) / (2 * width ** 2))
        soft = np.exp(-(dist ** 2) / (2 * blur_sigma ** 2))

        band_shadow += alpha * np.maximum(band, soft)

    # 阴影强度上限 0.30
    band_shadow = np.clip(band_shadow, 0, 0.30)

    ax.imshow(
        np.zeros((canvas_size, canvas_size)),
        cmap="gray",
        vmin=0,
        vmax=255,
        alpha=band_shadow,
        origin="upper",
        extent=[0, canvas_size, canvas_size, 0]
    )


# ==========================================
# 颜色微调生成
# ==========================================
def get_fluctuated_colors(profile):
    fluc = np.random.uniform(-0.05, 0.05, 3)
    top_f = np.clip(profile["top"] + fluc, 0.0, 1.0)
    base_f = np.clip(profile["base"] + fluc * 0.6, 0.0, 1.0)
    return top_f, base_f


# ==========================================
# 绘制单个立体圆台物料
# ==========================================
def draw_3d_material(ax, cx, cy, S, phi, X_c, Y_c):
    """
    在指定的圆心 (cx, cy) 绘制插塞物料。
    """
    profile = np.random.choice(COLOR_PROFILES)
    col_top, col_base = get_fluctuated_colors(profile)

    # 1. 绘制叠层软阴影 (阴影范围缩小为原来的 0.6 倍，强度不变)
    theta_sh = np.linspace(0, 2 * np.pi, 100)
    shadow_layers = [
        (4.5 * 0.6, 27.5, 0.08), 
        (3.0 * 0.6, 26.5, 0.12), 
        (1.5 * 0.6, 25.5, 0.16)
    ]
    for offset_val, r_val, alpha_val in shadow_layers:
        x_sh = cx + offset_val + r_val * np.cos(theta_sh)
        y_sh = cy + offset_val + r_val * np.sin(theta_sh)
        xt_sh, yt_sh = transform(x_sh, y_sh, S, phi, X_c, Y_c, CANVAS_SIZE)
        ax.fill(xt_sh, yt_sh, color='black', alpha=alpha_val, zorder=3)

    # 2. 绘制立体圆台渐变侧面
    R_base = 27.0
    R_top = 18.5
    dx_top, dy_top = -2.5, -2.5

    N_steps = 35
    for i in range(N_steps):
        t = i / (N_steps - 1)
        
        # 3D 打印错落纹理
        layer_jitter_x = np.random.normal(0, 0.22)
        layer_jitter_y = np.random.normal(0, 0.22)
        layer_r_noise = np.random.normal(0, 0.12)
        
        r = R_base + t * (R_top - R_base) + layer_r_noise
        dx = t * dx_top + layer_jitter_x
        dy = t * dy_top + layer_jitter_y
        
        col = col_base + t * (col_top - col_base)

        theta_c = np.linspace(0, 2 * np.pi, 120)
        px = cx + dx + r * np.cos(theta_c)
        py = cy + dy + r * np.sin(theta_c)
        ptx, pty = transform(px, py, S, phi, X_c, Y_c, CANVAS_SIZE)
        ax.fill(ptx, pty, color=col, zorder=4)

    # 3. 散射柔和白光区
    hl_glow_r = np.random.uniform(7.0, 11.5)
    hl_glow_theta = np.linspace(0, 2 * np.pi, 60)
    hl_glow_x = cx + dx_top - 3.5 + hl_glow_r * np.cos(hl_glow_theta)
    hl_glow_y = cy + dy_top - 3.5 + hl_glow_r * np.sin(hl_glow_theta)
    hlt_glow_x, hlt_glow_y = transform(hl_glow_x, hl_glow_y, S, phi, X_c, Y_c, CANVAS_SIZE)
    ax.fill(hlt_glow_x, hlt_glow_y, color='white', alpha=0.05, zorder=5)

    # 4. 绘制斑驳反光高光
    num_highlights = np.random.randint(1, 4)
    for _ in range(num_highlights):
        hl_r_base = np.random.uniform(3.0, 5.8)  
        hl_alpha = np.random.uniform(0.07, 0.20)
        
        hl_offset_x = dx_top - np.random.uniform(1.8, 5.5)
        hl_offset_y = dy_top - np.random.uniform(1.8, 5.5)
        
        hl_theta = np.linspace(0, 2 * np.pi, 45)
        hl_noise = hl_r_base + np.random.normal(0, 0.45, len(hl_theta))
        
        hl_x = cx + hl_offset_x + hl_noise * np.cos(hl_theta)
        hl_y = cy + hl_offset_y + hl_noise * np.sin(hl_theta)
        hlt_x, hlt_y = transform(hl_x, hl_y, S, phi, X_c, Y_c, CANVAS_SIZE)
        ax.fill(hlt_x, hlt_y, color='white', alpha=hl_alpha, zorder=5)


# ==========================================
# 单个图像渲染与保存逻辑
# ==========================================
def render_single_image(n_empty, save_path, S, phi, X_c, Y_c):
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')
    ax.axis('off')

    # 1. 基础底色
    initial_bg = np.full((CANVAS_SIZE, CANVAS_SIZE), BG_BASE)
    ax.imshow(initial_bg, cmap='gray', vmin=0, vmax=255, origin='upper', extent=[0, CANVAS_SIZE, CANVAS_SIZE, 0])

    # 2. 生成纸张噪声
    add_noise_background(ax, CANVAS_SIZE)

    R_outer = 145
    R_pitch = 100
    R_hole = 25

    # 3.1 外圆 (线宽随缩放比例 S 自适应变化)
    theta = np.linspace(0, 2 * np.pi, 600)
    x_out = R_outer * np.cos(theta)
    y_out = R_outer * np.sin(theta)
    xt_out, yt_out = transform(x_out, y_out, S, phi, X_c, Y_c, CANVAS_SIZE)
    ax.plot(xt_out, yt_out, color='#282828', linewidth=1.8 * S, alpha=0.9, zorder=2)

    # 3.2 分度圆 (线宽随缩放比例 S 自适应变化)
    x_pitch = R_pitch * np.cos(theta)
    y_pitch = R_pitch * np.sin(theta)
    xt_pitch, yt_pitch = transform(x_pitch, y_pitch, S, phi, X_c, Y_c, CANVAS_SIZE)
    ax.plot(xt_pitch, yt_pitch, color='#404040', linewidth=1.0 * S, linestyle='-.', alpha=0.85, zorder=2)

    # 4. 物料填充逻辑
    empty_indices = np.random.choice([0, 1, 2], n_empty, replace=False)

    angles = [270, 30, 150]
    for idx, angle in enumerate(angles):
        rad = np.radians(angle)
        cx = R_pitch * np.cos(rad)
        cy = R_pitch * np.sin(rad)

        if idx in empty_indices:
            # 该位置为空
            theta_hole = np.linspace(0, 2 * np.pi, 250)
            x_hole = cx + R_hole * np.cos(theta_hole)
            y_hole = cy + R_hole * np.sin(theta_hole)
            xt_hole, yt_hole = transform(x_hole, y_hole, S, phi, X_c, Y_c, CANVAS_SIZE)
            ax.plot(xt_hole, yt_hole, color='#282828', linewidth=1.8 * S, alpha=0.9, zorder=2)
        else:
            # 绘制具有大反光的立体圆台
            draw_3d_material(ax, cx, cy, S, phi, X_c, Y_c)

    # 5. 辅助十字中心线 (线宽随缩放比例 S 自适应变化)
    x_line = np.linspace(-15, 15, 20)
    y_line = np.full_like(x_line, -100)
    xt_line, yt_line = transform(x_line, y_line, S, phi, X_c, Y_c, CANVAS_SIZE)
    ax.plot(xt_line, yt_line, color='#404040', linewidth=0.9 * S, alpha=0.85, zorder=2)

    # 6. 叠加阴影
    add_band_shadows(ax, CANVAS_SIZE)

    ax.set_xlim(0, CANVAS_SIZE)
    ax.set_ylim(CANVAS_SIZE, 0)
    
    # 强制保存为 JPEG 格式，防止出现白边
    plt.savefig(save_path, format='jpg', dpi=150, bbox_inches='tight', pad_inches=0)
    plt.close(fig)


# ==========================================
# 批量控制生成函数
# ==========================================
def batch_generate(num_images=12, output_dir=r"assets\物料盘"):
    os.makedirs(output_dir, exist_ok=True)

    if num_images < 4:
        print("警告: 生成数量少于 4 张，无法完全满足涵盖 0-3 个空位的所有要求。")
        n_empty_list = [0, 1, 2, 3][:num_images]
    else:
        n_empty_list = [0, 1, 2, 3]
        remaining_count = num_images - 4
        if remaining_count > 0:
            n_empty_list += list(np.random.choice([0, 1, 2, 3], remaining_count))
        np.random.shuffle(n_empty_list)

    print(f"正在启动带有缩放、旋转、散布变换的批量生成任务...")
    print(f"目标目录: '{output_dir}'")
    print(f"总计划数: {num_images} 张")
    print("-" * 65)

    for i, n_empty in enumerate(n_empty_list):
        # 1. 随机生成缩放比例 S (0.4 - 1.6)
        S = np.random.uniform(0.4, 1.6)
        
        # 2. 随机生成旋转角 phi_deg (0 - 120 度)
        phi_deg = np.random.uniform(0, 120)
        phi = np.radians(phi_deg)
        
        # 3. 随机散射中心点 (X_c, Y_c)，根据缩放因子限制范围，防止大物体完全移出画布
        min_pos = max(100, int(150 * S))
        max_pos = min(500, int(600 - 150 * S))
        if min_pos >= max_pos:
            X_c = CANVAS_SIZE / 2.0
            Y_c = CANVAS_SIZE / 2.0
        else:
            X_c = np.random.uniform(min_pos, max_pos)
            Y_c = np.random.uniform(min_pos, max_pos)
        
        # 四舍五入取整，用于文件名命名
        X_c_int = int(round(X_c))
        Y_c_int = int(round(Y_c))
        
        # 按照格式命名：(84, 225).jpg
        file_name = f"({X_c_int}, {Y_c_int}).jpg"
        save_path = os.path.join(output_dir, file_name)
        
        # 执行单张渲染
        render_single_image(n_empty, save_path, S, phi, X_c, Y_c)
        
        material_count = 3 - n_empty
        print(f"[{i+1}/{num_images}] 已保存 -> {file_name} (缩放: {S:.2f}, 旋转: {phi_deg:.1f}°, 空位: {n_empty})")

    print("-" * 65)
    print(f"生成任务结束。所有图像已保存至 '{output_dir}' 文件夹。")


# ==========================================
# 执行批量生成
# ==========================================
if __name__ == '__main__':
    # 计划生成 16 张图，保存路径设为 assets\物料盘
    batch_generate(num_images=16, output_dir=r"assets\物料盘")