import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches

# ==========================================
# 全局参数与常量定义
# ==========================================
CANVAS_SIZE = 600

# 噪点控制常量
NOISE_BASE = 3.5
NOISE_REGION_COUNT = 6
NOISE_SIGMA_MIN = 30.0
NOISE_SIGMA_MAX = 120.0
NOISE_REGION_STRENGTH_MIN = 0.5
NOISE_REGION_STRENGTH_MAX = 2.5
BG_BASE = 242.0  # 基础纸张偏白色
SPOT_DENSITY = 0.015
SPOT_VALUES = np.array([-30.0, -15.0, -5.0, 5.0, 15.0])
SPOT_PROBS = np.array([0.3, 0.3, 0.2, 0.1, 0.1])
BG_CLIP_MIN = 0.0
BG_CLIP_MAX = 255.0

# 阴影条带常量
BAND_COUNT_MIN = 2
BAND_COUNT_MAX = 4
BAND_WIDTH_MIN = 25.0
BAND_WIDTH_MAX = 90.0
BAND_ALPHA_MIN = 0.04
BAND_ALPHA_MAX = 0.15
BAND_BLUR_SIGMA_MIN = 40.0
BAND_BLUR_SIGMA_MAX = 120.0


# ==========================================
# 第一步：物理扭曲与坐标变换
# ==========================================
def transform(x, y, canvas_size):
    """
    将 CAD 相对坐标 [-150, 150] 转换为画布像素坐标 [0, canvas_size]，
    并施加轻微的仿射倾斜与波浪形物理扭曲。
    """
    # 1. 缩放与居中
    scale = (canvas_size * 0.7) / 300.0
    xc = canvas_size / 2.0 + x * scale
    yc = canvas_size / 2.0 + y * scale

    # 2. 模拟扫描/纸张形变（切变变换 + 低频正弦弯曲）
    xc_s = xc + 0.04 * yc
    yc_s = yc - 0.02 * xc_s

    # 引入波动
    wave_x = 1.2 * np.sin(yc_s * 0.04 + 1.2)
    wave_y = 1.2 * np.cos(xc_s * 0.03 + 0.8)

    return xc_s + wave_x, yc_s + wave_y


# ==========================================
# 第二步：增加噪点 (原函数集成)
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


# ==========================================
# 第三步：增加条状带阴影 (原函数集成)
# ==========================================
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

    band_shadow = np.clip(band_shadow, 0, 0.25)

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
# 绘制主流程
# ==========================================
def draw_distorted_drawing():
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_aspect('equal')
    ax.axis('off')

    # 1. 建立基础背景图层
    initial_bg = np.full((CANVAS_SIZE, CANVAS_SIZE), BG_BASE)
    ax.imshow(initial_bg, cmap='gray', vmin=0, vmax=255, origin='upper', extent=[0, CANVAS_SIZE, CANVAS_SIZE, 0])

    # 2. 应用噪点背景生成
    add_noise_background(ax, CANVAS_SIZE)

    # 3. 绘制扭曲后的 CAD 图形线段 (采用离散点插值，使其能被平滑扭曲)
    R_outer = 145
    R_pitch = 100
    R_hole = 25

    # 3.1 绘制外圆 (略带不均匀墨迹效果，使用 dark charcoal 色)
    theta = np.linspace(0, 2 * np.pi, 600)
    x_out = R_outer * np.cos(theta)
    y_out = R_outer * np.sin(theta)
    xt_out, yt_out = transform(x_out, y_out, CANVAS_SIZE)
    ax.plot(xt_out, yt_out, color='#282828', linewidth=1.8, alpha=0.9)

    # 3.2 绘制分度圆 (双点划线/虚线样式)
    x_pitch = R_pitch * np.cos(theta)
    y_pitch = R_pitch * np.sin(theta)
    xt_pitch, yt_pitch = transform(x_pitch, y_pitch, CANVAS_SIZE)
    ax.plot(xt_pitch, yt_pitch, color='#404040', linewidth=1.0, linestyle='-.', alpha=0.85)

    # 3.3 绘制三个 120° 分布的小圆 (270°, 30°, 150°)
    angles = [270, 30, 150]
    for angle in angles:
        rad = np.radians(angle)
        cx = R_pitch * np.cos(rad)
        cy = R_pitch * np.sin(rad)
        
        theta_hole = np.linspace(0, 2 * np.pi, 250)
        x_hole = cx + R_hole * np.cos(theta_hole)
        y_hole = cy + R_hole * np.sin(theta_hole)
        xt_hole, yt_hole = transform(x_hole, y_hole, CANVAS_SIZE)
        ax.plot(xt_hole, yt_hole, color='#282828', linewidth=1.8, alpha=0.9)

    # 3.4 绘制最下方圆孔的小中心线段
    x_line = np.linspace(-15, 15, 20)
    y_line = np.full_like(x_line, -100)
    xt_line, yt_line = transform(x_line, y_line, CANVAS_SIZE)
    ax.plot(xt_line, yt_line, color='#404040', linewidth=0.9, alpha=0.85)

    # 4. 叠加带状阴影层
    add_band_shadows(ax, CANVAS_SIZE)

    # 5. 设置画布显示边界
    ax.set_xlim(0, CANVAS_SIZE)
    ax.set_ylim(CANVAS_SIZE, 0)

    plt.show()

# 执行绘图
if __name__ == '__main__':
    draw_distorted_drawing()