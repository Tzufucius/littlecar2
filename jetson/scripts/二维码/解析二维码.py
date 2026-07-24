import cv2
import numpy as np
from pathlib import Path

# 1. 定义二维码图片所在的目录
WORK_PATH = Path.cwd()
SAVE_PATH = WORK_PATH / "assets" / "二维码"

def decode_qr_code(file_path: Path):
    """
    读取并解析单张二维码图片
    """
    # 针对 Windows 系统含有中文路径（如“二维码”）的健壮读取方式
    # 直接使用 cv2.imread(str(file_path)) 可能会因为中文路径返回 None
    try:
        img_data = np.fromfile(str(file_path), dtype=np.uint8)
        img = cv2.imdecode(img_data, cv2.IMREAD_COLOR)
    except Exception as e:
        print(f"读取图片失败 {file_path.name}: {e}")
        return None

    if img is None:
        print(f"图片无法解码（可能文件损坏）: {file_path.name}")
        return None

    # 2. 初始化 OpenCV 二维码检测器
    detector = cv2.QRCodeDetector()
    
    # 3. 检测并解析二维码
    # detectAndDecode 返回三个值：
    #   data: 解码后的文本内容（如果未检测到则为空字符串）
    #   bbox: 二维码四个顶点的坐标
    #   straight_qrcode: 矫正后的二维码二值化图像
    data, bbox, straight_qrcode = detector.detectAndDecode(img)
    
    if bbox is not None and data:
        print(f"【成功】文件名: {file_path.name} ---> 解析结果: {data}")
        return data
    else:
        print(f"【失败】文件名: {file_path.name} ---> 未检测到有效的二维码")
        return None

def main():
    if not SAVE_PATH.exists():
        print(f"目标目录不存在: {SAVE_PATH}")
        return
    
    # 4. 使用 pathlib 查找目录下所有的 .png 文件
    qr_files = list(SAVE_PATH.glob("*.png"))
    
    if not qr_files:
        print("未在目录下找到任何 .png 格式的二维码图片。")
        return
        
    print(f"共找到 {len(qr_files)} 张图片，开始解析...\n")
    for file_path in qr_files:
        decode_qr_code(file_path)

if __name__ == "__main__":
    main()