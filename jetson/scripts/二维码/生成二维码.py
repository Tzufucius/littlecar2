import qrcode
import random
from pathlib import Path

WORK_PATH = Path.cwd()
SAVE_PATH = WORK_PATH / "assets" / "二维码"
N = 10

if not SAVE_PATH.exists():
    SAVE_PATH.mkdir(parents=True)

def random_task_code():
    color = "123456"
    position = "123"
    task_color1 = ''.join(random.sample(color, 3))
    task_color2 = ''.join(random.sample(color, 3))
    task_position1 = ''.join(random.sample(position, 3))
    task_position2 = ''.join(random.sample(position, 3))
    task_code = f"{task_color1}+{task_color2}+{task_position1}+{task_position2}"
    return task_code

def from_task_generate_qr(task_code, folder_path = SAVE_PATH, save = False):
    img = qrcode.make(task_code)
    if save:
        file_path = folder_path / f"{task_code}.png"
        img.save(file_path)
    print("任务码：", task_code)

def random_generate_qr(folder_path = SAVE_PATH, save = False):
    task_code = random_task_code()
    img = qrcode.make(task_code)
    if save:
        file_path = folder_path / f"{task_code}.png"
        img.save(file_path)
    print("任务码：", task_code)

def main():
    for _ in range(N):
        random_generate_qr(save=True)
    
if __name__ == "__main__":
    main()