import argparse
from pathlib import Path

from app.robot_main import build_camera, build_robot_main, load_config


def main() -> None:
    parser = argparse.ArgumentParser(description="littlecar2 Jetson 侧入口")
    parser.add_argument("--config", default="configs/default.yaml", help="配置文件路径")
    parser.add_argument("--once", help="处理一张图片并输出识别结果")
    parser.add_argument("--camera", action="store_true", help="打开摄像头运行主循环")
    parser.add_argument("--max-frames", type=int, default=None, help="摄像头循环最大帧数")
    parser.add_argument("--mock-comm", action="store_true", help="使用 mock STM32 通信")
    args = parser.parse_args()

    project_root = Path.cwd()
    config = load_config(project_root / args.config)
    robot = build_robot_main(config, project_root, use_mock_comm=args.mock_comm)

    if args.once:
        event = robot.run_once_image(project_root / args.once)
        print(event)
        return

    if args.camera:
        robot.run_camera_loop(build_camera(config), max_frames=args.max_frames)
        return

    parser.print_help()
