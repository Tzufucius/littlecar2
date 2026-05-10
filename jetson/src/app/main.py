from app.robot_main import PROJECT_ROOT, build_camera, build_robot_main


RUN_CAMERA_LOOP = False
RUN_ONCE_IMAGE = True
MAX_FRAMES = None

ONCE_IMAGE_PATH = PROJECT_ROOT / "assets" / "同色.png"


def main() -> None:
    robot = build_robot_main()

    if RUN_ONCE_IMAGE:
        event = robot.run_once_image(ONCE_IMAGE_PATH)
        print(event)
        return

    if RUN_CAMERA_LOOP:
        robot.run_camera_loop(build_camera(), max_frames=MAX_FRAMES)
        return

    print("未启用运行模式，请在 app/main.py 中设置 RUN_ONCE_IMAGE 或 RUN_CAMERA_LOOP。")


if __name__ == "__main__":
    main()
