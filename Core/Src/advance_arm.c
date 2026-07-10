#include "advance_arm.h"

/*id
- 步进电机
    - 5 控制丝杆上下运动
    - 6 控制机械臂前后平移
- 舵机
    - 1 物机械臂旋转
    - 2 机械臂抓取
    - 3 料盘旋转
 */

/* 功能配置
- 物料盘旋转到指定颜色（舵机3）
- 整体机械臂旋转（舵机1）
- 丝杆（步进电机5）
    - 上升
    - 下降
- 夹爪（舵机2）
    - 抓取
    - 松开
- 臂（步进电机6）
    - 前移
    - 后移

# define CHASSIS_MOTOR_ARM_LIFT_ID 5U
# define CHASSIS_MOTOR_ARM_SWING_ID 6U
# define CHASSIS_SERVO_ARM_ROTATE_ID 1U
# define CHASSIS_SERVO_ARM_GRAB_ID 2U
# define CHASSIS_SERVO_ARM_PLATE_ID 3U

*/

/*
 * 夹爪动作只归属 advance_arm；上位机无需知道总线舵机 ID 或标定位置。
 * closed: 0 松开，1 夹取。
 */
BusServo_Status_t AdvanceArm_Grab(uint8_t closed)
{
    int32_t position = (closed != 0U) ? ADVANCE_ARM_GRAB_CLOSE_POSITION : ADVANCE_ARM_GRAB_RELEASE_POSITION;

    return BusServo_SetPositionEx(CHASSIS_SERVO_ARM_GRAB_ID, 0U, position, 0U);
}
