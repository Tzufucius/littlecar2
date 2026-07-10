#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#include "drive_bus_servo.h"
#include "drive_emm.h"
#include <stdint.h>

/*
- 步进电机
    - 5 控制丝杆上下运动
    - 6 控制机械臂前后平移
- 舵机
    - 1 物机械臂旋转
    - 2 机械臂抓取
    - 3 料盘旋转
 */
# define CHASSIS_MOTOR_ARM_LIFT_ID 5U
# define CHASSIS_MOTOR_ARM_SWING_ID 6U
# define CHASSIS_SERVO_ARM_ROTATE_ID 1U
# define CHASSIS_SERVO_ARM_GRAB_ID 2U
# define CHASSIS_SERVO_ARM_PLATE_ID 3U

/* 夹爪开合位置集中在下位机，实车标定后只修改这两个宏。 */
#define ADVANCE_ARM_GRAB_RELEASE_POSITION ((int32_t)0)
#define ADVANCE_ARM_GRAB_CLOSE_POSITION ((int32_t)1000)

/* 上位机协议层只开放夹爪开合；其他机械臂能力后续单独设计命令。 */
BusServo_Status_t AdvanceArm_Grab(uint8_t closed);

#endif
