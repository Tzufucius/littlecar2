#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__
#endif

#include "drive_bus_servo.h"
#include "drive_emm.h"

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

void arm_init(void);
void arm_rotate_plate(uint16_t acceleration, int32_t position, uint16_t speed);
void arm_grab(uint8_t state);
void arm_rotate(uint16_t acceleration, int32_t position, uint16_t speed);
void arm_lift(uint16_t acceleration, int32_t position, uint16_t speed);
void arm_swing(uint16_t acceleration, int32_t position, uint16_t speed);