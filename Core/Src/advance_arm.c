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

void arm_init(void){
    // 舵机已经在主函数中初始化完毕
    // 步进电机初始化
    drive_emm_En_Control(CHASSIS_MOTOR_ARM_LIFT_ID, true, false); // 不用多机同步
    drive_emm_En_Control(CHASSIS_MOTOR_ARM_SWING_ID, true, false); 

    // 理论上需要把步进电机回零，但是现在还没想好，而且掉电丢失的问题还没有解决
    // TODO
    // ？？？？？
}

// 基本动作

// 物料盘旋转
void arm_rotate_plate(uint16_t acceleration, int32_t position, uint16_t speed){
    BusServo_SetPositionEx(CHASSIS_SERVO_ARM_PLATE_ID, acceleration, position, speed);
}

// 夹爪夹取
// 1抓取 0放开
void arm_grab(uint8_t state){
    if(state){
        BusServo_SetPositionEx(CHASSIS_SERVO_ARM_GRAB_ID, 0, 1000, 0); // 角度需要调试
    } else {
        BusServo_SetPositionEx(CHASSIS_SERVO_ARM_GRAB_ID, 0, 0, 0);
    }
}

// 机械臂旋转
void arm_rotate(uint16_t acceleration, int32_t position, uint16_t speed){
    BusServo_SetPositionEx(CHASSIS_SERVO_ARM_ROTATE_ID, acceleration, position, speed);
}

// 丝杆上下运动
void arm_lift(uint16_t acceleration, int32_t position, uint16_t speed){
    BusServo_SetPositionEx(CHASSIS_MOTOR_ARM_LIFT_ID, acceleration, position, speed);
}

// 机械臂前后平移
void arm_swing(uint16_t acceleration, int32_t position, uint16_t speed){
    BusServo_SetPositionEx(CHASSIS_MOTOR_ARM_SWING_ID, acceleration, position, speed);
}

// 高级封装动作

// 旋转到对应颜色物料

// 伸手抓一个

// 放置一个