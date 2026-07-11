#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "drive_bus_servo.h"

/* Emm 位置模式的方向位：0 为正向，1 为反向。实际机构方向由调用方传入。 */
typedef enum
{
  ADVANCE_ARM_MOTOR_DIRECTION_FORWARD = 0U,
  ADVANCE_ARM_MOTOR_DIRECTION_REVERSE = 1U
} AdvanceArm_MotorDirection_t;

typedef enum
{
  ADVANCE_ARM_STATUS_OK = 0,
  ADVANCE_ARM_STATUS_INVALID_PARAM
} AdvanceArm_Status_t;

/*
 * 物料盘（舵机）、机械臂旋转（舵机）、夹爪（舵机）均使用位置控制。
 * servo_id、加速度、目标位置和速度均由调用方传入，便于按实车标定。
 */
BusServo_Status_t AdvanceArm_RotatePlate(uint8_t servo_id, uint16_t acceleration,
                                         int32_t position, uint16_t speed);
BusServo_Status_t AdvanceArm_RotateBase(uint8_t servo_id, uint16_t acceleration,
                                        int32_t position, uint16_t speed);

/*
 * closed 为 false 时使用 release_position，为 true 时使用 close_position。
 * 两个位置均为形参，避免在模块内固化夹爪的开合标定值。
 */
BusServo_Status_t AdvanceArm_Grab(uint8_t servo_id, bool closed,
                                  int32_t release_position, int32_t close_position,
                                  uint16_t acceleration, uint16_t speed);

/*
 * 丝杆和前后平移机构均使用 Emm 位置模式。
 * pulse_count 是目标脉冲数；relative 为 true 时按相对位移执行；
 * synchronous 为 true 时仅装入多机同步命令缓存，需由调用方触发同步执行。
 */
AdvanceArm_Status_t AdvanceArm_MoveLift(uint8_t motor_id,
                                        AdvanceArm_MotorDirection_t direction,
                                        uint16_t velocity, uint8_t acceleration,
                                        uint32_t pulse_count, bool relative,
                                        bool synchronous);
AdvanceArm_Status_t AdvanceArm_MoveSwing(uint8_t motor_id,
                                         AdvanceArm_MotorDirection_t direction,
                                         uint16_t velocity, uint8_t acceleration,
                                         uint32_t pulse_count, bool relative,
                                         bool synchronous);

/* 立即停止指定步进电机；synchronous 的含义与 AdvanceArm_MoveLift 相同。 */
AdvanceArm_Status_t AdvanceArm_StopMotor(uint8_t motor_id, bool synchronous);

#ifdef __cplusplus
}
#endif

#endif
