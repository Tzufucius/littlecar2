#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "drive_bus_servo.h"

typedef enum
{
  ADVANCE_ARM_MOTOR_DIRECTION_FORWARD = 0U,
  ADVANCE_ARM_MOTOR_DIRECTION_REVERSE = 1U
} AdvanceArm_MotorDirection_t;

typedef enum
{
  ADVANCE_ARM_STATUS_OK = 0,
  ADVANCE_ARM_STATUS_INVALID_PARAM,
  ADVANCE_ARM_STATUS_NOT_READY,
  ADVANCE_ARM_STATUS_BUSY,
  ADVANCE_ARM_STATUS_FAULT
} AdvanceArm_Status_t;

typedef enum
{
  ADVANCE_ARM_POSITION_UNKNOWN = 0,
  ADVANCE_ARM_POSITION_VALID,
  ADVANCE_ARM_POSITION_FAULT
} AdvanceArm_PositionValidity_t;

typedef enum
{
  ADVANCE_ARM_TASK_BOOT = 0,
  ADVANCE_ARM_TASK_READY,
  ADVANCE_ARM_TASK_PICK_EXTEND,
  ADVANCE_ARM_TASK_PICK_LOWER,
  ADVANCE_ARM_TASK_PICK_GRIP,
  ADVANCE_ARM_TASK_PICK_LIFT,
  ADVANCE_ARM_TASK_PICK_RETRACT,
  ADVANCE_ARM_TASK_PLACE_EXTEND,
  ADVANCE_ARM_TASK_PLACE_LOWER,
  ADVANCE_ARM_TASK_PLACE_RELEASE,
  ADVANCE_ARM_TASK_PLACE_LIFT,
  ADVANCE_ARM_TASK_PLACE_RETRACT,
  ADVANCE_ARM_TASK_COMPLETE,
  ADVANCE_ARM_TASK_FAULT,
  ADVANCE_ARM_TASK_ESTOP
} AdvanceArm_TaskState_t;

typedef struct
{
  uint8_t lift_motor_id;
  uint8_t swing_motor_id;
  uint8_t gripper_servo_id;
  AdvanceArm_MotorDirection_t lift_down_direction;
  AdvanceArm_MotorDirection_t swing_extend_direction;
  uint16_t lift_velocity;
  uint16_t swing_velocity;
  uint8_t lift_acceleration;
  uint8_t swing_acceleration;
  int32_t position_tolerance_pulse;
} AdvanceArm_Config_t;

/* 所有脉冲均为从人工零点开始的相对位移，正方向由 Config 指定。 */
typedef struct
{
  uint32_t swing_extend_pulse;
  uint32_t lift_lower_pulse;
  uint32_t lift_raise_pulse;
  uint32_t swing_retract_pulse;
  int32_t gripper_close_position;
  int32_t gripper_release_position;
  uint16_t gripper_acceleration;
  uint16_t gripper_speed;
  uint32_t servo_wait_ms;
  uint32_t task_timeout_ms;
} AdvanceArm_TaskPlan_t;

typedef struct
{
  AdvanceArm_PositionValidity_t lift_position_validity;
  AdvanceArm_PositionValidity_t swing_position_validity;
  AdvanceArm_TaskState_t task_state;
  int32_t lift_current_pulse;
  int32_t lift_target_pulse;
  int32_t swing_current_pulse;
  int32_t swing_target_pulse;
  uint8_t active;
  uint8_t faulted;
  uint32_t updated_tick;
} AdvanceArm_RuntimeStatus_t;

void AdvanceArm_Init(void);
AdvanceArm_Status_t AdvanceArm_Configure(const AdvanceArm_Config_t *config);
void AdvanceArm_Poll(void);
AdvanceArm_Status_t AdvanceArm_StartPick(const AdvanceArm_TaskPlan_t *plan);
AdvanceArm_Status_t AdvanceArm_StartPlace(const AdvanceArm_TaskPlan_t *plan);
void AdvanceArm_Abort(void);
void AdvanceArm_EStop(void);
void AdvanceArm_InvalidateCoordinates(void);
AdvanceArm_Status_t AdvanceArm_GetStatus(AdvanceArm_RuntimeStatus_t *status);

BusServo_Status_t AdvanceArm_RotatePlate(uint8_t servo_id, uint16_t acceleration,
                                         int32_t position, uint16_t speed);
BusServo_Status_t AdvanceArm_RotateBase(uint8_t servo_id, uint16_t acceleration,
                                        int32_t position, uint16_t speed);
/* 非阻塞夹爪动作；完成状态由 AdvanceArm_Poll 推进。 */
AdvanceArm_Status_t AdvanceArm_Grab(uint8_t servo_id, bool closed,
                                    int32_t release_position, int32_t close_position,
                                    uint16_t acceleration, uint16_t speed);
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
AdvanceArm_Status_t AdvanceArm_StopMotor(uint8_t motor_id, bool synchronous);

#ifdef __cplusplus
}
#endif

#endif
