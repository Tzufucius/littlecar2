#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "drive_bus_servo.h"

#define ARM_LIFT_MOTOR_ID ((uint8_t)5U)
#define ARM_SWING_MOTOR_ID ((uint8_t)6U)
#define ARM_GRIPPER_SERVO_ID ((uint8_t)2U)

#define ARM_LIFT_SPEED ((uint16_t)300U)
#define ARM_SWING_SPEED ((uint16_t)300U)
#define ARM_LIFT_ACC ((uint8_t)10U)
#define ARM_SWING_ACC ((uint8_t)10U)
#define ARM_POSITION_TOLERANCE_PULSE ((int32_t)100)

#define ARM_SWING_EXTEND_PULSE ((int32_t)28000)
#define ARM_SWING_RETRACT_PULSE ((int32_t)28000)
#define ARM_LIFT_LOWER_PULSE ((int32_t)18000)
#define ARM_LIFT_RAISE_PULSE ((int32_t)18000)

#define ARM_GRIPPER_OPEN_POS ((int32_t)800)
#define ARM_GRIPPER_CLOSE_POS ((int32_t)1800)
#define ARM_GRIPPER_SPEED ((uint16_t)500U)
#define ARM_GRIPPER_ACC ((uint16_t)20U)
#define ARM_GRIPPER_WAIT_MS ((uint32_t)500U)
#define ARM_TASK_TIMEOUT_MS ((uint32_t)10000U)

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
    ADVANCE_ARM_RUN_BOOT = 0,
    ADVANCE_ARM_RUN_READY,
    ADVANCE_ARM_RUN_RUNNING,
    ADVANCE_ARM_RUN_COMPLETE,
    ADVANCE_ARM_RUN_FAULT,
    ADVANCE_ARM_RUN_ESTOP
  } AdvanceArm_RunState_t;

  typedef enum
  {
    ADVANCE_ARM_TASK_NONE = 0,
    ADVANCE_ARM_TASK_PICK,
    ADVANCE_ARM_TASK_PLACE
  } AdvanceArm_TaskType_t;

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

  /* 仅用于兼容一个协议发布周期的 36 字节取放参数。 */
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
    AdvanceArm_RunState_t run_state;
    AdvanceArm_TaskType_t task_type;
    uint8_t step;
    int32_t lift_current_pulse;
    int32_t lift_target_pulse;
    int32_t swing_current_pulse;
    int32_t swing_target_pulse;
    uint32_t updated_tick;
  } AdvanceArm_RuntimeStatus_t;

  void AdvanceArm_Init(void);
  void AdvanceArm_Poll(void);
  AdvanceArm_Status_t AdvanceArm_StartPick(void);
  AdvanceArm_Status_t AdvanceArm_StartPlace(void);
  AdvanceArm_Status_t AdvanceArm_StartLegacyTask(AdvanceArm_TaskType_t task_type,
                                                  const AdvanceArm_TaskPlan_t *plan);
  AdvanceArm_Status_t AdvanceArm_ResetZero(void);
  bool AdvanceArm_IsFixedConfig(const AdvanceArm_Config_t *config);
  void AdvanceArm_Abort(void);
  void AdvanceArm_EStop(void);
  void AdvanceArm_InvalidateCoordinates(void);
  AdvanceArm_Status_t AdvanceArm_GetStatus(AdvanceArm_RuntimeStatus_t *status);

  AdvanceArm_Status_t AdvanceArm_Grab(bool closed);
  BusServo_Status_t AdvanceArm_SetServo(uint8_t servo_id, uint16_t acceleration,
                                        int32_t position, uint16_t speed);
  AdvanceArm_Status_t AdvanceArm_MoveAxis(uint8_t motor_id,
                                          AdvanceArm_MotorDirection_t direction,
                                          uint16_t velocity, uint8_t acceleration,
                                          uint32_t pulse_count, bool relative,
                                          bool synchronous);
  AdvanceArm_Status_t AdvanceArm_StopAxis(uint8_t motor_id, bool synchronous);

#ifdef __cplusplus
}
#endif

#endif
