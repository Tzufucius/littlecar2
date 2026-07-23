#include "advance_arm.h"

#include "drive_bus_servo.h"
#include "drive_emm.h"
#include "sensor_limit.h"

/* 单个步进轴动作只做限位预检查、发相对位置命令与固定等待。 */
static AdvanceArm_Status_t AdvanceArm_MoveStep(uint8_t motor_id,
                                                uint8_t direction,
                                                uint16_t speed,
                                                uint8_t acceleration,
                                                uint32_t pulse,
                                                SensorLimitId_t limit_id)
{
  if (SensorLimit_IsActive(limit_id))
  {
    return ADVANCE_ARM_STATUS_LIMIT_BLOCKED;
  }

  drive_emm_Pos_Control(motor_id, direction, speed, acceleration, pulse, true, false);
  HAL_Delay(1000U);
  return ADVANCE_ARM_STATUS_OK;
}

/* 伸缩轴向前伸出。 */
static AdvanceArm_Status_t AdvanceArm_Extend(void)
{
  return AdvanceArm_MoveStep(ARM_SWING_MOTOR_ID,
                             ARM_SWING_EXTEND_DIRECTION,
                             ARM_SWING_SPEED,
                             ARM_SWING_ACC,
                             ARM_SWING_EXTEND_PULSE,
                             SENSOR_LIMIT_SLIDE_FRONT);
}

/* 伸缩轴向后回收。 */
static AdvanceArm_Status_t AdvanceArm_Retract(void)
{
  return AdvanceArm_MoveStep(ARM_SWING_MOTOR_ID,
                             ARM_SWING_RETRACT_DIRECTION,
                             ARM_SWING_SPEED,
                             ARM_SWING_ACC,
                             ARM_SWING_RETRACT_PULSE,
                             SENSOR_LIMIT_SLIDE_REAR);
}

/* 升降轴下降。 */
static AdvanceArm_Status_t AdvanceArm_Lower(void)
{
  return AdvanceArm_MoveStep(ARM_LIFT_MOTOR_ID,
                             ARM_LIFT_DOWN_DIRECTION,
                             ARM_LIFT_SPEED,
                             ARM_LIFT_ACC,
                             ARM_LIFT_LOWER_PULSE,
                             SENSOR_LIMIT_LIFT_DOWN);
}

/* 升降轴上升。 */
static AdvanceArm_Status_t AdvanceArm_Raise(void)
{
  return AdvanceArm_MoveStep(ARM_LIFT_MOTOR_ID,
                             ARM_LIFT_UP_DIRECTION,
                             ARM_LIFT_SPEED,
                             ARM_LIFT_ACC,
                             ARM_LIFT_RAISE_PULSE,
                             SENSOR_LIMIT_LIFT_UP);
}

void AdvanceArm_Init(void)
{
  /* 阻塞式实现不维护高层运行状态。 */
}

AdvanceArm_Status_t AdvanceArm_Grab(bool closed)
{
  int32_t position = closed ? ARM_GRIPPER_CLOSE_POS : ARM_GRIPPER_OPEN_POS;

  if (BusServo_SetPositionEx(ARM_GRIPPER_SERVO_ID,
                             ARM_GRIPPER_ACC,
                             position,
                             ARM_GRIPPER_SPEED) != drive_bus_servo_STATUS_OK)
  {
    return ADVANCE_ARM_STATUS_SERVO_ERROR;
  }

  HAL_Delay(500U);
  return ADVANCE_ARM_STATUS_OK;
}

AdvanceArm_Status_t AdvanceArm_Pick(void)
{
  AdvanceArm_Status_t status;

  status = AdvanceArm_Extend();
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  status = AdvanceArm_Lower();
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  status = AdvanceArm_Grab(true);
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  status = AdvanceArm_Raise();
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  status = AdvanceArm_Retract();
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  return ADVANCE_ARM_STATUS_OK;
}

AdvanceArm_Status_t AdvanceArm_Place(void)
{
  AdvanceArm_Status_t status;

  status = AdvanceArm_Extend();
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  status = AdvanceArm_Lower();
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  status = AdvanceArm_Grab(false);
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  status = AdvanceArm_Raise();
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  status = AdvanceArm_Retract();
  if (status != ADVANCE_ARM_STATUS_OK)
  {
    AdvanceArm_Stop();
    return status;
  }

  return ADVANCE_ARM_STATUS_OK;
}

void AdvanceArm_Stop(void)
{
  drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
  drive_emm_Stop_Now(ARM_SWING_MOTOR_ID, false);
}

void AdvanceArm_EStop(void)
{
  AdvanceArm_Stop();
}
