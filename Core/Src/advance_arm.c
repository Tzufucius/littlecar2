#include "advance_arm.h"
#include "drive_bus_servo.h"
#include "drive_emm.h"

/* 伸缩滑台向前伸出。 */
static void AdvanceArm_Extend(void)
{
  drive_emm_Pos_Control(ARM_SLIDE_MOTOR_ID, ARM_SLIDE_EXTEND_DIRECTION, ARM_SLIDE_SPEED, ARM_SLIDE_ACC, ARM_SLIDE_EXTEND_PULSE, false, false);
  HAL_Delay(1000U);
}

/* 伸缩滑台向后收回。 */
static void AdvanceArm_Retract(void)
{
  drive_emm_Pos_Control(ARM_SLIDE_MOTOR_ID, ARM_SLIDE_RETRACT_DIRECTION, ARM_SLIDE_SPEED, ARM_SLIDE_ACC, ARM_SLIDE_RETRACT_PULSE, false, false);
  HAL_Delay(1000U);
}

/* 升降轴下降。 */
static void AdvanceArm_Lower(void)
{
  drive_emm_Pos_Control(ARM_LIFT_MOTOR_ID, ARM_LIFT_DOWN_DIRECTION, ARM_LIFT_SPEED, ARM_LIFT_ACC, ARM_LIFT_LOWER_PULSE, false, false);
  HAL_Delay(1000U);
}

/* 升降轴上升。 */
static void AdvanceArm_Raise(void)
{
  drive_emm_Pos_Control(ARM_LIFT_MOTOR_ID, ARM_LIFT_UP_DIRECTION, ARM_LIFT_SPEED, ARM_LIFT_ACC, ARM_LIFT_RAISE_PULSE, false, false);
  HAL_Delay(1000U);
}

void AdvanceArm_Init(void)
{
}

void AdvanceArm_Grab(bool closed)
{
  int32_t position = closed ? ARM_GRIPPER_CLOSE_POS : ARM_GRIPPER_OPEN_POS;

  (void)BusServo_SetPositionEx(ARM_GRIPPER_SERVO_ID, ARM_GRIPPER_ACC, position, ARM_GRIPPER_SPEED);
  HAL_Delay(1000U);
}

void AdvanceArm_Pick(void)
{
  AdvanceArm_Extend();
  AdvanceArm_Lower();
  AdvanceArm_Grab(true);
  AdvanceArm_Raise();
  AdvanceArm_Retract();
}

void AdvanceArm_Place(void)
{
  AdvanceArm_Extend();
  AdvanceArm_Lower();
  AdvanceArm_Grab(false);
  AdvanceArm_Raise();
  AdvanceArm_Retract();
}

void AdvanceArm_Stop(void)
{
  drive_emm_Stop_Now(ARM_LIFT_MOTOR_ID, false);
  drive_emm_Stop_Now(ARM_SLIDE_MOTOR_ID, false);
}

void AdvanceArm_EStop(void)
{
  AdvanceArm_Stop();
}
