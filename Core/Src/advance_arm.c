#include "advance_arm.h"

#include "drive_emm.h"

static AdvanceArm_Status_t AdvanceArm_MoveStepper(uint8_t motor_id,
                                                  AdvanceArm_MotorDirection_t direction,
                                                  uint16_t velocity, uint8_t acceleration,
                                                  uint32_t pulse_count, bool relative,
                                                  bool synchronous)
{
  if ((motor_id == 0U) ||
      ((direction != ADVANCE_ARM_MOTOR_DIRECTION_FORWARD) &&
       (direction != ADVANCE_ARM_MOTOR_DIRECTION_REVERSE)))
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }

  drive_emm_Pos_Control(motor_id, (uint8_t)direction, velocity, acceleration,
                        pulse_count, relative, synchronous);
  return ADVANCE_ARM_STATUS_OK;
}

BusServo_Status_t AdvanceArm_RotatePlate(uint8_t servo_id, uint16_t acceleration,
                                         int32_t position, uint16_t speed)
{
  return BusServo_SetPositionEx(servo_id, acceleration, position, speed);
}

BusServo_Status_t AdvanceArm_RotateBase(uint8_t servo_id, uint16_t acceleration,
                                        int32_t position, uint16_t speed)
{
  return BusServo_SetPositionEx(servo_id, acceleration, position, speed);
}

BusServo_Status_t AdvanceArm_Grab(uint8_t servo_id, bool closed,
                                  int32_t release_position, int32_t close_position,
                                  uint16_t acceleration, uint16_t speed)
{
  int32_t position = closed ? close_position : release_position;

  return BusServo_SetPositionEx(servo_id, acceleration, position, speed);
}

AdvanceArm_Status_t AdvanceArm_MoveLift(uint8_t motor_id,
                                        AdvanceArm_MotorDirection_t direction,
                                        uint16_t velocity, uint8_t acceleration,
                                        uint32_t pulse_count, bool relative,
                                        bool synchronous)
{
  return AdvanceArm_MoveStepper(motor_id, direction, velocity, acceleration,
                                pulse_count, relative, synchronous);
}

AdvanceArm_Status_t AdvanceArm_MoveSwing(uint8_t motor_id,
                                         AdvanceArm_MotorDirection_t direction,
                                         uint16_t velocity, uint8_t acceleration,
                                         uint32_t pulse_count, bool relative,
                                         bool synchronous)
{
  return AdvanceArm_MoveStepper(motor_id, direction, velocity, acceleration,
                                pulse_count, relative, synchronous);
}

AdvanceArm_Status_t AdvanceArm_StopMotor(uint8_t motor_id, bool synchronous)
{
  if (motor_id == 0U)
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }

  drive_emm_Stop_Now(motor_id, synchronous);
  return ADVANCE_ARM_STATUS_OK;
}
