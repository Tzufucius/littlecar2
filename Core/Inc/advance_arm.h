#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#define ARM_LIFT_MOTOR_ID ((uint8_t)5U)
#define ARM_SWING_MOTOR_ID ((uint8_t)6U)
#define ARM_GRIPPER_SERVO_ID ((uint8_t)2U)

#define ARM_LIFT_DOWN_DIRECTION ((uint8_t)0U)
#define ARM_LIFT_UP_DIRECTION ((uint8_t)1U)
#define ARM_SWING_EXTEND_DIRECTION ((uint8_t)0U)
#define ARM_SWING_RETRACT_DIRECTION ((uint8_t)1U)

#define ARM_LIFT_SPEED ((uint16_t)300U)
#define ARM_SWING_SPEED ((uint16_t)300U)
#define ARM_LIFT_ACC ((uint8_t)10U)
#define ARM_SWING_ACC ((uint8_t)10U)

#define ARM_SWING_EXTEND_PULSE ((uint32_t)28000U)
#define ARM_SWING_RETRACT_PULSE ((uint32_t)28000U)
#define ARM_LIFT_LOWER_PULSE ((uint32_t)18000U)
#define ARM_LIFT_RAISE_PULSE ((uint32_t)18000U)

#define ARM_GRIPPER_OPEN_POS ((int32_t)800)
#define ARM_GRIPPER_CLOSE_POS ((int32_t)1800)
#define ARM_GRIPPER_SPEED ((uint16_t)500U)
#define ARM_GRIPPER_ACC ((uint16_t)20U)

typedef enum
{
  ADVANCE_ARM_STATUS_OK = 0,
  ADVANCE_ARM_STATUS_INVALID_PARAM,
  ADVANCE_ARM_STATUS_LIMIT_BLOCKED,
  ADVANCE_ARM_STATUS_SERVO_ERROR
} AdvanceArm_Status_t;

void AdvanceArm_Init(void);
AdvanceArm_Status_t AdvanceArm_Grab(bool closed);
AdvanceArm_Status_t AdvanceArm_Pick(void);
AdvanceArm_Status_t AdvanceArm_Place(void);
void AdvanceArm_Stop(void);
void AdvanceArm_EStop(void);

#ifdef __cplusplus
}
#endif

#endif
