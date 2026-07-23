#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#define ARM_LIFT_MOTOR_ID ((uint8_t)5U)
#define ARM_SLIDE_MOTOR_ID ((uint8_t)6U)
#define ARM_GRIPPER_SERVO_ID ((uint8_t)2U)

#define ARM_LIFT_DOWN_DIRECTION ((uint8_t)0U)
#define ARM_LIFT_UP_DIRECTION ((uint8_t)1U)
#define ARM_SLIDE_EXTEND_DIRECTION ((uint8_t)0U)
#define ARM_SLIDE_RETRACT_DIRECTION ((uint8_t)1U)

#define ARM_LIFT_SPEED ((uint16_t)300U)
#define ARM_SLIDE_SPEED ((uint16_t)300U)
#define ARM_LIFT_ACC ((uint8_t)10U)
#define ARM_SLIDE_ACC ((uint8_t)10U)

#define ARM_SLIDE_EXTEND_PULSE ((uint32_t)28000U)
#define ARM_SLIDE_RETRACT_PULSE ((uint32_t)28000U)
#define ARM_LIFT_LOWER_PULSE ((uint32_t)18000U)
#define ARM_LIFT_RAISE_PULSE ((uint32_t)18000U)

#define ARM_GRIPPER_OPEN_POS ((int32_t)800)
#define ARM_GRIPPER_CLOSE_POS ((int32_t)1800)
#define ARM_GRIPPER_SPEED ((uint16_t)500U)
#define ARM_GRIPPER_ACC ((uint16_t)20U)

/** @brief 初始化完全开环的机械臂高层模块。 */
void AdvanceArm_Init(void);

/** @brief 控制夹爪打开或闭合，并固定等待 1000 ms。 */
void AdvanceArm_Grab(bool closed);

/** @brief 依次执行伸出、下降、闭合夹爪、上升和收回。 */
void AdvanceArm_Pick(void);

/** @brief 依次执行伸出、下降、打开夹爪、上升和收回。 */
void AdvanceArm_Place(void);

/** @brief 立即停止升降轴与滑台轴。 */
void AdvanceArm_Stop(void);

/** @brief 执行机械臂紧急停止。 */
void AdvanceArm_EStop(void);

#ifdef __cplusplus
}
#endif

#endif
