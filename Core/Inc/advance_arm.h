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

/**
 * @brief 初始化机械臂高层模块。
 * @note 阻塞式方案不维护高层运行状态，此函数保留以兼容统一初始化流程。
 */
void AdvanceArm_Init(void);

/**
 * @brief 控制夹爪打开或闭合。
 * @param closed true 为闭合，false 为打开。
 * @return 发送失败时返回 ADVANCE_ARM_STATUS_SERVO_ERROR。
 * @note 命令发送成功后阻塞等待 500 ms。
 */
AdvanceArm_Status_t AdvanceArm_Grab(bool closed);

/**
 * @brief 按固定顺序执行抓取动作。
 * @return 限位阻塞或夹爪发送失败时停止机械臂并返回对应状态。
 * @note 函数阻塞约 4.5 秒，期间主循环不能处理新的上位机命令。
 */
AdvanceArm_Status_t AdvanceArm_Pick(void);

/**
 * @brief 按固定顺序执行放置动作。
 * @return 限位阻塞或夹爪发送失败时停止机械臂并返回对应状态。
 * @note 函数阻塞约 4.5 秒，期间主循环不能处理新的上位机命令。
 */
AdvanceArm_Status_t AdvanceArm_Place(void);

/**
 * @brief 立即停止升降轴与伸缩轴。
 */
void AdvanceArm_Stop(void);

/**
 * @brief 执行机械臂紧急停止。
 * @note 当前实现与 AdvanceArm_Stop 等价，不维护额外急停状态。
 */
void AdvanceArm_EStop(void);

#ifdef __cplusplus
}
#endif

#endif
