#ifndef __ADVANCE_MOTION_H__
#define __ADVANCE_MOTION_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "advance_world.h"

#define ADVANCE_MOTION_CONTROL_PERIOD_MS ((uint32_t)20U) /*!< 运动控制周期，单位为 ms。 */
#define ADVANCE_MOTION_POSE_TIMEOUT_MS ((uint32_t)100U) /*!< 位姿数据超时时间，单位为 ms。 */
#define ADVANCE_MOTION_YAW_TIMEOUT_MS ((uint32_t)100U) /*!< 航向角数据超时时间，单位为 ms。 */
#define ADVANCE_MOTION_ARRIVE_HOLD_MS ((uint32_t)150U) /*!< 到达判定保持时间，单位为 ms。 */
#define ADVANCE_MOTION_KP_POS (1.0f) /*!< 位置误差比例增益。 */
#define ADVANCE_MOTION_KP_YAW (2.0f) /*!< 航向角误差比例增益。 */
#define ADVANCE_MOTION_POS_TOLERANCE_MM (20.0f) /*!< 位置到达容差，单位为 mm。 */
#define ADVANCE_MOTION_YAW_TOLERANCE_DEG (2.0f) /*!< 航向角到达容差，单位为度。 */
#define ADVANCE_MOTION_DEFAULT_VMAX_MM_S (200.0f) /*!< 默认最大线速度，单位为 mm/s。 */
#define ADVANCE_MOTION_DEFAULT_WMAX_DEG_S (90.0f) /*!< 默认最大角速度，单位为度/s。 */

/*
 * GotoPose 输入边界。它们是软件安全限值，不替代现场的机械限位。
 * 修改前应确认场地尺寸、OPS 坐标单位和底盘的可制动距离。
 */
#define ADVANCE_MOTION_WORLD_X_MIN_MM (-5000.0f) /*!< 世界坐标 X 最小边界，单位为 mm。 */
#define ADVANCE_MOTION_WORLD_X_MAX_MM (5000.0f) /*!< 世界坐标 X 最大边界，单位为 mm。 */
#define ADVANCE_MOTION_WORLD_Y_MIN_MM (-5000.0f) /*!< 世界坐标 Y 最小边界，单位为 mm。 */
#define ADVANCE_MOTION_WORLD_Y_MAX_MM (5000.0f) /*!< 世界坐标 Y 最大边界，单位为 mm。 */
#define ADVANCE_MOTION_MAX_VMAX_MM_S (500.0f) /*!< 允许的最大线速度，单位为 mm/s。 */
#define ADVANCE_MOTION_MAX_WMAX_DEG_S (180.0f) /*!< 允许的最大角速度，单位为度/s。 */
#define ADVANCE_MOTION_MAX_TIMEOUT_MS ((uint32_t)60000U) /*!< 允许的最大目标超时时间，单位为 ms。 */

#define ADVANCE_MOTION_GOAL_USE_YAW ((uint8_t)0x01U) /*!< 目标标志：使用航向角约束。 */

  typedef enum
  {
    ADVANCE_MOTION_STATUS_OK = 0,
    ADVANCE_MOTION_STATUS_INVALID_PARAM,
    ADVANCE_MOTION_STATUS_NO_ORIGIN,
    ADVANCE_MOTION_STATUS_NO_POSE,
    ADVANCE_MOTION_STATUS_POSE_TIMEOUT
  } AdvanceMotion_Status_t;

  typedef enum
  {
    ADVANCE_MOTION_STATE_IDLE = 0,
    ADVANCE_MOTION_STATE_RUNNING,
    ADVANCE_MOTION_STATE_ARRIVED,
    ADVANCE_MOTION_STATE_TIMEOUT,
    ADVANCE_MOTION_STATE_NO_POSE,
    ADVANCE_MOTION_STATE_NO_ORIGIN,
    ADVANCE_MOTION_STATE_CANCELED
  } AdvanceMotion_RunState_t;

  /** @brief 运动控制模块运行状态及误差。 */
  typedef struct
  {
    AdvanceMotion_RunState_t state; /*!< 当前运行状态。 */
    WorldGoalPose2D_t goal; /*!< 当前目标位姿。 */
    WorldPose2D_t pose; /*!< 当前实际位姿。 */
    float error_x_mm; /*!< X 方向位置误差，单位为 mm。 */
    float error_y_mm; /*!< Y 方向位置误差，单位为 mm。 */
    float position_error_mm; /*!< 合成位置误差，单位为 mm。 */
    float yaw_error_deg; /*!< 航向角误差，单位为度。 */
    uint32_t started_tick; /*!< 任务开始时间，单位为 ms。 */
    uint32_t updated_tick; /*!< 状态更新时间，单位为 ms。 */
    uint8_t active; /*!< 是否存在活动目标：1-是，0-否。 */
  } AdvanceMotion_RuntimeStatus_t;

  void AdvanceMotion_Init(void);
  AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocity(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s);
  AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc);
  AdvanceMotion_Status_t AdvanceMotion_GotoPose(const WorldGoalPose2D_t *goal);
  AdvanceMotion_Status_t AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc);
  void AdvanceMotion_Poll(void);
  /** @brief 仅在存在活动目标时取消并平滑停车，避免空闲状态产生多余停止帧。 */
  void AdvanceMotion_CancelIfActive(void);
  void AdvanceMotion_Cancel(void);
  AdvanceMotion_Status_t AdvanceMotion_GetStatus(AdvanceMotion_RuntimeStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif
