#ifndef __ADVANCE_MOTION_H__
#define __ADVANCE_MOTION_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "advance_world.h"

#define ADVANCE_MOTION_CONTROL_PERIOD_MS ((uint32_t)20U)
#define ADVANCE_MOTION_POSE_TIMEOUT_MS ((uint32_t)100U)
#define ADVANCE_MOTION_YAW_TIMEOUT_MS ((uint32_t)100U)
#define ADVANCE_MOTION_ARRIVE_HOLD_MS ((uint32_t)150U)
#define ADVANCE_MOTION_KP_POS (1.0f)
#define ADVANCE_MOTION_KP_YAW (2.0f)
#define ADVANCE_MOTION_POS_TOLERANCE_MM (20.0f)
#define ADVANCE_MOTION_YAW_TOLERANCE_DEG (2.0f)
#define ADVANCE_MOTION_DEFAULT_VMAX_MM_S (200.0f)
#define ADVANCE_MOTION_DEFAULT_WMAX_DEG_S (90.0f)

/*
 * GotoPose 输入边界。它们是软件安全限值，不替代现场的机械限位。
 * 修改前应确认场地尺寸、OPS 坐标单位和底盘的可制动距离。
 */
#define ADVANCE_MOTION_WORLD_X_MIN_MM (-5000.0f)
#define ADVANCE_MOTION_WORLD_X_MAX_MM (5000.0f)
#define ADVANCE_MOTION_WORLD_Y_MIN_MM (-5000.0f)
#define ADVANCE_MOTION_WORLD_Y_MAX_MM (5000.0f)
#define ADVANCE_MOTION_MAX_VMAX_MM_S (500.0f)
#define ADVANCE_MOTION_MAX_WMAX_DEG_S (180.0f)
#define ADVANCE_MOTION_MAX_TIMEOUT_MS ((uint32_t)60000U)

#define ADVANCE_MOTION_GOAL_USE_YAW ((uint8_t)0x01U)

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

  typedef struct
  {
    AdvanceMotion_RunState_t state;
    WorldGoalPose2D_t goal;
    WorldPose2D_t pose;
    float error_x_mm;
    float error_y_mm;
    float position_error_mm;
    float yaw_error_deg;
    uint32_t started_tick;
    uint32_t updated_tick;
    uint8_t active;
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
