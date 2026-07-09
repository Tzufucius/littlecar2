#include "advance_motion.h"

#include "advance_chassis.h"
#include "advance_world.h"
#include "main.h"
#include <math.h>

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
  uint32_t last_control_tick;
  uint32_t arrive_hold_start_tick;
  uint8_t active;
  uint8_t acc;
} AdvanceMotion_Context_t;

static AdvanceMotion_Context_t g_motion = {0};

static float AdvanceMotion_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

static float AdvanceMotion_LimitVector(float *vx, float *vy, float max_value)
{
  float magnitude;
  float scale;

  if ((vx == 0) || (vy == 0))
  {
    return 0.0f;
  }

  magnitude = sqrtf((*vx * *vx) + (*vy * *vy));
  if ((max_value > 0.0f) && (magnitude > max_value))
  {
    scale = max_value / magnitude;
    *vx *= scale;
    *vy *= scale;
    magnitude = max_value;
  }

  return magnitude;
}

static float AdvanceMotion_GetGoalVmax(const WorldGoalPose2D_t *goal)
{
  return (goal->vmax_mm_s > 0.0f) ? goal->vmax_mm_s : ADVANCE_MOTION_DEFAULT_VMAX_MM_S;
}

static float AdvanceMotion_GetGoalWmax(const WorldGoalPose2D_t *goal)
{
  return (goal->wmax_deg_s > 0.0f) ? goal->wmax_deg_s : ADVANCE_MOTION_DEFAULT_WMAX_DEG_S;
}

static AdvanceMotion_Status_t AdvanceMotion_GetFreshPose(WorldPose2D_t *pose)
{
  AdvanceWorld_Status_t world_status;

  if (pose == 0)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  world_status = AdvanceWorld_GetPoseCopy(pose);
  if (world_status == ADVANCE_WORLD_STATUS_NO_ORIGIN)
  {
    return ADVANCE_MOTION_STATUS_NO_ORIGIN;
  }

  if ((world_status != ADVANCE_WORLD_STATUS_OK) || (pose->valid == 0U))
  {
    return ADVANCE_MOTION_STATUS_NO_POSE;
  }

  if ((HAL_GetTick() - pose->updated_tick) > ADVANCE_MOTION_POSE_TIMEOUT_MS)
  {
    return ADVANCE_MOTION_STATUS_POSE_TIMEOUT;
  }

  return ADVANCE_MOTION_STATUS_OK;
}

static void AdvanceMotion_SetTerminalState(AdvanceMotion_RunState_t state, uint8_t stop_acc)
{
  g_motion.state = state;
  g_motion.active = 0U;
  g_motion.updated_tick = HAL_GetTick();
  Chassis_SmoothStop(stop_acc);
}

static AdvanceMotion_Status_t AdvanceMotion_ApplyWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc)
{
  WorldPose2D_t pose;
  float vx_body_mm_s;
  float vy_body_mm_s;
  AdvanceMotion_Status_t pose_status = AdvanceMotion_GetFreshPose(&pose);

  if (pose_status == ADVANCE_MOTION_STATUS_NO_ORIGIN)
  {
    Chassis_SmoothStop(acc);
    return ADVANCE_MOTION_STATUS_NO_ORIGIN;
  }

  if (pose_status == ADVANCE_MOTION_STATUS_NO_POSE)
  {
    Chassis_SmoothStop(acc);
    return ADVANCE_MOTION_STATUS_NO_POSE;
  }

  if (pose_status == ADVANCE_MOTION_STATUS_POSE_TIMEOUT)
  {
    Chassis_SmoothStop(acc);
    return ADVANCE_MOTION_STATUS_POSE_TIMEOUT;
  }

  if (pose_status != ADVANCE_MOTION_STATUS_OK)
  {
    Chassis_SmoothStop(acc);
    return pose_status;
  }

  AdvanceWorld_WorldToBodyVelocity(vx_world_mm_s, vy_world_mm_s, pose.yaw_deg, &vx_body_mm_s, &vy_body_mm_s);
  Chassis_SetBodyVelocityEx(vx_body_mm_s, vy_body_mm_s, wz_ccw_deg_s, acc);
  return ADVANCE_MOTION_STATUS_OK;
}

void AdvanceMotion_Init(void)
{
  g_motion = (AdvanceMotion_Context_t){0};
  g_motion.state = ADVANCE_MOTION_STATE_IDLE;
}

AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocity(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s)
{
  return AdvanceMotion_SetWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, CHASSIS_DEFAULT_ACC);
}

AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc)
{
  if (g_motion.state == ADVANCE_MOTION_STATE_RUNNING)
  {
    g_motion.active = 0U;
    g_motion.state = ADVANCE_MOTION_STATE_CANCELED;
    g_motion.updated_tick = HAL_GetTick();
  }

  return AdvanceMotion_ApplyWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, acc);
}

AdvanceMotion_Status_t AdvanceMotion_GotoPose(const WorldGoalPose2D_t *goal)
{
  return AdvanceMotion_GotoPoseEx(goal, CHASSIS_DEFAULT_ACC);
}

AdvanceMotion_Status_t AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc)
{
  if (goal == 0)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  g_motion.goal = *goal;
  g_motion.started_tick = HAL_GetTick();
  g_motion.updated_tick = g_motion.started_tick;
  g_motion.last_control_tick = 0U;
  g_motion.arrive_hold_start_tick = 0U;
  g_motion.error_x_mm = 0.0f;
  g_motion.error_y_mm = 0.0f;
  g_motion.position_error_mm = 0.0f;
  g_motion.yaw_error_deg = 0.0f;
  g_motion.active = 1U;
  g_motion.acc = acc;
  g_motion.state = ADVANCE_MOTION_STATE_RUNNING;
  return ADVANCE_MOTION_STATUS_OK;
}

void AdvanceMotion_Poll(void)
{
  uint32_t now_tick = HAL_GetTick();
  AdvanceMotion_Status_t pose_status;
  float vx_world_mm_s;
  float vy_world_mm_s;
  float wz_ccw_deg_s = 0.0f;
  float vmax_mm_s;
  float wmax_deg_s;
  uint8_t yaw_required;

  if ((g_motion.active == 0U) || (g_motion.state != ADVANCE_MOTION_STATE_RUNNING))
  {
    return;
  }

  if ((g_motion.last_control_tick != 0U) &&
      ((now_tick - g_motion.last_control_tick) < ADVANCE_MOTION_CONTROL_PERIOD_MS))
  {
    return;
  }
  g_motion.last_control_tick = now_tick;

  if ((g_motion.goal.timeout_ms > 0U) &&
      ((now_tick - g_motion.started_tick) >= g_motion.goal.timeout_ms))
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_TIMEOUT, g_motion.acc);
    return;
  }

  pose_status = AdvanceMotion_GetFreshPose(&g_motion.pose);
  if (pose_status == ADVANCE_MOTION_STATUS_NO_ORIGIN)
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_NO_ORIGIN, g_motion.acc);
    return;
  }
  if (pose_status != ADVANCE_MOTION_STATUS_OK)
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_NO_POSE, g_motion.acc);
    return;
  }

  g_motion.error_x_mm = g_motion.goal.x_mm - g_motion.pose.x_mm;
  g_motion.error_y_mm = g_motion.goal.y_mm - g_motion.pose.y_mm;
  g_motion.position_error_mm = sqrtf((g_motion.error_x_mm * g_motion.error_x_mm) +
                                     (g_motion.error_y_mm * g_motion.error_y_mm));
  yaw_required = ((g_motion.goal.goal_flags & ADVANCE_MOTION_GOAL_USE_YAW) != 0U) ? 1U : 0U;
  g_motion.yaw_error_deg = yaw_required ? AdvanceWorld_WrapAngleDeg(g_motion.goal.yaw_deg - g_motion.pose.yaw_deg) : 0.0f;

  if ((g_motion.position_error_mm <= ADVANCE_MOTION_POS_TOLERANCE_MM) &&
      ((yaw_required == 0U) || (AdvanceMotion_AbsFloat(g_motion.yaw_error_deg) <= ADVANCE_MOTION_YAW_TOLERANCE_DEG)))
  {
    if (g_motion.arrive_hold_start_tick == 0U)
    {
      g_motion.arrive_hold_start_tick = now_tick;
    }
    if ((now_tick - g_motion.arrive_hold_start_tick) >= ADVANCE_MOTION_ARRIVE_HOLD_MS)
    {
      AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_ARRIVED, g_motion.acc);
    }
    return;
  }
  g_motion.arrive_hold_start_tick = 0U;

  vx_world_mm_s = ADVANCE_MOTION_KP_POS * g_motion.error_x_mm;
  vy_world_mm_s = ADVANCE_MOTION_KP_POS * g_motion.error_y_mm;
  vmax_mm_s = AdvanceMotion_GetGoalVmax(&g_motion.goal);
  (void)AdvanceMotion_LimitVector(&vx_world_mm_s, &vy_world_mm_s, vmax_mm_s);

  if (yaw_required != 0U)
  {
    wmax_deg_s = AdvanceMotion_GetGoalWmax(&g_motion.goal);
    wz_ccw_deg_s = AdvanceWorld_LimitFloat(
        ADVANCE_MOTION_KP_YAW * g_motion.yaw_error_deg,
        -wmax_deg_s,
        wmax_deg_s);
  }

  (void)AdvanceMotion_ApplyWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, g_motion.acc);
  g_motion.updated_tick = now_tick;
}

void AdvanceMotion_Cancel(void)
{
  AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_CANCELED, CHASSIS_DEFAULT_ACC);
}

AdvanceMotion_Status_t AdvanceMotion_GetStatus(AdvanceMotion_RuntimeStatus_t *status)
{
  if (status == 0)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  status->state = g_motion.state;
  status->goal = g_motion.goal;
  status->pose = g_motion.pose;
  status->error_x_mm = g_motion.error_x_mm;
  status->error_y_mm = g_motion.error_y_mm;
  status->position_error_mm = g_motion.position_error_mm;
  status->yaw_error_deg = g_motion.yaw_error_deg;
  status->started_tick = g_motion.started_tick;
  status->updated_tick = g_motion.updated_tick;
  status->active = g_motion.active;
  return ADVANCE_MOTION_STATUS_OK;
}
