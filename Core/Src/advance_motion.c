#include "advance_motion.h"

#include "advance_chassis.h"
#include "advance_world.h"
#include "main.h"
#include <math.h>

typedef struct
{
  uint32_t arrive_hold_start_tick;
  uint32_t pid_last_tick;
  uint32_t no_progress_start_tick;
  float pid_integral_x_mm_s;
  float pid_integral_y_mm_s;
  float pid_integral_yaw_deg_s;
  float pid_last_x_mm;
  float pid_last_y_mm;
  float pid_last_yaw_deg;
  float no_progress_reference_error_mm;
  uint8_t arrival_stop_sent;
  uint8_t pid_history_valid;
  uint8_t acc;
} AdvanceMotion_Control_t;

static AdvanceMotion_RuntimeStatus_t g_motion = {0};
static AdvanceMotion_Control_t g_motion_control = {0};

/* 返回浮点数的绝对值。 */
static float AdvanceMotion_AbsFloat(float value)
{
  return (value < 0.0f) ? -value : value;
}

/* 清除仅属于一轮 GotoPose 的 PID 与外部进展校验历史。 */
static void AdvanceMotion_ResetPidAndProgress(void)
{
  g_motion_control.pid_last_tick = 0U;
  g_motion_control.no_progress_start_tick = 0U;
  g_motion_control.pid_integral_x_mm_s = 0.0f;
  g_motion_control.pid_integral_y_mm_s = 0.0f;
  g_motion_control.pid_integral_yaw_deg_s = 0.0f;
  g_motion_control.pid_last_x_mm = 0.0f;
  g_motion_control.pid_last_y_mm = 0.0f;
  g_motion_control.pid_last_yaw_deg = 0.0f;
  g_motion_control.no_progress_reference_error_mm = 0.0f;
  g_motion_control.pid_history_valid = 0U;
}

/* 记录当前位姿，作为下一周期的实测速度差分基准。 */
static void AdvanceMotion_SavePidPose(const WorldPose2D_t *pose, uint32_t now_tick)
{
  g_motion_control.pid_last_x_mm = pose->x_mm;
  g_motion_control.pid_last_y_mm = pose->y_mm;
  g_motion_control.pid_last_yaw_deg = pose->yaw_deg;
  g_motion_control.pid_last_tick = now_tick;
  g_motion_control.pid_history_valid = 1U;
}

/* 在速度未朝同方向饱和时累积积分，避免目标较远时积分继续堆积。 */
static void AdvanceMotion_UpdatePidIntegral(float vx_world_mm_s, float vy_world_mm_s,
                                             float wz_ccw_deg_s, float dt_s,
                                             uint8_t linear_saturated, uint8_t yaw_saturated,
                                             uint8_t yaw_required)
{
  if ((linear_saturated == 0U) ||
      ((g_motion.error_x_mm * vx_world_mm_s) + (g_motion.error_y_mm * vy_world_mm_s) <= 0.0f))
  {
    g_motion_control.pid_integral_x_mm_s = AdvanceWorld_LimitFloat(
        g_motion_control.pid_integral_x_mm_s + (g_motion.error_x_mm * dt_s),
        -ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S,
        ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S);
    g_motion_control.pid_integral_y_mm_s = AdvanceWorld_LimitFloat(
        g_motion_control.pid_integral_y_mm_s + (g_motion.error_y_mm * dt_s),
        -ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S,
        ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S);
  }

  if ((yaw_required != 0U) &&
      ((yaw_saturated == 0U) || ((g_motion.yaw_error_deg * wz_ccw_deg_s) <= 0.0f)))
  {
    g_motion_control.pid_integral_yaw_deg_s = AdvanceWorld_LimitFloat(
        g_motion_control.pid_integral_yaw_deg_s + (g_motion.yaw_error_deg * dt_s),
        -ADVANCE_MOTION_PID_YAW_INTEGRAL_LIMIT_DEG_S,
        ADVANCE_MOTION_PID_YAW_INTEGRAL_LIMIT_DEG_S);
  }
}

/* 以位置误差的下降量校验外部闭环是否仍在取得进展。 */
static uint8_t AdvanceMotion_HasNoProgress(uint32_t now_tick, float command_magnitude)
{
  if ((g_motion.position_error_mm <= ADVANCE_MOTION_POS_TOLERANCE_MM) ||
      (command_magnitude < ADVANCE_MOTION_NO_PROGRESS_MIN_COMMAND_MM_S))
  {
    g_motion_control.no_progress_start_tick = 0U;
    return 0U;
  }

  if (g_motion_control.no_progress_start_tick == 0U)
  {
    g_motion_control.no_progress_start_tick = now_tick;
    g_motion_control.no_progress_reference_error_mm = g_motion.position_error_mm;
    return 0U;
  }

  if ((g_motion_control.no_progress_reference_error_mm - g_motion.position_error_mm) >=
      ADVANCE_MOTION_NO_PROGRESS_MIN_REDUCTION_MM)
  {
    g_motion_control.no_progress_start_tick = now_tick;
    g_motion_control.no_progress_reference_error_mm = g_motion.position_error_mm;
    return 0U;
  }

  return ((now_tick - g_motion_control.no_progress_start_tick) >=
          ADVANCE_MOTION_NO_PROGRESS_WINDOW_MS) ? 1U : 0U;
}

/* 按最大模长限制二维速度向量，同时保持其方向不变。 */
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

/* 获取目标点的线速度上限，未设置时使用默认值。 */
static float AdvanceMotion_GetGoalVmax(const WorldGoalPose2D_t *goal)
{
  return (goal->vmax_mm_s > 0.0f) ? goal->vmax_mm_s : ADVANCE_MOTION_DEFAULT_VMAX_MM_S;
}

/* 获取目标点的角速度上限，未设置时使用默认值。 */
static float AdvanceMotion_GetGoalWmax(const WorldGoalPose2D_t *goal)
{
  return (goal->wmax_deg_s > 0.0f) ? goal->wmax_deg_s : ADVANCE_MOTION_DEFAULT_WMAX_DEG_S;
}

/* 校验目标位姿、速度上限、超时时间和标志位。 */
static uint8_t AdvanceMotion_IsGoalValid(const WorldGoalPose2D_t *goal)
{
  if ((goal == 0) ||
      (isfinite(goal->x_mm) == 0) ||
      (isfinite(goal->y_mm) == 0) ||
      (isfinite(goal->yaw_deg) == 0) ||
      (isfinite(goal->vmax_mm_s) == 0) ||
      (isfinite(goal->wmax_deg_s) == 0))
  {
    return 0U;
  }

  if ((goal->x_mm < ADVANCE_MOTION_WORLD_X_MIN_MM) ||
      (goal->x_mm > ADVANCE_MOTION_WORLD_X_MAX_MM) ||
      (goal->y_mm < ADVANCE_MOTION_WORLD_Y_MIN_MM) ||
      (goal->y_mm > ADVANCE_MOTION_WORLD_Y_MAX_MM) ||
      (goal->vmax_mm_s < 0.0f) ||
      (goal->vmax_mm_s > ADVANCE_MOTION_MAX_VMAX_MM_S) ||
      (goal->wmax_deg_s < 0.0f) ||
      (goal->wmax_deg_s > ADVANCE_MOTION_MAX_WMAX_DEG_S) ||
      (goal->timeout_ms > ADVANCE_MOTION_MAX_TIMEOUT_MS) ||
      ((goal->goal_flags & (uint8_t)(~ADVANCE_MOTION_GOAL_USE_YAW)) != 0U))
  {
    return 0U;
  }

  return 1U;
}

/* 获取未超时的有效世界坐标，并转换为运动控制状态码。 */
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
  g_motion.updated_tick = HAL_GetTick();
  if ((state != ADVANCE_MOTION_STATE_ARRIVED) || (g_motion_control.arrival_stop_sent == 0U))
  {
    Chassis_SmoothStop(stop_acc);
  }
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.arrival_stop_sent = 0U;
  AdvanceMotion_ResetPidAndProgress();
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

/* 初始化世界坐标运动控制器。 */
void AdvanceMotion_Init(void)
{
  g_motion = (AdvanceMotion_RuntimeStatus_t){0};
  g_motion_control = (AdvanceMotion_Control_t){0};
  g_motion.state = ADVANCE_MOTION_STATE_IDLE;
}

/* 设置世界坐标系速度，并取消正在执行的到点任务。 */
AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc)
{
  if (g_motion.state == ADVANCE_MOTION_STATE_RUNNING)
  {
    g_motion.state = ADVANCE_MOTION_STATE_CANCELED;
    g_motion.updated_tick = HAL_GetTick();
    g_motion_control.arrive_hold_start_tick = 0U;
    g_motion_control.arrival_stop_sent = 0U;
    AdvanceMotion_ResetPidAndProgress();
  }

  return AdvanceMotion_ApplyWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, acc);
}

/* 设置目标位姿并启动闭环到点运动任务。 */
AdvanceMotion_Status_t AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc)
{
  if (AdvanceMotion_IsGoalValid(goal) == 0U)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  g_motion.goal = *goal;
  g_motion.started_tick = HAL_GetTick();
  g_motion.updated_tick = g_motion.started_tick;
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.arrival_stop_sent = 0U;
  AdvanceMotion_ResetPidAndProgress();
  g_motion.error_x_mm = 0.0f;
  g_motion.error_y_mm = 0.0f;
  g_motion.position_error_mm = 0.0f;
  g_motion.yaw_error_deg = 0.0f;
  g_motion_control.acc = acc;
  g_motion.state = ADVANCE_MOTION_STATE_RUNNING;
  return ADVANCE_MOTION_STATUS_OK;
}

/* 周期性读取世界位姿，计算误差并驱动到点控制状态机。 */
void AdvanceMotion_Poll(void)
{
  uint32_t now_tick = HAL_GetTick();
  AdvanceMotion_Status_t pose_status;
  float vx_world_mm_s;
  float vy_world_mm_s;
  float wz_ccw_deg_s = 0.0f;
  float vmax_mm_s;
  float wmax_deg_s;
  float measured_vx_world_mm_s = 0.0f;
  float measured_vy_world_mm_s = 0.0f;
  float measured_wz_ccw_deg_s = 0.0f;
  float dt_s = 0.0f;
  float raw_linear_magnitude;
  float raw_wz_ccw_deg_s;
  float command_magnitude;
  uint8_t yaw_required;
  uint8_t linear_saturated;
  uint8_t yaw_saturated = 0U;

  if (g_motion.state != ADVANCE_MOTION_STATE_RUNNING)
  {
    return;
  }

  if ((g_motion.goal.timeout_ms > 0U) &&
      ((now_tick - g_motion.started_tick) >= g_motion.goal.timeout_ms))
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_TIMEOUT, g_motion_control.acc);
    return;
  }

  pose_status = AdvanceMotion_GetFreshPose(&g_motion.pose);
  if (pose_status == ADVANCE_MOTION_STATUS_NO_ORIGIN)
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_NO_ORIGIN, g_motion_control.acc);
    return;
  }
  if (pose_status != ADVANCE_MOTION_STATUS_OK)
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_NO_POSE, g_motion_control.acc);
    return;
  }

  g_motion.error_x_mm = g_motion.goal.x_mm - g_motion.pose.x_mm;
  g_motion.error_y_mm = g_motion.goal.y_mm - g_motion.pose.y_mm;
  g_motion.position_error_mm = sqrtf((g_motion.error_x_mm * g_motion.error_x_mm) +
                                     (g_motion.error_y_mm * g_motion.error_y_mm));
  yaw_required = ((g_motion.goal.goal_flags & ADVANCE_MOTION_GOAL_USE_YAW) != 0U) ? 1U : 0U;
  if ((yaw_required != 0U) &&
      ((now_tick - g_motion.pose.yaw_updated_tick) > ADVANCE_MOTION_YAW_TIMEOUT_MS))
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_NO_POSE, g_motion_control.acc);
    return;
  }
  g_motion.yaw_error_deg = yaw_required ? AdvanceWorld_WrapAngleDeg(g_motion.goal.yaw_deg - g_motion.pose.yaw_deg) : 0.0f;

  if ((g_motion.position_error_mm <= ADVANCE_MOTION_POS_TOLERANCE_MM) &&
      ((yaw_required == 0U) || (AdvanceMotion_AbsFloat(g_motion.yaw_error_deg) <= ADVANCE_MOTION_YAW_TOLERANCE_DEG)))
  {
    AdvanceMotion_ResetPidAndProgress();
    if (g_motion_control.arrive_hold_start_tick == 0U)
    {
      g_motion_control.arrive_hold_start_tick = now_tick;
    }
    if (g_motion_control.arrival_stop_sent == 0U)
    {
      /* 保持判定期间已不再输出上一周期的非零速度。 */
      Chassis_SmoothStop(g_motion_control.acc);
      g_motion_control.arrival_stop_sent = 1U;
    }
    if ((now_tick - g_motion_control.arrive_hold_start_tick) >= ADVANCE_MOTION_ARRIVE_HOLD_MS)
    {
      AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_ARRIVED, g_motion_control.acc);
    }
    return;
  }
  g_motion_control.arrive_hold_start_tick = 0U;
  g_motion_control.arrival_stop_sent = 0U;

  if ((g_motion_control.pid_history_valid != 0U) &&
      ((now_tick - g_motion_control.pid_last_tick) > 0U) &&
      ((now_tick - g_motion_control.pid_last_tick) <= ADVANCE_MOTION_PID_MAX_DT_MS))
  {
    dt_s = (float)(now_tick - g_motion_control.pid_last_tick) / 1000.0f;
    measured_vx_world_mm_s = (g_motion.pose.x_mm - g_motion_control.pid_last_x_mm) / dt_s;
    measured_vy_world_mm_s = (g_motion.pose.y_mm - g_motion_control.pid_last_y_mm) / dt_s;
    measured_wz_ccw_deg_s = AdvanceWorld_WrapAngleDeg(
        g_motion.pose.yaw_deg - g_motion_control.pid_last_yaw_deg) / dt_s;
  }

  vx_world_mm_s = (ADVANCE_MOTION_KP_POS * g_motion.error_x_mm) +
                  (ADVANCE_MOTION_KI_POS * g_motion_control.pid_integral_x_mm_s) -
                  (ADVANCE_MOTION_KD_POS * measured_vx_world_mm_s);
  vy_world_mm_s = (ADVANCE_MOTION_KP_POS * g_motion.error_y_mm) +
                  (ADVANCE_MOTION_KI_POS * g_motion_control.pid_integral_y_mm_s) -
                  (ADVANCE_MOTION_KD_POS * measured_vy_world_mm_s);
  vmax_mm_s = AdvanceMotion_GetGoalVmax(&g_motion.goal);
  raw_linear_magnitude = sqrtf((vx_world_mm_s * vx_world_mm_s) +
                                (vy_world_mm_s * vy_world_mm_s));
  (void)AdvanceMotion_LimitVector(&vx_world_mm_s, &vy_world_mm_s, vmax_mm_s);
  linear_saturated = (raw_linear_magnitude > vmax_mm_s) ? 1U : 0U;

  if (yaw_required != 0U)
  {
    wmax_deg_s = AdvanceMotion_GetGoalWmax(&g_motion.goal);
    raw_wz_ccw_deg_s = (ADVANCE_MOTION_KP_YAW * g_motion.yaw_error_deg) +
                        (ADVANCE_MOTION_KI_YAW * g_motion_control.pid_integral_yaw_deg_s) -
                        (ADVANCE_MOTION_KD_YAW * measured_wz_ccw_deg_s);
    wz_ccw_deg_s = AdvanceWorld_LimitFloat(
        raw_wz_ccw_deg_s,
        -wmax_deg_s,
        wmax_deg_s);
    yaw_saturated = (AdvanceMotion_AbsFloat(raw_wz_ccw_deg_s) > wmax_deg_s) ? 1U : 0U;
  }

  AdvanceMotion_UpdatePidIntegral(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, dt_s,
                                   linear_saturated, yaw_saturated, yaw_required);
  AdvanceMotion_SavePidPose(&g_motion.pose, now_tick);
  command_magnitude = sqrtf((vx_world_mm_s * vx_world_mm_s) +
                             (vy_world_mm_s * vy_world_mm_s));
  if (AdvanceMotion_HasNoProgress(now_tick, command_magnitude) != 0U)
  {
    AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_CANCELED, g_motion_control.acc);
    return;
  }

  (void)AdvanceMotion_ApplyWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, g_motion_control.acc);
  g_motion.updated_tick = now_tick;
}

/* 取消当前运动任务并平滑停止底盘。 */
void AdvanceMotion_Cancel(void)
{
  AdvanceMotion_SetTerminalState(ADVANCE_MOTION_STATE_CANCELED, CHASSIS_DEFAULT_ACC);
}

/* 仅在存在运行中任务时取消运动。 */
void AdvanceMotion_CancelIfActive(void)
{
  if (g_motion.state == ADVANCE_MOTION_STATE_RUNNING)
  {
    AdvanceMotion_Cancel();
  }
}

/* 取消当前任务但不向底盘发送停止命令。 */
void AdvanceMotion_CancelWithoutStop(void)
{
  if (g_motion.state == ADVANCE_MOTION_STATE_RUNNING)
  {
    g_motion.state = ADVANCE_MOTION_STATE_CANCELED;
    g_motion.updated_tick = HAL_GetTick();
    g_motion_control.arrive_hold_start_tick = 0U;
    g_motion_control.arrival_stop_sent = 0U;
    AdvanceMotion_ResetPidAndProgress();
  }
}

/* 读取当前运动状态、目标位姿和误差。 */
AdvanceMotion_Status_t AdvanceMotion_GetStatus(AdvanceMotion_RuntimeStatus_t *status)
{
  if (status == 0)
  {
    return ADVANCE_MOTION_STATUS_INVALID_PARAM;
  }

  *status = g_motion;
  return ADVANCE_MOTION_STATUS_OK;
}
