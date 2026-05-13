#include "advance_world.h"

#include "car_pose.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

#define ADVANCE_WORLD_PI (3.14159265358979323846f)

typedef struct
{
  float raw_x_mm;
  float raw_y_mm;
  float raw_yaw_deg;
  uint8_t ready;
} AdvanceWorld_Origin_t;

static volatile WorldPose2D_t g_world_pose = {0};
static AdvanceWorld_Origin_t g_world_origin = {0};
static uint32_t g_last_debug_print_tick = 0U;

static float AdvanceWorld_DegToRad(float deg)
{
  return deg * ADVANCE_WORLD_PI / 180.0f;
}

static uint8_t AdvanceWorld_HasValidOps(void)
{
  return ((carpose_ops != NULL) && (carpose_ops->valid != 0U)) ? 1U : 0U;
}

static void AdvanceWorld_UpdatePoseFromOps(void)
{
  float dx_raw;
  float dy_raw;
  float yaw_rad;
  float cos_yaw;
  float sin_yaw;

  if ((AdvanceWorld_HasValidOps() == 0U) || (g_world_origin.ready == 0U))
  {
    g_world_pose.valid = 0U;
    g_world_pose.origin_ready = g_world_origin.ready;
    return;
  }

  dx_raw = carpose_ops->pos_x_mm - g_world_origin.raw_x_mm;
  dy_raw = carpose_ops->pos_y_mm - g_world_origin.raw_y_mm;
  yaw_rad = AdvanceWorld_DegToRad(g_world_origin.raw_yaw_deg);
  cos_yaw = cosf(yaw_rad);
  sin_yaw = sinf(yaw_rad);

  g_world_pose.x_mm = cos_yaw * dx_raw + sin_yaw * dy_raw;
  g_world_pose.y_mm = -sin_yaw * dx_raw + cos_yaw * dy_raw;
  g_world_pose.yaw_deg = AdvanceWorld_WrapAngleDeg(carpose_ops->zangle_deg - g_world_origin.raw_yaw_deg);
  g_world_pose.updated_tick = carpose_ops->updated_tick;
  g_world_pose.valid = 1U;
  g_world_pose.origin_ready = 1U;
}

void AdvanceWorld_Init(void)
{
  g_world_origin = (AdvanceWorld_Origin_t){0};
  g_world_pose = (WorldPose2D_t){0};
  g_last_debug_print_tick = 0U;
}

AdvanceWorld_Status_t AdvanceWorld_ResetOrigin(void)
{
  if (AdvanceWorld_HasValidOps() == 0U)
  {
    g_world_origin.ready = 0U;
    g_world_pose.valid = 0U;
    g_world_pose.origin_ready = 0U;
    return ADVANCE_WORLD_STATUS_NO_OPS;
  }

  g_world_origin.raw_x_mm = carpose_ops->pos_x_mm;
  g_world_origin.raw_y_mm = carpose_ops->pos_y_mm;
  g_world_origin.raw_yaw_deg = carpose_ops->zangle_deg;
  g_world_origin.ready = 1U;

  g_world_pose.x_mm = 0.0f;
  g_world_pose.y_mm = 0.0f;
  g_world_pose.yaw_deg = 0.0f;
  g_world_pose.updated_tick = HAL_GetTick();
  g_world_pose.valid = 1U;
  g_world_pose.origin_ready = 1U;
  return ADVANCE_WORLD_STATUS_OK;
}

void AdvanceWorld_Poll(void)
{
  AdvanceWorld_UpdatePoseFromOps();
}

const volatile WorldPose2D_t *AdvanceWorld_GetPose(void)
{
  return &g_world_pose;
}

AdvanceWorld_Status_t AdvanceWorld_GetPoseCopy(WorldPose2D_t *pose)
{
  if (pose == NULL)
  {
    return ADVANCE_WORLD_STATUS_NOT_READY;
  }

  pose->x_mm = g_world_pose.x_mm;
  pose->y_mm = g_world_pose.y_mm;
  pose->yaw_deg = g_world_pose.yaw_deg;
  pose->updated_tick = g_world_pose.updated_tick;
  pose->valid = g_world_pose.valid;
  pose->origin_ready = g_world_pose.origin_ready;

  if (g_world_origin.ready == 0U)
  {
    return ADVANCE_WORLD_STATUS_NO_ORIGIN;
  }

  return (pose->valid != 0U) ? ADVANCE_WORLD_STATUS_OK : ADVANCE_WORLD_STATUS_NO_OPS;
}

float AdvanceWorld_WrapAngleDeg(float angle_deg)
{
  while (angle_deg > 180.0f)
  {
    angle_deg -= 360.0f;
  }

  while (angle_deg < -180.0f)
  {
    angle_deg += 360.0f;
  }

  return angle_deg;
}

float AdvanceWorld_LimitFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}

void AdvanceWorld_WorldToBodyVelocity(float vx_w, float vy_w, float yaw_deg, float *vx_b, float *vy_b)
{
  float yaw = AdvanceWorld_DegToRad(yaw_deg);
  float cos_yaw = cosf(yaw);
  float sin_yaw = sinf(yaw);

  if ((vx_b == NULL) || (vy_b == NULL))
  {
    return;
  }

  *vx_b = cos_yaw * vx_w + sin_yaw * vy_w;
  *vy_b = -sin_yaw * vx_w + cos_yaw * vy_w;
}

void AdvanceWorld_BodyToWorldVelocity(float vx_b, float vy_b, float yaw_deg, float *vx_w, float *vy_w)
{
  float yaw = AdvanceWorld_DegToRad(yaw_deg);
  float cos_yaw = cosf(yaw);
  float sin_yaw = sinf(yaw);

  if ((vx_w == NULL) || (vy_w == NULL))
  {
    return;
  }

  *vx_w = cos_yaw * vx_b - sin_yaw * vy_b;
  *vy_w = sin_yaw * vx_b + cos_yaw * vy_b;
}

void AdvanceWorld_PrintDebug(void)
{
  uint32_t now_tick = HAL_GetTick();

  if ((now_tick - g_last_debug_print_tick) < ADVANCE_WORLD_DEBUG_PRINT_MS)
  {
    return;
  }
  g_last_debug_print_tick = now_tick;

  if (carpose_ops != NULL)
  {
    printf("OPS_RAW: valid=%u x_mm=%ld y_mm=%ld yaw_cdeg=%ld\r\n",
           carpose_ops->valid,
           (long)carpose_ops->pos_x_mm,
           (long)carpose_ops->pos_y_mm,
           (long)(carpose_ops->zangle_deg * 100.0f));
  }

  printf("POSE_WORLD: valid=%u origin=%u x_mm=%ld y_mm=%ld yaw_cdeg=%ld\r\n",
         g_world_pose.valid,
         g_world_pose.origin_ready,
         (long)g_world_pose.x_mm,
         (long)g_world_pose.y_mm,
         (long)(g_world_pose.yaw_deg * 100.0f));
}
