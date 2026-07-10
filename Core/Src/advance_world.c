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
  float raw_imu_yaw_deg;
  uint8_t ready;
  uint8_t imu_yaw_ready;
} AdvanceWorld_Origin_t;

static volatile WorldPose2D_t g_world_pose = {0};
static AdvanceWorld_Origin_t g_world_origin = {0};
static uint32_t g_last_debug_print_tick = 0U;

static float AdvanceWorld_DegToRad(float deg)
{
  return deg * ADVANCE_WORLD_PI / 180.0f;
}

static float AdvanceWorld_NormalizeSensorYawDeg(float yaw_deg, uint8_t reversed, float offset_deg)
{
  if (reversed != 0U)
  {
    yaw_deg = -yaw_deg;
  }

  return AdvanceWorld_WrapAngleDeg(yaw_deg + offset_deg);
}

static float AdvanceWorld_GetOpsYawDeg(void)
{
  return AdvanceWorld_NormalizeSensorYawDeg(
      carpose_ops->zangle_deg,
      ADVANCE_WORLD_OPS_YAW_REVERSED,
      ADVANCE_WORLD_OPS_YAW_OFFSET_DEG);
}

static float AdvanceWorld_GetImuYawDeg(void)
{
  return AdvanceWorld_NormalizeSensorYawDeg(
      carpose_imu->angle_deg.z,
      ADVANCE_WORLD_WIT_YAW_REVERSED,
      ADVANCE_WORLD_WIT_YAW_OFFSET_DEG);
}

static uint8_t AdvanceWorld_HasValidOps(void)
{
  return ((carpose_ops != NULL) && (carpose_ops->valid != 0U)) ? 1U : 0U;
}

static uint8_t AdvanceWorld_HasValidImuYaw(void)
{
  return ((carpose_imu != NULL) && (carpose_imu->angle_deg.valid != 0U)) ? 1U : 0U;
}

static float AdvanceWorld_GetRelativeYawDeg(void)
{
  if (g_world_origin.imu_yaw_ready != 0U)
  {
    return AdvanceWorld_WrapAngleDeg(AdvanceWorld_GetImuYawDeg() - g_world_origin.raw_imu_yaw_deg);
  }

  return AdvanceWorld_WrapAngleDeg(AdvanceWorld_GetOpsYawDeg() - g_world_origin.raw_yaw_deg);
}

static void AdvanceWorld_GetOpsChassisPosition(float *x_mm, float *y_mm)
{
  float x = carpose_ops->pos_x_mm;
  float y = carpose_ops->pos_y_mm;
  float yaw_rad;
  float cos_yaw;
  float sin_yaw;

  if (ADVANCE_WORLD_OPS_XY_SWAPPED != 0U)
  {
    float tmp = x;
    x = y;
    y = tmp;
  }
  if (ADVANCE_WORLD_OPS_X_REVERSED != 0U)
  {
    x = -x;
  }
  if (ADVANCE_WORLD_OPS_Y_REVERSED != 0U)
  {
    y = -y;
  }

  /* OPS 坐标通常是传感器位置，转换为底盘旋转中心后再建 world 原点。 */
  yaw_rad = AdvanceWorld_DegToRad(AdvanceWorld_GetOpsYawDeg());
  cos_yaw = cosf(yaw_rad);
  sin_yaw = sinf(yaw_rad);
  *x_mm = x - (cos_yaw * ADVANCE_WORLD_OPS_OFFSET_X_MM) + (sin_yaw * ADVANCE_WORLD_OPS_OFFSET_Y_MM);
  *y_mm = y - (sin_yaw * ADVANCE_WORLD_OPS_OFFSET_X_MM) - (cos_yaw * ADVANCE_WORLD_OPS_OFFSET_Y_MM);
}

static void AdvanceWorld_UpdatePoseFromOps(void)
{
  float dx_raw;
  float dy_raw;
  float ops_x_mm;
  float ops_y_mm;
  float yaw_rad;
  float cos_yaw;
  float sin_yaw;

  if ((AdvanceWorld_HasValidOps() == 0U) || (g_world_origin.ready == 0U))
  {
    g_world_pose.valid = 0U;
    g_world_pose.origin_ready = g_world_origin.ready;
    return;
  }

  /* 原点选择了 WIT 航向时，不允许在 WIT 超时后悄然切换到 OPS 航向。 */
  if ((g_world_origin.imu_yaw_ready != 0U) && (AdvanceWorld_HasValidImuYaw() == 0U))
  {
    g_world_pose.valid = 0U;
    g_world_pose.origin_ready = 1U;
    return;
  }

  AdvanceWorld_GetOpsChassisPosition(&ops_x_mm, &ops_y_mm);
  dx_raw = ops_x_mm - g_world_origin.raw_x_mm;
  dy_raw = ops_y_mm - g_world_origin.raw_y_mm;
  yaw_rad = AdvanceWorld_DegToRad(g_world_origin.raw_yaw_deg);
  cos_yaw = cosf(yaw_rad);
  sin_yaw = sinf(yaw_rad);

  g_world_pose.x_mm = cos_yaw * dx_raw + sin_yaw * dy_raw;
  g_world_pose.y_mm = -sin_yaw * dx_raw + cos_yaw * dy_raw;
  g_world_pose.yaw_deg = AdvanceWorld_GetRelativeYawDeg();
  g_world_pose.updated_tick = carpose_ops->updated_tick;
  g_world_pose.yaw_updated_tick = (g_world_origin.imu_yaw_ready != 0U) ?
                                     carpose_imu->angle_deg.updated_tick : carpose_ops->updated_tick;
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

  AdvanceWorld_GetOpsChassisPosition(&g_world_origin.raw_x_mm, &g_world_origin.raw_y_mm);
  g_world_origin.raw_yaw_deg = AdvanceWorld_GetOpsYawDeg();
  if (AdvanceWorld_HasValidImuYaw() != 0U)
  {
    g_world_origin.raw_imu_yaw_deg = AdvanceWorld_GetImuYawDeg();
    g_world_origin.imu_yaw_ready = 1U;
  }
  else
  {
    g_world_origin.raw_imu_yaw_deg = 0.0f;
    g_world_origin.imu_yaw_ready = 0U;
  }
  g_world_origin.ready = 1U;

  g_world_pose.x_mm = 0.0f;
  g_world_pose.y_mm = 0.0f;
  g_world_pose.yaw_deg = 0.0f;
  g_world_pose.updated_tick = HAL_GetTick();
  g_world_pose.yaw_updated_tick = g_world_pose.updated_tick;
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
  pose->yaw_updated_tick = g_world_pose.yaw_updated_tick;
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
#if (ADVANCE_WORLD_DEBUG_ENABLE != 0U)
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

  if (carpose_imu != NULL)
  {
    printf("WIT_RAW: angle_valid=%u yaw_cdeg=%ld origin=%u\r\n",
           carpose_imu->angle_deg.valid,
           (long)(carpose_imu->angle_deg.z * 100.0f),
           g_world_origin.imu_yaw_ready);
  }

  printf("POSE_WORLD: valid=%u origin=%u x_mm=%ld y_mm=%ld yaw_cdeg=%ld\r\n",
         g_world_pose.valid,
         g_world_pose.origin_ready,
         (long)g_world_pose.x_mm,
         (long)g_world_pose.y_mm,
         (long)(g_world_pose.yaw_deg * 100.0f));
#endif
}
