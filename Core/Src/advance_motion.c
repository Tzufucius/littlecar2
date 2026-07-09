#include "advance_motion.h"

#include "advance_chassis.h"
#include "advance_world.h"
#include "main.h"

void AdvanceMotion_Init(void)
{
}

AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocity(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s)
{
  return AdvanceMotion_SetWorldVelocityEx(vx_world_mm_s, vy_world_mm_s, wz_ccw_deg_s, CHASSIS_DEFAULT_ACC);
}

AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc)
{
  WorldPose2D_t pose;
  float vx_body_mm_s;
  float vy_body_mm_s;
  AdvanceWorld_Status_t world_status = AdvanceWorld_GetPoseCopy(&pose);

  if (world_status == ADVANCE_WORLD_STATUS_NO_ORIGIN)
  {
    Chassis_SmoothStop(acc);
    return ADVANCE_MOTION_STATUS_NO_ORIGIN;
  }

  if ((world_status != ADVANCE_WORLD_STATUS_OK) || (pose.valid == 0U))
  {
    Chassis_SmoothStop(acc);
    return ADVANCE_MOTION_STATUS_NO_POSE;
  }

  if ((HAL_GetTick() - pose.updated_tick) > ADVANCE_MOTION_POSE_TIMEOUT_MS)
  {
    Chassis_SmoothStop(acc);
    return ADVANCE_MOTION_STATUS_POSE_TIMEOUT;
  }

  AdvanceWorld_WorldToBodyVelocity(vx_world_mm_s, vy_world_mm_s, pose.yaw_deg, &vx_body_mm_s, &vy_body_mm_s);
  Chassis_SetBodyVelocityEx(vx_body_mm_s, vy_body_mm_s, wz_ccw_deg_s, acc);
  return ADVANCE_MOTION_STATUS_OK;
}

void AdvanceMotion_Cancel(void)
{
  Chassis_SmoothStop(CHASSIS_DEFAULT_ACC);
}
