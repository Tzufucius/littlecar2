#ifndef __ADVANCE_MOTION_H__
#define __ADVANCE_MOTION_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define ADVANCE_MOTION_POSE_TIMEOUT_MS ((uint32_t)100U)

  typedef enum
  {
    ADVANCE_MOTION_STATUS_OK = 0,
    ADVANCE_MOTION_STATUS_NO_ORIGIN,
    ADVANCE_MOTION_STATUS_NO_POSE,
    ADVANCE_MOTION_STATUS_POSE_TIMEOUT
  } AdvanceMotion_Status_t;

  void AdvanceMotion_Init(void);
  AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocity(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s);
  AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc);
  void AdvanceMotion_Cancel(void);

#ifdef __cplusplus
}
#endif

#endif
