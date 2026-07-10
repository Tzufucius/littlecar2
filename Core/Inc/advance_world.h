#ifndef __ADVANCE_WORLD_H__
#define __ADVANCE_WORLD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define ADVANCE_WORLD_DEBUG_PRINT_MS ((uint32_t)500U)
#define ADVANCE_WORLD_DEBUG_ENABLE (0U)

/*
 * world yaw 统一约定：俯视小车时，逆时针旋转为角度增大。
 * 如果某个传感器安装后读数方向相反，将对应宏改为 1。
 */
#define ADVANCE_WORLD_OPS_YAW_REVERSED ((uint8_t)0U)
#define ADVANCE_WORLD_WIT_YAW_REVERSED ((uint8_t)0U)

/*
 * OPS 安装标定。先使用单轴低速测试确认轴向，再填写这些编译期配置。
 * 偏移以车体旋转中心为原点，+X 向右、+Y 向前，单位为 mm。
 */
#define ADVANCE_WORLD_OPS_X_REVERSED ((uint8_t)0U)
#define ADVANCE_WORLD_OPS_Y_REVERSED ((uint8_t)0U)
#define ADVANCE_WORLD_OPS_XY_SWAPPED ((uint8_t)0U)
#define ADVANCE_WORLD_OPS_YAW_OFFSET_DEG (0.0f)
#define ADVANCE_WORLD_WIT_YAW_OFFSET_DEG (0.0f)
#define ADVANCE_WORLD_OPS_OFFSET_X_MM (0.0f)
#define ADVANCE_WORLD_OPS_OFFSET_Y_MM (0.0f)

  typedef enum
  {
    ADVANCE_WORLD_STATUS_OK = 0,
    ADVANCE_WORLD_STATUS_NOT_READY,
    ADVANCE_WORLD_STATUS_NO_OPS,
    ADVANCE_WORLD_STATUS_NO_ORIGIN
  } AdvanceWorld_Status_t;

  typedef struct
  {
    float x_mm;
    float y_mm;
    float yaw_deg;
    uint32_t updated_tick;
    uint32_t yaw_updated_tick;
    uint8_t valid;
    uint8_t origin_ready;
  } WorldPose2D_t;

  typedef struct
  {
    float x_mm;
    float y_mm;
    float yaw_deg;
    float vmax_mm_s;
    float wmax_deg_s;
    uint32_t timeout_ms;
    uint8_t goal_flags;
  } WorldGoalPose2D_t;

  typedef struct
  {
    float vx_mm_s;
    float vy_mm_s;
    float wz_deg_s;
  } WorldVelocity2D_t;

  void AdvanceWorld_Init(void);
  AdvanceWorld_Status_t AdvanceWorld_ResetOrigin(void);
  void AdvanceWorld_Poll(void);
  const volatile WorldPose2D_t *AdvanceWorld_GetPose(void);
  AdvanceWorld_Status_t AdvanceWorld_GetPoseCopy(WorldPose2D_t *pose);

  float AdvanceWorld_WrapAngleDeg(float angle_deg);
  float AdvanceWorld_LimitFloat(float value, float min_value, float max_value);
  void AdvanceWorld_WorldToBodyVelocity(float vx_w, float vy_w, float yaw_deg, float *vx_b, float *vy_b);
  void AdvanceWorld_BodyToWorldVelocity(float vx_b, float vy_b, float yaw_deg, float *vx_w, float *vy_w);
  void AdvanceWorld_PrintDebug(void);

#ifdef __cplusplus
}
#endif

#endif
