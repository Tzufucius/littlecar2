#ifndef __ADVANCE_WORLD_H__
#define __ADVANCE_WORLD_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define ADVANCE_WORLD_DEBUG_PRINT_MS ((uint32_t)500U) /*!< 世界坐标调试打印周期，单位为 ms。 */
#define ADVANCE_WORLD_DEBUG_ENABLE (0U) /*!< 世界坐标调试打印开关：0-关闭，1-开启。 */

/*
 * world yaw 统一约定：俯视小车时，逆时针旋转为角度增大。
 * 如果某个传感器安装后读数方向相反，将对应宏改为 1。
 */
#define ADVANCE_WORLD_OPS_YAW_REVERSED ((uint8_t)0U) /*!< OPS 航向角方向反转开关。 */
#define ADVANCE_WORLD_WIT_YAW_REVERSED ((uint8_t)0U) /*!< WIT 航向角方向反转开关。 */

/*
 * OPS 安装标定。先使用单轴低速测试确认轴向，再填写这些编译期配置。
 * 偏移以车体旋转中心为原点，+X 向右、+Y 向前，单位为 mm。
 */
#define ADVANCE_WORLD_OPS_X_REVERSED ((uint8_t)0U) /*!< OPS X 轴方向反转开关。 */
#define ADVANCE_WORLD_OPS_Y_REVERSED ((uint8_t)0U) /*!< OPS Y 轴方向反转开关。 */
#define ADVANCE_WORLD_OPS_XY_SWAPPED ((uint8_t)0U) /*!< OPS X/Y 轴交换开关。 */
#define ADVANCE_WORLD_OPS_YAW_OFFSET_DEG (0.0f) /*!< OPS 航向角安装偏移，单位为度。 */
#define ADVANCE_WORLD_WIT_YAW_OFFSET_DEG (0.0f) /*!< WIT 航向角安装偏移，单位为度。 */
#define ADVANCE_WORLD_OPS_OFFSET_X_MM (0.0f) /*!< OPS 相对车体中心 X 偏移，单位为 mm。 */
#define ADVANCE_WORLD_OPS_OFFSET_Y_MM (0.0f) /*!< OPS 相对车体中心 Y 偏移，单位为 mm。 */

  typedef enum
  {
    ADVANCE_WORLD_STATUS_OK = 0,
    ADVANCE_WORLD_STATUS_NOT_READY,
    ADVANCE_WORLD_STATUS_NO_OPS,
    ADVANCE_WORLD_STATUS_NO_ORIGIN
  } AdvanceWorld_Status_t;

  /** @brief 世界坐标系下的二维位姿及数据有效性。 */
  typedef struct
  {
    float x_mm; /*!< X 坐标，单位为 mm。 */
    float y_mm; /*!< Y 坐标，单位为 mm。 */
    float yaw_deg; /*!< 航向角，单位为度。 */
    uint32_t updated_tick; /*!< 位姿更新时间，单位为 ms。 */
    uint32_t yaw_updated_tick; /*!< 航向角更新时间，单位为 ms。 */
    uint8_t valid; /*!< 位姿有效标志：1-有效，0-无效。 */
    uint8_t origin_ready; /*!< 原点是否已建立：1-是，0-否。 */
  } WorldPose2D_t;

  /** @brief 世界坐标系下的目标位姿及运动约束。 */
  typedef struct
  {
    float x_mm; /*!< 目标 X 坐标，单位为 mm。 */
    float y_mm; /*!< 目标 Y 坐标，单位为 mm。 */
    float yaw_deg; /*!< 目标航向角，单位为度。 */
    float vmax_mm_s; /*!< 最大线速度，单位为 mm/s。 */
    float wmax_deg_s; /*!< 最大角速度，单位为度/s。 */
    uint32_t timeout_ms; /*!< 目标超时时间，单位为 ms。 */
    uint8_t goal_flags; /*!< 目标约束标志位。 */
  } WorldGoalPose2D_t;

  /** @brief 二维线速度和角速度。 */
  typedef struct
  {
    float vx_mm_s; /*!< X 方向速度，单位为 mm/s。 */
    float vy_mm_s; /*!< Y 方向速度，单位为 mm/s。 */
    float wz_deg_s; /*!< 绕 Z 轴角速度，单位为度/s。 */
  } WorldVelocity2D_t;

  /** @brief 初始化世界坐标模块。 */
  void AdvanceWorld_Init(void);
  /** @brief 将当前位姿设置为世界坐标原点。 @return 重置结果状态。 */
  AdvanceWorld_Status_t AdvanceWorld_ResetOrigin(void);
  /** @brief 轮询更新世界坐标和传感器状态。 */
  void AdvanceWorld_Poll(void);
  /** @brief 获取当前世界位姿只读引用。 @return 世界位姿指针。 */
  const volatile WorldPose2D_t *AdvanceWorld_GetPose(void);
  /** @brief 获取当前世界位姿副本。 @param pose 输出位姿结构体。 @return 获取结果状态。 */
  AdvanceWorld_Status_t AdvanceWorld_GetPoseCopy(WorldPose2D_t *pose);

  /** @brief 将角度归一化到标准范围。 @param angle_deg 输入角度，单位为度。 @return 归一化后的角度。 */
  float AdvanceWorld_WrapAngleDeg(float angle_deg);
  /** @brief 将浮点数限制在指定范围内。 @return 限幅后的数值。 */
  float AdvanceWorld_LimitFloat(float value, float min_value, float max_value);
  /** @brief 将世界坐标系速度转换为车体坐标系速度。 */
  void AdvanceWorld_WorldToBodyVelocity(float vx_w, float vy_w, float yaw_deg, float *vx_b, float *vy_b);
  /** @brief 将车体坐标系速度转换为世界坐标系速度。 */
  void AdvanceWorld_BodyToWorldVelocity(float vx_b, float vy_b, float yaw_deg, float *vx_w, float *vy_w);
  /** @brief 输出世界坐标调试信息。 */
  void AdvanceWorld_PrintDebug(void);

#ifdef __cplusplus
}
#endif

#endif
