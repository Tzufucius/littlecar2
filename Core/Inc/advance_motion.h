#ifndef __ADVANCE_MOTION_H__
#define __ADVANCE_MOTION_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "advance_world.h"

#define ADVANCE_MOTION_POSE_TIMEOUT_MS ((uint32_t)100U) /*!< 位姿数据超时时间，单位为 ms。 */
#define ADVANCE_MOTION_YAW_TIMEOUT_MS ((uint32_t)100U) /*!< 航向角数据超时时间，单位为 ms。 */
#define ADVANCE_MOTION_ARRIVE_HOLD_MS ((uint32_t)150U) /*!< 到达判定保持时间，单位为 ms。 */
#define ADVANCE_MOTION_KP_POS (1.0f) /*!< 位置误差比例增益。 */
#define ADVANCE_MOTION_KP_YAW (2.0f) /*!< 航向角误差比例增益。 */
#define ADVANCE_MOTION_KI_POS (0.03f) /*!< 位置误差积分增益。 */
#define ADVANCE_MOTION_KD_POS (0.10f) /*!< 基于实测速度的位置微分增益。 */
#define ADVANCE_MOTION_KI_YAW (0.05f) /*!< 航向角误差积分增益。 */
#define ADVANCE_MOTION_KD_YAW (0.08f) /*!< 基于实测角速度的航向微分增益。 */
#define ADVANCE_MOTION_PID_MAX_DT_MS ((uint32_t)100U) /*!< PID 历史允许的最大间隔，单位为 ms。 */
#define ADVANCE_MOTION_PID_POS_INTEGRAL_LIMIT_MM_S (1000.0f) /*!< 位置误差积分限幅，单位为 mm*s。 */
#define ADVANCE_MOTION_PID_YAW_INTEGRAL_LIMIT_DEG_S (180.0f) /*!< 航向角误差积分限幅，单位为 deg*s。 */
#define ADVANCE_MOTION_POS_TOLERANCE_MM (20.0f) /*!< 位置到达容差，单位为 mm。 */
#define ADVANCE_MOTION_YAW_TOLERANCE_DEG (2.0f) /*!< 航向角到达容差，单位为度。 */
#define ADVANCE_MOTION_DEFAULT_VMAX_MM_S (200.0f) /*!< 默认最大线速度，单位为 mm/s。 */
#define ADVANCE_MOTION_DEFAULT_WMAX_DEG_S (90.0f) /*!< 默认最大角速度，单位为度/s。 */
#define ADVANCE_MOTION_NO_PROGRESS_WINDOW_MS ((uint32_t)1000U) /*!< 无进展判定观察窗口，单位为 ms。 */
#define ADVANCE_MOTION_NO_PROGRESS_MIN_REDUCTION_MM (15.0f) /*!< 观察窗口内要求的最小误差下降量，单位为 mm。 */
#define ADVANCE_MOTION_NO_PROGRESS_MIN_COMMAND_MM_S (30.0f) /*!< 启用无进展判定的最小线速度指令，单位为 mm/s。 */

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

  /** @brief 运动控制的对外状态快照，用于上位机与调试查询。 */
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
  } AdvanceMotion_RuntimeStatus_t;

  /** @brief 初始化运动控制模块。 */
  void AdvanceMotion_Init(void);
  /** @brief 设置世界坐标系速度及加速度。 @return 设置结果状态。 */
  AdvanceMotion_Status_t AdvanceMotion_SetWorldVelocityEx(float vx_world_mm_s, float vy_world_mm_s, float wz_ccw_deg_s, uint8_t acc);
  /** @brief 启动带加速度参数的位姿导航。 @param goal 目标位姿指针。 @return 启动结果状态。 */
  AdvanceMotion_Status_t AdvanceMotion_GotoPoseEx(const WorldGoalPose2D_t *goal, uint8_t acc);
  /**
   * @brief 阻塞执行位姿导航，返回前不会继续执行调用方后续代码。
   * @details UART/DMA 中断在阻塞期间仍可运行，但本函数不处理上位机协议队列。
   * 外部通信控制应使用异步 AdvanceMotion_GotoPoseEx 和主循环调度；本接口用于本地测试和固定业务流程。
   * @return 最终 AdvanceMotion_RunState_t，不得从中断上下文调用。
   */
  AdvanceMotion_RunState_t AdvanceMotion_GotoPoseBlocking(const WorldGoalPose2D_t *goal, uint8_t acc);
  /** @brief 轮询推进位姿导航控制器。 */
  void AdvanceMotion_Poll(void);
  /** @brief 仅在存在活动目标时取消并平滑停车，避免空闲状态产生多余停止帧。 */
  void AdvanceMotion_CancelIfActive(void);
  void AdvanceMotion_CancelWithoutStop(void);
  /** @brief 取消当前导航目标并停车。 */
  void AdvanceMotion_Cancel(void);
  /** @brief 获取运动控制运行状态。 @param status 输出状态结构体。 @return 获取结果状态。 */
  AdvanceMotion_Status_t AdvanceMotion_GetStatus(AdvanceMotion_RuntimeStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif
