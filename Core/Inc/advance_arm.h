#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "drive_bus_servo.h"

#define ARM_LIFT_MOTOR_ID ((uint8_t)5U) /*!< 升降轴总线舵机 ID。 */
#define ARM_SWING_MOTOR_ID ((uint8_t)6U) /*!< 摆动轴总线舵机 ID。 */
#define ARM_GRIPPER_SERVO_ID ((uint8_t)2U) /*!< 夹爪舵机 ID。 */

#define ARM_LIFT_SPEED ((uint16_t)300U) /*!< 升降轴默认速度。 */
#define ARM_SWING_SPEED ((uint16_t)300U) /*!< 摆动轴默认速度。 */
#define ARM_LIFT_ACC ((uint8_t)10U) /*!< 升降轴默认加速度。 */
#define ARM_SWING_ACC ((uint8_t)10U) /*!< 摆动轴默认加速度。 */
#define ARM_POSITION_TOLERANCE_PULSE ((int32_t)100) /*!< 位置到达判定容差，单位为脉冲。 */

#define ARM_SWING_EXTEND_PULSE ((int32_t)28000) /*!< 摆臂伸出动作的位移，单位为脉冲。 */
#define ARM_SWING_RETRACT_PULSE ((int32_t)28000) /*!< 摆臂收回动作的位移，单位为脉冲。 */
#define ARM_LIFT_LOWER_PULSE ((int32_t)18000) /*!< 升降轴下降动作的位移，单位为脉冲。 */
#define ARM_LIFT_RAISE_PULSE ((int32_t)18000) /*!< 升降轴上升动作的位移，单位为脉冲。 */

#define ARM_GRIPPER_OPEN_POS ((int32_t)800) /*!< 夹爪打开位置。 */
#define ARM_GRIPPER_CLOSE_POS ((int32_t)1800) /*!< 夹爪闭合位置。 */
#define ARM_GRIPPER_SPEED ((uint16_t)500U) /*!< 夹爪默认速度。 */
#define ARM_GRIPPER_ACC ((uint16_t)20U) /*!< 夹爪默认加速度。 */
#define ARM_GRIPPER_WAIT_MS ((uint32_t)500U) /*!< 夹爪动作后的等待时间，单位为 ms。 */
#define ARM_TASK_TIMEOUT_MS ((uint32_t)10000U) /*!< 单个机械臂任务的超时时间，单位为 ms。 */

  /** @brief 电机正方向定义。 */
  typedef enum
  {
    ADVANCE_ARM_MOTOR_DIRECTION_FORWARD = 0U,
    ADVANCE_ARM_MOTOR_DIRECTION_REVERSE = 1U
  } AdvanceArm_MotorDirection_t;

  /** @brief 机械臂接口返回状态。 */
  typedef enum
  {
    ADVANCE_ARM_STATUS_OK = 0,
    ADVANCE_ARM_STATUS_INVALID_PARAM,
    ADVANCE_ARM_STATUS_NOT_READY,
    ADVANCE_ARM_STATUS_BUSY,
    ADVANCE_ARM_STATUS_FAULT
  } AdvanceArm_Status_t;

  /** @brief 轴当前位置有效性。 */
  typedef enum
  {
    ADVANCE_ARM_POSITION_UNKNOWN = 0,
    ADVANCE_ARM_POSITION_VALID,
    ADVANCE_ARM_POSITION_FAULT
  } AdvanceArm_PositionValidity_t;

  /** @brief 机械臂运行状态。 */
  typedef enum
  {
    ADVANCE_ARM_RUN_BOOT = 0,
    ADVANCE_ARM_RUN_READY,
    ADVANCE_ARM_RUN_RUNNING,
    ADVANCE_ARM_RUN_COMPLETE,
    ADVANCE_ARM_RUN_FAULT,
    ADVANCE_ARM_RUN_ESTOP
  } AdvanceArm_RunState_t;

  /** @brief 机械臂任务类型。 */
  typedef enum
  {
    ADVANCE_ARM_TASK_NONE = 0,
    ADVANCE_ARM_TASK_PICK,
    ADVANCE_ARM_TASK_PLACE
  } AdvanceArm_TaskType_t;

  /** @brief 机械臂固定配置及运动参数。 */
  typedef struct
  {
    uint8_t lift_motor_id; /*!< 升降轴电机 ID。 */
    uint8_t swing_motor_id; /*!< 摆动轴电机 ID。 */
    uint8_t gripper_servo_id; /*!< 夹爪舵机 ID。 */
    AdvanceArm_MotorDirection_t lift_down_direction; /*!< 升降轴下降方向。 */
    AdvanceArm_MotorDirection_t swing_extend_direction; /*!< 摆动轴伸出方向。 */
    uint16_t lift_velocity; /*!< 升降轴运动速度。 */
    uint16_t swing_velocity; /*!< 摆动轴运动速度。 */
    uint8_t lift_acceleration; /*!< 升降轴加速度。 */
    uint8_t swing_acceleration; /*!< 摆动轴加速度。 */
    int32_t position_tolerance_pulse; /*!< 位置到达判定容差，单位为脉冲。 */
  } AdvanceArm_Config_t;

  /* 仅用于兼容一个协议发布周期的 36 字节取放参数。 */
  /** @brief 兼容旧版通信协议的任务参数。 */
  typedef struct
  {
    uint32_t swing_extend_pulse; /*!< 摆动轴伸出位移，单位为脉冲。 */
    uint32_t lift_lower_pulse; /*!< 升降轴下降位移，单位为脉冲。 */
    uint32_t lift_raise_pulse; /*!< 升降轴上升位移，单位为脉冲。 */
    uint32_t swing_retract_pulse; /*!< 摆动轴收回位移，单位为脉冲。 */
    int32_t gripper_close_position; /*!< 夹爪闭合位置。 */
    int32_t gripper_release_position; /*!< 夹爪释放位置。 */
    uint16_t gripper_acceleration; /*!< 夹爪运动加速度。 */
    uint16_t gripper_speed; /*!< 夹爪运动速度。 */
    uint32_t servo_wait_ms; /*!< 夹爪动作后的等待时间，单位为 ms。 */
    uint32_t task_timeout_ms; /*!< 任务超时时间，单位为 ms。 */
  } AdvanceArm_TaskPlan_t;

  /** @brief 机械臂运行时状态快照。 */
  typedef struct
  {
    AdvanceArm_PositionValidity_t lift_position_validity; /*!< 升降轴位置有效性。 */
    AdvanceArm_PositionValidity_t swing_position_validity; /*!< 摆动轴位置有效性。 */
    AdvanceArm_RunState_t run_state; /*!< 当前运行状态。 */
    AdvanceArm_TaskType_t task_type; /*!< 当前任务类型。 */
    uint8_t step; /*!< 当前任务步骤。 */
    int32_t lift_current_pulse; /*!< 升降轴当前位置，单位为脉冲。 */
    int32_t lift_target_pulse; /*!< 升降轴目标位置，单位为脉冲。 */
    int32_t swing_current_pulse; /*!< 摆动轴当前位置，单位为脉冲。 */
    int32_t swing_target_pulse; /*!< 摆动轴目标位置，单位为脉冲。 */
    uint32_t updated_tick; /*!< 状态更新时间，单位为 ms。 */
  } AdvanceArm_RuntimeStatus_t;

  /** @brief 初始化机械臂控制模块。 */
  void AdvanceArm_Init(void);
  /** @brief 周期执行机械臂状态机。 */
  void AdvanceArm_Poll(void);
  /** @brief 启动抓取任务。 */
  AdvanceArm_Status_t AdvanceArm_StartPick(void);
  /** @brief 启动放置任务。 */
  AdvanceArm_Status_t AdvanceArm_StartPlace(void);
  /** @brief 启动指定类型的兼容任务。 */
  AdvanceArm_Status_t AdvanceArm_StartLegacyTask(AdvanceArm_TaskType_t task_type,
                                                  const AdvanceArm_TaskPlan_t *plan);
  /** @brief 将当前机械臂位置记录为零点。 */
  AdvanceArm_Status_t AdvanceArm_ResetZero(void);
  /** @brief 判断配置是否与当前固定机械结构匹配。 */
  bool AdvanceArm_IsFixedConfig(const AdvanceArm_Config_t *config);
  /** @brief 中止当前任务并停止运动。 */
  void AdvanceArm_Abort(void);
  /** @brief 执行紧急停止。 */
  void AdvanceArm_EStop(void);
  /** @brief 使已缓存的坐标状态失效。 */
  void AdvanceArm_InvalidateCoordinates(void);
  /** @brief 获取机械臂当前运行状态。 */
  AdvanceArm_Status_t AdvanceArm_GetStatus(AdvanceArm_RuntimeStatus_t *status);

  /** @brief 控制夹爪打开或闭合。 */
  AdvanceArm_Status_t AdvanceArm_Grab(bool closed);
  /** @brief 向指定舵机发送位置控制命令。 */
  BusServo_Status_t AdvanceArm_SetServo(uint8_t servo_id, uint16_t acceleration,
                                        int32_t position, uint16_t speed);
  /** @brief 控制指定轴按绝对或相对方式运动。 */
  AdvanceArm_Status_t AdvanceArm_MoveAxis(uint8_t motor_id,
                                          AdvanceArm_MotorDirection_t direction,
                                          uint16_t velocity, uint8_t acceleration,
                                          uint32_t pulse_count, bool relative,
                                          bool synchronous);
  /** @brief 停止指定轴运动。 */
  AdvanceArm_Status_t AdvanceArm_StopAxis(uint8_t motor_id, bool synchronous);

#ifdef __cplusplus
}
#endif

#endif
