#ifndef __ADVANCE_ARM_H__
#define __ADVANCE_ARM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "drive_bus_servo.h"

  /** @brief 机械臂电机运动方向。 */
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

  /** @brief 机械臂位置有效性状态。 */
  typedef enum
  {
    ADVANCE_ARM_POSITION_UNKNOWN = 0,
    ADVANCE_ARM_POSITION_VALID,
    ADVANCE_ARM_POSITION_FAULT
  } AdvanceArm_PositionValidity_t;

  /** @brief 机械臂任务状态。 */
  typedef enum
  {
    ADVANCE_ARM_TASK_BOOT = 0,
    ADVANCE_ARM_TASK_READY,
    ADVANCE_ARM_TASK_PICK_EXTEND,
    ADVANCE_ARM_TASK_PICK_LOWER,
    ADVANCE_ARM_TASK_PICK_GRIP,
    ADVANCE_ARM_TASK_PICK_LIFT,
    ADVANCE_ARM_TASK_PICK_RETRACT,
    ADVANCE_ARM_TASK_PLACE_EXTEND,
    ADVANCE_ARM_TASK_PLACE_LOWER,
    ADVANCE_ARM_TASK_PLACE_RELEASE,
    ADVANCE_ARM_TASK_PLACE_LIFT,
    ADVANCE_ARM_TASK_PLACE_RETRACT,
    ADVANCE_ARM_TASK_COMPLETE,
    ADVANCE_ARM_TASK_FAULT,
    ADVANCE_ARM_TASK_ESTOP
  } AdvanceArm_TaskState_t;

  /** @brief 机械臂电机、舵机及运动参数配置。 */
  typedef struct
  {
    uint8_t lift_motor_id; /*!< 升降电机 ID。 */
    uint8_t swing_motor_id; /*!< 摆臂电机 ID。 */
    uint8_t gripper_servo_id; /*!< 夹爪舵机 ID。 */
    AdvanceArm_MotorDirection_t lift_down_direction; /*!< 升降下降方向。 */
    AdvanceArm_MotorDirection_t swing_extend_direction; /*!< 摆臂伸出方向。 */
    uint16_t lift_velocity; /*!< 升降速度。 */
    uint16_t swing_velocity; /*!< 摆臂速度。 */
    uint8_t lift_acceleration; /*!< 升降加速度。 */
    uint8_t swing_acceleration; /*!< 摆臂加速度。 */
    int32_t position_tolerance_pulse; /*!< 位置到达容差，单位为脉冲。 */
  } AdvanceArm_Config_t;

  /* 所有脉冲均为从人工零点开始的相对位移，正方向由 Config 指定。 */
  /** @brief 机械臂取放任务的目标位置和时序参数。 */
  typedef struct
  {
    uint32_t swing_extend_pulse; /*!< 摆臂伸出目标脉冲。 */
    uint32_t lift_lower_pulse; /*!< 升降下降目标脉冲。 */
    uint32_t lift_raise_pulse; /*!< 升降抬升目标脉冲。 */
    uint32_t swing_retract_pulse; /*!< 摆臂收回目标脉冲。 */
    int32_t gripper_close_position; /*!< 夹爪闭合位置。 */
    int32_t gripper_release_position; /*!< 夹爪释放位置。 */
    uint16_t gripper_acceleration; /*!< 夹爪加速度。 */
    uint16_t gripper_speed; /*!< 夹爪速度。 */
    uint32_t servo_wait_ms; /*!< 舵机动作后的等待时间，单位为 ms。 */
    uint32_t task_timeout_ms; /*!< 单个任务超时时间，单位为 ms。 */
  } AdvanceArm_TaskPlan_t;

  /** @brief 机械臂当前执行状态和反馈位置。 */
  typedef struct
  {
    AdvanceArm_PositionValidity_t lift_position_validity; /*!< 升降位置有效性。 */
    AdvanceArm_PositionValidity_t swing_position_validity; /*!< 摆臂位置有效性。 */
    AdvanceArm_TaskState_t task_state; /*!< 当前任务状态。 */
    int32_t lift_current_pulse; /*!< 升降当前位置脉冲。 */
    int32_t lift_target_pulse; /*!< 升降目标位置脉冲。 */
    int32_t swing_current_pulse; /*!< 摆臂当前位置脉冲。 */
    int32_t swing_target_pulse; /*!< 摆臂目标位置脉冲。 */
    uint32_t updated_tick; /*!< 状态更新时间，单位为 ms。 */
  } AdvanceArm_RuntimeStatus_t;

  /** @brief 初始化机械臂控制模块。 */
  void AdvanceArm_Init(void);
  /** @brief 设置机械臂配置。 @param config 配置结构体指针。 @return 配置结果状态。 */
  AdvanceArm_Status_t AdvanceArm_Configure(const AdvanceArm_Config_t *config);
  /** @brief 轮询推进机械臂任务状态机。 */
  void AdvanceArm_Poll(void);
  /** @brief 启动取物任务。 @param plan 取物任务计划指针。 @return 启动结果状态。 */
  AdvanceArm_Status_t AdvanceArm_StartPick(const AdvanceArm_TaskPlan_t *plan);
  /** @brief 启动放物任务。 @param plan 放物任务计划指针。 @return 启动结果状态。 */
  AdvanceArm_Status_t AdvanceArm_StartPlace(const AdvanceArm_TaskPlan_t *plan);
  /** @brief 中止当前机械臂任务。 */
  void AdvanceArm_Abort(void);
  /** @brief 执行机械臂紧急停止。 */
  void AdvanceArm_EStop(void);
  /** @brief 使机械臂位置反馈失效，等待重新建立坐标。 */
  void AdvanceArm_InvalidateCoordinates(void);
  /** @brief 获取机械臂运行状态。 @param status 输出状态结构体指针。 @return 获取结果状态。 */
  AdvanceArm_Status_t AdvanceArm_GetStatus(AdvanceArm_RuntimeStatus_t *status);

  /** @brief 控制夹爪舵机转动托盘。 @param servo_id 舵机 ID。 @param acceleration 加速度。 @param position 目标位置。 @param speed 目标速度。 @return 舵机操作状态。 */
  BusServo_Status_t AdvanceArm_RotatePlate(uint8_t servo_id, uint16_t acceleration,
                                           int32_t position, uint16_t speed);
  /** @brief 控制夹爪舵机转动底座。 @param servo_id 舵机 ID。 @param acceleration 加速度。 @param position 目标位置。 @param speed 目标速度。 @return 舵机操作状态。 */
  BusServo_Status_t AdvanceArm_RotateBase(uint8_t servo_id, uint16_t acceleration,
                                          int32_t position, uint16_t speed);
  /* 非阻塞夹爪动作；完成状态由 AdvanceArm_Poll 推进。 */
  /** @brief 发起非阻塞夹爪动作。 @param closed true-闭合，false-释放。 @return 动作启动状态。 */
  AdvanceArm_Status_t AdvanceArm_Grab(uint8_t servo_id, bool closed,
                                      int32_t release_position, int32_t close_position,
                                      uint16_t acceleration, uint16_t speed);
  /** @brief 发起升降电机运动。 @param motor_id 电机 ID。 @return 电机操作状态。 */
  AdvanceArm_Status_t AdvanceArm_MoveLift(uint8_t motor_id,
                                          AdvanceArm_MotorDirection_t direction,
                                          uint16_t velocity, uint8_t acceleration,
                                          uint32_t pulse_count, bool relative,
                                          bool synchronous);
  /** @brief 发起摆臂电机运动。 @param motor_id 电机 ID。 @return 电机操作状态。 */
  AdvanceArm_Status_t AdvanceArm_MoveSwing(uint8_t motor_id,
                                           AdvanceArm_MotorDirection_t direction,
                                           uint16_t velocity, uint8_t acceleration,
                                           uint32_t pulse_count, bool relative,
                                           bool synchronous);
  /** @brief 停止指定电机。 @param motor_id 电机 ID。 @param synchronous 是否同步执行。 @return 停止结果状态。 */
  AdvanceArm_Status_t AdvanceArm_StopMotor(uint8_t motor_id, bool synchronous);

#ifdef __cplusplus
}
#endif

#endif
