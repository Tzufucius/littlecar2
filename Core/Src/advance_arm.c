/*
 * @file advance_arm.c
 * @brief 机械臂任务协调器（2轴步进 + 1轴总线舵机）
 *
 * 设计逻辑：
 * 1. 组合：将两个 ZDT 步进电机（升降/伸缩）与一个总线舵机（夹爪）逻辑绑定。
 * 2. 状态机：通过 AdvanceArm_Poll 驱动非阻塞任务序列，支持 Pick/Place 原子操作。
 * 3. 坐标系：系统不带自动寻原点功能，需手动归位后调用 ResetZero 建立软件坐标。
 */

#include "advance_arm.h"
#include <limits.h>
#include <string.h>
#include "advance_chassis.h"
#include "drive_emm.h"
#include "sensor_limit.h"

/* --- 内部私有定义 --- */

/** @brief 原子动作指令 */
typedef enum
{
  ARM_ACTION_MOVE_SWING = 0, /* 相对运动：伸缩轴 */
  ARM_ACTION_MOVE_LIFT,      /* 相对运动：升降轴 */
  ARM_ACTION_SET_GRIPPER,    /* 位置控制：夹爪舵机 */
  ARM_ACTION_WAIT            /* 时间延时：单位 ms */
} AdvanceArm_ActionType_t;

/** @brief 动作描述符 */
typedef struct
{
  AdvanceArm_ActionType_t type;
  int32_t value;         /* 电机脉冲 / 舵机位置 / 延时时间 */
  uint16_t speed;        /* 运动速度 (等待动作不适用) */
  uint16_t acceleration; /* 运动加速度 (等待动作不适用) */
} AdvanceArm_Action_t;

/** @brief 关节轴实时信息 */
typedef struct
{
  uint8_t motor_id;
  AdvanceArm_PositionValidity_t validity; /* 坐标有效性状态 */
  int32_t zero_pulse;                     /* 全局零点对应的绝对脉冲值 */
  int32_t current_pulse;                  /* 相对于零点的偏移脉冲 */
  int32_t target_pulse;                   /* 目标相对脉冲 */
  SensorLimitId_t positive_limit;
  SensorLimitId_t negative_limit;
  AxisMotionState_t motion_state;
} AdvanceArm_Axis_t;

/** @brief 任务执行状态机 */
typedef struct
{
  const AdvanceArm_Action_t *actions;
  uint8_t action_count;
  AdvanceArm_TaskType_t task_type;
  uint8_t step;                /* 当前执行的步骤索引 */
  uint8_t action_started;      /* 标记当前步骤指令是否已下发 */
  uint32_t step_started_tick;  /* 步骤起始时间戳 */
  uint32_t task_deadline_tick; /* 任务整体超时截止点 */
} AdvanceArm_Executor_t;

/** @brief 机械臂全局控制块 */
typedef struct
{
  AdvanceArm_Axis_t lift;
  AdvanceArm_Axis_t swing;
  AdvanceArm_Executor_t executor;
  AdvanceArm_Action_t legacy_actions[6]; /* 兼容动态任务的临时存储 */
  AdvanceArm_RunState_t state;
  uint32_t state_tick;
  uint8_t configured;
  uint8_t zero_pending;
} AdvanceArm_Controller_t;

/* --- 固定配置与任务序列 --- */

static const AdvanceArm_Config_t g_arm_config = {
    .lift_motor_id = ARM_LIFT_MOTOR_ID,
    .swing_motor_id = ARM_SWING_MOTOR_ID,
    .gripper_servo_id = ARM_GRIPPER_SERVO_ID,
    .lift_down_direction = ADVANCE_ARM_MOTOR_DIRECTION_FORWARD,
    .swing_extend_direction = ADVANCE_ARM_MOTOR_DIRECTION_FORWARD,
    .lift_velocity = ARM_LIFT_SPEED,
    .swing_velocity = ARM_SWING_SPEED,
    .lift_acceleration = ARM_LIFT_ACC,
    .swing_acceleration = ARM_SWING_ACC,
    .position_tolerance_pulse = ARM_POSITION_TOLERANCE_PULSE};

/* 固定抓取流程：伸出 -> 下降 -> 闭合 -> 等待 -> 上升 -> 回收 */
static const AdvanceArm_Action_t g_pick_actions[] = {
    {ARM_ACTION_MOVE_SWING, ARM_SWING_EXTEND_PULSE, ARM_SWING_SPEED, ARM_SWING_ACC},
    {ARM_ACTION_MOVE_LIFT, ARM_LIFT_LOWER_PULSE, ARM_LIFT_SPEED, ARM_LIFT_ACC},
    {ARM_ACTION_SET_GRIPPER, ARM_GRIPPER_CLOSE_POS, ARM_GRIPPER_SPEED, ARM_GRIPPER_ACC},
    {ARM_ACTION_WAIT, (int32_t)ARM_GRIPPER_WAIT_MS, 0U, 0U},
    {ARM_ACTION_MOVE_LIFT, -ARM_LIFT_RAISE_PULSE, ARM_LIFT_SPEED, ARM_LIFT_ACC},
    {ARM_ACTION_MOVE_SWING, -ARM_SWING_RETRACT_PULSE, ARM_SWING_SPEED, ARM_SWING_ACC}};

/* 固定放置流程：伸出 -> 下降 -> 打开 -> 等待 -> 上升 -> 回收 */
static const AdvanceArm_Action_t g_place_actions[] = {
    {ARM_ACTION_MOVE_SWING, ARM_SWING_EXTEND_PULSE, ARM_SWING_SPEED, ARM_SWING_ACC},
    {ARM_ACTION_MOVE_LIFT, ARM_LIFT_LOWER_PULSE, ARM_LIFT_SPEED, ARM_LIFT_ACC},
    {ARM_ACTION_SET_GRIPPER, ARM_GRIPPER_OPEN_POS, ARM_GRIPPER_SPEED, ARM_GRIPPER_ACC},
    {ARM_ACTION_WAIT, (int32_t)ARM_GRIPPER_WAIT_MS, 0U, 0U},
    {ARM_ACTION_MOVE_LIFT, -ARM_LIFT_RAISE_PULSE, ARM_LIFT_SPEED, ARM_LIFT_ACC},
    {ARM_ACTION_MOVE_SWING, -ARM_SWING_RETRACT_PULSE, ARM_SWING_SPEED, ARM_SWING_ACC}};

static AdvanceArm_Controller_t g_advance_arm = {0};

/* --- 核心内部逻辑 --- */

static void AdvanceArm_EnterState(AdvanceArm_RunState_t state)
{
  g_advance_arm.state = state;
  g_advance_arm.state_tick = HAL_GetTick();
}

static uint8_t AdvanceArm_IsChassisMotorId(uint8_t motor_id)
{
  return ((motor_id == CHASSIS_MOTOR_LF_ID) ||
          (motor_id == CHASSIS_MOTOR_RF_ID) ||
          (motor_id == CHASSIS_MOTOR_LR_ID) ||
          (motor_id == CHASSIS_MOTOR_RR_ID))
             ? 1U
             : 0U;
}

static uint8_t AdvanceArm_ConfigIsValid(const AdvanceArm_Config_t *config)
{
  if ((config == NULL) || (config->lift_motor_id == 0U) ||
      (config->swing_motor_id == 0U) || (config->gripper_servo_id == 0U) ||
      (config->lift_motor_id == config->swing_motor_id) ||
      (AdvanceArm_IsChassisMotorId(config->lift_motor_id) != 0U) ||
      (AdvanceArm_IsChassisMotorId(config->swing_motor_id) != 0U) ||
      (config->position_tolerance_pulse < 0))
  {
    return 0U;
  }
  return 1U;
}

static AdvanceArm_Axis_t *AdvanceArm_FindAxis(uint8_t motor_id)
{
  if (g_advance_arm.lift.motor_id == motor_id)
  {
    return &g_advance_arm.lift;
  }
  if (g_advance_arm.swing.motor_id == motor_id)
  {
    return &g_advance_arm.swing;
  }
  return NULL;
}

static SensorLimitId_t AdvanceArm_GetDirectionLimit(const AdvanceArm_Axis_t *axis,
                                                     AdvanceArm_MotorDirection_t direction)
{
  return (direction == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD)
             ? axis->positive_limit
             : axis->negative_limit;
}

static AxisMotionState_t AdvanceArm_GetMovingState(AdvanceArm_MotorDirection_t direction)
{
  return (direction == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD)
             ? AXIS_STATE_MOVING_POSITIVE
             : AXIS_STATE_MOVING_NEGATIVE;
}

static AxisMotionState_t AdvanceArm_GetBlockedState(AdvanceArm_MotorDirection_t direction)
{
  return (direction == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD)
             ? AXIS_STATE_BLOCKED_POSITIVE
             : AXIS_STATE_BLOCKED_NEGATIVE;
}

static void AdvanceArm_CancelTaskForLimit(AdvanceArm_Axis_t *axis,
                                          AdvanceArm_MotorDirection_t direction)
{
  drive_emm_Stop_Now(axis->motor_id, false);
  axis->target_pulse = axis->current_pulse;
  axis->motion_state = AdvanceArm_GetBlockedState(direction);

  memset(&g_advance_arm.executor, 0, sizeof(g_advance_arm.executor));
  if (g_advance_arm.state == ADVANCE_ARM_RUN_RUNNING)
  {
    AdvanceArm_EnterState(ADVANCE_ARM_RUN_READY);
  }
}

static uint8_t AdvanceArm_CheckMovingLimit(AdvanceArm_Axis_t *axis)
{
  AdvanceArm_MotorDirection_t direction;

  if (axis->motion_state == AXIS_STATE_MOVING_POSITIVE)
  {
    direction = ADVANCE_ARM_MOTOR_DIRECTION_FORWARD;
  }
  else if (axis->motion_state == AXIS_STATE_MOVING_NEGATIVE)
  {
    direction = ADVANCE_ARM_MOTOR_DIRECTION_REVERSE;
  }
  else
  {
    return 0U;
  }

  if (!SensorLimit_IsActive(AdvanceArm_GetDirectionLimit(axis, direction)))
  {
    return 0U;
  }

  AdvanceArm_CancelTaskForLimit(axis, direction);
  return 1U;
}

static uint8_t AdvanceArm_UpdateAxis(AdvanceArm_Axis_t *axis)
{
  DriveEmm_MotorFeedback_t feedback;

  /* 获取电机反馈并检查健康状态 */
  if ((axis == NULL) || (axis->motor_id == 0U) ||
      (drive_emm_GetMotorFeedback(axis->motor_id, &feedback) != HAL_OK))
  {
    return 0U;
  }

  if (drive_emm_IsMotorFeedbackHealthy(axis->motor_id,
                                       DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS) == 0U)
  {
    if (axis->validity == ADVANCE_ARM_POSITION_VALID)
    {
      axis->validity = ADVANCE_ARM_POSITION_FAULT;
    }
    return 0U;
  }

  /* 映射绝对坐标到软件零点坐标 */
  axis->current_pulse = feedback.position - axis->zero_pulse;
  return 1U;
}

/** @brief 检查指定轴是否到达目标位置 */
static uint8_t AdvanceArm_AxisReached(const AdvanceArm_Axis_t *axis)
{
  if ((axis == NULL) || (axis->validity != ADVANCE_ARM_POSITION_VALID))
  {
    return 0U;
  }
  return drive_emm_IsMotorReached(axis->motor_id,
                                  axis->zero_pulse + axis->target_pulse,
                                  g_arm_config.position_tolerance_pulse,
                                  DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS);
}

static AdvanceArm_MotorDirection_t AdvanceArm_ReverseDirection(
    AdvanceArm_MotorDirection_t direction)
{
  return (direction == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD)
             ? ADVANCE_ARM_MOTOR_DIRECTION_REVERSE
             : ADVANCE_ARM_MOTOR_DIRECTION_FORWARD;
}

static uint32_t AdvanceArm_AbsActionValue(int32_t value)
{
  return (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
}

/** @brief 发送带符号位移指令到电机 */
static AdvanceArm_Status_t AdvanceArm_CommandSignedMove(
    AdvanceArm_Axis_t *axis, AdvanceArm_MotorDirection_t positive_direction,
    const AdvanceArm_Action_t *action)
{
  AdvanceArm_MotorDirection_t direction;

  if ((axis == NULL) || (action == NULL) ||
      (axis->validity != ADVANCE_ARM_POSITION_VALID) || (action->value == 0))
  {
    return ADVANCE_ARM_STATUS_NOT_READY;
  }

  /* 根据 value 的符号确定实际运动方向 */
  direction = (action->value > 0) ? positive_direction
                                  : AdvanceArm_ReverseDirection(positive_direction);

  return AdvanceArm_MoveAxis(axis->motor_id, direction, action->speed,
                             (uint8_t)action->acceleration,
                             AdvanceArm_AbsActionValue(action->value), true, false);
}

static void AdvanceArm_StopConfiguredAxes(void)
{
  if (g_advance_arm.configured != 0U)
  {
    (void)AdvanceArm_StopAxis(g_advance_arm.lift.motor_id, false);
    (void)AdvanceArm_StopAxis(g_advance_arm.swing.motor_id, false);
  }
}

static void AdvanceArm_SetFault(void)
{
  if (g_advance_arm.state != ADVANCE_ARM_RUN_FAULT)
  {
    AdvanceArm_StopConfiguredAxes();
  }
  g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_FAULT;
  g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_FAULT;
  g_advance_arm.lift.motion_state = AXIS_STATE_FAULT;
  g_advance_arm.swing.motion_state = AXIS_STATE_FAULT;
  g_advance_arm.zero_pending = 0U;
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_FAULT);
}

/** @brief 推进到任务序列的下一步 */
static void AdvanceArm_AdvanceStep(void)
{
  ++g_advance_arm.executor.step;
  g_advance_arm.executor.action_started = 0U;
  g_advance_arm.executor.step_started_tick = HAL_GetTick();
}

/** @brief 配置并启动新任务 */
static AdvanceArm_Status_t AdvanceArm_StartTask(
    AdvanceArm_TaskType_t task_type, const AdvanceArm_Action_t *actions,
    uint8_t action_count, uint32_t timeout_ms)
{
  if ((actions == NULL) || (action_count == 0U) || (timeout_ms == 0U) ||
      ((task_type != ADVANCE_ARM_TASK_PICK) &&
       (task_type != ADVANCE_ARM_TASK_PLACE)))
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }

  if (g_advance_arm.state == ADVANCE_ARM_RUN_RUNNING)
  {
    return ADVANCE_ARM_STATUS_BUSY;
  }

  if ((g_advance_arm.state != ADVANCE_ARM_RUN_READY) ||
      (g_advance_arm.lift.validity != ADVANCE_ARM_POSITION_VALID) ||
      (g_advance_arm.swing.validity != ADVANCE_ARM_POSITION_VALID))
  {
    return ADVANCE_ARM_STATUS_NOT_READY;
  }

  g_advance_arm.executor.actions = actions;
  g_advance_arm.executor.action_count = action_count;
  g_advance_arm.executor.task_type = task_type;
  g_advance_arm.executor.step = 0U;
  g_advance_arm.executor.action_started = 0U;
  g_advance_arm.executor.step_started_tick = HAL_GetTick();
  g_advance_arm.executor.task_deadline_tick = HAL_GetTick() + timeout_ms;
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_RUNNING);
  return ADVANCE_ARM_STATUS_OK;
}

/** @brief 任务执行器的具体实现（由 Poll 调用） */
static void AdvanceArm_RunCurrentAction(uint32_t now)
{
  const AdvanceArm_Action_t *action;
  AdvanceArm_Axis_t *axis;
  AdvanceArm_MotorDirection_t positive_direction;
  AdvanceArm_Status_t status;

  /* 检查序列是否结束 */
  if (g_advance_arm.executor.step >= g_advance_arm.executor.action_count)
  {
    AdvanceArm_EnterState(ADVANCE_ARM_RUN_COMPLETE);
    return;
  }

  action = &g_advance_arm.executor.actions[g_advance_arm.executor.step];
  switch (action->type)
  {
  case ARM_ACTION_MOVE_SWING:
  case ARM_ACTION_MOVE_LIFT:
    axis = (action->type == ARM_ACTION_MOVE_SWING) ? &g_advance_arm.swing
                                                   : &g_advance_arm.lift;
    positive_direction = (action->type == ARM_ACTION_MOVE_SWING)
                             ? g_arm_config.swing_extend_direction
                             : g_arm_config.lift_down_direction;
    if (g_advance_arm.executor.action_started == 0U)
    {
      /* 指令只下发一次 */
      status = AdvanceArm_CommandSignedMove(axis, positive_direction, action);
      if (status != ADVANCE_ARM_STATUS_OK)
      {
        AdvanceArm_SetFault();
        return;
      }
      g_advance_arm.executor.action_started = 1U;
      g_advance_arm.executor.step_started_tick = now;
    }
    else if (AdvanceArm_AxisReached(axis) != 0U)
    {
      /* 等待回读确认到位 */
      AdvanceArm_AdvanceStep();
    }
    break;

  case ARM_ACTION_SET_GRIPPER:
    /* 舵机控制指令，命令成功即认为步骤开始（到位由随后的 WAIT 保证） */
    if (BusServo_SetPositionEx(g_arm_config.gripper_servo_id, action->acceleration,
                               action->value, action->speed) != drive_bus_servo_STATUS_OK)
    {
      AdvanceArm_SetFault();
      return;
    }
    AdvanceArm_AdvanceStep();
    break;

  case ARM_ACTION_WAIT:
    if (g_advance_arm.executor.action_started == 0U)
    {
      g_advance_arm.executor.action_started = 1U;
      g_advance_arm.executor.step_started_tick = now;
    }
    if ((now - g_advance_arm.executor.step_started_tick) >= (uint32_t)action->value)
    {
      AdvanceArm_AdvanceStep();
    }
    break;

  default:
    AdvanceArm_SetFault();
    break;
  }
}

/* 初始化机械臂控制器、注册电机反馈并进入启动状态。 */
void AdvanceArm_Init(void)
{
  /* 初始化只注册并监控两个步进轴，不会自动寻找机械零点。 */
  memset(&g_advance_arm, 0, sizeof(g_advance_arm));
  g_advance_arm.lift.motor_id = g_arm_config.lift_motor_id;
  g_advance_arm.swing.motor_id = g_arm_config.swing_motor_id;
  g_advance_arm.lift.positive_limit = SENSOR_LIMIT_LIFT_DOWN;
  g_advance_arm.lift.negative_limit = SENSOR_LIMIT_LIFT_UP;
  g_advance_arm.swing.positive_limit = SENSOR_LIMIT_SLIDE_FRONT;
  g_advance_arm.swing.negative_limit = SENSOR_LIMIT_SLIDE_REAR;
  g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.lift.motion_state = AXIS_STATE_IDLE;
  g_advance_arm.swing.motion_state = AXIS_STATE_IDLE;

  if ((AdvanceArm_ConfigIsValid(&g_arm_config) == 0U) ||
      (drive_emm_MonitorMotor(g_arm_config.lift_motor_id) != HAL_OK) ||
      (drive_emm_MonitorMotor(g_arm_config.swing_motor_id) != HAL_OK))
  {
    AdvanceArm_SetFault();
    return;
  }
  g_advance_arm.configured = 1U;
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_BOOT);
}

/* 检查传入配置是否与当前固定配置完全一致。 */
bool AdvanceArm_IsFixedConfig(const AdvanceArm_Config_t *config)
{
  if (AdvanceArm_ConfigIsValid(config) == 0U)
  {
    return false;
  }
  return (config->lift_motor_id == g_arm_config.lift_motor_id) &&
         (config->swing_motor_id == g_arm_config.swing_motor_id) &&
         (config->gripper_servo_id == g_arm_config.gripper_servo_id) &&
         (config->lift_down_direction == g_arm_config.lift_down_direction) &&
         (config->swing_extend_direction == g_arm_config.swing_extend_direction) &&
         (config->lift_velocity == g_arm_config.lift_velocity) &&
         (config->swing_velocity == g_arm_config.swing_velocity) &&
         (config->lift_acceleration == g_arm_config.lift_acceleration) &&
         (config->swing_acceleration == g_arm_config.swing_acceleration) &&
         (config->position_tolerance_pulse == g_arm_config.position_tolerance_pulse);
}

/* 清除旧零点，等待下一次轮询建立软件零点。 */
AdvanceArm_Status_t AdvanceArm_ResetZero(void)
{
  if (g_advance_arm.configured == 0U)
    return ADVANCE_ARM_STATUS_FAULT;
  if (g_advance_arm.state == ADVANCE_ARM_RUN_RUNNING)
    return ADVANCE_ARM_STATUS_BUSY;

  /*
   * 重置坐标系：清空当前零点偏移。
   * Poll 函数在确认电机连接正常后，会将当前的绝对位置记录为零点。
   */
  g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.lift.zero_pulse = 0;
  g_advance_arm.swing.zero_pulse = 0;
  g_advance_arm.lift.current_pulse = 0;
  g_advance_arm.swing.current_pulse = 0;
  g_advance_arm.lift.target_pulse = 0;
  g_advance_arm.swing.target_pulse = 0;
  g_advance_arm.lift.motion_state = AXIS_STATE_IDLE;
  g_advance_arm.swing.motion_state = AXIS_STATE_IDLE;

  memset(&g_advance_arm.executor, 0, sizeof(g_advance_arm.executor));
  g_advance_arm.zero_pending = 1U;
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_BOOT);
  return ADVANCE_ARM_STATUS_OK;
}

/* 使两个轴的坐标失效，并清除当前任务状态。 */
void AdvanceArm_InvalidateCoordinates(void)
{
  g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.lift.motion_state = AXIS_STATE_IDLE;
  g_advance_arm.swing.motion_state = AXIS_STATE_IDLE;
  memset(&g_advance_arm.executor, 0, sizeof(g_advance_arm.executor));
  g_advance_arm.zero_pending = 0U;
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_BOOT);
}

/* 中止当前任务、停止已配置轴并使坐标失效。 */
void AdvanceArm_Abort(void)
{
  /* 中止任务并失能轴 */
  AdvanceArm_StopConfiguredAxes();
  AdvanceArm_InvalidateCoordinates();
}

/* 执行机械臂紧急停止，并进入紧急停止状态。 */
void AdvanceArm_EStop(void)
{
  /* 紧急停止 */
  AdvanceArm_StopConfiguredAxes();
  AdvanceArm_InvalidateCoordinates();
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_ESTOP);
}

/* 读取机械臂当前状态、坐标和任务执行信息。 */
AdvanceArm_Status_t AdvanceArm_GetStatus(AdvanceArm_RuntimeStatus_t *status)
{
  if (status == NULL)
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }

  status->lift_position_validity = g_advance_arm.lift.validity;
  status->swing_position_validity = g_advance_arm.swing.validity;
  status->lift_motion_state = g_advance_arm.lift.motion_state;
  status->slide_motion_state = g_advance_arm.swing.motion_state;
  status->run_state = g_advance_arm.state;
  status->task_type = g_advance_arm.executor.task_type;
  status->step = g_advance_arm.executor.step;
  status->lift_current_pulse = g_advance_arm.lift.current_pulse;
  status->lift_target_pulse = g_advance_arm.lift.target_pulse;
  status->swing_current_pulse = g_advance_arm.swing.current_pulse;
  status->swing_target_pulse = g_advance_arm.swing.target_pulse;
  status->updated_tick = HAL_GetTick();

  return ADVANCE_ARM_STATUS_OK;
}

/* 周期性刷新反馈、建立零点并驱动任务状态机。 */
void AdvanceArm_Poll(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t lift_healthy;
  uint8_t swing_healthy;

  if (g_advance_arm.configured == 0U)
    return;

  /* 1. 刷新反馈 */
  lift_healthy = AdvanceArm_UpdateAxis(&g_advance_arm.lift);
  swing_healthy = AdvanceArm_UpdateAxis(&g_advance_arm.swing);

  /* 2. 检查异常 */
  if ((lift_healthy == 0U) || (swing_healthy == 0U))
  {
    if ((g_advance_arm.state != ADVANCE_ARM_RUN_FAULT) &&
        (g_advance_arm.state != ADVANCE_ARM_RUN_ESTOP) &&
        ((g_advance_arm.lift.validity == ADVANCE_ARM_POSITION_FAULT) ||
         (g_advance_arm.swing.validity == ADVANCE_ARM_POSITION_FAULT)))
    {
      AdvanceArm_SetFault();
    }
    return;
  }

  if ((AdvanceArm_CheckMovingLimit(&g_advance_arm.lift) != 0U) ||
      (AdvanceArm_CheckMovingLimit(&g_advance_arm.swing) != 0U))
  {
    return;
  }

  if ((g_advance_arm.lift.motion_state == AXIS_STATE_MOVING_POSITIVE ||
       g_advance_arm.lift.motion_state == AXIS_STATE_MOVING_NEGATIVE) &&
      (AdvanceArm_AxisReached(&g_advance_arm.lift) != 0U))
  {
    g_advance_arm.lift.motion_state = AXIS_STATE_IDLE;
  }

  if ((g_advance_arm.swing.motion_state == AXIS_STATE_MOVING_POSITIVE ||
       g_advance_arm.swing.motion_state == AXIS_STATE_MOVING_NEGATIVE) &&
      (AdvanceArm_AxisReached(&g_advance_arm.swing) != 0U))
  {
    g_advance_arm.swing.motion_state = AXIS_STATE_IDLE;
  }

  /* 3. 确立零点 (手动归零后的确认) */
  if ((g_advance_arm.state == ADVANCE_ARM_RUN_BOOT) &&
      (g_advance_arm.zero_pending != 0U))
  {
    /* 此刻反馈健康，建立坐标系 */
    g_advance_arm.lift.zero_pulse = g_advance_arm.lift.current_pulse + g_advance_arm.lift.zero_pulse;
    g_advance_arm.swing.zero_pulse = g_advance_arm.swing.current_pulse + g_advance_arm.swing.zero_pulse;
    g_advance_arm.lift.current_pulse = 0;
    g_advance_arm.swing.current_pulse = 0;
    g_advance_arm.lift.target_pulse = 0;
    g_advance_arm.swing.target_pulse = 0;
    g_advance_arm.lift.motion_state = AXIS_STATE_IDLE;
    g_advance_arm.swing.motion_state = AXIS_STATE_IDLE;
    g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_VALID;
    g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_VALID;
    g_advance_arm.zero_pending = 0U;
    AdvanceArm_EnterState(ADVANCE_ARM_RUN_READY);
    return;
  }

  /* 4. 任务调度与执行 */
  if (g_advance_arm.state == ADVANCE_ARM_RUN_COMPLETE)
  {
    memset(&g_advance_arm.executor, 0, sizeof(g_advance_arm.executor));
    AdvanceArm_EnterState(ADVANCE_ARM_RUN_READY);
    return;
  }

  if (g_advance_arm.state != ADVANCE_ARM_RUN_RUNNING)
    return;

  /* 超时判定 */
  if ((int32_t)(now - g_advance_arm.executor.task_deadline_tick) >= 0)
  {
    AdvanceArm_SetFault();
    return;
  }

  AdvanceArm_RunCurrentAction(now);
}

/* 启动一轮抓取动作任务。 */
AdvanceArm_Status_t AdvanceArm_StartPick(void)
{
  return AdvanceArm_StartTask(ADVANCE_ARM_TASK_PICK, g_pick_actions,
                              (uint8_t)(sizeof(g_pick_actions) / sizeof(g_pick_actions[0])),
                              ARM_TASK_TIMEOUT_MS);
}

/* 启动一轮放置动作任务。 */
AdvanceArm_Status_t AdvanceArm_StartPlace(void)
{
  return AdvanceArm_StartTask(ADVANCE_ARM_TASK_PLACE, g_place_actions,
                              (uint8_t)(sizeof(g_place_actions) / sizeof(g_place_actions[0])),
                              ARM_TASK_TIMEOUT_MS);
}

/* 启动由调用方提供动作序列的兼容任务。 */
AdvanceArm_Status_t AdvanceArm_StartLegacyTask(AdvanceArm_TaskType_t task_type,
                                               const AdvanceArm_TaskPlan_t *plan)
{
  int32_t gripper_position;

  if ((plan == NULL) || (plan->swing_extend_pulse > INT32_MAX) ||
      (plan->lift_lower_pulse > INT32_MAX) ||
      (plan->lift_raise_pulse > INT32_MAX) ||
      (plan->swing_retract_pulse > INT32_MAX) ||
      (plan->servo_wait_ms == 0U) || (plan->servo_wait_ms > INT32_MAX) ||
      (plan->task_timeout_ms == 0U))
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }

  gripper_position = (task_type == ADVANCE_ARM_TASK_PICK)
                         ? plan->gripper_close_position
                         : plan->gripper_release_position;

  /* 解析兼容协议包并生成任务指令流 */
  g_advance_arm.legacy_actions[0] = (AdvanceArm_Action_t){
      ARM_ACTION_MOVE_SWING, (int32_t)plan->swing_extend_pulse,
      g_arm_config.swing_velocity, g_arm_config.swing_acceleration};
  g_advance_arm.legacy_actions[1] = (AdvanceArm_Action_t){
      ARM_ACTION_MOVE_LIFT, (int32_t)plan->lift_lower_pulse,
      g_arm_config.lift_velocity, g_arm_config.lift_acceleration};
  g_advance_arm.legacy_actions[2] = (AdvanceArm_Action_t){
      ARM_ACTION_SET_GRIPPER, gripper_position, plan->gripper_speed,
      plan->gripper_acceleration};
  g_advance_arm.legacy_actions[3] = (AdvanceArm_Action_t){
      ARM_ACTION_WAIT, (int32_t)plan->servo_wait_ms, 0U, 0U};
  g_advance_arm.legacy_actions[4] = (AdvanceArm_Action_t){
      ARM_ACTION_MOVE_LIFT, -(int32_t)plan->lift_raise_pulse,
      g_arm_config.lift_velocity, g_arm_config.lift_acceleration};
  g_advance_arm.legacy_actions[5] = (AdvanceArm_Action_t){
      ARM_ACTION_MOVE_SWING, -(int32_t)plan->swing_retract_pulse,
      g_arm_config.swing_velocity, g_arm_config.swing_acceleration};

  return AdvanceArm_StartTask(task_type, g_advance_arm.legacy_actions, 6U,
                              plan->task_timeout_ms);
}

/* 控制夹爪打开或闭合。 */
AdvanceArm_Status_t AdvanceArm_Grab(bool closed)
{
  /* 独立操控夹爪（非阻塞） */
  return (BusServo_SetPositionEx(g_arm_config.gripper_servo_id, ARM_GRIPPER_ACC,
                                 closed ? ARM_GRIPPER_CLOSE_POS : ARM_GRIPPER_OPEN_POS,
                                 ARM_GRIPPER_SPEED) == drive_bus_servo_STATUS_OK)
             ? ADVANCE_ARM_STATUS_OK
             : ADVANCE_ARM_STATUS_FAULT;
}

/* 控制指定机械臂轴运动，并更新其预期目标坐标。 */
AdvanceArm_Status_t AdvanceArm_MoveAxis(uint8_t motor_id,
                                        AdvanceArm_MotorDirection_t direction,
                                        uint16_t velocity, uint8_t acceleration,
                                        uint32_t pulse_count, bool relative,
                                        bool synchronous)
{
  AdvanceArm_Axis_t *axis = AdvanceArm_FindAxis(motor_id);

  /* 安全锁定：坐标未建立时拒绝常规位移 */
  if ((axis == NULL) || (axis->validity != ADVANCE_ARM_POSITION_VALID) ||
      (pulse_count == 0U))
  {
    return ADVANCE_ARM_STATUS_NOT_READY;
  }

  if ((direction != ADVANCE_ARM_MOTOR_DIRECTION_FORWARD) &&
      (direction != ADVANCE_ARM_MOTOR_DIRECTION_REVERSE))
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }

  if (SensorLimit_IsActive(AdvanceArm_GetDirectionLimit(axis, direction)))
  {
    if (axis->motion_state == AdvanceArm_GetMovingState(direction))
    {
      AdvanceArm_CancelTaskForLimit(axis, direction);
    }
    else if ((axis->motion_state != AXIS_STATE_MOVING_POSITIVE) &&
             (axis->motion_state != AXIS_STATE_MOVING_NEGATIVE))
    {
      axis->motion_state = AdvanceArm_GetBlockedState(direction);
    }
    return ADVANCE_ARM_STATUS_LIMIT_BLOCKED;
  }

  drive_emm_Pos_Control(motor_id, (uint8_t)direction, velocity, acceleration,
                        pulse_count, !relative, synchronous);

  axis->motion_state = AdvanceArm_GetMovingState(direction);

  /* 更新预期的目标坐标 */
  axis->target_pulse = relative
                           ? axis->current_pulse +
                                 ((direction == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD)
                                      ? (int32_t)pulse_count
                                      : -(int32_t)pulse_count)
                           : ((direction == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD)
                                  ? (int32_t)pulse_count
                                  : -(int32_t)pulse_count);
  return ADVANCE_ARM_STATUS_OK;
}

/* 立即停止指定机械臂轴。 */
AdvanceArm_Status_t AdvanceArm_StopAxis(uint8_t motor_id, bool synchronous)
{
  AdvanceArm_Axis_t *axis = AdvanceArm_FindAxis(motor_id);

  if (axis == NULL)
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }
  drive_emm_Stop_Now(motor_id, synchronous);
  axis->target_pulse = axis->current_pulse;
  axis->motion_state = AXIS_STATE_IDLE;
  return ADVANCE_ARM_STATUS_OK;
}
