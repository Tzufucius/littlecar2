#include "advance_arm.h"

#include <limits.h>
#include <string.h>

#include "advance_chassis.h"
#include "drive_emm.h"

typedef enum
{
  ARM_ACTION_MOVE_SWING = 0,
  ARM_ACTION_MOVE_LIFT,
  ARM_ACTION_SET_GRIPPER,
  ARM_ACTION_WAIT
} AdvanceArm_ActionType_t;

typedef struct
{
  AdvanceArm_ActionType_t type;
  int32_t value;
  uint16_t speed;
  uint16_t acceleration;
} AdvanceArm_Action_t;

typedef struct
{
  uint8_t motor_id;
  AdvanceArm_PositionValidity_t validity;
  int32_t zero_pulse;
  int32_t current_pulse;
  int32_t target_pulse;
} AdvanceArm_Axis_t;

typedef struct
{
  const AdvanceArm_Action_t *actions;
  uint8_t action_count;
  AdvanceArm_TaskType_t task_type;
  uint8_t step;
  uint8_t action_started;
  uint32_t step_started_tick;
  uint32_t task_deadline_tick;
} AdvanceArm_Executor_t;

typedef struct
{
  AdvanceArm_Axis_t lift;
  AdvanceArm_Axis_t swing;
  AdvanceArm_Executor_t executor;
  AdvanceArm_Action_t legacy_actions[6];
  AdvanceArm_RunState_t state;
  uint32_t state_tick;
  uint8_t configured;
  uint8_t zero_pending;
} AdvanceArm_Controller_t;

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

static const AdvanceArm_Action_t g_pick_actions[] = {
    {ARM_ACTION_MOVE_SWING, ARM_SWING_EXTEND_PULSE, ARM_SWING_SPEED, ARM_SWING_ACC},
    {ARM_ACTION_MOVE_LIFT, ARM_LIFT_LOWER_PULSE, ARM_LIFT_SPEED, ARM_LIFT_ACC},
    {ARM_ACTION_SET_GRIPPER, ARM_GRIPPER_CLOSE_POS, ARM_GRIPPER_SPEED, ARM_GRIPPER_ACC},
    {ARM_ACTION_WAIT, (int32_t)ARM_GRIPPER_WAIT_MS, 0U, 0U},
    {ARM_ACTION_MOVE_LIFT, -ARM_LIFT_RAISE_PULSE, ARM_LIFT_SPEED, ARM_LIFT_ACC},
    {ARM_ACTION_MOVE_SWING, -ARM_SWING_RETRACT_PULSE, ARM_SWING_SPEED, ARM_SWING_ACC}};

static const AdvanceArm_Action_t g_place_actions[] = {
    {ARM_ACTION_MOVE_SWING, ARM_SWING_EXTEND_PULSE, ARM_SWING_SPEED, ARM_SWING_ACC},
    {ARM_ACTION_MOVE_LIFT, ARM_LIFT_LOWER_PULSE, ARM_LIFT_SPEED, ARM_LIFT_ACC},
    {ARM_ACTION_SET_GRIPPER, ARM_GRIPPER_OPEN_POS, ARM_GRIPPER_SPEED, ARM_GRIPPER_ACC},
    {ARM_ACTION_WAIT, (int32_t)ARM_GRIPPER_WAIT_MS, 0U, 0U},
    {ARM_ACTION_MOVE_LIFT, -ARM_LIFT_RAISE_PULSE, ARM_LIFT_SPEED, ARM_LIFT_ACC},
    {ARM_ACTION_MOVE_SWING, -ARM_SWING_RETRACT_PULSE, ARM_SWING_SPEED, ARM_SWING_ACC}};

static AdvanceArm_Controller_t g_advance_arm = {0};

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

static uint8_t AdvanceArm_UpdateAxis(AdvanceArm_Axis_t *axis)
{
  DriveEmm_MotorFeedback_t feedback;

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
  axis->current_pulse = feedback.position - axis->zero_pulse;
  return 1U;
}

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
  g_advance_arm.zero_pending = 0U;
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_FAULT);
}

static void AdvanceArm_AdvanceStep(void)
{
  ++g_advance_arm.executor.step;
  g_advance_arm.executor.action_started = 0U;
  g_advance_arm.executor.step_started_tick = HAL_GetTick();
}

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

static void AdvanceArm_RunCurrentAction(uint32_t now)
{
  const AdvanceArm_Action_t *action;
  AdvanceArm_Axis_t *axis;
  AdvanceArm_MotorDirection_t positive_direction;
  AdvanceArm_Status_t status;

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
      AdvanceArm_AdvanceStep();
    }
    break;

  case ARM_ACTION_SET_GRIPPER:
    if (AdvanceArm_SetServo(g_arm_config.gripper_servo_id, action->acceleration,
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

void AdvanceArm_Init(void)
{
  memset(&g_advance_arm, 0, sizeof(g_advance_arm));
  g_advance_arm.lift.motor_id = g_arm_config.lift_motor_id;
  g_advance_arm.swing.motor_id = g_arm_config.swing_motor_id;
  g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_UNKNOWN;

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

AdvanceArm_Status_t AdvanceArm_ResetZero(void)
{
  if (g_advance_arm.configured == 0U)
  {
    return ADVANCE_ARM_STATUS_FAULT;
  }
  if (g_advance_arm.state == ADVANCE_ARM_RUN_RUNNING)
  {
    return ADVANCE_ARM_STATUS_BUSY;
  }

  g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.lift.zero_pulse = 0;
  g_advance_arm.swing.zero_pulse = 0;
  g_advance_arm.lift.current_pulse = 0;
  g_advance_arm.swing.current_pulse = 0;
  g_advance_arm.lift.target_pulse = 0;
  g_advance_arm.swing.target_pulse = 0;
  memset(&g_advance_arm.executor, 0, sizeof(g_advance_arm.executor));
  g_advance_arm.zero_pending = 1U;
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_BOOT);
  return ADVANCE_ARM_STATUS_OK;
}

void AdvanceArm_InvalidateCoordinates(void)
{
  g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  memset(&g_advance_arm.executor, 0, sizeof(g_advance_arm.executor));
  g_advance_arm.zero_pending = 0U;
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_BOOT);
}

void AdvanceArm_Abort(void)
{
  AdvanceArm_StopConfiguredAxes();
  AdvanceArm_InvalidateCoordinates();
}

void AdvanceArm_EStop(void)
{
  AdvanceArm_StopConfiguredAxes();
  AdvanceArm_InvalidateCoordinates();
  AdvanceArm_EnterState(ADVANCE_ARM_RUN_ESTOP);
}

void AdvanceArm_Poll(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t lift_healthy;
  uint8_t swing_healthy;

  if (g_advance_arm.configured == 0U)
  {
    return;
  }

  lift_healthy = AdvanceArm_UpdateAxis(&g_advance_arm.lift);
  swing_healthy = AdvanceArm_UpdateAxis(&g_advance_arm.swing);
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

  if ((g_advance_arm.state == ADVANCE_ARM_RUN_BOOT) &&
      (g_advance_arm.zero_pending != 0U))
  {
    g_advance_arm.lift.zero_pulse = g_advance_arm.lift.current_pulse;
    g_advance_arm.swing.zero_pulse = g_advance_arm.swing.current_pulse;
    g_advance_arm.lift.current_pulse = 0;
    g_advance_arm.swing.current_pulse = 0;
    g_advance_arm.lift.target_pulse = 0;
    g_advance_arm.swing.target_pulse = 0;
    g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_VALID;
    g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_VALID;
    g_advance_arm.zero_pending = 0U;
    AdvanceArm_EnterState(ADVANCE_ARM_RUN_READY);
    return;
  }

  if (g_advance_arm.state == ADVANCE_ARM_RUN_COMPLETE)
  {
    memset(&g_advance_arm.executor, 0, sizeof(g_advance_arm.executor));
    AdvanceArm_EnterState(ADVANCE_ARM_RUN_READY);
    return;
  }
  if (g_advance_arm.state != ADVANCE_ARM_RUN_RUNNING)
  {
    return;
  }
  if ((int32_t)(now - g_advance_arm.executor.task_deadline_tick) >= 0)
  {
    AdvanceArm_SetFault();
    return;
  }
  AdvanceArm_RunCurrentAction(now);
}

AdvanceArm_Status_t AdvanceArm_StartPick(void)
{
  return AdvanceArm_StartTask(ADVANCE_ARM_TASK_PICK, g_pick_actions,
                              (uint8_t)(sizeof(g_pick_actions) / sizeof(g_pick_actions[0])),
                              ARM_TASK_TIMEOUT_MS);
}

AdvanceArm_Status_t AdvanceArm_StartPlace(void)
{
  return AdvanceArm_StartTask(ADVANCE_ARM_TASK_PLACE, g_place_actions,
                              (uint8_t)(sizeof(g_place_actions) / sizeof(g_place_actions[0])),
                              ARM_TASK_TIMEOUT_MS);
}

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

AdvanceArm_Status_t AdvanceArm_GetStatus(AdvanceArm_RuntimeStatus_t *status)
{
  if (status == NULL)
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }
  *status = (AdvanceArm_RuntimeStatus_t){
      .lift_position_validity = g_advance_arm.lift.validity,
      .swing_position_validity = g_advance_arm.swing.validity,
      .run_state = g_advance_arm.state,
      .task_type = g_advance_arm.executor.task_type,
      .step = g_advance_arm.executor.step,
      .lift_current_pulse = g_advance_arm.lift.current_pulse,
      .lift_target_pulse = g_advance_arm.lift.target_pulse,
      .swing_current_pulse = g_advance_arm.swing.current_pulse,
      .swing_target_pulse = g_advance_arm.swing.target_pulse,
      .updated_tick = HAL_GetTick()};
  return ADVANCE_ARM_STATUS_OK;
}

AdvanceArm_Status_t AdvanceArm_Grab(bool closed)
{
  return (AdvanceArm_SetServo(g_arm_config.gripper_servo_id, ARM_GRIPPER_ACC,
                              closed ? ARM_GRIPPER_CLOSE_POS : ARM_GRIPPER_OPEN_POS,
                              ARM_GRIPPER_SPEED) == drive_bus_servo_STATUS_OK)
             ? ADVANCE_ARM_STATUS_OK
             : ADVANCE_ARM_STATUS_FAULT;
}

BusServo_Status_t AdvanceArm_SetServo(uint8_t servo_id, uint16_t acceleration,
                                      int32_t position, uint16_t speed)
{
  return BusServo_SetPositionEx(servo_id, acceleration, position, speed);
}

AdvanceArm_Status_t AdvanceArm_MoveAxis(uint8_t motor_id,
                                        AdvanceArm_MotorDirection_t direction,
                                        uint16_t velocity, uint8_t acceleration,
                                        uint32_t pulse_count, bool relative,
                                        bool synchronous)
{
  AdvanceArm_Axis_t *axis = AdvanceArm_FindAxis(motor_id);

  if ((axis == NULL) || (axis->validity != ADVANCE_ARM_POSITION_VALID) ||
      (pulse_count == 0U))
  {
    return ADVANCE_ARM_STATUS_NOT_READY;
  }
  drive_emm_Pos_Control(motor_id, (uint8_t)direction, velocity, acceleration,
                        pulse_count, !relative, synchronous);
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

AdvanceArm_Status_t AdvanceArm_StopAxis(uint8_t motor_id, bool synchronous)
{
  if (AdvanceArm_FindAxis(motor_id) == NULL)
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }
  drive_emm_Stop_Now(motor_id, synchronous);
  return ADVANCE_ARM_STATUS_OK;
}
