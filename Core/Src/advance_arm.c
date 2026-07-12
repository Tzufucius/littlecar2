#include "advance_arm.h"

#include <string.h>

#include "drive_emm.h"

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
  AdvanceArm_Config_t config;
  AdvanceArm_TaskPlan_t plan;
  AdvanceArm_Axis_t lift;
  AdvanceArm_Axis_t swing;
  AdvanceArm_TaskState_t state;
  uint32_t state_tick;
  uint32_t deadline_tick;
  uint8_t configured;
  uint8_t active;
  uint8_t faulted;
} AdvanceArm_Controller_t;

static AdvanceArm_Controller_t g_advance_arm = {0};

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

static void AdvanceArm_EnterState(AdvanceArm_TaskState_t state)
{
  g_advance_arm.state = state;
  g_advance_arm.state_tick = HAL_GetTick();
}

static void AdvanceArm_SetFault(void)
{
  g_advance_arm.faulted = 1U;
  g_advance_arm.active = 0U;
  g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_FAULT;
  g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_FAULT;
  AdvanceArm_EnterState(ADVANCE_ARM_TASK_FAULT);
}

static uint8_t AdvanceArm_UpdateAxis(AdvanceArm_Axis_t *axis)
{
  DriveEmm_MotorFeedback_t feedback;

  if ((axis == NULL) || (axis->motor_id == 0U) ||
      (drive_emm_GetMotorFeedback(axis->motor_id, &feedback) != HAL_OK))
  {
    return 0U;
  }
  if (drive_emm_IsMotorFeedbackHealthy(axis->motor_id, DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS) == 0U)
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
  return drive_emm_IsMotorReached(axis->motor_id, axis->zero_pulse + axis->target_pulse,
                                  g_advance_arm.config.position_tolerance_pulse,
                                  DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS);
}

static AdvanceArm_Status_t AdvanceArm_CommandRelative(AdvanceArm_Axis_t *axis,
                                                       AdvanceArm_MotorDirection_t direction,
                                                       uint16_t velocity, uint8_t acceleration,
                                                       uint32_t pulse_count)
{
  if ((axis == NULL) || (axis->validity != ADVANCE_ARM_POSITION_VALID))
  {
    return ADVANCE_ARM_STATUS_NOT_READY;
  }
  drive_emm_Pos_Control(axis->motor_id, (uint8_t)direction, velocity, acceleration,
                        pulse_count, false, false);
  axis->target_pulse = axis->current_pulse +
      ((direction == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD) ? (int32_t)pulse_count : -(int32_t)pulse_count);
  return ADVANCE_ARM_STATUS_OK;
}

static AdvanceArm_Status_t AdvanceArm_StartTask(const AdvanceArm_TaskPlan_t *plan,
                                                 AdvanceArm_TaskState_t first_state)
{
  if ((plan == NULL) || (g_advance_arm.configured == 0U))
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }
  if ((g_advance_arm.state != ADVANCE_ARM_TASK_READY) ||
      (g_advance_arm.lift.validity != ADVANCE_ARM_POSITION_VALID) ||
      (g_advance_arm.swing.validity != ADVANCE_ARM_POSITION_VALID))
  {
    return ADVANCE_ARM_STATUS_NOT_READY;
  }
  if ((plan->task_timeout_ms == 0U) || (plan->servo_wait_ms == 0U))
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }
  g_advance_arm.plan = *plan;
  g_advance_arm.active = 1U;
  g_advance_arm.faulted = 0U;
  g_advance_arm.deadline_tick = HAL_GetTick() + plan->task_timeout_ms;
  AdvanceArm_EnterState(first_state);
  return ADVANCE_ARM_STATUS_OK;
}

void AdvanceArm_Init(void)
{
  memset(&g_advance_arm, 0, sizeof(g_advance_arm));
  AdvanceArm_EnterState(ADVANCE_ARM_TASK_BOOT);
}

AdvanceArm_Status_t AdvanceArm_Configure(const AdvanceArm_Config_t *config)
{
  if ((config == NULL) || (config->lift_motor_id == 0U) || (config->swing_motor_id == 0U) ||
      (config->lift_motor_id == config->swing_motor_id) || (config->gripper_servo_id == 0U) ||
      (config->position_tolerance_pulse < 0) ||
      (drive_emm_MonitorMotor(config->lift_motor_id) != HAL_OK) ||
      (drive_emm_MonitorMotor(config->swing_motor_id) != HAL_OK))
  {
    return ADVANCE_ARM_STATUS_INVALID_PARAM;
  }
  g_advance_arm.config = *config;
  g_advance_arm.lift = (AdvanceArm_Axis_t){.motor_id = config->lift_motor_id,
                                           .validity = ADVANCE_ARM_POSITION_UNKNOWN};
  g_advance_arm.swing = (AdvanceArm_Axis_t){.motor_id = config->swing_motor_id,
                                            .validity = ADVANCE_ARM_POSITION_UNKNOWN};
  g_advance_arm.configured = 1U;
  g_advance_arm.active = 0U;
  g_advance_arm.faulted = 0U;
  AdvanceArm_EnterState(ADVANCE_ARM_TASK_BOOT);
  return ADVANCE_ARM_STATUS_OK;
}

void AdvanceArm_InvalidateCoordinates(void)
{
  g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_UNKNOWN;
  g_advance_arm.active = 0U;
  g_advance_arm.faulted = 0U;
  AdvanceArm_EnterState(ADVANCE_ARM_TASK_BOOT);
}

void AdvanceArm_Abort(void)
{
  if (g_advance_arm.configured != 0U)
  {
    drive_emm_Stop_Now(g_advance_arm.lift.motor_id, false);
    drive_emm_Stop_Now(g_advance_arm.swing.motor_id, false);
  }
  AdvanceArm_InvalidateCoordinates();
}

void AdvanceArm_EStop(void)
{
  AdvanceArm_Abort();
  AdvanceArm_EnterState(ADVANCE_ARM_TASK_ESTOP);
}

void AdvanceArm_Poll(void)
{
  uint32_t now = HAL_GetTick();

  if (g_advance_arm.configured == 0U)
  {
    return;
  }
  if ((AdvanceArm_UpdateAxis(&g_advance_arm.lift) == 0U) ||
      (AdvanceArm_UpdateAxis(&g_advance_arm.swing) == 0U))
  {
    if ((g_advance_arm.lift.validity == ADVANCE_ARM_POSITION_FAULT) ||
        (g_advance_arm.swing.validity == ADVANCE_ARM_POSITION_FAULT))
    {
      AdvanceArm_SetFault();
    }
    return;
  }
  if (g_advance_arm.state == ADVANCE_ARM_TASK_BOOT)
  {
    /* 操作者已在上电前置零，首个新鲜反馈定义软件坐标零点。 */
    g_advance_arm.lift.zero_pulse += g_advance_arm.lift.current_pulse;
    g_advance_arm.swing.zero_pulse += g_advance_arm.swing.current_pulse;
    g_advance_arm.lift.current_pulse = 0;
    g_advance_arm.lift.target_pulse = 0;
    g_advance_arm.swing.current_pulse = 0;
    g_advance_arm.swing.target_pulse = 0;
    g_advance_arm.lift.validity = ADVANCE_ARM_POSITION_VALID;
    g_advance_arm.swing.validity = ADVANCE_ARM_POSITION_VALID;
    AdvanceArm_EnterState(ADVANCE_ARM_TASK_READY);
    return;
  }
  if ((g_advance_arm.active != 0U) && ((int32_t)(now - g_advance_arm.deadline_tick) >= 0))
  {
    AdvanceArm_SetFault();
    return;
  }

  switch (g_advance_arm.state)
  {
  case ADVANCE_ARM_TASK_PICK_EXTEND:
  case ADVANCE_ARM_TASK_PLACE_EXTEND:
    if (AdvanceArm_CommandRelative(&g_advance_arm.swing, g_advance_arm.config.swing_extend_direction,
                                   g_advance_arm.config.swing_velocity, g_advance_arm.config.swing_acceleration,
                                   g_advance_arm.plan.swing_extend_pulse) != ADVANCE_ARM_STATUS_OK)
    { AdvanceArm_SetFault(); break; }
    AdvanceArm_EnterState((g_advance_arm.state == ADVANCE_ARM_TASK_PICK_EXTEND) ?
                          ADVANCE_ARM_TASK_PICK_LOWER : ADVANCE_ARM_TASK_PLACE_LOWER);
    break;
  case ADVANCE_ARM_TASK_PICK_LOWER:
  case ADVANCE_ARM_TASK_PLACE_LOWER:
    if (AdvanceArm_AxisReached(&g_advance_arm.swing) == 0U) { break; }
    if (AdvanceArm_CommandRelative(&g_advance_arm.lift, g_advance_arm.config.lift_down_direction,
                                   g_advance_arm.config.lift_velocity, g_advance_arm.config.lift_acceleration,
                                   g_advance_arm.plan.lift_lower_pulse) != ADVANCE_ARM_STATUS_OK)
    { AdvanceArm_SetFault(); break; }
    AdvanceArm_EnterState((g_advance_arm.state == ADVANCE_ARM_TASK_PICK_LOWER) ?
                          ADVANCE_ARM_TASK_PICK_GRIP : ADVANCE_ARM_TASK_PLACE_RELEASE);
    break;
  case ADVANCE_ARM_TASK_PICK_GRIP:
  case ADVANCE_ARM_TASK_PLACE_RELEASE:
    if (AdvanceArm_AxisReached(&g_advance_arm.lift) == 0U) { break; }
    if (BusServo_SetPositionEx(g_advance_arm.config.gripper_servo_id,
                               g_advance_arm.plan.gripper_acceleration,
                               (g_advance_arm.state == ADVANCE_ARM_TASK_PICK_GRIP) ?
                               g_advance_arm.plan.gripper_close_position : g_advance_arm.plan.gripper_release_position,
                               g_advance_arm.plan.gripper_speed) != drive_bus_servo_STATUS_OK)
    { AdvanceArm_SetFault(); break; }
    AdvanceArm_EnterState((g_advance_arm.state == ADVANCE_ARM_TASK_PICK_GRIP) ?
                          ADVANCE_ARM_TASK_PICK_LIFT : ADVANCE_ARM_TASK_PLACE_LIFT);
    g_advance_arm.state_tick = now + g_advance_arm.plan.servo_wait_ms;
    break;
  case ADVANCE_ARM_TASK_PICK_LIFT:
  case ADVANCE_ARM_TASK_PLACE_LIFT:
    if ((int32_t)(now - g_advance_arm.state_tick) < 0) { break; }
    if (AdvanceArm_CommandRelative(&g_advance_arm.lift,
                                   (g_advance_arm.config.lift_down_direction == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD) ?
                                   ADVANCE_ARM_MOTOR_DIRECTION_REVERSE : ADVANCE_ARM_MOTOR_DIRECTION_FORWARD,
                                   g_advance_arm.config.lift_velocity, g_advance_arm.config.lift_acceleration,
                                   g_advance_arm.plan.lift_raise_pulse) != ADVANCE_ARM_STATUS_OK)
    { AdvanceArm_SetFault(); break; }
    AdvanceArm_EnterState((g_advance_arm.state == ADVANCE_ARM_TASK_PICK_LIFT) ?
                          ADVANCE_ARM_TASK_PICK_RETRACT : ADVANCE_ARM_TASK_PLACE_RETRACT);
    break;
  case ADVANCE_ARM_TASK_PICK_RETRACT:
  case ADVANCE_ARM_TASK_PLACE_RETRACT:
    if (AdvanceArm_AxisReached(&g_advance_arm.lift) == 0U) { break; }
    if (AdvanceArm_CommandRelative(&g_advance_arm.swing,
                                   (g_advance_arm.config.swing_extend_direction == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD) ?
                                   ADVANCE_ARM_MOTOR_DIRECTION_REVERSE : ADVANCE_ARM_MOTOR_DIRECTION_FORWARD,
                                   g_advance_arm.config.swing_velocity, g_advance_arm.config.swing_acceleration,
                                   g_advance_arm.plan.swing_retract_pulse) != ADVANCE_ARM_STATUS_OK)
    { AdvanceArm_SetFault(); break; }
    AdvanceArm_EnterState(ADVANCE_ARM_TASK_COMPLETE);
    break;
  case ADVANCE_ARM_TASK_COMPLETE:
    if (AdvanceArm_AxisReached(&g_advance_arm.swing) != 0U)
    { g_advance_arm.active = 0U; AdvanceArm_EnterState(ADVANCE_ARM_TASK_READY); }
    break;
  default:
    break;
  }
}

AdvanceArm_Status_t AdvanceArm_StartPick(const AdvanceArm_TaskPlan_t *plan)
{ return AdvanceArm_StartTask(plan, ADVANCE_ARM_TASK_PICK_EXTEND); }

AdvanceArm_Status_t AdvanceArm_StartPlace(const AdvanceArm_TaskPlan_t *plan)
{ return AdvanceArm_StartTask(plan, ADVANCE_ARM_TASK_PLACE_EXTEND); }

AdvanceArm_Status_t AdvanceArm_GetStatus(AdvanceArm_RuntimeStatus_t *status)
{
  if (status == NULL) { return ADVANCE_ARM_STATUS_INVALID_PARAM; }
  *status = (AdvanceArm_RuntimeStatus_t){
      .lift_position_validity = g_advance_arm.lift.validity,
      .swing_position_validity = g_advance_arm.swing.validity,
      .task_state = g_advance_arm.state,
      .lift_current_pulse = g_advance_arm.lift.current_pulse,
      .lift_target_pulse = g_advance_arm.lift.target_pulse,
      .swing_current_pulse = g_advance_arm.swing.current_pulse,
      .swing_target_pulse = g_advance_arm.swing.target_pulse,
      .active = g_advance_arm.active,
      .faulted = g_advance_arm.faulted,
      .updated_tick = HAL_GetTick()};
  return ADVANCE_ARM_STATUS_OK;
}

BusServo_Status_t AdvanceArm_RotatePlate(uint8_t id, uint16_t acc, int32_t pos, uint16_t speed)
{ return BusServo_SetPositionEx(id, acc, pos, speed); }
BusServo_Status_t AdvanceArm_RotateBase(uint8_t id, uint16_t acc, int32_t pos, uint16_t speed)
{ return BusServo_SetPositionEx(id, acc, pos, speed); }

AdvanceArm_Status_t AdvanceArm_Grab(uint8_t id, bool closed, int32_t release, int32_t close,
                                    uint16_t acc, uint16_t speed)
{
  return (BusServo_SetPositionEx(id, acc, closed ? close : release, speed) == drive_bus_servo_STATUS_OK) ?
      ADVANCE_ARM_STATUS_OK : ADVANCE_ARM_STATUS_FAULT;
}

static AdvanceArm_Status_t AdvanceArm_MoveStepper(uint8_t id, AdvanceArm_MotorDirection_t dir,
                                                  uint16_t velocity, uint8_t acc, uint32_t pulse,
                                                  bool relative, bool synchronous)
{
  AdvanceArm_Axis_t *axis = AdvanceArm_FindAxis(id);
  if ((axis == NULL) || (axis->validity != ADVANCE_ARM_POSITION_VALID))
  { return ADVANCE_ARM_STATUS_NOT_READY; }
  drive_emm_Pos_Control(id, (uint8_t)dir, velocity, acc, pulse, !relative, synchronous);
  axis->target_pulse = relative ? axis->current_pulse + ((dir == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD) ?
      (int32_t)pulse : -(int32_t)pulse) : ((dir == ADVANCE_ARM_MOTOR_DIRECTION_FORWARD) ? (int32_t)pulse : -(int32_t)pulse);
  return ADVANCE_ARM_STATUS_OK;
}

AdvanceArm_Status_t AdvanceArm_MoveLift(uint8_t id, AdvanceArm_MotorDirection_t dir, uint16_t v,
                                        uint8_t a, uint32_t p, bool relative, bool sync)
{ return AdvanceArm_MoveStepper(id, dir, v, a, p, relative, sync); }
AdvanceArm_Status_t AdvanceArm_MoveSwing(uint8_t id, AdvanceArm_MotorDirection_t dir, uint16_t v,
                                         uint8_t a, uint32_t p, bool relative, bool sync)
{ return AdvanceArm_MoveStepper(id, dir, v, a, p, relative, sync); }
AdvanceArm_Status_t AdvanceArm_StopMotor(uint8_t id, bool sync)
{
  if (AdvanceArm_FindAxis(id) == NULL) { return ADVANCE_ARM_STATUS_INVALID_PARAM; }
  drive_emm_Stop_Now(id, sync);
  return ADVANCE_ARM_STATUS_OK;
}
