#include "advance_test.h"

#include <stdio.h>

#include "advance_chassis.h"
#include "drive_emm.h"
#include "main.h"

typedef enum
{
  ADVANCE_TEST_ACTION_MECANUM = 0,
  ADVANCE_TEST_ACTION_STOP
} AdvanceTest_ActionType_t;

typedef struct
{
  AdvanceTest_ActionType_t type;
  uint32_t duration_ms;
  union
  {
    struct
    {
      int16_t forward_rpm;
      int16_t strafe_rpm;
      int16_t rotate_rpm;
      uint8_t acceleration;
    } mecanum;
    struct
    {
      uint8_t acceleration;
    } stop;
  } param;
} AdvanceTest_Action_t;

static const AdvanceTest_Action_t g_nonblocking_actions[] = {
    {ADVANCE_TEST_ACTION_MECANUM, 1000U, {.mecanum = {100, 0, 0, 10U}}},
    {ADVANCE_TEST_ACTION_STOP, 500U, {.stop = {10U}}},
    {ADVANCE_TEST_ACTION_MECANUM, 1000U, {.mecanum = {-100, 0, 0, 10U}}},
    {ADVANCE_TEST_ACTION_STOP, 500U, {.stop = {10U}}},
    {ADVANCE_TEST_ACTION_MECANUM, 1000U, {.mecanum = {0, 0, 100, 10U}}},
    {ADVANCE_TEST_ACTION_STOP, 500U, {.stop = {10U}}},
    {ADVANCE_TEST_ACTION_MECANUM, 1000U, {.mecanum = {0, 0, -100, 10U}}},
    {ADVANCE_TEST_ACTION_STOP, 500U, {.stop = {10U}}},
};

static uint32_t g_nonblocking_action_index = 0U;
static uint8_t g_nonblocking_action_started = 0U;
static uint8_t g_nonblocking_active = 0U;
static uint32_t g_nonblocking_action_started_tick = 0U;

static void AdvanceTest_ExecuteAction(const AdvanceTest_Action_t *action)
{
  if (action->type == ADVANCE_TEST_ACTION_MECANUM)
  {
    Chassis_MoveMecanumEx(action->param.mecanum.forward_rpm,
                          action->param.mecanum.strafe_rpm,
                          action->param.mecanum.rotate_rpm,
                          action->param.mecanum.acceleration);
    return;
  }

  Chassis_SmoothStop(action->param.stop.acceleration);
}

void AdvanceTest_BlockingMain(void)
{
  printf("[TEST] four chassis motor layered test start\r\n");

  printf("[TEST] layer 1: drive_emm direct motor control\r\n");
  printf("[TEST][DRIVE] ID1 enable\r\n");
  drive_emm_En_Control(1, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID1 forward\r\n");
  drive_emm_Vel_Control(1, ZDT_DIR_CW, 150, 10, false);
  HAL_Delay(2000);
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID1 reverse\r\n");
  drive_emm_Vel_Control(1, ZDT_DIR_CCW, 150, 10, false);
  HAL_Delay(2000);
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);
  drive_emm_En_Control(1, false, false);
  printf("[TEST][DRIVE] ID1 complete\r\n");

  printf("[TEST][DRIVE] ID2 enable\r\n");
  drive_emm_En_Control(2, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID2 forward\r\n");
  drive_emm_Vel_Control(2, ZDT_DIR_CW, 150, 10, false);
  HAL_Delay(2000);
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID2 reverse\r\n");
  drive_emm_Vel_Control(2, ZDT_DIR_CCW, 150, 10, false);
  HAL_Delay(2000);
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  drive_emm_En_Control(2, false, false);
  printf("[TEST][DRIVE] ID2 complete\r\n");

  printf("[TEST][DRIVE] ID3 enable\r\n");
  drive_emm_En_Control(3, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID3 forward\r\n");
  drive_emm_Vel_Control(3, ZDT_DIR_CW, 150, 10, false);
  HAL_Delay(2000);
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID3 reverse\r\n");
  drive_emm_Vel_Control(3, ZDT_DIR_CCW, 150, 10, false);
  HAL_Delay(2000);
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  drive_emm_En_Control(3, false, false);
  printf("[TEST][DRIVE] ID3 complete\r\n");

  printf("[TEST][DRIVE] ID4 enable\r\n");
  drive_emm_En_Control(4, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID4 forward\r\n");
  drive_emm_Vel_Control(4, ZDT_DIR_CW, 150, 10, false);
  HAL_Delay(2000);
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID4 reverse\r\n");
  drive_emm_Vel_Control(4, ZDT_DIR_CCW, 150, 10, false);
  HAL_Delay(2000);
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  drive_emm_En_Control(4, false, false);
  printf("[TEST][DRIVE] ID4 complete\r\n");

  printf("[TEST] layer 2: advance_chassis single-wheel mapping\r\n");
  Chassis_Enable(true);
  printf("[TEST][CHASSIS] ID1 forward\r\n");
  Chassis_SetMotorRPMEx(150, 0, 0, 0, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);
  printf("[TEST][CHASSIS] ID1 reverse\r\n");
  Chassis_SetMotorRPMEx(-150, 0, 0, 0, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);

  printf("[TEST][CHASSIS] ID2 forward\r\n");
  Chassis_SetMotorRPMEx(0, 0, 150, 0, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);
  printf("[TEST][CHASSIS] ID2 reverse\r\n");
  Chassis_SetMotorRPMEx(0, 0, -150, 0, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);

  printf("[TEST][CHASSIS] ID3 forward\r\n");
  Chassis_SetMotorRPMEx(0, 150, 0, 0, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);
  printf("[TEST][CHASSIS] ID3 reverse\r\n");
  Chassis_SetMotorRPMEx(0, -150, 0, 0, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);

  printf("[TEST][CHASSIS] ID4 forward\r\n");
  Chassis_SetMotorRPMEx(0, 0, 0, 150, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);
  printf("[TEST][CHASSIS] ID4 reverse\r\n");
  Chassis_SetMotorRPMEx(0, 0, 0, -150, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);

  printf("[TEST] layer 3: advance_chassis vehicle motion\r\n");
  printf("[TEST][ADVANCE] mecanum forward\r\n");
  Chassis_MoveMecanumEx(150, 0, 0, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);

  printf("[TEST][ADVANCE] mecanum reverse\r\n");
  Chassis_MoveMecanumEx(-150, 0, 0, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);

  printf("[TEST][ADVANCE] mecanum rotate CCW\r\n");
  Chassis_MoveMecanumEx(0, 0, 150, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);

  printf("[TEST][ADVANCE] body velocity\r\n");
  Chassis_SetBodyVelocityEx(0.0f, 100.0f, 0.0f, 10);
  HAL_Delay(2000);
  Chassis_SmoothStop(10);
  HAL_Delay(1000);

  Chassis_Stop();
  Chassis_Enable(false);
  printf("[TEST] four chassis motor layered test complete\r\n");
}

void AdvanceTest_NonBlockingMain(void)
{
  g_nonblocking_action_index = 0U;
  g_nonblocking_action_started = 0U;
  g_nonblocking_active = 1U;
  g_nonblocking_action_started_tick = 0U;
  Chassis_Enable(true);
}

void AdvanceTest_NonBlockingPoll(void)
{
  const AdvanceTest_Action_t *action;
  uint32_t now;

  if (g_nonblocking_active == 0U)
  {
    return;
  }

  if (drive_emm_IsChassisFeedbackHealthy() == 0U)
  {
    Chassis_Stop();
    g_nonblocking_active = 0U;
    return;
  }

  if (g_nonblocking_action_index >=
      (uint32_t)(sizeof(g_nonblocking_actions) / sizeof(g_nonblocking_actions[0])))
  {
    Chassis_SmoothStop(10U);
    g_nonblocking_active = 0U;
    return;
  }

  now = HAL_GetTick();
  action = &g_nonblocking_actions[g_nonblocking_action_index];
  if (g_nonblocking_action_started == 0U)
  {
    AdvanceTest_ExecuteAction(action);
    g_nonblocking_action_started_tick = now;
    g_nonblocking_action_started = 1U;
    return;
  }

  if ((now - g_nonblocking_action_started_tick) >= action->duration_ms)
  {
    ++g_nonblocking_action_index;
    g_nonblocking_action_started = 0U;
  }
}

void AdvanceTest_ScrewMotor(uint8_t id, bool is_x)
{
  const uint16_t vel_rpm = 150U;
  const uint16_t vel_deg_0p1 = vel_rpm * 10U;
  const uint8_t acc = 10U;
  const uint32_t one_turn_deg_0p1 = 3600U;

  drive_emm_En_Control(id, true, false);
  HAL_Delay(200U);

  if (is_x)
  {
    drive_emm_SetSpeedX(id, ZDT_DIR_CCW, vel_deg_0p1, acc, false);
    HAL_Delay(2000U);
    drive_emm_Stop_Now(id, false);
    HAL_Delay(1000U);
    drive_emm_SetSpeedX(id, ZDT_DIR_CW, vel_deg_0p1, acc, false);
    HAL_Delay(2000U);
    drive_emm_Stop_Now(id, false);
    HAL_Delay(1000U);
    drive_emm_SetTrapezoidPositionX(id, ZDT_DIR_CCW, 100U, 100U,
                                    vel_deg_0p1, one_turn_deg_0p1,
                                    ZDT_POS_RELATIVE_CURRENT, false);
    HAL_Delay(3000U);
    return;
  }

  drive_emm_Vel_Control(id, 0U, vel_rpm, acc, false);
  HAL_Delay(2000U);
  drive_emm_Stop_Now(id, false);
  HAL_Delay(1000U);
  drive_emm_Vel_Control(id, 1U, vel_rpm, acc, false);
  HAL_Delay(2000U);
  drive_emm_Stop_Now(id, false);
  HAL_Delay(1000U);
}
