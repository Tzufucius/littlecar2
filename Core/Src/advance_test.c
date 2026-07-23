#include "advance_test.h"

#include <stdio.h>

#include "drive_emm.h"
#include "main.h"

void AdvanceTest_BlockingMain(void)
{
  DriveEmm_MotorFeedback_t feedback;
  DriveEmm_Diagnostics_t diagnostics;
  int32_t target_pulse;
  uint16_t poll_count;
  uint8_t reached;

  printf("[TEST] drive normal control modes start\r\n");
  if (drive_emm_GetDiagnostics(&diagnostics) == HAL_OK)
  {
    printf("[TEST][DRIVE] diag q=%u/%u active=%u wait=%u monitor=%u txerr=%lu rx=%lu ack=%lu bad=%lu drop=%lu unknown=%lu timeout=%lu\r\n",
           (unsigned int)diagnostics.tx_queue_count, (unsigned int)diagnostics.tx_queue_depth,
           (unsigned int)diagnostics.tx_active, (unsigned int)diagnostics.query_waiting,
           (unsigned int)diagnostics.feedback_monitor_enabled,
           (unsigned long)diagnostics.tx_error_count, (unsigned long)diagnostics.rx_reply_count,
           (unsigned long)diagnostics.rx_ack_count,
           (unsigned long)diagnostics.rx_invalid_frame_count,
           (unsigned long)diagnostics.rx_resync_drop_count,
           (unsigned long)diagnostics.rx_unknown_motor_count,
           (unsigned long)diagnostics.query_timeout_count);
  }

  printf("[TEST][DRIVE] ID1 enable\r\n");
  drive_emm_En_Control(1, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID1 normal velocity CW\r\n");
  drive_emm_Vel_Control(1, ZDT_DIR_CW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    drive_emm_Poll();
    HAL_Delay(20);
  }
  HAL_Delay(1000);
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID1 normal velocity CCW\r\n");
  drive_emm_Vel_Control(1, ZDT_DIR_CCW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    drive_emm_Poll();
    HAL_Delay(20);
  }
  HAL_Delay(1000);
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);

  printf("[TEST][DRIVE] ID1 normal position CW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(1, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position + 3200;
    drive_emm_Pos_Control(1, ZDT_DIR_CW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      drive_emm_Poll();
      if (drive_emm_IsMotorReached(1, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID1 position CW reached=%u target=%ld\r\n",
           (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID1 position CW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);

  printf("[TEST][DRIVE] ID1 normal position CCW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(1, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position - 3200;
    drive_emm_Pos_Control(1, ZDT_DIR_CCW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      drive_emm_Poll();
      if (drive_emm_IsMotorReached(1, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID1 position CCW reached=%u target=%ld\r\n",
           (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID1 position CCW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(1, false);
  HAL_Delay(1000);
  if (drive_emm_GetMotorFeedback(1, &feedback) == HAL_OK)
  {
    printf("[TEST][DRIVE] ID1 feedback pos=%ld speed=%d en=%u stall=%u fault=%u valid=%u\r\n",
           (long)feedback.position, (int)feedback.speed_rpm,
           (unsigned int)feedback.enabled, (unsigned int)feedback.stalled,
           (unsigned int)feedback.fault, (unsigned int)feedback.valid);
  }
  drive_emm_En_Control(1, false, false);
  printf("[TEST][DRIVE] ID1 complete\r\n");

  printf("[TEST][DRIVE] ID2 enable\r\n");
  drive_emm_En_Control(2, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID2 normal velocity CW\r\n");
  drive_emm_Vel_Control(2, ZDT_DIR_CW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    drive_emm_Poll();
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID2 normal velocity CCW\r\n");
  drive_emm_Vel_Control(2, ZDT_DIR_CCW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    drive_emm_Poll();
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID2 normal position CW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(2, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position + 3200;
    drive_emm_Pos_Control(2, ZDT_DIR_CW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      drive_emm_Poll();
      if (drive_emm_IsMotorReached(2, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID2 position CW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID2 position CW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID2 normal position CCW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(2, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position - 3200;
    drive_emm_Pos_Control(2, ZDT_DIR_CCW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      drive_emm_Poll();
      if (drive_emm_IsMotorReached(2, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID2 position CCW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID2 position CCW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(2, false);
  HAL_Delay(1000);
  if (drive_emm_GetMotorFeedback(2, &feedback) == HAL_OK)
  {
    printf("[TEST][DRIVE] ID2 feedback pos=%ld speed=%d en=%u stall=%u fault=%u valid=%u\r\n",
           (long)feedback.position, (int)feedback.speed_rpm,
           (unsigned int)feedback.enabled, (unsigned int)feedback.stalled,
           (unsigned int)feedback.fault, (unsigned int)feedback.valid);
  }
  drive_emm_En_Control(2, false, false);
  printf("[TEST][DRIVE] ID2 complete\r\n");

  printf("[TEST][DRIVE] ID3 enable\r\n");
  drive_emm_En_Control(3, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID3 normal velocity CW\r\n");
  drive_emm_Vel_Control(3, ZDT_DIR_CW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    drive_emm_Poll();
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID3 normal velocity CCW\r\n");
  drive_emm_Vel_Control(3, ZDT_DIR_CCW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    drive_emm_Poll();
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID3 normal position CW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(3, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position + 3200;
    drive_emm_Pos_Control(3, ZDT_DIR_CW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      drive_emm_Poll();
      if (drive_emm_IsMotorReached(3, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID3 position CW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID3 position CW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID3 normal position CCW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(3, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position - 3200;
    drive_emm_Pos_Control(3, ZDT_DIR_CCW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      drive_emm_Poll();
      if (drive_emm_IsMotorReached(3, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID3 position CCW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID3 position CCW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(3, false);
  HAL_Delay(1000);
  if (drive_emm_GetMotorFeedback(3, &feedback) == HAL_OK)
  {
    printf("[TEST][DRIVE] ID3 feedback pos=%ld speed=%d en=%u stall=%u fault=%u valid=%u\r\n",
           (long)feedback.position, (int)feedback.speed_rpm,
           (unsigned int)feedback.enabled, (unsigned int)feedback.stalled,
           (unsigned int)feedback.fault, (unsigned int)feedback.valid);
  }
  drive_emm_En_Control(3, false, false);
  printf("[TEST][DRIVE] ID3 complete\r\n");

  printf("[TEST][DRIVE] ID4 enable\r\n");
  drive_emm_En_Control(4, true, false);
  HAL_Delay(200);
  printf("[TEST][DRIVE] ID4 normal velocity CW\r\n");
  drive_emm_Vel_Control(4, ZDT_DIR_CW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    drive_emm_Poll();
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID4 normal velocity CCW\r\n");
  drive_emm_Vel_Control(4, ZDT_DIR_CCW, 150, 10, false);
  for (poll_count = 0; poll_count < 100; ++poll_count)
  {
    drive_emm_Poll();
    HAL_Delay(20);
  }
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID4 normal position CW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(4, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position + 3200;
    drive_emm_Pos_Control(4, ZDT_DIR_CW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      drive_emm_Poll();
      if (drive_emm_IsMotorReached(4, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID4 position CW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID4 position CW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  printf("[TEST][DRIVE] ID4 normal position CCW\r\n");
  reached = 0U;
  if (drive_emm_GetMotorFeedback(4, &feedback) == HAL_OK && feedback.valid != 0U)
  {
    target_pulse = feedback.position - 3200;
    drive_emm_Pos_Control(4, ZDT_DIR_CCW, 150, 10, 3200, false, false);
    for (poll_count = 0; poll_count < 250; ++poll_count)
    {
      drive_emm_Poll();
      if (drive_emm_IsMotorReached(4, target_pulse, 100, 500) != 0U)
      {
        reached = 1U;
        break;
      }
      HAL_Delay(20);
    }
    printf("[TEST][DRIVE] ID4 position CCW reached=%u target=%ld\r\n", (unsigned int)reached, (long)target_pulse);
  }
  else
  {
    printf("[TEST][DRIVE] ID4 position CCW feedback invalid\r\n");
  }
  drive_emm_Stop_Now(4, false);
  HAL_Delay(1000);
  if (drive_emm_GetMotorFeedback(4, &feedback) == HAL_OK)
  {
    printf("[TEST][DRIVE] ID4 feedback pos=%ld speed=%d en=%u stall=%u fault=%u valid=%u\r\n",
           (long)feedback.position, (int)feedback.speed_rpm,
           (unsigned int)feedback.enabled, (unsigned int)feedback.stalled,
           (unsigned int)feedback.fault, (unsigned int)feedback.valid);
  }
  drive_emm_En_Control(4, false, false);
  drive_emm_Stop_Now(1, false);
  drive_emm_Stop_Now(2, false);
  drive_emm_Stop_Now(3, false);
  drive_emm_Stop_Now(4, false);
  if (drive_emm_GetDiagnostics(&diagnostics) == HAL_OK)
  {
    printf("[TEST][DRIVE] diag q=%u/%u active=%u wait=%u monitor=%u txerr=%lu rx=%lu ack=%lu bad=%lu drop=%lu unknown=%lu timeout=%lu\r\n",
           (unsigned int)diagnostics.tx_queue_count, (unsigned int)diagnostics.tx_queue_depth,
           (unsigned int)diagnostics.tx_active, (unsigned int)diagnostics.query_waiting,
           (unsigned int)diagnostics.feedback_monitor_enabled,
           (unsigned long)diagnostics.tx_error_count, (unsigned long)diagnostics.rx_reply_count,
           (unsigned long)diagnostics.rx_ack_count,
           (unsigned long)diagnostics.rx_invalid_frame_count,
           (unsigned long)diagnostics.rx_resync_drop_count,
           (unsigned long)diagnostics.rx_unknown_motor_count,
           (unsigned long)diagnostics.query_timeout_count);
  }
  printf("[TEST] drive normal control modes complete\r\n");
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
