#include "advance_chassis.h"

#include "drive_emm.h"

#define CHASSIS_PI (3.14159265358979323846f)

typedef struct
{
  uint8_t id;
  int8_t sign;
} ChassisMotorConfig;

static const ChassisMotorConfig g_chassis_motors[4] = {
    {CHASSIS_MOTOR_LF_ID, CHASSIS_MOTOR_LF_SIGN},
    {CHASSIS_MOTOR_RF_ID, CHASSIS_MOTOR_RF_SIGN},
    {CHASSIS_MOTOR_LR_ID, CHASSIS_MOTOR_LR_SIGN},
    {CHASSIS_MOTOR_RR_ID, CHASSIS_MOTOR_RR_SIGN},
};
static uint8_t g_chassis_motion_command_active = 0U;

static uint16_t Chassis_AbsLimitRpm(int16_t rpm)
{
  int32_t value = rpm;

  if (value < 0)
  {
    value = -value;
  }

  if (value > (int32_t)CHASSIS_MAX_RPM)
  {
    value = (int32_t)CHASSIS_MAX_RPM;
  }

  return (uint16_t)value;
}

static int32_t Chassis_AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static int32_t Chassis_RoundFloatToI32(float value)
{
  return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

static float Chassis_LimitFloat(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}

static int16_t Chassis_ScaleOneRpm(int32_t rpm, int32_t max_abs)
{
  if (max_abs <= (int32_t)CHASSIS_MAX_RPM)
  {
    return (int16_t)rpm;
  }

  if (rpm >= 0)
  {
    return (int16_t)(((int64_t)rpm * (int32_t)CHASSIS_MAX_RPM + (max_abs / 2)) / max_abs);
  }

  return (int16_t)(-(((int64_t)(-rpm) * (int32_t)CHASSIS_MAX_RPM + (max_abs / 2)) / max_abs));
}

static void Chassis_ScaleWheelRpm(int32_t *lf, int32_t *rf, int32_t *lr, int32_t *rr)
{
  int32_t max_abs = Chassis_AbsI32(*lf);
  int32_t abs_value = Chassis_AbsI32(*rf);

  if (abs_value > max_abs)
  {
    max_abs = abs_value;
  }
  abs_value = Chassis_AbsI32(*lr);
  if (abs_value > max_abs)
  {
    max_abs = abs_value;
  }
  abs_value = Chassis_AbsI32(*rr);
  if (abs_value > max_abs)
  {
    max_abs = abs_value;
  }

  if (max_abs > (int32_t)CHASSIS_MAX_RPM)
  {
    *lf = Chassis_ScaleOneRpm(*lf, max_abs);
    *rf = Chassis_ScaleOneRpm(*rf, max_abs);
    *lr = Chassis_ScaleOneRpm(*lr, max_abs);
    *rr = Chassis_ScaleOneRpm(*rr, max_abs);
  }
}

static void Chassis_SetMotorRPMScaledEx(int32_t lf_rpm, int32_t rf_rpm, int32_t lr_rpm, int32_t rr_rpm, uint8_t acc)
{
  Chassis_ScaleWheelRpm(&lf_rpm, &rf_rpm, &lr_rpm, &rr_rpm);
  Chassis_SetMotorRPMEx((int16_t)lf_rpm, (int16_t)rf_rpm, (int16_t)lr_rpm, (int16_t)rr_rpm, acc);
}

static void Chassis_SendLoadedCommand(void)
{
  /* drive_emm 内部 DMA 队列保证两帧按顺序发送，控制周期不阻塞等待 UART。 */
  drive_emm_Multi_Motor_Cmd(CHASSIS_SYNC_ADDR);
  drive_emm_Synchronous_motion(CHASSIS_SYNC_ADDR);
}

static void Chassis_LoadMotorSpeed(const ChassisMotorConfig *motor, int16_t rpm, uint8_t acc)
{
  int16_t actual_rpm = (int16_t)(rpm * motor->sign);
  uint16_t abs_rpm = Chassis_AbsLimitRpm(actual_rpm);
  uint8_t dir = (actual_rpm >= 0) ? 0U : 1U;

  /* snF=true 表示把命令装入 Emm 多电机同步命令缓存。 */
  drive_emm_MMCL_Vel_Control(motor->id, dir, abs_rpm, acc, true);
}

void Chassis_Enable(bool enable)
{
  uint8_t i;

  for (i = 0U; i < 4U; ++i)
  {
    drive_emm_MMCL_En_Control(g_chassis_motors[i].id, enable, true);
  }

  Chassis_SendLoadedCommand();
}

void Chassis_Stop(void)
{
  uint8_t i;

  for (i = 0U; i < 4U; ++i)
  {
    drive_emm_MMCL_Stop_Now(g_chassis_motors[i].id, true);
  }

  Chassis_SendLoadedCommand();
  g_chassis_motion_command_active = 0U;
}

void Chassis_SmoothStop(uint8_t acc)
{
  Chassis_SetMotorRPMEx(0, 0, 0, 0, acc);
}

void Chassis_SetMotorRPMEx(int16_t lf_rpm, int16_t rf_rpm, int16_t lr_rpm, int16_t rr_rpm, uint8_t acc)
{
  /* 先装入四个轮子的速度命令，再一次性发送多电机命令。 */
  Chassis_LoadMotorSpeed(&g_chassis_motors[0], lf_rpm, acc);
  Chassis_LoadMotorSpeed(&g_chassis_motors[1], rf_rpm, acc);
  Chassis_LoadMotorSpeed(&g_chassis_motors[2], lr_rpm, acc);
  Chassis_LoadMotorSpeed(&g_chassis_motors[3], rr_rpm, acc);

  Chassis_SendLoadedCommand();
  g_chassis_motion_command_active = ((lf_rpm != 0) || (rf_rpm != 0) ||
                                     (lr_rpm != 0) || (rr_rpm != 0))
                                        ? 1U
                                        : 0U;
}

uint8_t Chassis_IsMotionCommandActive(void)
{
  return g_chassis_motion_command_active;
}

void Chassis_SetBodyVelocityEx(float vx_right_mm_s, float vy_forward_mm_s, float wz_ccw_deg_s, uint8_t acc)
{
  float vx = Chassis_LimitFloat(vx_right_mm_s, CHASSIS_MAX_BODY_SPEED_MM_S) * (float)CHASSIS_BODY_X_SIGN;
  float vy = Chassis_LimitFloat(vy_forward_mm_s, CHASSIS_MAX_BODY_SPEED_MM_S) * (float)CHASSIS_BODY_Y_SIGN;
  float wz_rad_s = Chassis_LimitFloat(wz_ccw_deg_s, CHASSIS_MAX_BODY_WZ_DEG_S) *
                   (float)CHASSIS_BODY_WZ_SIGN * CHASSIS_PI / 180.0f;
  float wheel_base = CHASSIS_HALF_LENGTH_MM + CHASSIS_HALF_WIDTH_MM;
  float rpm_per_mm_s = (60.0f * CHASSIS_MOTOR_GEAR_RATIO) / (2.0f * CHASSIS_PI * CHASSIS_WHEEL_RADIUS_MM);
  int32_t lf;
  int32_t rf;
  int32_t lr;
  int32_t rr;

  lf = Chassis_RoundFloatToI32((vy + vx - (wheel_base * wz_rad_s)) * rpm_per_mm_s);
  rf = Chassis_RoundFloatToI32((vy - vx + (wheel_base * wz_rad_s)) * rpm_per_mm_s);
  lr = Chassis_RoundFloatToI32((vy - vx - (wheel_base * wz_rad_s)) * rpm_per_mm_s);
  rr = Chassis_RoundFloatToI32((vy + vx + (wheel_base * wz_rad_s)) * rpm_per_mm_s);

  Chassis_SetMotorRPMScaledEx(lf, rf, lr, rr, acc);
}
void Chassis_MoveMecanumEx(int16_t forward_rpm, int16_t strafe_rpm, int16_t wz_ccw_rpm, uint8_t acc)
{
  /*
   * 麦克纳姆轮速度合成：
   * forward 控制前后，strafe 控制左右平移，rotate 控制原地转向。
   * 单个电机的方向修正统一在 Chassis_LoadMotorSpeed() 中处理。
   */
  int32_t lf = (int32_t)forward_rpm + strafe_rpm - wz_ccw_rpm;
  int32_t rf = (int32_t)forward_rpm - strafe_rpm + wz_ccw_rpm;
  int32_t lr = (int32_t)forward_rpm - strafe_rpm - wz_ccw_rpm;
  int32_t rr = (int32_t)forward_rpm + strafe_rpm + wz_ccw_rpm;

  Chassis_SetMotorRPMScaledEx(lf, rf, lr, rr, acc);
}
