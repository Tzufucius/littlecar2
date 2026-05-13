#ifndef __ADVANCE_CHASSIS_H__
#define __ADVANCE_CHASSIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * 四个电机地址在这里配置，用于匹配 ZDT 驱动器实际 ID。
 * LF: 左前轮，RF: 右前轮，LR: 左后轮，RR: 右后轮。
 */
#define CHASSIS_MOTOR_LF_ID ((uint8_t)1U)
#define CHASSIS_MOTOR_RF_ID ((uint8_t)2U)
#define CHASSIS_MOTOR_LR_ID ((uint8_t)3U)
#define CHASSIS_MOTOR_RR_ID ((uint8_t)4U)

/*
 * 如果某个电机正转方向与底盘约定相反，将对应 SIGN 改为 -1。
 * 底盘约定：轮子 RPM 为正时，对小车产生向前运动贡献。
 */
#define CHASSIS_MOTOR_LF_SIGN (1)
#define CHASSIS_MOTOR_RF_SIGN (1)
#define CHASSIS_MOTOR_LR_SIGN (1)
#define CHASSIS_MOTOR_RR_SIGN (1)

#define CHASSIS_DEFAULT_ACC ((uint8_t)10U)
#define CHASSIS_SYNC_ADDR ((uint8_t)0x00U)
#define CHASSIS_MAX_RPM ((uint16_t)3000U)
#define CHASSIS_UART_WAIT_TIMEOUT_MS ((uint32_t)20U)

/*
 * 各运动动作的平滑移动预设参数。
 * acc 越大响应越快；acc 越小启动和变速越平滑。
 */
#define CHASSIS_FORWARD_PRESET_RPM ((uint16_t)100U)
#define CHASSIS_FORWARD_PRESET_ACC ((uint8_t)10U)
#define CHASSIS_BACKWARD_PRESET_RPM ((uint16_t)100U)
#define CHASSIS_BACKWARD_PRESET_ACC ((uint8_t)10U)
#define CHASSIS_STRAFE_PRESET_RPM ((uint16_t)100U)
#define CHASSIS_STRAFE_PRESET_ACC ((uint8_t)10U)
#define CHASSIS_ROTATE_PRESET_RPM ((uint16_t)80U)
#define CHASSIS_ROTATE_PRESET_ACC ((uint8_t)8U)
#define CHASSIS_DIFF_LEFT_PRESET_RPM ((int16_t)80)
#define CHASSIS_DIFF_RIGHT_PRESET_RPM ((int16_t)120)
#define CHASSIS_DIFF_PRESET_ACC ((uint8_t)8U)
#define CHASSIS_MECANUM_FORWARD_PRESET_RPM ((int16_t)80)
#define CHASSIS_MECANUM_STRAFE_PRESET_RPM ((int16_t)0)
#define CHASSIS_MECANUM_ROTATE_PRESET_RPM ((int16_t)0)
#define CHASSIS_MECANUM_PRESET_ACC ((uint8_t)8U)

/*
 * 使能或失能四个底盘电机。
 * enable=true 表示打开电机输出/保持力；enable=false 表示关闭电机输出。
 */
void Chassis_Enable(bool enable);

/*
 * 四个底盘电机立即停止。
 * 内部向四个已配置的电机 ID 发送 Emm 停止命令。
 */
void Chassis_Stop(void);

/*
 * 平滑停止四个底盘电机。
 * 内部发送四轮 0RPM 速度命令，acc 越小减速越柔和。
 */
void Chassis_SmoothStop(uint8_t acc);

/*
 * 直接设置四个轮子的转速。
 * lf_rpm/rf_rpm/lr_rpm/rr_rpm 使用有符号 RPM：
 * 正数表示向前运动贡献，负数表示向后运动贡献。
 */
void Chassis_SetMotorRPM(int16_t lf_rpm, int16_t rf_rpm, int16_t lr_rpm, int16_t rr_rpm);

/*
 * 直接设置四个轮子的转速，并指定加速度参数。
 * acc 会传给 Emm 速度模式；在 Emm 协议中，0 表示直接启动。
 */
void Chassis_SetMotorRPMEx(int16_t lf_rpm, int16_t rf_rpm, int16_t lr_rpm, int16_t rr_rpm, uint8_t acc);

/*
 * 前进。
 * 轮速分配：LF +，RF +，LR +，RR +。
 */
void Chassis_Forward(uint16_t rpm);

/*
 * 前进，并指定加速度，用于调整平滑程度和响应速度。
 */
void Chassis_ForwardEx(uint16_t rpm, uint8_t acc);

/*
 * 按 CHASSIS_FORWARD_PRESET_RPM/ACC 预设参数前进。
 */
void Chassis_ForwardPreset(void);

/*
 * 后退。
 * 轮速分配：LF -，RF -，LR -，RR -。
 */
void Chassis_Backward(uint16_t rpm);

/*
 * 后退，并指定加速度。
 */
void Chassis_BackwardEx(uint16_t rpm, uint8_t acc);

/*
 * 按 CHASSIS_BACKWARD_PRESET_RPM/ACC 预设参数后退。
 */
void Chassis_BackwardPreset(void);

/*
 * 左平移，车头方向不变。
 * 轮速分配：LF -，RF +，LR +，RR -。
 */
void Chassis_StrafeLeft(uint16_t rpm);

/*
 * 左平移，并指定加速度。
 */
void Chassis_StrafeLeftEx(uint16_t rpm, uint8_t acc);

/*
 * 按 CHASSIS_STRAFE_PRESET_RPM/ACC 预设参数左平移。
 */
void Chassis_StrafeLeftPreset(void);

/*
 * 右平移，车头方向不变。
 * 轮速分配：LF +，RF -，LR -，RR +。
 */
void Chassis_StrafeRight(uint16_t rpm);

/*
 * 右平移，并指定加速度。
 */
void Chassis_StrafeRightEx(uint16_t rpm, uint8_t acc);

/*
 * 按 CHASSIS_STRAFE_PRESET_RPM/ACC 预设参数右平移。
 */
void Chassis_StrafeRightPreset(void);

/*
 * 左原地旋转。
 * 轮速分配：LF -，RF +，LR -，RR +。
 */
void Chassis_RotateLeft(uint16_t rpm);

/*
 * 左原地旋转，并指定加速度。
 */
void Chassis_RotateLeftEx(uint16_t rpm, uint8_t acc);

/*
 * 按 CHASSIS_ROTATE_PRESET_RPM/ACC 预设参数左原地旋转。
 */
void Chassis_RotateLeftPreset(void);

/*
 * 右原地旋转。
 * 轮速分配：LF +，RF -，LR +，RR -。
 */
void Chassis_RotateRight(uint16_t rpm);

/*
 * 右原地旋转，并指定加速度。
 */
void Chassis_RotateRightEx(uint16_t rpm, uint8_t acc);

/*
 * 按 CHASSIS_ROTATE_PRESET_RPM/ACC 预设参数右原地旋转。
 */
void Chassis_RotateRightPreset(void);

/*
 * 差速转向。
 * left_rpm 控制 LF/LR，right_rpm 控制 RF/RR。
 * 例如 left_rpm=100、right_rpm=50 时，小车一边前进一边向右转。
 */
void Chassis_DifferentialTurn(int16_t left_rpm, int16_t right_rpm);

/*
 * 差速转向，并指定加速度。
 */
void Chassis_DifferentialTurnEx(int16_t left_rpm, int16_t right_rpm, uint8_t acc);

/*
 * 按 CHASSIS_DIFF_LEFT_PRESET_RPM、
 * CHASSIS_DIFF_RIGHT_PRESET_RPM 和 CHASSIS_DIFF_PRESET_ACC 预设参数差速转向。
 */
void Chassis_DifferentialTurnPreset(void);

/*
 * 通用麦克纳姆轮运动命令。
 * forward_rpm：正数前进，负数后退。
 * strafe_rpm：正数右平移，负数左平移。
 * rotate_rpm：正数右旋转，负数左旋转。
 */
void Chassis_MoveMecanum(int16_t forward_rpm, int16_t strafe_rpm, int16_t rotate_rpm);

/*
 * 通用麦克纳姆轮运动命令，并指定加速度。
 * 轮速合成公式：
 *   LF = forward + strafe + rotate
 *   RF = forward - strafe - rotate
 *   LR = forward - strafe + rotate
 *   RR = forward + strafe - rotate
 */
void Chassis_MoveMecanumEx(int16_t forward_rpm, int16_t strafe_rpm, int16_t rotate_rpm, uint8_t acc);

/*
 * 按 CHASSIS_MECANUM_*_PRESET_* 参数执行通用麦克纳姆轮运动。
 */
void Chassis_MoveMecanumPreset(void);

#ifdef __cplusplus
}
#endif

#endif
