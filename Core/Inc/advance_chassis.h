#ifndef __ADVANCE_CHASSIS_H__
#define __ADVANCE_CHASSIS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/*
 * 四个电机地址在这里配置，用于匹配 ZDT 驱动器实际 ID。
 * LF: 左前轮，RF: 右前轮，LR: 左后轮，RR: 右后轮。
 */
#define CHASSIS_MOTOR_LF_ID ((uint8_t)1U)
#define CHASSIS_MOTOR_RF_ID ((uint8_t)3U)
#define CHASSIS_MOTOR_LR_ID ((uint8_t)2U)
#define CHASSIS_MOTOR_RR_ID ((uint8_t)4U)

/*
 * 如果某个电机正转方向与底盘约定相反，将对应 SIGN 改为 -1。
 * 底盘约定：轮子 RPM 为正时，对小车产生向前运动贡献。
 */
#define CHASSIS_MOTOR_LF_SIGN (1)
#define CHASSIS_MOTOR_RF_SIGN (1)
#define CHASSIS_MOTOR_LR_SIGN (1)
#define CHASSIS_MOTOR_RR_SIGN (1)

#define CHASSIS_DEFAULT_ACC ((uint8_t)10U) /*!< 底盘默认加速度参数。 */
#define CHASSIS_SYNC_ADDR ((uint8_t)0x00U) /*!< 四轮同步控制地址。 */
#define CHASSIS_MAX_RPM ((uint16_t)3000U) /*!< 底盘轮速上限，单位为 RPM。 */
#define CHASSIS_UART_WAIT_TIMEOUT_MS ((uint32_t)20U) /*!< 串口发送等待超时时间，单位为 ms。 */

/*
 * Body velocity axis convention:
 *   +X: right, +Y: forward, +WZ: counter-clockwise when viewed from above.
 * Change one sign to reverse that motion axis without touching control code.
 */
#define CHASSIS_BODY_X_SIGN (1) /*!< 车体 X 轴速度方向符号。 */
#define CHASSIS_BODY_Y_SIGN (1) /*!< 车体 Y 轴速度方向符号。 */
#define CHASSIS_BODY_WZ_SIGN (1) /*!< 车体角速度方向符号。 */

/*
 * 物理速度接口的输入限值。超出时底盘层会钳制，协议层应同时拒绝异常命令。
 * 修改前应完成轮组方向、制动距离和场地边界验证。
 */
#define CHASSIS_MAX_BODY_SPEED_MM_S (500.0f) /*!< 车体线速度上限，单位为 mm/s。 */
#define CHASSIS_MAX_BODY_WZ_DEG_S (180.0f) /*!< 车体角速度上限，单位为度/s。 */

/*
 * Physical chassis parameters for mm/s and deg/s conversion.
 * Measure and calibrate these values on the real chassis before high-speed use.
 */
#define CHASSIS_WHEEL_RADIUS_MM (48.0f) /*!< 车轮半径，单位为 mm。 */
#define CHASSIS_HALF_LENGTH_MM (110.0f) /*!< 车体半长，单位为 mm。 */
#define CHASSIS_HALF_WIDTH_MM (90.0f) /*!< 车体半宽，单位为 mm。 */
#define CHASSIS_MOTOR_GEAR_RATIO (1.0f) /*!< 电机与车轮传动比。 */

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
    /* 仅表示最近一次底盘速度命令是否非零，用于反馈超时的停车保护。 */
    uint8_t Chassis_IsMotionCommandActive(void);

    /*
     * 直接设置四个轮子的转速，并指定加速度参数。
     * acc 会传给 Emm 速度模式；在 Emm 协议中，0 表示直接启动。
     */
    void Chassis_SetMotorRPMEx(int16_t lf_rpm, int16_t rf_rpm, int16_t lr_rpm, int16_t rr_rpm, uint8_t acc);

    /*
     * Set chassis velocity with physical units.
     * vx_right_mm_s > 0: move right; vy_forward_mm_s > 0: move forward.
     * wz_ccw_deg_s > 0: rotate counter-clockwise when viewed from above.
     */
    void Chassis_SetBodyVelocityEx(float vx_right_mm_s, float vy_forward_mm_s, float wz_ccw_deg_s, uint8_t acc);

    /*
     * 通用麦克纳姆轮运动命令，并指定加速度。
     * 轮速合成公式：
     *   LF = forward + strafe + rotate
     *   RF = forward - strafe - rotate
     *   LR = forward - strafe + rotate
     *   RR = forward + strafe - rotate
     */
    void Chassis_MoveMecanumEx(int16_t forward_rpm, int16_t strafe_rpm, int16_t wz_ccw_rpm, uint8_t acc);


#ifdef __cplusplus
}
#endif

#endif
