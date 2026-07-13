#ifndef __drive_emm_H
#define __drive_emm_H

#include <stdbool.h>
#include "main.h"

/**********************************************************
***	drive_emm.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：http s://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

/** @brief 返回整数表达式的绝对值。 */
#define ABS(x) ((x) > 0 ? (x) : -(x))

/*
 * 官方例程默认硬编码 huart1。当前工程 ZDT 电机接在 USART3，
 * 如需切换串口，只修改此宏，不要在 drive_emm.c 内直接改句柄。
 */
#ifndef drive_emm_UART_HANDLE
/** @brief 指定 EMM 驱动使用的 UART 句柄。 */
#define drive_emm_UART_HANDLE huart3
#endif

/** @brief 获取 EMM 驱动使用的 UART 句柄地址。 */
#define drive_emm_UART (&drive_emm_UART_HANDLE)

/** @brief EMM 驱动使用的 UART 句柄。 */
extern UART_HandleTypeDef drive_emm_UART_HANDLE;

typedef enum
{
	/** @brief 读取总线电压。 */
	S_VBUS = 5,	  // 读取总线电压
	/** @brief 读取总线电流。 */
	S_CBUS = 6,	  // 读取总线电流
	/** @brief 读取相电流。 */
	S_CPHA = 7,	  // 读取相电流
	/** @brief 读取编码器原始值。 */
	S_ENCO = 8,	  // 读取编码器原始值
	/** @brief 读取实时脉冲数。 */
	S_CLKC = 9,	  // 读取实时脉冲数
	/** @brief 读取经过线性化校准后的编码器值。 */
	S_ENCL = 10,  // 读取经过线性化校准后的编码器值
	/** @brief 读取输入脉冲数。 */
	S_CLKI = 11,  // 读取输入脉冲数
	/** @brief 读取电机目标位置。 */
	S_TPOS = 12,  // 读取电机目标位置
	/** @brief 读取电机实时设定的目标位置。 */
	S_SPOS = 13,  // 读取电机实时设定的目标位置
	/** @brief 读取电机实时转速。 */
	S_VEL = 14,	  // 读取电机实时转速
	/** @brief 读取电机实时位置。 */
	S_CPOS = 15,  // 读取电机实时位置
	/** @brief 读取电机位置误差。 */
	S_PERR = 16,  // 读取电机位置误差
	/** @brief 读取多圈编码器电池电压。 */
	S_VBAT = 17,  // 读取多圈编码器电池电压（Y42）
	/** @brief 读取电机实时温度。 */
	S_TEMP = 18,  // 读取电机实时温度（Y42）
	/** @brief 读取电机状态标志位。 */
	S_FLAG = 19,  // 读取电机状态标志位
	/** @brief 读取回零状态标志位。 */
	S_OFLAG = 20, // 读取回零状态标志位
	/** @brief 读取电机状态标志位和回零状态标志位。 */
	S_OAF = 21,	  // 读取电机状态标志位 + 回零状态标志位（Y42）
	/** @brief 读取引脚状态。 */
	S_PIN = 22,	  // 读取引脚状态（Y42）
} SysParams_t;

/** @brief 多机命令缓存的最大元素数量。 */
#define MMCL_LEN 512
/** @brief 多机命令缓存当前元素数量及缓存数组。 */
extern __IO uint16_t MMCL_count, MMCL_cmd[MMCL_LEN];

/* USART3 发送队列与反馈监督配置。数值需与电机回包速率一起实车验证。 */
/** @brief USART 发送队列深度。 */
#define DRIVE_EMM_TX_QUEUE_DEPTH ((uint8_t)8U)
/** @brief 单次 USART 发送超时时间，单位为毫秒。 */
#define DRIVE_EMM_TX_TIMEOUT_MS ((uint32_t)100U)
/** @brief 电机反馈轮询周期，单位为毫秒。 */
#define DRIVE_EMM_FEEDBACK_PERIOD_MS ((uint32_t)20U)
/** @brief 电机反馈超时时间，单位为毫秒。 */
#define DRIVE_EMM_FEEDBACK_TIMEOUT_MS ((uint32_t)500U)
/** @brief 启动阶段反馈监督宽限时间，单位为毫秒。 */
#define DRIVE_EMM_FEEDBACK_STARTUP_GRACE_MS ((uint32_t)1000U)
/** @brief 底盘反馈监督启动阶段的反馈超时时间，单位为毫秒。 */
#define DRIVE_EMM_ARM_FEEDBACK_TIMEOUT_MS ((uint32_t)800U)
/** @brief 是否启用心跳保护功能。 */
#define DRIVE_EMM_ENABLE_HEARTBEAT_PROTECTION (1U)
/** @brief 心跳保护时间，单位为毫秒。 */
#define DRIVE_EMM_HEARTBEAT_PROTECT_MS ((uint32_t)500U)

typedef struct
{
	/** @brief 电机转速，单位为转每分钟。 */
  int16_t speed_rpm;
	/** @brief 电机当前位置。 */
  int32_t position;
	/** @brief 电机位置误差。 */
  int32_t position_error;
	/** @brief 电机使能状态。 */
  uint8_t enabled;
	/** @brief 电机堵转状态。 */
  uint8_t stalled;
	/** @brief 电机故障状态。 */
  uint8_t fault;
	/** @brief 反馈数据是否有效。 */
  uint8_t valid;
	/** @brief 反馈数据最近一次更新时间戳。 */
  uint32_t updated_tick;
} DriveEmm_MotorFeedback_t;

/* 初始化 USART3 DMA 接收、发送队列和轮询反馈。 */
/** @brief 初始化 EMM 驱动的 DMA、发送队列和反馈轮询。 */
/** @retval HAL_OK 初始化成功。 */
/** @retval 其他值 初始化失败。 */
HAL_StatusTypeDef drive_emm_Init(void);
/* 配置需要接受反馈监督的四个底盘电机 ID。 */
/** @brief 配置需要接受反馈监督的四个底盘电机 ID。 */
/** @param lf_id 左前电机 ID。 */
/** @param rf_id 右前电机 ID。 */
/** @param lr_id 左后电机 ID。 */
/** @param rr_id 右后电机 ID。 */
void drive_emm_ConfigureChassisFeedback(uint8_t lf_id, uint8_t rf_id, uint8_t lr_id, uint8_t rr_id);
/** @brief 启用指定电机的反馈监督。 */
/** @param id 电机 ID。 */
/** @retval HAL_OK 配置成功。 */
/** @retval 其他值 配置失败。 */
HAL_StatusTypeDef drive_emm_MonitorMotor(uint8_t id);
/* 主循环轮询：推进 DMA 队列、查询四电机反馈并检测发送超时。 */
/** @brief 在主循环中推进 DMA 队列、查询反馈并检测发送超时。 */
void drive_emm_Poll(void);
/** @brief 处理 UART 接收事件。 */
/** @param huart 产生接收事件的 UART 句柄。 */
/** @param size 本次接收的数据长度。 */
void drive_emm_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
/** @brief 处理 UART 发送完成事件。 */
/** @param huart 完成发送的 UART 句柄。 */
void drive_emm_OnTxComplete(UART_HandleTypeDef *huart);
/** @brief 处理 UART 错误事件。 */
/** @param huart 发生错误的 UART 句柄。 */
void drive_emm_OnUartError(UART_HandleTypeDef *huart);
/** @brief 判断底盘电机反馈是否整体健康。 */
/** @retval 非零值 反馈健康。 */
/** @retval 0 反馈不健康。 */
uint8_t drive_emm_IsChassisFeedbackHealthy(void);
/** @brief 获取指定电机的反馈数据。 */
/** @param id 电机 ID。 */
/** @param feedback 用于接收反馈数据的输出结构体指针。 */
/** @retval HAL_OK 获取成功。 */
/** @retval 其他值 获取失败。 */
HAL_StatusTypeDef drive_emm_GetMotorFeedback(uint8_t id, DriveEmm_MotorFeedback_t *feedback);
/** @brief 判断指定电机的反馈是否在超时时间内更新。 */
/** @param id 电机 ID。 */
/** @param timeout_ms 允许的最大反馈间隔，单位为毫秒。 */
/** @retval 非零值 反馈健康。 */
/** @retval 0 反馈超时或无效。 */
uint8_t drive_emm_IsMotorFeedbackHealthy(uint8_t id, uint32_t timeout_ms);
/** @brief 判断指定电机是否到达目标位置。 */
/** @param id 电机 ID。 */
/** @param target_pulse 目标脉冲位置。 */
/** @param tolerance_pulse 允许的位置误差脉冲数。 */
/** @param timeout_ms 允许的最大反馈间隔，单位为毫秒。 */
/** @retval 非零值 已到达目标位置。 */
/** @retval 0 未到达目标位置或反馈无效。 */
uint8_t drive_emm_IsMotorReached(uint8_t id, int32_t target_pulse,
                                  int32_t tolerance_pulse, uint32_t timeout_ms);

/**
***********************************************************
***********************************************************
***
***
*** @brief	后缀带有（Y42）为Y42新增命令，X42不能用，其他通用
***
***
***********************************************************
***********************************************************
***/
/**********************************************************
*** 触发动作命令
**********************************************************/
// 触发编码器校准
/** @brief 触发编码器校准。 */
/** @param addr 电机地址。 */
void drive_emm_Trig_Encoder_Cal(uint8_t addr);
// 重启电机（Y42）
/** @brief 重启电机。 */
/** @param addr 电机地址。 */
void drive_emm_Reset_Motor(uint8_t addr);
// 将当前位置清零
/** @brief 将电机当前位置清零。 */
/** @param addr 电机地址。 */
void drive_emm_Reset_CurPos_To_Zero(uint8_t addr);
// 解除堵转保护
/** @brief 解除电机堵转保护。 */
/** @param addr 电机地址。 */
void drive_emm_Reset_Clog_Pro(uint8_t addr);
// 恢复出厂设置
/** @brief 恢复电机出厂设置。 */
/** @param addr 电机地址。 */
void drive_emm_Restore_Motor(uint8_t addr);
/**********************************************************
*** 运动控制命令
**********************************************************/
// 多电机命令（Y42）
/** @brief 发送多电机命令。 */
/** @param addr 电机地址。 */
void drive_emm_Multi_Motor_Cmd(uint8_t addr);
// 电机使能控制
/** @brief 控制电机使能状态。 */
/** @param addr 电机地址。 */
/** @param state 使能状态。 */
/** @param snF 是否保存设置。 */
void drive_emm_En_Control(uint8_t addr, bool state, bool snF);
// 速度模式控制
/** @brief 以速度模式控制电机。 */
/** @param addr 电机地址。 */
/** @param dir 运动方向。 */
/** @param vel 目标速度。 */
/** @param acc 加速度。 */
/** @param snF 是否保存设置。 */
void drive_emm_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
// 位置模式控制
/** @brief 以位置模式控制电机。 */
/** @param addr 电机地址。 */
/** @param dir 运动方向。 */
/** @param vel 目标速度。 */
/** @param acc 加速度。 */
/** @param clk 目标脉冲数。 */
/** @param raF 是否相对位置运动。 */
/** @param snF 是否保存设置。 */
void drive_emm_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
// 让电机立即停止运动
/** @brief 让电机立即停止运动。 */
/** @param addr 电机地址。 */
/** @param snF 是否保存设置。 */
void drive_emm_Stop_Now(uint8_t addr, bool snF);
// 触发多机同步开始运动
/** @brief 触发多机同步开始运动。 */
/** @param addr 电机地址。 */
void drive_emm_Synchronous_motion(uint8_t addr);
/**********************************************************
*** 原点回零命令
**********************************************************/
// 设置单圈回零的零点位置
/** @brief 设置单圈回零的零点位置。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
void drive_emm_Origin_Set_O(uint8_t addr, bool svF);
// 触发回零
/** @brief 触发电机回零。 */
/** @param addr 电机地址。 */
/** @param o_mode 回零模式。 */
/** @param snF 是否保存设置。 */
void drive_emm_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
// 强制中断并退出回零
/** @brief 强制中断并退出回零。 */
/** @param addr 电机地址。 */
void drive_emm_Origin_Interrupt(uint8_t addr);
// 读取回零参数
/** @brief 读取回零参数。 */
/** @param addr 电机地址。 */
void drive_emm_Origin_Read_Params(uint8_t addr);
// 修改回零参数
/** @brief 修改回零参数。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param o_mode 回零模式。 */
/** @param o_dir 回零方向。 */
/** @param o_vel 回零速度。 */
/** @param o_tm 回零超时时间。 */
/** @param sl_vel 切换速度。 */
/** @param sl_ma 切换电流。 */
/** @param sl_ms 切换时间。 */
/** @param potF 是否启用正限位。 */
void drive_emm_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
/**********************************************************
*** 读取系统参数命令
**********************************************************/
// 定时返回信息命令（Y42）
/** @brief 配置定时返回系统参数信息。 */
/** @param addr 电机地址。 */
/** @param s 系统参数类型。 */
/** @param time_ms 返回周期，单位为毫秒。 */
void drive_emm_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
// 读取系统参数
/** @brief 读取指定系统参数。 */
/** @param addr 电机地址。 */
/** @param s 系统参数类型。 */
void drive_emm_Read_Sys_Params(uint8_t addr, SysParams_t s);
/**********************************************************
*** 读写驱动参数命令
**********************************************************/
// 修改电机ID地址
/** @brief 修改电机 ID 地址。 */
/** @param addr 当前电机地址。 */
/** @param svF 是否保存设置。 */
/** @param id 新的电机 ID。 */
void drive_emm_Modify_Motor_ID(uint8_t addr, bool svF, uint8_t id);
// 修改细分值
/** @brief 修改电机细分值。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param mstep 细分值。 */
void drive_emm_Modify_MicroStep(uint8_t addr, bool svF, uint8_t mstep);
// 修改掉电标志
/** @brief 修改掉电保存标志。 */
/** @param addr 电机地址。 */
/** @param pdf 掉电保存标志。 */
void drive_emm_Modify_PDFlag(uint8_t addr, bool pdf);
// 读取选项参数状态（Y42）
/** @brief 读取选项参数状态。 */
/** @param addr 电机地址。 */
void drive_emm_Read_Opt_Param_Sta(uint8_t addr);
// 修改电机类型（Y42）
/** @brief 修改电机类型。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param mottype 电机类型。 */
void drive_emm_Modify_Motor_Type(uint8_t addr, bool svF, bool mottype);
// 修改固件类型（Y42）
/** @brief 修改固件类型。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param fwtype 固件类型。 */
void drive_emm_Modify_Firmware_Type(uint8_t addr, bool svF, bool fwtype);
// 修改开环/闭环控制模式（Y42）
/** @brief 修改开环或闭环控制模式。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param ctrl_mode 控制模式。 */
void drive_emm_Modify_Ctrl_Mode(uint8_t addr, bool svF, bool ctrl_mode);
// 修改电机运动正方向（Y42）
/** @brief 修改电机运动正方向。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param dir 运动正方向。 */
void drive_emm_Modify_Motor_Dir(uint8_t addr, bool svF, bool dir);
// 修改锁定按键功能（Y42）
/** @brief 修改锁定按键功能。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param lockbtn 锁定按键功能。 */
void drive_emm_Modify_Lock_Btn(uint8_t addr, bool svF, bool lockbtn);
// 修改命令速度值是否缩小10倍输入（Y42）
/** @brief 修改命令速度值是否缩小十倍输入。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param s_vel 速度值缩小十倍输入开关。 */
void drive_emm_Modify_S_Vel(uint8_t addr, bool svF, bool s_vel);
// 修改开环模式工作电流
/** @brief 修改开环模式工作电流。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param om_ma 开环模式工作电流。 */
void drive_emm_Modify_OM_ma(uint8_t addr, bool svF, uint16_t om_ma);
// 修改闭环模式最大电流
/** @brief 修改闭环模式最大电流。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param foc_mA 闭环模式最大电流。 */
void drive_emm_Modify_FOC_mA(uint8_t addr, bool svF, uint16_t foc_mA);
// 读取PID参数
/** @brief 读取 PID 参数。 */
/** @param addr 电机地址。 */
void drive_emm_Read_PID_Params(uint8_t addr);
// 修改PID参数
/** @brief 修改 PID 参数。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param kp 比例系数。 */
/** @param ki 积分系数。 */
/** @param kd 微分系数。 */
void drive_emm_Modify_PID_Params(uint8_t addr, bool svF, uint32_t kp, uint32_t ki, uint32_t kd);
// 读取DMX512协议参数（Y42）
/** @brief 读取 DMX512 协议参数。 */
/** @param addr 电机地址。 */
void drive_emm_Read_DMX512_Params(uint8_t addr);
// 修改DMX512协议参数（Y42）
/** @brief 修改 DMX512 协议参数。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param tch 通道起始地址。 */
/** @param nch 通道数量。 */
/** @param mode 工作模式。 */
/** @param vel 目标速度。 */
/** @param acc 加速度。 */
/** @param vel_step 速度步进值。 */
/** @param pos_step 位置步进值。 */
void drive_emm_Modify_DMX512_Params(uint8_t addr, bool svF, uint16_t tch, uint8_t nch, uint8_t mode, uint16_t vel, uint16_t acc, uint16_t vel_step, uint32_t pos_step);
// 读取位置到达窗口（Y42）
/** @brief 读取位置到达窗口。 */
/** @param addr 电机地址。 */
void drive_emm_Read_Pos_Window(uint8_t addr);
// 修改位置到达窗口（Y42）
/** @brief 修改位置到达窗口。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param prw 位置到达窗口。 */
void drive_emm_Modify_Pos_Window(uint8_t addr, bool svF, uint16_t prw);
// 读取过热过流保护检测阈值（Y42）
/** @brief 读取过热过流保护检测阈值。 */
/** @param addr 电机地址。 */
void drive_emm_Read_Otocp(uint8_t addr);
// 修改过热过流保护检测阈值（Y42）
/** @brief 修改过热过流保护检测阈值。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param otp 过热保护阈值。 */
/** @param ocp 过流保护阈值。 */
/** @param time_ms 保护时间，单位为毫秒。 */
void drive_emm_Modify_Otocp(uint8_t addr, bool svF, uint16_t otp, uint16_t ocp, uint16_t time_ms);
// 读取心跳保护功能时间（Y42）
/** @brief 读取心跳保护功能时间。 */
/** @param addr 电机地址。 */
void drive_emm_Read_Heart_Protect(uint8_t addr);
// 修改心跳保护功能时间（Y42）
/** @brief 修改心跳保护功能时间。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param hp 心跳保护时间。 */
void drive_emm_Modify_Heart_Protect(uint8_t addr, bool svF, uint32_t hp);
// 读取积分限幅/刚性系数（Y42）
/** @brief 读取积分限幅和刚性系数。 */
/** @param addr 电机地址。 */
void drive_emm_Read_Integral_Limit(uint8_t addr);
// 修改积分限幅/刚性系数（Y42）
/** @brief 修改积分限幅和刚性系数。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param il 积分限幅或刚性系数。 */
void drive_emm_Modify_Integral_Limit(uint8_t addr, bool svF, uint32_t il);
/**********************************************************
*** 读取所有驱动参数命令
**********************************************************/
// 读取系统状态参数
/** @brief 读取系统状态参数。 */
/** @param addr 电机地址。 */
void drive_emm_Read_System_State_Params(uint8_t addr);
// 读取驱动配置参数
/** @brief 读取驱动配置参数。 */
/** @param addr 电机地址。 */
void drive_emm_Read_Motor_Conf_Params(uint8_t addr);

/**
***********************************************************
***********************************************************
***
***
*** @brief	以下是把相应命令加载到Y42多电机命令上的函数（Y42）
***
***
***********************************************************
***********************************************************
***/
/**********************************************************
*** 触发动作命令
**********************************************************/
// 触发编码器校准 - 加载到多电机指令上
/** @brief 将编码器校准命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
void drive_emm_MMCL_Trig_Encoder_Cal(uint8_t addr);
// 重启电机 - 加载到多电机指令上
/** @brief 将电机重启命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
void drive_emm_MMCL_Reset_Motor(uint8_t addr);
// 将当前位置清零 - 加载到多电机指令上
/** @brief 将当前位置清零命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
void drive_emm_MMCL_Reset_CurPos_To_Zero(uint8_t addr);
// 解除堵转保护 - 加载到多电机指令上
/** @brief 将解除堵转保护命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
void drive_emm_MMCL_Reset_Clog_Pro(uint8_t addr);
// 恢复出厂设置 - 加载到多电机指令上
/** @brief 将恢复出厂设置命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
void drive_emm_MMCL_Restore_Motor(uint8_t addr);
/**********************************************************
*** 运动控制命令
**********************************************************/
// 电机使能控制 - 加载到多电机指令上
/** @brief 将电机使能命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
/** @param state 使能状态。 */
/** @param snF 是否保存设置。 */
void drive_emm_MMCL_En_Control(uint8_t addr, bool state, bool snF);
// 速度模式控制 - 加载到多电机指令上
/** @brief 将速度控制命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
/** @param dir 运动方向。 */
/** @param vel 目标速度。 */
/** @param acc 加速度。 */
/** @param snF 是否保存设置。 */
void drive_emm_MMCL_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);
// 位置模式控制 - 加载到多电机指令上
/** @brief 将位置控制命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
/** @param dir 运动方向。 */
/** @param vel 目标速度。 */
/** @param acc 加速度。 */
/** @param clk 目标脉冲数。 */
/** @param raF 是否相对位置运动。 */
/** @param snF 是否保存设置。 */
void drive_emm_MMCL_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
// 让电机立即停止运动 - 加载到多电机指令上
/** @brief 将立即停止命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
/** @param snF 是否保存设置。 */
void drive_emm_MMCL_Stop_Now(uint8_t addr, bool snF);
// 触发多机同步开始运动 - 加载到多电机指令上
/** @brief 将多机同步开始运动命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
void drive_emm_MMCL_Synchronous_motion(uint8_t addr);
/**********************************************************
*** 原点回零命令
**********************************************************/
// 设置单圈回零的零点位置 - 加载到多电机指令上
/** @brief 将设置单圈回零零点命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
void drive_emm_MMCL_Origin_Set_O(uint8_t addr, bool svF);
// 触发回零 - 加载到多电机指令上
/** @brief 将触发回零命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
/** @param o_mode 回零模式。 */
/** @param snF 是否保存设置。 */
void drive_emm_MMCL_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);
// 强制中断并退出回零 - 加载到多电机指令上
/** @brief 将中断并退出回零命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
void drive_emm_MMCL_Origin_Interrupt(uint8_t addr);
// 修改回零参数 - 加载到多电机指令上
/** @brief 将修改回零参数命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
/** @param svF 是否保存设置。 */
/** @param o_mode 回零模式。 */
/** @param o_dir 回零方向。 */
/** @param o_vel 回零速度。 */
/** @param o_tm 回零超时时间。 */
/** @param sl_vel 切换速度。 */
/** @param sl_ma 切换电流。 */
/** @param sl_ms 切换时间。 */
/** @param potF 是否启用正限位。 */
void drive_emm_MMCL_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);
/**********************************************************
*** 读取系统参数命令
**********************************************************/
// 定时返回信息命令（Y42） - 加载到多电机指令上
/** @brief 将定时返回系统参数命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
/** @param s 系统参数类型。 */
/** @param time_ms 返回周期，单位为毫秒。 */
void drive_emm_MMCL_Auto_Return_Sys_Params_Timed(uint8_t addr, SysParams_t s, uint16_t time_ms);
// 读取系统参数 - 加载到多电机指令上
/** @brief 将读取系统参数命令加载到多电机命令缓存。 */
/** @param addr 电机地址。 */
/** @param s 系统参数类型。 */
void drive_emm_MMCL_Read_Sys_Params(uint8_t addr, SysParams_t s);
/**********************************************************
*** 读写驱动参数命令
**********************************************************/

#endif
