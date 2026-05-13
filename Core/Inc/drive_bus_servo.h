#ifndef __drive_bus_servo_H__
#define __drive_bus_servo_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define drive_bus_servo_BROADCAST_ID         ((uint8_t)0xFE)
#define drive_bus_servo_DEFAULT_ACCELERATION ((uint16_t)1500U)
#define drive_bus_servo_DEFAULT_SPEED        ((uint16_t)2000U)
#define drive_bus_servo_MIN_ID               ((uint8_t)0x00)
#define drive_bus_servo_MAX_ID               ((uint8_t)0xFD)
#define drive_bus_servo_MAX_POSITION         ((int32_t)0x7FFF)
#define drive_bus_servo_MIN_POSITION         ((int32_t)-0x7FFF)

typedef enum
{
  drive_bus_servo_STATUS_OK = 0,
  drive_bus_servo_STATUS_INVALID_PARAM,
  drive_bus_servo_STATUS_NOT_READY,
  drive_bus_servo_STATUS_TX_ERROR,
  drive_bus_servo_STATUS_RX_ERROR
} BusServo_Status_t;

typedef struct
{
  uint8_t id;
  uint16_t acceleration;
  int32_t position;
  uint16_t speed;
} BusServo_Command_t;

/**
  * @brief  初始化总线舵机设备层并绑定当前工程使用的串口句柄。
  * @param  huart: 用于舵机通信的串口句柄，当前规划固定传入 `&huart4`。
  * @retval 初始化结果状态。
  */
BusServo_Status_t BusServo_Init(UART_HandleTypeDef *huart);

/**
  * @brief  启动 UART 单字节中断接收，用于兼容当前工程的统一串口回调链路。
  * @retval 启动接收结果状态。
  */
BusServo_Status_t BusServo_StartReceive(void);

/**
  * @brief  处理 UART 单字节接收完成事件，并在模块内部重启下一次接收。
  * @retval None
  */
void BusServo_OnByteReceived(void);

/**
  * @brief  处理串口错误并重启单字节接收。
  * @retval None
  */
void BusServo_OnUartError(void);

/**
  * @brief  轮询舵机模块内部状态。
  * @note   当前首版为纯发送设备层，该接口保留为空实现，方便后续扩展回读协议。
  * @retval None
  */
void BusServo_Poll(void);

/**
  * @brief  使用默认加速度和速度发送位置控制命令。
  * @param  id: 舵机 ID，支持 0~253，254 作为广播地址仅建议上层显式使用扩展接口。
  * @param  position: 目标位置，允许沿用旧项目中的有符号位姿写法。
  * @retval 发送结果状态。
  */
BusServo_Status_t BusServo_SetPosition(uint8_t id, int32_t position);

/**
  * @brief  发送带加速度和速度参数的位置控制命令。
  * @param  id: 舵机 ID。
  * @param  acceleration: 加速度参数，为兼容旧项目调用习惯，协议层仅使用其低 8 位。
  * @param  position: 目标位置。
  * @param  speed: 目标速度。
  * @retval 发送结果状态。
  */
BusServo_Status_t BusServo_SetPositionEx(uint8_t id, uint16_t acceleration, int32_t position, uint16_t speed);

/**
  * @brief  按结构化命令发送单条舵机控制帧。
  * @param  command: 待发送命令结构体指针。
  * @retval 发送结果状态。
  */
BusServo_Status_t BusServo_SendCommand(const BusServo_Command_t *command);

/**
  * @brief  按顺序逐条发送一组舵机控制命令。
  * @param  commands: 命令数组指针。
  * @param  count: 命令数量。
  * @retval 发送过程中最后一次失败状态，全部成功则返回 `drive_bus_servo_STATUS_OK`。
  */
BusServo_Status_t BusServo_SendGroup(const BusServo_Command_t *commands, uint8_t count);

/**
  * @brief  获取最近一次舵机设备层操作状态。
  * @retval 最近一次状态码。
  */
BusServo_Status_t BusServo_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif
