#ifndef __comm_protocol_H__
#define __comm_protocol_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  comm_protocol_SOURCE_PC = 0,
  comm_protocol_SOURCE_JETSON,
  comm_protocol_SOURCE_COUNT
} HostProtocol_Source_t;

typedef enum
{
  comm_protocol_STATUS_OK = 0,
  comm_protocol_STATUS_INVALID_PARAM,
  comm_protocol_STATUS_OVERFLOW
} HostProtocol_Status_t;

/*
 * 绑定某一路上位机接收源对应的 UART 句柄。
 *
 * 作用：
 * - 协议层收到有效命令后，需要从同一路 UART 回发 ACK。
 * - PC 和 Jetson 可以共用同一套帧解析逻辑，但 ACK 必须回到各自来源。
 */
void HostProtocol_RegisterSource(HostProtocol_Source_t source, UART_HandleTypeDef *huart);

/*
 * 向协议解析器输入原始字节流。
 *
 * 调用位置：
 * - USART DMA + IDLE 回调中只搬运数据，不直接执行电机动作。
 * - 完整帧被解析并校验通过后，会进入主循环队列等待执行。
 */
void HostProtocol_OnBytes(HostProtocol_Source_t source, const uint8_t *data, uint16_t length);

/*
 * 在主循环中轮询执行协议命令。
 *
 * 该函数负责：
 * - 检查心跳超时。
 * - 从命令队列取出完整帧。
 * - 分发到 SYSTEM / SAFETY / CHASSIS 处理函数。
 * - 回发 ACK。
 */
void HostProtocol_Poll(void);

HostProtocol_Status_t HostProtocol_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif
