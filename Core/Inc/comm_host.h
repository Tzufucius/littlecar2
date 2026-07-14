#ifndef __COMM_HOST_H__
#define __COMM_HOST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "comm_common.h"
#include "main.h"
#include <stdint.h>

/** @brief 主机通信模块返回状态。 */
typedef enum
{
  HOST_COMM_STATUS_OK = 0,
  HOST_COMM_STATUS_INVALID_PARAM,
  HOST_COMM_STATUS_NOT_READY,
  HOST_COMM_STATUS_RX_ERROR,
  HOST_COMM_STATUS_OVERFLOW
} HostComm_Status_t;

/** @brief 初始化主机来源与 UART 通道的绑定关系。 */
HostComm_Status_t HostComm_InitChannel(HostSource_t source, UART_HandleTypeDef *huart);
/** @brief 周期处理主机通信接收状态。 */
void HostComm_Poll(void);
/** @brief 处理 UART 接收事件。 */
void HostComm_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
/** @brief 处理 UART 错误事件。 */
void HostComm_OnUartError(UART_HandleTypeDef *huart);
/** @brief 获取指定主机来源最近一次通信状态。 */
HostComm_Status_t HostComm_GetStatus(HostSource_t source);

#ifdef __cplusplus
}
#endif

#endif
