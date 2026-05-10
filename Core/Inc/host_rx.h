#ifndef __HOST_RX_H__
#define __HOST_RX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  HOST_RX_STATUS_OK = 0,
  HOST_RX_STATUS_INVALID_PARAM,
  HOST_RX_STATUS_NOT_READY,
  HOST_RX_STATUS_RX_ERROR,
  HOST_RX_STATUS_OVERFLOW
} HostRx_Status_t;

typedef enum
{
  HOST_RX_SOURCE_PC = 0,
  HOST_RX_SOURCE_JETSON,
  HOST_RX_SOURCE_COUNT
} HostRx_Source_t;

HostRx_Status_t HostRx_InitPc(UART_HandleTypeDef *huart);
HostRx_Status_t HostRx_InitJetson(UART_HandleTypeDef *huart);
void HostRx_Poll(void);
void HostRx_OnPcByteReceived(UART_HandleTypeDef *huart);
void HostRx_OnJetsonUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void HostRx_OnUartError(UART_HandleTypeDef *huart);
HostRx_Status_t HostRx_GetLastStatus(HostRx_Source_t source);

#ifdef __cplusplus
}
#endif

#endif
