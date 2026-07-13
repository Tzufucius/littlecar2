#ifndef __COMM_HOST_H__
#define __COMM_HOST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "comm_common.h"
#include "main.h"
#include <stdint.h>

typedef enum
{
  HOST_COMM_STATUS_OK = 0,
  HOST_COMM_STATUS_INVALID_PARAM,
  HOST_COMM_STATUS_NOT_READY,
  HOST_COMM_STATUS_RX_ERROR,
  HOST_COMM_STATUS_OVERFLOW
} HostComm_Status_t;

HostComm_Status_t HostComm_InitChannel(HostSource_t source, UART_HandleTypeDef *huart);
void HostComm_Poll(void);
void HostComm_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void HostComm_OnUartError(UART_HandleTypeDef *huart);
HostComm_Status_t HostComm_GetStatus(HostSource_t source);

#ifdef __cplusplus
}
#endif

#endif
