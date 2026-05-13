#ifndef __comm_pc_H__
#define __comm_pc_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  comm_pc_STATUS_OK = 0,
  comm_pc_STATUS_INVALID_PARAM,
  comm_pc_STATUS_NOT_READY,
  comm_pc_STATUS_RX_ERROR,
  comm_pc_STATUS_OVERFLOW
} HostRx_Status_t;

typedef enum
{
  comm_pc_SOURCE_PC = 0,
  comm_pc_SOURCE_JETSON,
  comm_pc_SOURCE_COUNT
} HostRx_Source_t;

HostRx_Status_t HostRx_InitPc(UART_HandleTypeDef *huart);
HostRx_Status_t HostRx_InitJetson(UART_HandleTypeDef *huart);
void HostRx_Poll(void);
void HostRx_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void HostRx_OnUartError(UART_HandleTypeDef *huart);
HostRx_Status_t HostRx_GetLastStatus(HostRx_Source_t source);

#ifdef __cplusplus
}
#endif

#endif
