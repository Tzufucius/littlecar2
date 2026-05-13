#ifndef __comm_jetson_H__
#define __comm_jetson_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  comm_jetson_STATUS_OK = 0,
  comm_jetson_STATUS_INVALID_PARAM,
  comm_jetson_STATUS_NOT_READY,
  comm_jetson_STATUS_RX_ERROR,
  comm_jetson_STATUS_OVERFLOW
} JetsonDebug_Status_t;

JetsonDebug_Status_t JetsonDebug_Init(UART_HandleTypeDef *huart);
void JetsonDebug_Poll(void);
void JetsonDebug_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void JetsonDebug_OnUartError(UART_HandleTypeDef *huart);
JetsonDebug_Status_t JetsonDebug_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif
