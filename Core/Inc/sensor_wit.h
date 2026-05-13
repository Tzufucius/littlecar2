#ifndef __WIT_IMU_H__
#define __WIT_IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  WIT_STATUS_OK = 0,
  WIT_STATUS_NOT_READY,
  WIT_STATUS_RX_ERROR,
  WIT_STATUS_FRAME_ERROR
} WIT_Status_t;

typedef struct
{
  float x;
  float y;
  float z;
  uint32_t updated_tick;
  uint8_t valid;
} WIT_Vector3f_t;

typedef struct
{
  WIT_Vector3f_t accel_g;
  WIT_Vector3f_t gyro_dps;
  WIT_Vector3f_t angle_deg;
} WIT_Data_t;

WIT_Status_t WIT_Init(void);
void WIT_Poll(void);

void WIT_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
void WIT_OnUartError(UART_HandleTypeDef *huart);
const volatile WIT_Data_t *WIT_GetData(void);

#ifdef __cplusplus
}
#endif

#endif
