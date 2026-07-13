#ifndef __comm_jetson_H__
#define __comm_jetson_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

  /** @brief Jetson 调试串口模块状态。 */
  typedef enum
  {
    comm_jetson_STATUS_OK = 0,
    comm_jetson_STATUS_INVALID_PARAM,
    comm_jetson_STATUS_NOT_READY,
    comm_jetson_STATUS_RX_ERROR,
    comm_jetson_STATUS_OVERFLOW
  } JetsonDebug_Status_t;

  /** @brief 初始化 Jetson 调试串口。 @param huart 串口句柄。 @return 初始化结果状态。 */
  JetsonDebug_Status_t JetsonDebug_Init(UART_HandleTypeDef *huart);
  /** @brief 轮询处理 Jetson 调试数据。 */
  void JetsonDebug_Poll(void);
  /** @brief 处理 Jetson 串口接收事件。 @param huart 串口句柄。 @param size 本次接收字节数。 */
  void JetsonDebug_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
  /** @brief 处理 Jetson 串口错误。 @param huart 串口句柄。 */
  void JetsonDebug_OnUartError(UART_HandleTypeDef *huart);
  /** @brief 获取 Jetson 调试模块最近一次状态。 @return 最近一次状态码。 */
  JetsonDebug_Status_t JetsonDebug_GetLastStatus(void);

#ifdef __cplusplus
}
#endif

#endif
