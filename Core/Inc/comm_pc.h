#ifndef __comm_pc_H__
#define __comm_pc_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

  /** @brief 上位机接收模块状态。 */
  typedef enum
  {
    comm_pc_STATUS_OK = 0,
    comm_pc_STATUS_INVALID_PARAM,
    comm_pc_STATUS_NOT_READY,
    comm_pc_STATUS_RX_ERROR,
    comm_pc_STATUS_OVERFLOW
  } HostRx_Status_t;

  /** @brief 上位机命令来源。 */
  typedef enum
  {
    comm_pc_SOURCE_PC = 0,
    comm_pc_SOURCE_JETSON,
    comm_pc_SOURCE_COUNT
  } HostRx_Source_t;

  /** @brief 初始化 PC 接收通道。 @param huart 串口句柄。 @return 初始化结果状态。 */
  HostRx_Status_t HostRx_InitPc(UART_HandleTypeDef *huart);
  /** @brief 初始化 Jetson 接收通道。 @param huart 串口句柄。 @return 初始化结果状态。 */
  HostRx_Status_t HostRx_InitJetson(UART_HandleTypeDef *huart);
  /** @brief 轮询处理上位机接收数据。 */
  void HostRx_Poll(void);
  /** @brief 处理上位机串口接收事件。 @param huart 串口句柄。 @param size 本次接收字节数。 */
  void HostRx_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
  /** @brief 处理上位机串口错误。 @param huart 串口句柄。 */
  void HostRx_OnUartError(UART_HandleTypeDef *huart);
  /** @brief 获取指定来源最近一次接收状态。 @param source 命令来源。 @return 最近一次状态码。 */
  HostRx_Status_t HostRx_GetLastStatus(HostRx_Source_t source);

#ifdef __cplusplus
}
#endif

#endif
