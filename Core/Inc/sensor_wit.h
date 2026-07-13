#ifndef __WIT_IMU_H__
#define __WIT_IMU_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"
#include <stdint.h>

  /** @brief WIT IMU 模块状态。 */
  typedef enum
  {
    WIT_STATUS_OK = 0,
    WIT_STATUS_NOT_READY,
    WIT_STATUS_RX_ERROR,
    WIT_STATUS_FRAME_ERROR
  } WIT_Status_t;

  /** @brief 三维浮点数据及其有效性信息。 */
  typedef struct
  {
    float x; /*!< X 轴数据，具体单位由所属数据类型决定。 */
    float y; /*!< Y 轴数据，具体单位由所属数据类型决定。 */
    float z; /*!< Z 轴数据，具体单位由所属数据类型决定。 */
    uint32_t updated_tick; /*!< 数据更新时间，单位为 ms。 */
    uint8_t valid; /*!< 数据有效标志：1-有效，0-无效或超时。 */
  } WIT_Vector3f_t;

  /** @brief WIT 加速度、角速度和姿态角数据。 */
  typedef struct
  {
    WIT_Vector3f_t accel_g; /*!< 三轴加速度，单位为 g。 */
    WIT_Vector3f_t gyro_dps; /*!< 三轴角速度，单位为度/s。 */
    WIT_Vector3f_t angle_deg; /*!< 三轴姿态角，单位为度。 */
  } WIT_Data_t;

  /** @brief 初始化 WIT IMU。 @return 初始化结果状态。 */
  WIT_Status_t WIT_Init(void);
  /** @brief 轮询解析 WIT IMU 数据。 */
  void WIT_Poll(void);

  /** @brief 处理 WIT 串口接收事件。 @param huart 串口句柄。 @param size 本次接收字节数。 */
  void WIT_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size);
  /** @brief 处理 WIT 串口错误。 @param huart 串口句柄。 */
  void WIT_OnUartError(UART_HandleTypeDef *huart);
  /** @brief 获取最新 WIT 数据，只读。 @return WIT 数据指针。 */
  const volatile WIT_Data_t *WIT_GetData(void);

#ifdef __cplusplus
}
#endif

#endif
