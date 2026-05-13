#ifndef __OPS_SENSOR_H__
#define __OPS_SENSOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef enum
{
  OPS_STATUS_OK = 0,
  OPS_STATUS_INVALID_PARAM,
  OPS_STATUS_NOT_READY,
  OPS_STATUS_RX_ERROR,
  OPS_STATUS_FRAME_ERROR,
  OPS_STATUS_NO_DATA
} OPS_Status_t;

/**
 * @brief OPS 姿态与位置数据结构体
 */
typedef struct
{
  float zangle_deg;   /*!< Z 轴角度/航向角 (单位: 度) */
  float xangle_deg;   /*!< X 轴角度 (单位: 度) */
  float yangle_deg;   /*!< Y 轴角度 (单位: 度) */
  float pos_x_mm;     /*!< X 坐标 (单位: 毫米) */
  float pos_y_mm;     /*!< Y 坐标 (单位: 毫米) */
  float w_z_dps;      /*!< Z 轴角速度 (单位: °/s) */
  uint32_t updated_tick; /*!< 上次数据更新时的系统滴答时间 (ms) */
  uint32_t frame_count;  /*!< 累计接收到的有效帧数 */
  uint8_t valid;      /*!< 数据有效性标志: 1-有效, 0-无效/超时 */
} OPS_Pose_t;

/**
 * @brief 初始化 OPS 传感器
 * @param huart 指向 UART5 的句柄
 * @return OPS_STATUS_OK: 成功; 其他: 失败
 */
OPS_Status_t OPS_Init(UART_HandleTypeDef *huart);

/**
 * @brief 数据存活轮询 (建议在 main 循环中调用)
 * @details 若超过 500ms 未收到新数据，将自动把 valid 标志位置 0
 */
void OPS_Poll(void);

/**
 * @brief 串口接收中断回调处理 (必须在 HAL_UART_RxCpltCallback 中调用)
 */
void OPS_OnByteReceived(void);

/**
 * @brief 串口错误处理回调 (必须在 HAL_UART_ErrorCallback 中调用)
 */
void OPS_OnUartError(void);

/* --- 数据获取接口 --- */

/**
 * @brief 获取完整的位姿结构体
 */
OPS_Status_t OPS_GetPose(OPS_Pose_t *pose);
const volatile OPS_Pose_t *OPS_GetPoseRef(void);

#ifdef __cplusplus
}
#endif

#endif
