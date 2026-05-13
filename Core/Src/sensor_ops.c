#include "sensor_ops.h"

#define OPS_FRAME_HEADER_FIRST   ((uint8_t)0x0DU)
#define OPS_FRAME_HEADER_SECOND  ((uint8_t)0x0AU)
#define OPS_FRAME_FOOTER_FIRST   ((uint8_t)0x0AU)
#define OPS_FRAME_FOOTER_SECOND  ((uint8_t)0x0DU)
#define OPS_FRAME_PAYLOAD_SIZE   ((uint16_t)24U)
#define OPS_STALE_TIMEOUT_MS     ((uint32_t)500U)

typedef enum
{
  OPS_PARSER_WAIT_HEADER_FIRST = 0,
  OPS_PARSER_WAIT_HEADER_SECOND,
  OPS_PARSER_READ_PAYLOAD,
  OPS_PARSER_WAIT_FOOTER_FIRST,
  OPS_PARSER_WAIT_FOOTER_SECOND
} OPS_ParserState_t;

typedef union
{
  uint8_t bytes[OPS_FRAME_PAYLOAD_SIZE]; /*!< 原始字节数组 (24字节) */
  float values[6];                       /*!< 映射后的浮点数数组 (6个) */
} OPS_Payload_t;

static UART_HandleTypeDef *g_ops_uart = NULL;
static uint8_t g_ops_rx_byte = 0U;
static OPS_ParserState_t g_ops_parser_state = OPS_PARSER_WAIT_HEADER_FIRST;
static OPS_Payload_t g_ops_payload = {0};
static uint16_t g_ops_payload_index = 0U;
static OPS_Pose_t g_ops_pose = {0};
static OPS_Status_t g_ops_last_status = OPS_STATUS_NOT_READY;
static uint8_t g_ops_initialized = 0U;

/**
 * @brief 重置解析器状态
 * @details 当发生帧错误、丢包或完成一帧接收后，调用此函数将状态机归位。
 */
static void OPS_ResetParser(void)
{
  g_ops_parser_state = OPS_PARSER_WAIT_HEADER_FIRST;
  g_ops_payload_index = 0U;
}

static UART_HandleTypeDef *OPS_GetUartHandle(void)
{
  return g_ops_uart;
}

static void OPS_CopyPose(OPS_Pose_t *pose)
{
  __disable_irq();
  *pose = g_ops_pose;
  __enable_irq();
}

/**
 * @brief 更新位姿数据
 * @details
 * 当一帧数据完整接收后调用此函数。
 * 我们在这里把 union 里的数据提取到全局变量 g_ops_pose 中。
 */
static void OPS_UpdatePose(void)
{
  __disable_irq();

  g_ops_pose.zangle_deg = g_ops_payload.values[0];
  g_ops_pose.xangle_deg = g_ops_payload.values[1];
  g_ops_pose.yangle_deg = g_ops_payload.values[2];
  g_ops_pose.pos_x_mm   = g_ops_payload.values[3];
  g_ops_pose.pos_y_mm   = g_ops_payload.values[4];
  g_ops_pose.w_z_dps    = g_ops_payload.values[5];

  g_ops_pose.updated_tick = HAL_GetTick();
  g_ops_pose.frame_count++;
  g_ops_pose.valid = 1U;

  __enable_irq();

  g_ops_last_status = OPS_STATUS_OK;
}

/**
 * @brief 启动下一次异步接收
 * @return OPS_STATUS_OK: 启动成功; OPS_STATUS_RX_ERROR: 硬件或驱动错误
 */
static OPS_Status_t OPS_StartReceive(void)
{
  if ((OPS_GetUartHandle() == NULL) ||
      (HAL_UART_Receive_IT(OPS_GetUartHandle(), &g_ops_rx_byte, 1U) != HAL_OK))
  {
    g_ops_last_status = OPS_STATUS_RX_ERROR;
    return g_ops_last_status;
  }

  // 如果开启了 DMA 接收，则禁用“半完成”中断以节省 CPU 开销
  if (OPS_GetUartHandle()->hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(OPS_GetUartHandle()->hdmarx, DMA_IT_HT);
  }

  g_ops_last_status = OPS_STATUS_OK;
  return g_ops_last_status;
}

/**
 * @brief 检查数据是否超时
 * @details 比较当前系统时间与上次数据更新时间，若超过 500ms 则标记数据无效。
 */
static void OPS_InvalidateStaleData(void)
{
  uint32_t now_tick = HAL_GetTick();

  __disable_irq();
  if ((g_ops_pose.valid != 0U) && ((now_tick - g_ops_pose.updated_tick) > OPS_STALE_TIMEOUT_MS))
  {
    g_ops_pose.valid = 0U;
  }
  __enable_irq();
}

/**
 * @brief 处理接收到的每一个字节 (状态机解析协议帧)
 * @details
 * 数据帧结构 (28字节):
 * 帧头: 0x0D 0x0A (2字节)
 * 负载: 6个 float (24字节) -> zangle, xangle, yangle, pos_x, pos_y, w_z
 * 帧尾: 0x0A 0x0D (2字节)
 */
static void OPS_HandleFrameByte(uint8_t byte)
{
  switch (g_ops_parser_state)
  {
    case OPS_PARSER_WAIT_HEADER_FIRST:
      // 等待第一个帧头 0x0D
      if (byte == OPS_FRAME_HEADER_FIRST)
      {
        g_ops_parser_state = OPS_PARSER_WAIT_HEADER_SECOND;
      }
      break;

    case OPS_PARSER_WAIT_HEADER_SECOND:
      // 等待第二个帧头 0x0A
      if (byte == OPS_FRAME_HEADER_SECOND)
      {
        g_ops_parser_state = OPS_PARSER_READ_PAYLOAD;
        g_ops_payload_index = 0U;
      }
      else if (byte == OPS_FRAME_HEADER_FIRST)
      {
        // 如果连续收到两个 0x0D，保持在当前状态
        g_ops_parser_state = OPS_PARSER_WAIT_HEADER_SECOND;
      }
      else
      {
        OPS_ResetParser();
      }
      break;

    case OPS_PARSER_READ_PAYLOAD:
      // 连续读取 24 字节的浮点数数据
      g_ops_payload.bytes[g_ops_payload_index++] = byte;
      if (g_ops_payload_index >= OPS_FRAME_PAYLOAD_SIZE)
      {
        g_ops_parser_state = OPS_PARSER_WAIT_FOOTER_FIRST;
      }
      break;

    case OPS_PARSER_WAIT_FOOTER_FIRST:
      // 等待第一个帧尾 0x0A
      if (byte == OPS_FRAME_FOOTER_FIRST)
      {
        g_ops_parser_state = OPS_PARSER_WAIT_FOOTER_SECOND;
      }
      else if (byte == OPS_FRAME_HEADER_FIRST)
      {
        // 容错处理：如果意外收到新的帧头
        g_ops_parser_state = OPS_PARSER_WAIT_HEADER_SECOND;
      }
      else
      {
        OPS_ResetParser();
      }
      break;

    case OPS_PARSER_WAIT_FOOTER_SECOND:
      // 等待第二个帧尾 0x0D，成功则更新位姿
      if (byte == OPS_FRAME_FOOTER_SECOND)
      {
        OPS_UpdatePose();
        OPS_ResetParser();
      }
      else if (byte == OPS_FRAME_HEADER_FIRST)
      {
        g_ops_parser_state = OPS_PARSER_WAIT_HEADER_SECOND;
        g_ops_payload_index = 0U;
      }
      else
      {
        g_ops_last_status = OPS_STATUS_FRAME_ERROR;
        OPS_ResetParser();
      }
      break;

    default:
      OPS_ResetParser();
      break;
  }
}

OPS_Status_t OPS_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    g_ops_last_status = OPS_STATUS_INVALID_PARAM;
    return g_ops_last_status;
  }

  g_ops_uart = huart;
  g_ops_initialized = 0U;
  g_ops_pose = (OPS_Pose_t){0};
  OPS_ResetParser();

  g_ops_last_status = OPS_StartReceive();
  if (g_ops_last_status != OPS_STATUS_OK)
  {
    return g_ops_last_status;
  }

  g_ops_initialized = 1U;
  return OPS_STATUS_OK;
}

void OPS_Poll(void)
{
  OPS_InvalidateStaleData();
}

void OPS_OnByteReceived(void)
{
  if ((OPS_GetUartHandle() == NULL) || (g_ops_initialized == 0U))
  {
    return;
  }

  OPS_HandleFrameByte(g_ops_rx_byte);
  (void)OPS_StartReceive();
}

void OPS_OnUartError(void)
{
  if (OPS_GetUartHandle() == NULL)
  {
    return;
  }

  g_ops_last_status = OPS_STATUS_RX_ERROR;
  OPS_ResetParser();
  (void)OPS_StartReceive();
}

OPS_Status_t OPS_GetPose(OPS_Pose_t *pose)
{
  if (pose == NULL)
  {
    return OPS_STATUS_INVALID_PARAM;
  }

  OPS_CopyPose(pose);
  return (pose->valid != 0U) ? OPS_STATUS_OK : OPS_STATUS_NO_DATA;
}

const volatile OPS_Pose_t *OPS_GetPoseRef(void)
{
  return &g_ops_pose;
}
