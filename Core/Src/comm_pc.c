#include "comm_pc.h"
#include "comm_protocol.h"
#include <stdio.h>
#include <string.h>

/*
 * 上位机原始接收桥接层。
 *
 * 本文件负责两件事：
 * - 使用 DMA + UART IDLE 接收 PC / Jetson 的原始字节流，并打印 hex/ascii 方便调试。
 * - 将同一份字节流送入 comm_protocol，由协议层负责找帧、CRC 校验、命令入队和 ACK。
 *
 * 注意：
 * - 这里不直接执行底盘、舵机、传感器业务命令。
 * - 如果串口助手只发 hello\r\n，会看到原始回显日志，但不会触发正式协议命令。
 */
#define comm_pc_DMA_BUFFER_SIZE    ((uint16_t)256U)
#define comm_pc_PRINT_BUFFER_SIZE  ((uint16_t)256U)

typedef struct
{
  const char *name;
  UART_HandleTypeDef *uart;
  uint8_t dma_buffer[comm_pc_DMA_BUFFER_SIZE];
  uint16_t dma_last_pos;
  uint8_t print_buffer[comm_pc_PRINT_BUFFER_SIZE];
  volatile uint16_t print_length;
  volatile uint8_t print_ready;
  volatile uint8_t error_ready;
  volatile uint32_t overflow_count;
  volatile uint32_t uart_error_code;
  HostRx_Status_t last_status;
} HostRx_Channel_t;

static HostRx_Channel_t g_comm_pc_channels[comm_pc_SOURCE_COUNT] = {
  [comm_pc_SOURCE_PC] = {
    .name = "PC",
    .uart = NULL,
    .dma_last_pos = 0U,
    .print_length = 0U,
    .print_ready = 0U,
    .error_ready = 0U,
    .overflow_count = 0U,
    .uart_error_code = 0U,
    .last_status = comm_pc_STATUS_NOT_READY
  },
  [comm_pc_SOURCE_JETSON] = {
    .name = "JETSON",
    .uart = NULL,
    .dma_last_pos = 0U,
    .print_length = 0U,
    .print_ready = 0U,
    .error_ready = 0U,
    .overflow_count = 0U,
    .uart_error_code = 0U,
    .last_status = comm_pc_STATUS_NOT_READY
  }
};

static HostRx_Status_t HostRx_StartDmaReceive(HostRx_Source_t source)
{
  HostRx_Channel_t *channel;

  if (source >= comm_pc_SOURCE_COUNT)
  {
    return comm_pc_STATUS_INVALID_PARAM;
  }

  channel = &g_comm_pc_channels[source];

  if (channel->uart == NULL)
  {
    channel->last_status = comm_pc_STATUS_NOT_READY;
    return channel->last_status;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(channel->uart,
                                   channel->dma_buffer,
                                   comm_pc_DMA_BUFFER_SIZE) != HAL_OK)
  {
    channel->last_status = comm_pc_STATUS_RX_ERROR;
    return channel->last_status;
  }

  if (channel->uart->hdmarx != NULL)
  {
    /* 半传输中断对当前协议无意义，关闭后只处理 IDLE / 满缓冲事件。 */
    __HAL_DMA_DISABLE_IT(channel->uart->hdmarx, DMA_IT_HT);
  }

  channel->dma_last_pos = 0U;
  channel->last_status = comm_pc_STATUS_OK;
  return channel->last_status;
}

static uint8_t HostRx_IsVisibleAscii(uint8_t byte)
{
  return (byte >= 0x20U) && (byte <= 0x7EU);
}

static void HostRx_ResetChannel(HostRx_Channel_t *channel)
{
  channel->dma_last_pos = 0U;
  channel->print_length = 0U;
  channel->print_ready = 0U;
  channel->error_ready = 0U;
  channel->overflow_count = 0U;
  channel->uart_error_code = 0U;
  memset(channel->dma_buffer, 0, sizeof(channel->dma_buffer));
  memset(channel->print_buffer, 0, sizeof(channel->print_buffer));
}

static void HostRx_AppendReceivedData(HostRx_Source_t source, const uint8_t *data, uint16_t length)
{
  HostRx_Channel_t *channel;
  uint16_t copy_length;

  if ((source >= comm_pc_SOURCE_COUNT) || (data == NULL) || (length == 0U))
  {
    return;
  }

  /*
   * 同一份原始接收数据同时进入协议层。
   * 协议层能够处理拆包、粘包和连续多帧；这里不需要按帧切分。
   */
  HostProtocol_OnBytes((HostProtocol_Source_t)source, data, length);

  channel = &g_comm_pc_channels[source];

  if (channel->print_length >= comm_pc_PRINT_BUFFER_SIZE)
  {
    ++channel->overflow_count;
    channel->last_status = comm_pc_STATUS_OVERFLOW;
    return;
  }

  copy_length = (uint16_t)(comm_pc_PRINT_BUFFER_SIZE - channel->print_length);
  if (copy_length > length)
  {
    copy_length = length;
  }

  memcpy(&channel->print_buffer[channel->print_length], data, copy_length);
  channel->print_length = (uint16_t)(channel->print_length + copy_length);
  channel->print_ready = 1U;

  if (copy_length < length)
  {
    ++channel->overflow_count;
    channel->last_status = comm_pc_STATUS_OVERFLOW;
  }
}

static void HostRx_HandleDmaRxEvent(HostRx_Source_t source, uint16_t size)
{
  HostRx_Channel_t *channel;
  uint16_t current_pos = size;

  if (source >= comm_pc_SOURCE_COUNT)
  {
    return;
  }

  channel = &g_comm_pc_channels[source];

  if (current_pos > comm_pc_DMA_BUFFER_SIZE)
  {
    current_pos = comm_pc_DMA_BUFFER_SIZE;
  }

  if (current_pos == channel->dma_last_pos)
  {
    return;
  }

  if (current_pos > channel->dma_last_pos)
  {
    /* 普通情况：DMA 写指针向前移动，只取新增区间。 */
    HostRx_AppendReceivedData(source,
                              &channel->dma_buffer[channel->dma_last_pos],
                              (uint16_t)(current_pos - channel->dma_last_pos));
  }
  else
  {
    /* 环形缓冲区回绕：先取尾部，再取头部。 */
    HostRx_AppendReceivedData(source,
                              &channel->dma_buffer[channel->dma_last_pos],
                              (uint16_t)(comm_pc_DMA_BUFFER_SIZE - channel->dma_last_pos));
    if (current_pos > 0U)
    {
      HostRx_AppendReceivedData(source, &channel->dma_buffer[0], current_pos);
    }
  }

  channel->dma_last_pos = (current_pos == comm_pc_DMA_BUFFER_SIZE) ? 0U : current_pos;
}

static void HostRx_PrintChannel(HostRx_Channel_t *channel)
{
  uint8_t local_buffer[comm_pc_PRINT_BUFFER_SIZE];
  uint16_t local_length;
  uint32_t overflow_count;
  uint32_t error_code;
  uint16_t index;

  if ((channel->print_ready == 0U) && (channel->error_ready == 0U))
  {
    return;
  }

  __disable_irq();
  /* 将中断侧累积的打印数据搬到局部缓冲，随后立即恢复中断。 */
  error_code = channel->uart_error_code;
  local_length = channel->print_length;
  if (local_length > comm_pc_PRINT_BUFFER_SIZE)
  {
    local_length = comm_pc_PRINT_BUFFER_SIZE;
  }
  memcpy(local_buffer, channel->print_buffer, local_length);
  overflow_count = channel->overflow_count;
  channel->print_length = 0U;
  channel->print_ready = 0U;
  channel->error_ready = 0U;
  channel->overflow_count = 0U;
  channel->uart_error_code = 0U;
  __enable_irq();

  if (error_code != 0U)
  {
    printf("%s ERR code=0x%08lX\r\n", channel->name, (unsigned long)error_code);
  }

  if (local_length == 0U)
  {
    return;
  }

  printf("%s RX len=%u hex=", channel->name, (unsigned int)local_length);
  for (index = 0U; index < local_length; ++index)
  {
    if (index > 0U)
    {
      printf(" ");
    }
    printf("%02X", local_buffer[index]);
  }

  printf(" ascii=");
  for (index = 0U; index < local_length; ++index)
  {
    if (local_buffer[index] == '\r')
    {
      printf("\\r");
    }
    else if (local_buffer[index] == '\n')
    {
      printf("\\n");
    }
    else if (local_buffer[index] == '\t')
    {
      printf("\\t");
    }
    else if (HostRx_IsVisibleAscii(local_buffer[index]) != 0U)
    {
      printf("%c", local_buffer[index]);
    }
    else
    {
      printf(".");
    }
  }

  if (overflow_count > 0U)
  {
    printf(" overflow=%lu", (unsigned long)overflow_count);
  }

  printf("\r\n");
}

HostRx_Status_t HostRx_InitPc(UART_HandleTypeDef *huart)
{
  HostRx_Channel_t *channel = &g_comm_pc_channels[comm_pc_SOURCE_PC];

  if (huart == NULL)
  {
    channel->last_status = comm_pc_STATUS_INVALID_PARAM;
    return channel->last_status;
  }

  channel->uart = huart;
  HostRx_ResetChannel(channel);
  /* PC 调试口也注册到协议层，便于用串口助手验证 0x5A 0xA5 帧。 */
  HostProtocol_RegisterSource(comm_protocol_SOURCE_PC, huart);
  return HostRx_StartDmaReceive(comm_pc_SOURCE_PC);
}

HostRx_Status_t HostRx_InitJetson(UART_HandleTypeDef *huart)
{
  HostRx_Channel_t *channel = &g_comm_pc_channels[comm_pc_SOURCE_JETSON];

  if (huart == NULL)
  {
    channel->last_status = comm_pc_STATUS_INVALID_PARAM;
    return channel->last_status;
  }

  channel->uart = huart;
  HostRx_ResetChannel(channel);
  /* Jetson 是正式上位机入口，ACK 会从 huart6 回发。 */
  HostProtocol_RegisterSource(comm_protocol_SOURCE_JETSON, huart);
  return HostRx_StartDmaReceive(comm_pc_SOURCE_JETSON);
}

void HostRx_Poll(void)
{
  /* 先执行正式协议命令，再输出原始收包日志，方便观察命令和 ACK。 */
  HostProtocol_Poll();
  HostRx_PrintChannel(&g_comm_pc_channels[comm_pc_SOURCE_PC]);
  HostRx_PrintChannel(&g_comm_pc_channels[comm_pc_SOURCE_JETSON]);
}

void HostRx_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  HostRx_Source_t source;

  for (source = comm_pc_SOURCE_PC; source < comm_pc_SOURCE_COUNT; ++source)
  {
    HostRx_Channel_t *channel = &g_comm_pc_channels[source];
    if ((channel->uart != NULL) && (huart == channel->uart))
    {
      HostRx_HandleDmaRxEvent(source, size);
      return;
    }
  }
}

void HostRx_OnUartError(UART_HandleTypeDef *huart)
{
  HostRx_Channel_t *pc_channel = &g_comm_pc_channels[comm_pc_SOURCE_PC];
  HostRx_Channel_t *jetson_channel = &g_comm_pc_channels[comm_pc_SOURCE_JETSON];

  if ((pc_channel->uart != NULL) && (huart == pc_channel->uart))
  {
    pc_channel->uart_error_code = huart->ErrorCode;
    pc_channel->error_ready = 1U;
    pc_channel->last_status = comm_pc_STATUS_RX_ERROR;
    (void)HostRx_StartDmaReceive(comm_pc_SOURCE_PC);
  }

  if ((jetson_channel->uart != NULL) && (huart == jetson_channel->uart))
  {
    jetson_channel->uart_error_code = huart->ErrorCode;
    jetson_channel->error_ready = 1U;
    jetson_channel->last_status = comm_pc_STATUS_RX_ERROR;
    (void)HostRx_StartDmaReceive(comm_pc_SOURCE_JETSON);
  }
}

HostRx_Status_t HostRx_GetLastStatus(HostRx_Source_t source)
{
  if (source >= comm_pc_SOURCE_COUNT)
  {
    return comm_pc_STATUS_INVALID_PARAM;
  }

  return g_comm_pc_channels[source].last_status;
}
