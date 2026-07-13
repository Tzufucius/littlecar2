#include "comm_host.h"
#include "comm_protocol.h"

/* 发布固件关闭原始收包日志，避免协议调度路径执行阻塞式 printf。 */
#define COMM_HOST_RAW_DEBUG_ENABLE (0U)
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
#define comm_host_DMA_BUFFER_SIZE ((uint16_t)256U)
#define comm_host_PRINT_BUFFER_SIZE ((uint16_t)256U)

typedef struct
{
  const char *name;
  UART_HandleTypeDef *uart;
  uint8_t dma_buffer[comm_host_DMA_BUFFER_SIZE];
  uint16_t dma_last_pos;
  uint8_t print_buffer[comm_host_PRINT_BUFFER_SIZE];
  volatile uint16_t print_length;
  volatile uint8_t print_ready;
  volatile uint8_t error_ready;
  volatile uint32_t overflow_count;
  volatile uint32_t uart_error_code;
  HostComm_Status_t last_status;
} HostComm_Channel_t;

static HostComm_Channel_t g_comm_host_channels[HOST_SOURCE_COUNT] = {
    [HOST_SOURCE_PC] = {
        .name = "PC",
        .uart = NULL,
        .dma_last_pos = 0U,
        .print_length = 0U,
        .print_ready = 0U,
        .error_ready = 0U,
        .overflow_count = 0U,
        .uart_error_code = 0U,
        .last_status = HOST_COMM_STATUS_NOT_READY},
    [HOST_SOURCE_JETSON] = {.name = "JETSON", .uart = NULL, .dma_last_pos = 0U, .print_length = 0U, .print_ready = 0U, .error_ready = 0U, .overflow_count = 0U, .uart_error_code = 0U, .last_status = HOST_COMM_STATUS_NOT_READY}};

static HostComm_Status_t HostComm_StartDmaReceive(HostSource_t source)
{
  HostComm_Channel_t *channel;

  if (source >= HOST_SOURCE_COUNT)
  {
    return HOST_COMM_STATUS_INVALID_PARAM;
  }

  channel = &g_comm_host_channels[source];

  if (channel->uart == NULL)
  {
    channel->last_status = HOST_COMM_STATUS_NOT_READY;
    return channel->last_status;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(channel->uart,
                                   channel->dma_buffer,
                                   comm_host_DMA_BUFFER_SIZE) != HAL_OK)
  {
    channel->last_status = HOST_COMM_STATUS_RX_ERROR;
    return channel->last_status;
  }

  if (channel->uart->hdmarx != NULL)
  {
    /* 半传输中断对当前协议无意义，关闭后只处理 IDLE / 满缓冲事件。 */
    __HAL_DMA_DISABLE_IT(channel->uart->hdmarx, DMA_IT_HT);
  }

  channel->dma_last_pos = 0U;
  channel->last_status = HOST_COMM_STATUS_OK;
  return channel->last_status;
}

static uint8_t HostComm_IsVisibleAscii(uint8_t byte)
{
  return (byte >= 0x20U) && (byte <= 0x7EU);
}

static void HostComm_ResetChannel(HostComm_Channel_t *channel)
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

static void HostComm_AppendReceivedData(HostSource_t source, const uint8_t *data, uint16_t length)
{
  HostComm_Channel_t *channel;
  uint16_t copy_length;

  if ((source >= HOST_SOURCE_COUNT) || (data == NULL) || (length == 0U))
  {
    return;
  }

  /*
   * 同一份原始接收数据同时进入协议层。
   * 协议层能够处理拆包、粘包和连续多帧；这里不需要按帧切分。
   */
  HostProtocol_OnBytes((HostSource_t)source, data, length);

  channel = &g_comm_host_channels[source];

  if (channel->print_length >= comm_host_PRINT_BUFFER_SIZE)
  {
    ++channel->overflow_count;
    channel->last_status = HOST_COMM_STATUS_OVERFLOW;
    return;
  }

  copy_length = (uint16_t)(comm_host_PRINT_BUFFER_SIZE - channel->print_length);
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
    channel->last_status = HOST_COMM_STATUS_OVERFLOW;
  }
}

static void HostComm_HandleDmaRxEvent(HostSource_t source, uint16_t size)
{
  HostComm_Channel_t *channel;
  uint16_t current_pos = size;

  if (source >= HOST_SOURCE_COUNT)
  {
    return;
  }

  channel = &g_comm_host_channels[source];

  if (current_pos > comm_host_DMA_BUFFER_SIZE)
  {
    current_pos = comm_host_DMA_BUFFER_SIZE;
  }

  if (current_pos == channel->dma_last_pos)
  {
    return;
  }

  if (current_pos > channel->dma_last_pos)
  {
    /* 普通情况：DMA 写指针向前移动，只取新增区间。 */
    HostComm_AppendReceivedData(source,
                              &channel->dma_buffer[channel->dma_last_pos],
                              (uint16_t)(current_pos - channel->dma_last_pos));
  }
  else
  {
    /* 环形缓冲区回绕：先取尾部，再取头部。 */
    HostComm_AppendReceivedData(source,
                              &channel->dma_buffer[channel->dma_last_pos],
                              (uint16_t)(comm_host_DMA_BUFFER_SIZE - channel->dma_last_pos));
    if (current_pos > 0U)
    {
      HostComm_AppendReceivedData(source, &channel->dma_buffer[0], current_pos);
    }
  }

  channel->dma_last_pos = (current_pos == comm_host_DMA_BUFFER_SIZE) ? 0U : current_pos;
}

static void HostComm_PrintChannel(HostComm_Channel_t *channel)
{
  uint8_t local_buffer[comm_host_PRINT_BUFFER_SIZE];
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
  if (local_length > comm_host_PRINT_BUFFER_SIZE)
  {
    local_length = comm_host_PRINT_BUFFER_SIZE;
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
    else if (HostComm_IsVisibleAscii(local_buffer[index]) != 0U)
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

HostComm_Status_t HostComm_InitChannel(HostSource_t source, UART_HandleTypeDef *huart)
{
  HostComm_Channel_t *channel;

  if ((source >= HOST_SOURCE_COUNT) || (huart == NULL))
  {
    return HOST_COMM_STATUS_INVALID_PARAM;
  }

  channel = &g_comm_host_channels[source];
  channel->uart = huart;
  HostComm_ResetChannel(channel);
  HostProtocol_RegisterSource(source, huart);
  return HostComm_StartDmaReceive(source);
}

void HostComm_Poll(void)
{
  /* 协议业务由主循环的 TIM6 调度任务执行，不在 UART 中断中运行。 */
  HostProtocol_Poll();
#if (COMM_HOST_RAW_DEBUG_ENABLE != 0U)
  HostComm_PrintChannel(&g_comm_host_channels[HOST_SOURCE_PC]);
  HostComm_PrintChannel(&g_comm_host_channels[HOST_SOURCE_JETSON]);
#endif
}

void HostComm_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  HostSource_t source;

  for (source = HOST_SOURCE_PC; source < HOST_SOURCE_COUNT; ++source)
  {
    HostComm_Channel_t *channel = &g_comm_host_channels[source];
    if ((channel->uart != NULL) && (huart == channel->uart))
    {
      HostComm_HandleDmaRxEvent(source, size);
      return;
    }
  }
}

void HostComm_OnUartError(UART_HandleTypeDef *huart)
{
  HostComm_Channel_t *pc_channel = &g_comm_host_channels[HOST_SOURCE_PC];
  HostComm_Channel_t *jetson_channel = &g_comm_host_channels[HOST_SOURCE_JETSON];

  if ((pc_channel->uart != NULL) && (huart == pc_channel->uart))
  {
    pc_channel->uart_error_code = huart->ErrorCode;
    pc_channel->error_ready = 1U;
    pc_channel->last_status = HOST_COMM_STATUS_RX_ERROR;
    (void)HostComm_StartDmaReceive(HOST_SOURCE_PC);
  }

  if ((jetson_channel->uart != NULL) && (huart == jetson_channel->uart))
  {
    jetson_channel->uart_error_code = huart->ErrorCode;
    jetson_channel->error_ready = 1U;
    jetson_channel->last_status = HOST_COMM_STATUS_RX_ERROR;
    (void)HostComm_StartDmaReceive(HOST_SOURCE_JETSON);
  }
}

HostComm_Status_t HostComm_GetStatus(HostSource_t source)
{
  if (source >= HOST_SOURCE_COUNT)
  {
    return HOST_COMM_STATUS_INVALID_PARAM;
  }

  return g_comm_host_channels[source].last_status;
}
