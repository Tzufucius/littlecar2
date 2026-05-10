#include "host_rx.h"
#include <stdio.h>
#include <string.h>

#define HOST_RX_DMA_BUFFER_SIZE    ((uint16_t)256U)
#define HOST_RX_PRINT_BUFFER_SIZE  ((uint16_t)256U)

typedef struct
{
  const char *name;
  UART_HandleTypeDef *uart;
  uint8_t dma_buffer[HOST_RX_DMA_BUFFER_SIZE];
  uint16_t dma_last_pos;
  uint8_t pc_rx_byte;
  uint8_t print_buffer[HOST_RX_PRINT_BUFFER_SIZE];
  volatile uint16_t print_length;
  volatile uint8_t print_ready;
  volatile uint8_t error_ready;
  volatile uint32_t overflow_count;
  volatile uint32_t uart_error_code;
  HostRx_Status_t last_status;
} HostRx_Channel_t;

static HostRx_Channel_t g_host_rx_channels[HOST_RX_SOURCE_COUNT] = {
  [HOST_RX_SOURCE_PC] = {
    .name = "PC",
    .uart = NULL,
    .dma_last_pos = 0U,
    .pc_rx_byte = 0U,
    .print_length = 0U,
    .print_ready = 0U,
    .error_ready = 0U,
    .overflow_count = 0U,
    .uart_error_code = 0U,
    .last_status = HOST_RX_STATUS_NOT_READY
  },
  [HOST_RX_SOURCE_JETSON] = {
    .name = "JETSON",
    .uart = NULL,
    .dma_last_pos = 0U,
    .pc_rx_byte = 0U,
    .print_length = 0U,
    .print_ready = 0U,
    .error_ready = 0U,
    .overflow_count = 0U,
    .uart_error_code = 0U,
    .last_status = HOST_RX_STATUS_NOT_READY
  }
};

static HostRx_Status_t HostRx_StartPcReceive(void)
{
  HostRx_Channel_t *channel = &g_host_rx_channels[HOST_RX_SOURCE_PC];

  if (channel->uart == NULL)
  {
    channel->last_status = HOST_RX_STATUS_NOT_READY;
    return channel->last_status;
  }

  if (HAL_UART_Receive_IT(channel->uart, &channel->pc_rx_byte, 1U) != HAL_OK)
  {
    channel->last_status = HOST_RX_STATUS_RX_ERROR;
    return channel->last_status;
  }

  channel->last_status = HOST_RX_STATUS_OK;
  return channel->last_status;
}

static HostRx_Status_t HostRx_StartJetsonReceive(void)
{
  HostRx_Channel_t *channel = &g_host_rx_channels[HOST_RX_SOURCE_JETSON];

  if (channel->uart == NULL)
  {
    channel->last_status = HOST_RX_STATUS_NOT_READY;
    return channel->last_status;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(channel->uart,
                                   channel->dma_buffer,
                                   HOST_RX_DMA_BUFFER_SIZE) != HAL_OK)
  {
    channel->last_status = HOST_RX_STATUS_RX_ERROR;
    return channel->last_status;
  }

  if (channel->uart->hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(channel->uart->hdmarx, DMA_IT_HT);
  }

  channel->dma_last_pos = 0U;
  channel->last_status = HOST_RX_STATUS_OK;
  return channel->last_status;
}

static uint8_t HostRx_IsVisibleAscii(uint8_t byte)
{
  return (byte >= 0x20U) && (byte <= 0x7EU);
}

static void HostRx_ResetChannel(HostRx_Channel_t *channel)
{
  channel->dma_last_pos = 0U;
  channel->pc_rx_byte = 0U;
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

  if ((source >= HOST_RX_SOURCE_COUNT) || (data == NULL) || (length == 0U))
  {
    return;
  }

  channel = &g_host_rx_channels[source];

  if (channel->print_length >= HOST_RX_PRINT_BUFFER_SIZE)
  {
    ++channel->overflow_count;
    channel->last_status = HOST_RX_STATUS_OVERFLOW;
    return;
  }

  copy_length = (uint16_t)(HOST_RX_PRINT_BUFFER_SIZE - channel->print_length);
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
    channel->last_status = HOST_RX_STATUS_OVERFLOW;
  }
}

static void HostRx_HandleJetsonRxEvent(uint16_t size)
{
  HostRx_Channel_t *channel = &g_host_rx_channels[HOST_RX_SOURCE_JETSON];
  uint16_t current_pos = size;

  if (current_pos > HOST_RX_DMA_BUFFER_SIZE)
  {
    current_pos = HOST_RX_DMA_BUFFER_SIZE;
  }

  if (current_pos == channel->dma_last_pos)
  {
    return;
  }

  if (current_pos > channel->dma_last_pos)
  {
    HostRx_AppendReceivedData(HOST_RX_SOURCE_JETSON,
                              &channel->dma_buffer[channel->dma_last_pos],
                              (uint16_t)(current_pos - channel->dma_last_pos));
  }
  else
  {
    HostRx_AppendReceivedData(HOST_RX_SOURCE_JETSON,
                              &channel->dma_buffer[channel->dma_last_pos],
                              (uint16_t)(HOST_RX_DMA_BUFFER_SIZE - channel->dma_last_pos));
    if (current_pos > 0U)
    {
      HostRx_AppendReceivedData(HOST_RX_SOURCE_JETSON, &channel->dma_buffer[0], current_pos);
    }
  }

  channel->dma_last_pos = (current_pos == HOST_RX_DMA_BUFFER_SIZE) ? 0U : current_pos;
}

static void HostRx_PrintChannel(HostRx_Channel_t *channel)
{
  uint8_t local_buffer[HOST_RX_PRINT_BUFFER_SIZE];
  uint16_t local_length;
  uint32_t overflow_count;
  uint32_t error_code;
  uint16_t index;

  if ((channel->print_ready == 0U) && (channel->error_ready == 0U))
  {
    return;
  }

  __disable_irq();
  error_code = channel->uart_error_code;
  local_length = channel->print_length;
  if (local_length > HOST_RX_PRINT_BUFFER_SIZE)
  {
    local_length = HOST_RX_PRINT_BUFFER_SIZE;
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
  HostRx_Channel_t *channel = &g_host_rx_channels[HOST_RX_SOURCE_PC];

  if (huart == NULL)
  {
    channel->last_status = HOST_RX_STATUS_INVALID_PARAM;
    return channel->last_status;
  }

  channel->uart = huart;
  HostRx_ResetChannel(channel);
  return HostRx_StartPcReceive();
}

HostRx_Status_t HostRx_InitJetson(UART_HandleTypeDef *huart)
{
  HostRx_Channel_t *channel = &g_host_rx_channels[HOST_RX_SOURCE_JETSON];

  if (huart == NULL)
  {
    channel->last_status = HOST_RX_STATUS_INVALID_PARAM;
    return channel->last_status;
  }

  channel->uart = huart;
  HostRx_ResetChannel(channel);
  return HostRx_StartJetsonReceive();
}

void HostRx_Poll(void)
{
  HostRx_PrintChannel(&g_host_rx_channels[HOST_RX_SOURCE_PC]);
  HostRx_PrintChannel(&g_host_rx_channels[HOST_RX_SOURCE_JETSON]);
}

void HostRx_OnPcByteReceived(UART_HandleTypeDef *huart)
{
  HostRx_Channel_t *channel = &g_host_rx_channels[HOST_RX_SOURCE_PC];

  if ((channel->uart != NULL) && (huart == channel->uart))
  {
    HostRx_AppendReceivedData(HOST_RX_SOURCE_PC, &channel->pc_rx_byte, 1U);
    (void)HostRx_StartPcReceive();
  }
}

void HostRx_OnJetsonUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  HostRx_Channel_t *channel = &g_host_rx_channels[HOST_RX_SOURCE_JETSON];

  if ((channel->uart != NULL) && (huart == channel->uart))
  {
    HostRx_HandleJetsonRxEvent(size);
  }
}

void HostRx_OnUartError(UART_HandleTypeDef *huart)
{
  HostRx_Channel_t *pc_channel = &g_host_rx_channels[HOST_RX_SOURCE_PC];
  HostRx_Channel_t *jetson_channel = &g_host_rx_channels[HOST_RX_SOURCE_JETSON];

  if ((pc_channel->uart != NULL) && (huart == pc_channel->uart))
  {
    pc_channel->uart_error_code = huart->ErrorCode;
    pc_channel->error_ready = 1U;
    pc_channel->last_status = HOST_RX_STATUS_RX_ERROR;
    (void)HostRx_StartPcReceive();
  }

  if ((jetson_channel->uart != NULL) && (huart == jetson_channel->uart))
  {
    jetson_channel->uart_error_code = huart->ErrorCode;
    jetson_channel->error_ready = 1U;
    jetson_channel->last_status = HOST_RX_STATUS_RX_ERROR;
    (void)HostRx_StartJetsonReceive();
  }
}

HostRx_Status_t HostRx_GetLastStatus(HostRx_Source_t source)
{
  if (source >= HOST_RX_SOURCE_COUNT)
  {
    return HOST_RX_STATUS_INVALID_PARAM;
  }

  return g_host_rx_channels[source].last_status;
}
