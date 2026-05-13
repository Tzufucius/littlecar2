#include "comm_jetson.h"
#include "comm_protocol.h"
#include <stdio.h>
#include <string.h>

#define comm_jetson_RX_DMA_BUFFER_SIZE  ((uint16_t)256U)
#define comm_jetson_PRINT_BUFFER_SIZE   ((uint16_t)256U)

static UART_HandleTypeDef *g_comm_jetson_uart = NULL;
static uint8_t g_jetson_rx_dma_buffer[comm_jetson_RX_DMA_BUFFER_SIZE] = {0};
static uint16_t g_jetson_rx_last_pos = 0U;

static volatile uint16_t g_jetson_print_length = 0U;
static volatile uint8_t g_jetson_print_ready = 0U;
static volatile uint8_t g_jetson_error_ready = 0U;
static volatile uint32_t g_jetson_overflow_count = 0U;
static volatile uint32_t g_jetson_uart_error_code = 0U;
static uint8_t g_jetson_print_buffer[comm_jetson_PRINT_BUFFER_SIZE] = {0};

static JetsonDebug_Status_t g_jetson_last_status = comm_jetson_STATUS_NOT_READY;

static JetsonDebug_Status_t JetsonDebug_StartReceive(void)
{
  if (g_comm_jetson_uart == NULL)
  {
    g_jetson_last_status = comm_jetson_STATUS_NOT_READY;
    return g_jetson_last_status;
  }

  if (HAL_UARTEx_ReceiveToIdle_DMA(g_comm_jetson_uart,
                                   g_jetson_rx_dma_buffer,
                                   comm_jetson_RX_DMA_BUFFER_SIZE) != HAL_OK)
  {
    g_jetson_last_status = comm_jetson_STATUS_RX_ERROR;
    return g_jetson_last_status;
  }

  if (g_comm_jetson_uart->hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(g_comm_jetson_uart->hdmarx, DMA_IT_HT);
  }

  g_jetson_rx_last_pos = 0U;
  g_jetson_last_status = comm_jetson_STATUS_OK;
  return g_jetson_last_status;
}

static uint8_t JetsonDebug_IsVisibleAscii(uint8_t byte)
{
  return (byte >= 0x20U) && (byte <= 0x7EU);
}

static void JetsonDebug_AppendReceivedData(const uint8_t *data, uint16_t length)
{
  uint16_t copy_length;

  if ((data == NULL) || (length == 0U))
  {
    return;
  }

  /* 兼容旧 Jetson 调试入口，将收到的字节交给统一协议解析层。 */
  HostProtocol_OnBytes(comm_protocol_SOURCE_JETSON, data, length);

  if (g_jetson_print_length >= comm_jetson_PRINT_BUFFER_SIZE)
  {
    ++g_jetson_overflow_count;
    g_jetson_last_status = comm_jetson_STATUS_OVERFLOW;
    return;
  }

  copy_length = (uint16_t)(comm_jetson_PRINT_BUFFER_SIZE - g_jetson_print_length);
  if (copy_length > length)
  {
    copy_length = length;
  }

  memcpy(&g_jetson_print_buffer[g_jetson_print_length], data, copy_length);
  g_jetson_print_length = (uint16_t)(g_jetson_print_length + copy_length);
  g_jetson_print_ready = 1U;

  if (copy_length < length)
  {
    ++g_jetson_overflow_count;
    g_jetson_last_status = comm_jetson_STATUS_OVERFLOW;
  }
}

static void JetsonDebug_HandleRxEvent(uint16_t size)
{
  uint16_t current_pos = size;

  if (current_pos > comm_jetson_RX_DMA_BUFFER_SIZE)
  {
    current_pos = comm_jetson_RX_DMA_BUFFER_SIZE;
  }

  if (current_pos == g_jetson_rx_last_pos)
  {
    return;
  }

  if (current_pos > g_jetson_rx_last_pos)
  {
    JetsonDebug_AppendReceivedData(&g_jetson_rx_dma_buffer[g_jetson_rx_last_pos],
                                   (uint16_t)(current_pos - g_jetson_rx_last_pos));
  }
  else
  {
    JetsonDebug_AppendReceivedData(&g_jetson_rx_dma_buffer[g_jetson_rx_last_pos],
                                   (uint16_t)(comm_jetson_RX_DMA_BUFFER_SIZE - g_jetson_rx_last_pos));
    if (current_pos > 0U)
    {
      JetsonDebug_AppendReceivedData(&g_jetson_rx_dma_buffer[0], current_pos);
    }
  }

  g_jetson_rx_last_pos = (current_pos == comm_jetson_RX_DMA_BUFFER_SIZE) ? 0U : current_pos;
}

JetsonDebug_Status_t JetsonDebug_Init(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    g_jetson_last_status = comm_jetson_STATUS_INVALID_PARAM;
    return g_jetson_last_status;
  }

  g_comm_jetson_uart = huart;
  g_jetson_rx_last_pos = 0U;
  g_jetson_print_length = 0U;
  g_jetson_print_ready = 0U;
  g_jetson_error_ready = 0U;
  g_jetson_overflow_count = 0U;
  g_jetson_uart_error_code = 0U;
  memset(g_jetson_rx_dma_buffer, 0, sizeof(g_jetson_rx_dma_buffer));
  memset(g_jetson_print_buffer, 0, sizeof(g_jetson_print_buffer));
  HostProtocol_RegisterSource(comm_protocol_SOURCE_JETSON, huart);

  return JetsonDebug_StartReceive();
}

void JetsonDebug_Poll(void)
{
  uint8_t local_buffer[comm_jetson_PRINT_BUFFER_SIZE];
  uint16_t local_length;
  uint32_t overflow_count;
  uint32_t error_code;
  uint16_t index;

  if ((g_jetson_print_ready == 0U) && (g_jetson_error_ready == 0U))
  {
    HostProtocol_Poll();
    return;
  }

  HostProtocol_Poll();

  __disable_irq();
  error_code = g_jetson_uart_error_code;
  local_length = g_jetson_print_length;
  if (local_length > comm_jetson_PRINT_BUFFER_SIZE)
  {
    local_length = comm_jetson_PRINT_BUFFER_SIZE;
  }
  memcpy(local_buffer, g_jetson_print_buffer, local_length);
  overflow_count = g_jetson_overflow_count;
  g_jetson_print_length = 0U;
  g_jetson_print_ready = 0U;
  g_jetson_error_ready = 0U;
  g_jetson_overflow_count = 0U;
  g_jetson_uart_error_code = 0U;
  __enable_irq();

  if (error_code != 0U)
  {
    printf("USART6 ERR code=0x%08lX\r\n", (unsigned long)error_code);
  }

  if (local_length == 0U)
  {
    return;
  }

  printf("USART6 RX len=%u hex=", (unsigned int)local_length);
  for (index = 0U; index < local_length; ++index)
  {
    printf("%02X ", local_buffer[index]);
  }

  printf("ascii=");
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
    else if (JetsonDebug_IsVisibleAscii(local_buffer[index]) != 0U)
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

void JetsonDebug_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  if ((g_comm_jetson_uart != NULL) && (huart == g_comm_jetson_uart))
  {
    JetsonDebug_HandleRxEvent(size);
  }
}

void JetsonDebug_OnUartError(UART_HandleTypeDef *huart)
{
  if ((g_comm_jetson_uart != NULL) && (huart == g_comm_jetson_uart))
  {
    g_jetson_uart_error_code = huart->ErrorCode;
    g_jetson_error_ready = 1U;
    g_jetson_last_status = comm_jetson_STATUS_RX_ERROR;
    (void)JetsonDebug_StartReceive();
  }
}

JetsonDebug_Status_t JetsonDebug_GetLastStatus(void)
{
  return g_jetson_last_status;
}
