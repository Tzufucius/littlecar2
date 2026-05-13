#include "sensor_wit.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

#define WIT_DATA_TIMEOUT_MS     ((uint32_t)500U)
#define WIT_RX_DMA_BUFFER_SIZE  ((uint16_t)128U)
#define WIT_FRAME_LENGTH        ((uint16_t)11U)
#define WIT_FRAME_HEADER        ((uint8_t)0x55U)
#define WIT_FRAME_TYPE_ACCEL    ((uint8_t)0x51U)
#define WIT_FRAME_TYPE_GYRO     ((uint8_t)0x52U)
#define WIT_FRAME_TYPE_ANGLE    ((uint8_t)0x53U)
#define WIT_PENDING_MAX_LENGTH  (WIT_FRAME_LENGTH - 1U)
#define WIT_ACCEL_RANGE_G       (16.0f)
#define WIT_GYRO_RANGE_DPS      (2000.0f)
#define WIT_ANGLE_RANGE_DEG     (180.0f)

static volatile WIT_Data_t g_wit_data = {0};

static uint8_t g_wit_rx_dma_buffer[WIT_RX_DMA_BUFFER_SIZE] = {0};
static uint16_t g_wit_rx_last_pos = 0U;
static uint8_t g_wit_pending_bytes[WIT_PENDING_MAX_LENGTH] = {0};
static uint16_t g_wit_pending_length = 0U;
static WIT_Status_t g_wit_last_status = WIT_STATUS_NOT_READY;

static UART_HandleTypeDef *WIT_GetUartHandle(void)
{
  return &huart2;
}

static int16_t WIT_GetInt16LE(const uint8_t *buffer)
{
  return (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
}

static float WIT_ScaleValue(int16_t raw, float range)
{
  return ((float)raw / 32768.0f) * range;
}

static uint8_t WIT_CalcChecksum(const uint8_t *frame)
{
  uint16_t index;
  uint8_t sum = 0U;

  for (index = 0U; index < (WIT_FRAME_LENGTH - 1U); ++index)
  {
    sum = (uint8_t)(sum + frame[index]);
  }

  return sum;
}

static void WIT_ResetPendingBytes(void)
{
  g_wit_pending_length = 0U;
}

static void WIT_MarkVectorStale(volatile WIT_Vector3f_t *vector, uint32_t now_tick)
{
  if (vector->valid == 0U)
  {
    return;
  }

  if ((now_tick - vector->updated_tick) > WIT_DATA_TIMEOUT_MS)
  {
    vector->valid = 0U;
  }
}

static void WIT_UpdateVector(volatile WIT_Vector3f_t *target, float x, float y, float z, uint32_t tick_ms)
{
  target->x = x;
  target->y = y;
  target->z = z;
  target->updated_tick = tick_ms;
  target->valid = 1U;
}

static void WIT_ReadScaledVector(const uint8_t *frame, float range, float *x, float *y, float *z)
{
  *x = WIT_ScaleValue(WIT_GetInt16LE(&frame[2]), range);
  *y = WIT_ScaleValue(WIT_GetInt16LE(&frame[4]), range);
  *z = WIT_ScaleValue(WIT_GetInt16LE(&frame[6]), range);
}

static void WIT_ParseVectorFrame(const uint8_t *frame, volatile WIT_Vector3f_t *target, float range)
{
  uint32_t now_tick = HAL_GetTick();
  float x;
  float y;
  float z;

  WIT_ReadScaledVector(frame, range, &x, &y, &z);
  WIT_UpdateVector(target, x, y, z, now_tick);
}

static void WIT_ParseFrame(const uint8_t *frame)
{
  switch (frame[1])
  {
    case WIT_FRAME_TYPE_ACCEL:
      WIT_ParseVectorFrame(frame, &g_wit_data.accel_g, WIT_ACCEL_RANGE_G);
      break;

    case WIT_FRAME_TYPE_GYRO:
      WIT_ParseVectorFrame(frame, &g_wit_data.gyro_dps, WIT_GYRO_RANGE_DPS);
      break;

    case WIT_FRAME_TYPE_ANGLE:
      WIT_ParseVectorFrame(frame, &g_wit_data.angle_deg, WIT_ANGLE_RANGE_DEG);
      break;

    default:
      break;
  }
}

static void WIT_ParseBytes(const uint8_t *data, uint16_t length)
{
  uint8_t window[WIT_PENDING_MAX_LENGTH + WIT_RX_DMA_BUFFER_SIZE];
  uint16_t window_length = 0U;
  uint16_t index = 0U;

  if (length == 0U)
  {
    return;
  }

  if (g_wit_pending_length > 0U)
  {
    memcpy(window, g_wit_pending_bytes, g_wit_pending_length);
    window_length = g_wit_pending_length;
  }

  memcpy(&window[window_length], data, length);
  window_length = (uint16_t)(window_length + length);

  while (index < window_length)
  {
    while ((index < window_length) && (window[index] != WIT_FRAME_HEADER))
    {
      ++index;
    }

    if ((uint16_t)(window_length - index) < WIT_FRAME_LENGTH)
    {
      break;
    }

    if (WIT_CalcChecksum(&window[index]) == window[index + WIT_FRAME_LENGTH - 1U])
    {
      WIT_ParseFrame(&window[index]);
      g_wit_last_status = WIT_STATUS_OK;
      index = (uint16_t)(index + WIT_FRAME_LENGTH);
    }
    else
    {
      g_wit_last_status = WIT_STATUS_FRAME_ERROR;
      ++index;
    }
  }

  g_wit_pending_length = (uint16_t)(window_length - index);
  if (g_wit_pending_length > WIT_PENDING_MAX_LENGTH)
  {
    g_wit_pending_length = WIT_PENDING_MAX_LENGTH;
    index = (uint16_t)(window_length - g_wit_pending_length);
  }

  if (g_wit_pending_length > 0U)
  {
    memcpy(g_wit_pending_bytes, &window[index], g_wit_pending_length);
  }
}

static void WIT_HandleRxEvent(uint16_t size)
{
  uint16_t current_pos = size;

  if (current_pos > WIT_RX_DMA_BUFFER_SIZE)
  {
    current_pos = WIT_RX_DMA_BUFFER_SIZE;
  }

  if (current_pos == g_wit_rx_last_pos)
  {
    return;
  }

  if (current_pos > g_wit_rx_last_pos)
  {
    WIT_ParseBytes(&g_wit_rx_dma_buffer[g_wit_rx_last_pos], current_pos - g_wit_rx_last_pos);
  }
  else
  {
    WIT_ParseBytes(&g_wit_rx_dma_buffer[g_wit_rx_last_pos], WIT_RX_DMA_BUFFER_SIZE - g_wit_rx_last_pos);
    if (current_pos > 0U)
    {
      WIT_ParseBytes(&g_wit_rx_dma_buffer[0], current_pos);
    }
  }

  g_wit_rx_last_pos = (current_pos == WIT_RX_DMA_BUFFER_SIZE) ? 0U : current_pos;
}

static WIT_Status_t WIT_StartReceive(void)
{
  if (HAL_UARTEx_ReceiveToIdle_DMA(WIT_GetUartHandle(), g_wit_rx_dma_buffer, WIT_RX_DMA_BUFFER_SIZE) != HAL_OK)
  {
    g_wit_last_status = WIT_STATUS_RX_ERROR;
    return g_wit_last_status;
  }

  if (WIT_GetUartHandle()->hdmarx != NULL)
  {
    __HAL_DMA_DISABLE_IT(WIT_GetUartHandle()->hdmarx, DMA_IT_HT);
  }

  g_wit_rx_last_pos = 0U;
  WIT_ResetPendingBytes();
  g_wit_last_status = WIT_STATUS_OK;
  return g_wit_last_status;
}

WIT_Status_t WIT_Init(void)
{
  g_wit_data = (WIT_Data_t){0};
  g_wit_last_status = WIT_STATUS_NOT_READY;
  g_wit_rx_last_pos = 0U;
  WIT_ResetPendingBytes();

  if (WIT_StartReceive() != WIT_STATUS_OK)
  {
    return g_wit_last_status;
  }

  return WIT_STATUS_OK;
}

void WIT_Poll(void)
{
  uint32_t now_tick = HAL_GetTick();

  WIT_MarkVectorStale(&g_wit_data.accel_g, now_tick);
  WIT_MarkVectorStale(&g_wit_data.gyro_dps, now_tick);
  WIT_MarkVectorStale(&g_wit_data.angle_deg, now_tick);
}

void WIT_OnUartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart == WIT_GetUartHandle())
  {
    WIT_HandleRxEvent(size);
  }
}

void WIT_OnUartError(UART_HandleTypeDef *huart)
{
  if (huart == WIT_GetUartHandle())
  {
    WIT_ResetPendingBytes();
    if (WIT_StartReceive() == WIT_STATUS_OK)
    {
      g_wit_last_status = WIT_STATUS_RX_ERROR;
    }
  }
}

const volatile WIT_Data_t *WIT_GetData(void)
{
  return &g_wit_data;
}
