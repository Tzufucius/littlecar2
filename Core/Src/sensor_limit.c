#include "sensor_limit.h"

/** @brief 限位传感器 GPIO 映射项。 */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} SensorLimitPinMap_t;

static const SensorLimitPinMap_t g_sensor_limit_pin_map[SENSOR_LIMIT_COUNT] = {
    [SENSOR_LIMIT_LIFT_UP] = {LIFT_UP_LIMIT_GPIO_Port, LIFT_UP_LIMIT_Pin},
    [SENSOR_LIMIT_LIFT_DOWN] = {LIFT_DOWN_LIMIT_GPIO_Port, LIFT_DOWN_LIMIT_Pin},
    [SENSOR_LIMIT_SLIDE_FRONT] = {SLIDE_FRONT_LIMIT_GPIO_Port, SLIDE_FRONT_LIMIT_Pin},
    [SENSOR_LIMIT_SLIDE_REAR] = {SLIDE_REAR_LIMIT_GPIO_Port, SLIDE_REAR_LIMIT_Pin}};

GPIO_PinState SensorLimit_ReadLevel(SensorLimitId_t id)
{
  if ((id < SENSOR_LIMIT_LIFT_UP) || (id >= SENSOR_LIMIT_COUNT))
  {
    return SENSOR_LIMIT_ACTIVE_LEVEL;
  }

  return HAL_GPIO_ReadPin(g_sensor_limit_pin_map[id].port,
                          g_sensor_limit_pin_map[id].pin);
}

bool SensorLimit_IsActive(SensorLimitId_t id)
{
  return SensorLimit_ReadLevel(id) == SENSOR_LIMIT_ACTIVE_LEVEL;
}
