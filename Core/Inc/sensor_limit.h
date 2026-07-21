#ifndef SENSOR_LIMIT_H
#define SENSOR_LIMIT_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#include "main.h"

/** @brief 光电限位传感器标识。 */
typedef enum
{
  SENSOR_LIMIT_LIFT_UP = 0,
  SENSOR_LIMIT_LIFT_DOWN,
  SENSOR_LIMIT_SLIDE_FRONT,
  SENSOR_LIMIT_SLIDE_REAR,
  SENSOR_LIMIT_COUNT
} SensorLimitId_t;

/** @brief 限位传感器的有效电平，可按实际接线在编译期覆盖。 */
#ifndef SENSOR_LIMIT_ACTIVE_LEVEL
#define SENSOR_LIMIT_ACTIVE_LEVEL GPIO_PIN_SET
#endif

/**
 * @brief 读取指定限位传感器的原始 GPIO 电平。
 * @param id 限位传感器标识。
 * @return GPIO_PIN_SET 或 GPIO_PIN_RESET；非法标识按故障安全原则返回有效电平。
 */
GPIO_PinState SensorLimit_ReadLevel(SensorLimitId_t id);

/**
 * @brief 判断指定限位传感器是否处于有效状态。
 * @param id 限位传感器标识。
 * @return true 表示已触发限位，false 表示未触发。
 */
bool SensorLimit_IsActive(SensorLimitId_t id);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_LIMIT_H */
