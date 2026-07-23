#ifndef __ADVANCE_TEST_H__
#define __ADVANCE_TEST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 执行使用 HAL_Delay 的底盘基础方向测试。
 * @note 测试期间阻塞主循环，仅用于现场调试。
 */
void AdvanceTest_BlockingMain(void);

/**
 * @brief 执行指定丝杆电机的阻塞式基础测试。
 * @param id 目标电机 ID。
 * @param is_x true 使用 X 系列命令，false 使用普通速度与位置命令。
 */
void AdvanceTest_ScrewMotor(uint8_t id, bool is_x);

#ifdef __cplusplus
}
#endif

#endif
