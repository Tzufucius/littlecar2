#ifndef __ADVANCE_TEST_H__
#define __ADVANCE_TEST_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

/* 执行使用 HAL_Delay 的底盘基础方向测试。 */
void AdvanceTest_BlockingMain(void);

/* 启动使用动作脚本的非阻塞底盘组合测试。 */
void AdvanceTest_NonBlockingMain(void);

/* 周期性推进非阻塞底盘组合测试。 */
void AdvanceTest_NonBlockingPoll(void);

/* 执行指定丝杆电机的基础测试。 */
void AdvanceTest_ScrewMotor(uint8_t id, bool is_x);

#ifdef __cplusplus
}
#endif

#endif
