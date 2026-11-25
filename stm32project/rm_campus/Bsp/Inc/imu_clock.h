#ifndef IMU_CLOCK_H
#define IMU_CLOCK_H
#include <stdbool.h>
#include <stdint.h>
/* 使用 M7 DWT 计数器测实际采集间隔，不占用新的 TIM。只初始化一次。 */
bool ImuClock_Init(void);
uint32_t ImuClock_Cycles(void);
/* previous/current 是先后读取的原始周期数，返回经过的秒数。间隔须小于计数器
 * 的完整回绕周期；在此范围内，即使 current 数值小于 previous 也能正确计算。 */
float ImuClock_Elapsed(uint32_t previous, uint32_t current);
#endif
