#ifndef IMU_CLOCK_H
#define IMU_CLOCK_H
#include <stdbool.h>
#include <stdint.h>
/* 使用 M7 DWT 计数器测实际采集间隔，不占用新的 TIM。只初始化一次。 */
bool ImuClock_Init(void);
uint32_t ImuClock_Cycles(void);
float ImuClock_Elapsed(uint32_t previous, uint32_t current);
#endif
