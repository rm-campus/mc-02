#include "imu_clock.h"
#include "main.h"

bool ImuClock_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;
    if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) != 0U || SystemCoreClock == 0U) return false;
    /* 不清零 CYCCNT，避免破坏今后其它模块共享计数器的时间基准。 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
    uint32_t before = DWT->CYCCNT;
    for (volatile uint32_t i = 0U; i < 32U; ++i) __NOP();
    return DWT->CYCCNT != before;
}

uint32_t ImuClock_Cycles(void) { return DWT->CYCCNT; }

float ImuClock_Elapsed(uint32_t previous, uint32_t current)
{
    /* 480 MHz 下约 8.95 秒回绕；任务正常间隔约 1 ms，另以 HAL tick 检查长时间停顿。 */
    return (float)(uint32_t)(current - previous) / (float)SystemCoreClock;
}
