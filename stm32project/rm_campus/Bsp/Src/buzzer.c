#include "buzzer.h"
#include "robot_control_config.h"
#if ROBOT_CONTROL_ENABLE
#include "tim.h"

void Buzzer_Set(bool enabled)
{
    if (enabled) {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, (__HAL_TIM_GET_AUTORELOAD(&htim12) + 1U) / 2U);
        (void)HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    } else {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0U);
        (void)HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
    }
}
#endif
