#include "tim.h"

TIM_HandleTypeDef htim12;

void MX_TIM12_Init(void)
{
    /* APB1 定时器时钟 240 MHz，经 240 分频后每 250 计数一个周期，即 4 kHz。 */
    TIM_OC_InitTypeDef channel = {0};
    htim12.Instance = TIM12;
    htim12.Init.Prescaler = 239;
    htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim12.Init.Period = 249;
    htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) Error_Handler();
    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim12, &channel, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM12) {
        __HAL_RCC_TIM12_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_InitTypeDef pin = {0};
        pin.Pin = GPIO_PIN_15;
        pin.Mode = GPIO_MODE_AF_PP;
        pin.Pull = GPIO_NOPULL;
        pin.Speed = GPIO_SPEED_FREQ_LOW;
        pin.Alternate = GPIO_AF2_TIM12;
        HAL_GPIO_Init(GPIOB, &pin);
    }
}
