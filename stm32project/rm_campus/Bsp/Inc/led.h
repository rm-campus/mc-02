/**
 * @file led.h
 * @brief 板载 WS2812 的颜色接口，参数为 R/G/B，而发送顺序为 G/R/B。
 * 每个颜色分量为 0～255；返回 true 表示 HAL 完成发送，不能证明灯珠实际显示成功。
 * 内部使用静态缓冲区，因此仅由 StatusTask 串行调用，不支持多个任务并发访问。
 */
#ifndef LED_H
#define LED_H
#include <stdbool.h>
#include <stdint.h>

/* WS2812：SPI6 MOSI / PA7，6 MHz；仅由状态任务调用。 */
/* 初始化通过发送全黑帧完成，不重新配置 CubeMX 已初始化的 SPI6。 */
bool Led_Init(void);
/* 阻塞发送一帧，HAL 超时设置为 5 ms；应从任务调用，避免在中断中等待。 */
bool Led_SetRGB(uint8_t red, uint8_t green, uint8_t blue);
/* 发送 RGB 全零使灯熄灭；不关闭 SPI 外设或灯珠电源。 */
bool Led_Off(void);
#endif
