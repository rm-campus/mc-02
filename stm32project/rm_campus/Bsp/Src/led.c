/**
 * @file led.c
 * @brief 用 SPI6 的 MOSI 波形模拟 WS2812 单线时序。
 * SPI 时钟为 6 MHz，编码字节的每一位约 0.167 us；每个 WS2812 位占用 8 个 SPI 位。
 * 24 个编码字节携带 24 位颜色，64 个全零字节产生约 85.3 us 的帧尾低电平。
 * 总计 88 个 SPI 字节，理想线速耗时约 117.3 us，实际还受中断和调度影响。
 */
#include "led.h"
#include "spi.h"

/* 沿用参考代码的 0x60/0x78 编码；加长帧尾低电平至约 85 us。 */
#define LED_COLOR_BYTES 24U
#define LED_RESET_BYTES 64U
/* 静态初始化全零；只改写前 24 字节，末尾复位段始终保持零。 */
static uint8_t s_tx[LED_COLOR_BYTES + LED_RESET_BYTES];

/**
 * @brief 把一个 8 位颜色分量展开为 8 个 SPI 字节。
 * @param out 至少 8 字节的输出空间；本函数不会修改帧尾复位段。
 */
static void encode(uint8_t value, uint8_t *out)
{
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        /* 从最高位向最低位编码：0x60 提供约 0.333 us 高电平，
         * 0x78 提供约 0.667 us 高电平；其余位形成低电平间隔。 */
        out[bit] = (value & (0x80U >> bit)) ? 0x78U : 0x60U;
    }
}

/**
 * @brief 将调用者的 RGB 参数重排为 WS2812 所需的 GRB 顺序并发送。
 * 发送使用 HAL，统一处理 SPI 的启动和结束标志，避免应用无限等待寄存器。
 * @return HAL_OK 时为 true；超时或 SPI 忙等情况为 false。
 * @note 不能与其他任务或中断并发操作同一个 hspi6/发送缓冲区。
 */
bool Led_SetRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    encode(green, &s_tx[0]);
    encode(red, &s_tx[8]);
    encode(blue, &s_tx[16]);
    /* 静态数组的复位段保持全零。HAL 发送带超时，避免无限等待。 */
    return HAL_SPI_Transmit(&hspi6, s_tx, sizeof(s_tx), 5U) == HAL_OK;
}

bool Led_Off(void) { return Led_SetRGB(0U, 0U, 0U); }
bool Led_Init(void) { return Led_Off(); }
