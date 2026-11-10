from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
out = root / 'tests' / 'host-build'
out.mkdir(parents=True, exist_ok=True)
(out / 'usart.h').write_text(r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { uint32_t CCR; } SCB_Type;
extern UART_HandleTypeDef huart5;
extern SCB_Type *SCB;
#define SCB_CCR_DC_Msk 0x10000U
#define HAL_OK 0
#define __DMB() ((void)0)
int HAL_UART_AbortReceive(UART_HandleTypeDef *h);
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n);
uint32_t HAL_GetTick(void);
void SCB_CleanInvalidateDCache_by_Addr(uint32_t *p, int32_t n);
void SCB_InvalidateDCache_by_Addr(uint32_t *p, int32_t n);
''', encoding='utf-8')
(out / 'cmsis_os2.h').write_text(r'''
#pragma once
#include <stdint.h>
typedef void *osThreadId_t;
osThreadId_t osThreadGetId(void);
uint32_t osThreadFlagsSet(osThreadId_t id, uint32_t flags);
''', encoding='utf-8')
(out / 'FreeRTOS.h').write_text('#pragma once\n', encoding='utf-8')
(out / 'task.h').write_text('#pragma once\n#define taskENTER_CRITICAL() ((void)0)\n#define taskEXIT_CRITICAL() ((void)0)\n', encoding='utf-8')
(out / 'can.h').write_text(r'''
#pragma once
#include <stdint.h>
#include <stdbool.h>
bool Can_Send(uint32_t id, const uint8_t *data, uint8_t size);
uint32_t HAL_GetTick(void);
''', encoding='utf-8')
(out / 'test.c').write_text(r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "cmsis_os2.h"
#include "fxi6x.h"
#include "motor_3508.h"

UART_HandleTypeDef huart5;
static SCB_Type scb;
SCB_Type *SCB = &scb;
static uint8_t *dma;
static unsigned dma_size, dma_pos, starts;
static uint32_t tick = 10;
static uint8_t sent[8];
static uint32_t sent_id;
int HAL_UART_AbortReceive(UART_HandleTypeDef *h) { (void)h; return HAL_OK; }
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *p, uint16_t n)
{ (void)h; dma = p; dma_size = n; dma_pos = 0; ++starts; return HAL_OK; }
uint32_t HAL_GetTick(void) { return tick; }
void SCB_CleanInvalidateDCache_by_Addr(uint32_t *p, int32_t n) { (void)p; (void)n; }
void SCB_InvalidateDCache_by_Addr(uint32_t *p, int32_t n) { (void)p; (void)n; }
osThreadId_t osThreadGetId(void) { return (void *)1; }
uint32_t osThreadFlagsSet(osThreadId_t id, uint32_t flags) { (void)id; return flags; }
bool Can_Send(uint32_t id, const uint8_t *p, uint8_t n)
{ assert(n == 8); sent_id = id; memcpy(sent, p, n); return true; }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);

static void feed(const uint8_t *p, unsigned n)
{
    for (unsigned i = 0; i < n; ++i)
    {
        dma[dma_pos++] = p[i];
        if (dma_pos == dma_size / 2) HAL_UARTEx_RxEventCallback(&huart5, dma_pos);
        if (dma_pos == dma_size)
        {
            HAL_UARTEx_RxEventCallback(&huart5, dma_size);
            dma_pos = 0;
        }
    }
    HAL_UARTEx_RxEventCallback(&huart5, dma_pos ? dma_pos : dma_size);
}

static void pack_frame(uint8_t f[25], const unsigned ch[16])
{
    memset(f, 0, 25); f[0] = 0x0F;
    for (unsigned i = 0; i < 16; ++i)
        for (unsigned b = 0; b < 11; ++b)
            if (ch[i] & (1U << b)) f[1 + (i * 11 + b) / 8] |= 1U << ((i * 11 + b) % 8);
}

static void make_frame(uint8_t f[25])
{
    const unsigned ch[16] = {1807, 240, 1124, 1024, 1024, 1024, 200, 1800, 1024, 200,
                            1024, 1024, 1024, 1024, 1024, 1024};
    pack_frame(f, ch);
}

static void test_document_values(void)
{
    /* 手工期望值覆盖端点、中心、正负取整；-98 映射为 -82.5，按文档取 -82。 */
    const int16_t input[] = {-784, -588, -98, -1, 0, 1, 391, 783};
    const int16_t expected[] = {-660, -495, -82, -1, 0, 1, 330, 660};
    const int16_t switch_input[] = {-784, -1, 0, 1, 783};
    const int8_t switch_expected[] = {1, 1, 0, -1, -1};
    unsigned ch[16]; uint8_t f[25]; Fxi6x_Data rc;
    assert(FXI6X_HIGH == 1 && FXI6X_MIDDLE == 0 && FXI6X_LOW == -1);
    for (unsigned n = 0; n < 8; ++n)
    {
        for (unsigned i = 0; i < 16; ++i) ch[i] = 1024;
        for (unsigned i = 0; i < 6; ++i) ch[i] = (unsigned)(1024 + input[(n + i) % 8]);
        for (unsigned i = 0; i < 4; ++i) ch[6 + i] = (unsigned)(1024 + switch_input[(n + i) % 5]);
        pack_frame(f, ch); ++tick; feed(f, 25);
        assert(Fxi6x_Update()); Fxi6x_CopyData(&rc);
        assert(rc.valid && rc.last_valid_ms == tick);
        /* 按文档 CH1～CH10 顺序检查字段，轮换输入以覆盖每个通道与开关。 */
        const int16_t actual[] = {rc.rocker_r_h, rc.rocker_r_v, rc.rocker_l_v,
                                  rc.rocker_l_h, rc.knob_l, rc.knob_r};
        const int8_t switches[] = {rc.switch_a, rc.switch_b, rc.switch_c, rc.switch_d};
        for (unsigned i = 0; i < 6; ++i) assert(actual[i] == expected[(n + i) % 8]);
        for (unsigned i = 0; i < 4; ++i) assert(switches[i] == switch_expected[(n + i) % 5]);
    }
    /* 保留原有越界拒收策略：任一模拟通道超出参考端点，整帧无效且时间戳不刷新。 */
    const uint32_t last_valid = rc.last_valid_ms;
    for (unsigned i = 0; i < 6; ++i)
    {
        for (unsigned end = 0; end < 2; ++end)
        {
            for (unsigned j = 0; j < 16; ++j) ch[j] = 1024;
            ch[i] = end == 0 ? 239 : 1808;
            pack_frame(f, ch); ++tick; feed(f, 25);
            assert(!Fxi6x_Update()); Fxi6x_CopyData(&rc);
            assert(!rc.valid && rc.last_valid_ms == last_valid);
            assert(rc.rocker_r_h == 0 && rc.rocker_r_v == 0 && rc.rocker_l_v == 0 &&
                   rc.rocker_l_h == 0 && rc.knob_l == 0 && rc.knob_r == 0);
        }
    }
    make_frame(f); feed(f, 25); assert(Fxi6x_Update());
}

int main(void)
{
    uint8_t f[25]; Fxi6x_Data rc;
    assert(Fxi6x_Init());
    make_frame(f);
    feed(f, 7); assert(!Fxi6x_Update());
    feed(f + 7, 18); assert(Fxi6x_Update()); Fxi6x_CopyData(&rc);
    assert(rc.valid && rc.received && rc.last_valid_ms == tick);
    assert(rc.rocker_r_h == 660);
    assert(rc.rocker_r_v == -660);
    assert(rc.switch_a == FXI6X_HIGH && rc.switch_b == FXI6X_LOW && rc.switch_c == FXI6X_MIDDLE);
    for (unsigned n = 0; n < 30; ++n) { ++tick; feed(f, 25); assert(Fxi6x_Update()); }
    Fxi6x_CopyData(&rc); assert(rc.frame_count == 31); /* DMA 半满/全满跨界与重复事件。 */
    f[23] = 8; feed(f, 25); assert(!Fxi6x_Update()); Fxi6x_CopyData(&rc);
    assert(rc.failsafe && !rc.valid && rc.rocker_r_h == 0);
    f[23] = 4; feed(f, 25); Fxi6x_Update(); Fxi6x_CopyData(&rc); assert(rc.frame_lost && !rc.valid);
    make_frame(f); uint8_t bad[25]; memcpy(bad, f, 25); bad[24] = 0xFF;
    feed(bad, 25); feed(f, 25); assert(Fxi6x_Update());
    uint8_t noise[600]; memset(noise, 0x55, sizeof(noise)); feed(noise, sizeof(noise));
    Fxi6x_Update(); Fxi6x_CopyData(&rc); assert(!rc.valid && rc.overflow_count > 0);
    feed(f, 25); assert(Fxi6x_Update());
    HAL_UART_ErrorCallback(&huart5); Fxi6x_Update(); Fxi6x_CopyData(&rc);
    assert(starts == 2 && rc.uart_error_count == 1 && !rc.valid);
    feed(f, 25); assert(Fxi6x_Update());
    test_document_values();

    Motor3508 motor; Motor3508_Init(&motor, 1U);
    Motor3508_SetCurrent(&motor, 0, 20000);
    Motor3508_SetCurrent(&motor, 1, -20000);
    assert(motor.command[0] == -16384 && motor.command[1] == -16384);
    assert(Motor3508_Send(&motor) && sent_id == 0x200);
    assert(sent[0] == 0xC0 && sent[1] == 0 && sent[2] == 0xC0 && sent[3] == 0);
    uint8_t feedback[8] = {0x1F,0xF0,0,60,0x20,0,35,0};
    Motor3508_Update(&motor, 0x201, feedback, 8);
    assert(motor.received_mask == 1 && motor.temperature[0] == 35 && motor.current[0] == -10.0f);
    assert(motor.velocity[0] < 0);
    feedback[0] = 0; feedback[1] = 16;
    Motor3508_Update(&motor, 0x201, feedback, 8); assert(motor.round[0] == 1);
    feedback[0] = 0x1F; feedback[1] = 0xF0;
    Motor3508_Update(&motor, 0x201, feedback, 8); assert(motor.round[0] == 0);
    uint32_t old_tick = motor.last_feedback_ms[0]; ++tick;
    Motor3508_Update(&motor, 0x205, feedback, 8); assert(motor.last_feedback_ms[0] == old_tick);
    feedback[0] = 0x20; feedback[1] = 0;
    Motor3508_Update(&motor, 0x201, feedback, 8); assert(motor.last_feedback_ms[0] == old_tick);
    puts("PASS: SBUS document mapping/rounding, all 10 channels, signed switches, range checks, fragments, DMA wrap, flags, resync, overflow/recovery; M3508 limits, direction, packing, feedback/wrap.");
    return 0;
}
''', encoding='utf-8')
compiler = Path(r'E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe')
cmd = [str(compiler), '-std=c11', '-Wall', '-Wextra', '-Werror', '-I' + str(out),
       '-I' + str(root / 'Bsp' / 'Inc'), str(out / 'test.c'),
       str(root / 'Bsp' / 'Src' / 'fxi6x.c'), str(root / 'Bsp' / 'Src' / 'motor_3508.c'),
       '-lm', '-o', str(out / 'driver_tests.exe')]
subprocess.run(cmd, check=True, timeout=30)
subprocess.run([str(out / 'driver_tests.exe')], check=True, timeout=10)
