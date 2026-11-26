#ifndef SERVO_UART_H
#define SERVO_UART_H
#include <stdbool.h>
#include <stdint.h>
#include "servo_config.h"
#if SERVO_RX_TRACE_ENABLE
#include "zp_servo.h"
enum { SERVO_TRACE_BYTES=48, SERVO_TRACE_BLOCKS=64, SERVO_TRACE_IDLE_MS=2 };
typedef struct {
    uint32_t offset, first_ms, last_ms, tx_done_ms;
    uint8_t length, bytes[SERVO_TRACE_BYTES];
    char last_tx[ZP_FRAME_MAX+1U]; /* 最后实际发送完成的命令，仅作为时序参照。 */
} ServoRxTraceBlock;
/* USB 任务独占读取；协议接收环及其恢复不影响此副本。 */
bool ServoUart_TakeTrace(ServoRxTraceBlock *out);
#endif
enum { SERVO_UART_TX_DONE=1U, SERVO_UART_ERROR=2U };
typedef struct {
    uint32_t rx_bytes, tx_frames, errors, overflows, last_rx_ms, tx_done_ms;
    bool ready, transmitting;
} ServoUartStatus;
/* Task owner: ServoBusTask. Callbacks only move bytes/change direction/notify. */
bool ServoUart_Init(void);
bool ServoUart_Recover(void);
/* 按长度原样发送字节，最后一个停止位发完后切回接收。 */
bool ServoUart_Send(const char *data, uint16_t length);
uint16_t ServoUart_Read(uint8_t *out, uint16_t capacity);
uint32_t ServoUart_TakeEvents(void);
void ServoUart_CopyStatus(ServoUartStatus *out);
#endif
