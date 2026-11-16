#ifndef SERVO_UART_H
#define SERVO_UART_H
#include <stdbool.h>
#include <stdint.h>
enum { SERVO_UART_TX_DONE=1U, SERVO_UART_ERROR=2U };
typedef struct {
    uint32_t rx_bytes, tx_frames, errors, overflows, last_rx_ms, tx_done_ms;
    bool ready, transmitting;
} ServoUartStatus;
/* Task owner: ServoBusTask. Callbacks only move bytes/change direction/notify. */
bool ServoUart_Init(void);
bool ServoUart_Recover(void);
bool ServoUart_Send(const char *data, uint16_t length);
uint16_t ServoUart_Read(uint8_t *out, uint16_t capacity);
uint32_t ServoUart_TakeEvents(void);
void ServoUart_CopyStatus(ServoUartStatus *out);
#endif
