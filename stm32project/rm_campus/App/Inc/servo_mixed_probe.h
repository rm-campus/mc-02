#ifndef SERVO_MIXED_PROBE_H
#define SERVO_MIXED_PROBE_H

#include <stddef.h>
#include <stdint.h>

/* 仅由独占 UART7 的 USB 维护任务调用。生成一次只读混接探测报告。 */
void ServoMixedProbe_Run(uint8_t fashionstar_id, uint32_t session, char *reply, size_t capacity);

#endif
