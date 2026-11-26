#ifndef FS_SERVO_H
#define FS_SERVO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { FS_SERVO_ANGLE_REPLY_SIZE = 8 };

/* 单圈 -180°～180° 映射到与现有示波器兼容的 P500～P2500。 */
uint16_t FsServo_PulseFromAngle(int angle_ddeg);
int FsServo_AngleFromPulse(uint16_t pulse);
size_t FsServo_BuildQuery(uint8_t *frame, size_t capacity, uint8_t id);
size_t FsServo_BuildMove(uint8_t *frame, size_t capacity, uint8_t id, uint16_t pulse,
                         uint16_t time_ms, uint16_t power_mw);
size_t FsServo_BuildHold(uint8_t *frame, size_t capacity, uint8_t id, uint16_t power_mw);
bool FsServo_ParsePosition(const uint8_t frame[FS_SERVO_ANGLE_REPLY_SIZE], uint8_t id, uint16_t *pulse);

#endif
