#include "fs_servo.h"
#include <string.h>

enum { FS_ANGLE = 0x0A, FS_MOVE = 0x08, FS_STOP = 0x18 };

uint16_t FsServo_PulseFromAngle(int angle_ddeg)
{
    return (uint16_t)(500 + ((angle_ddeg + 1800) * 5 + 4) / 9);
}

int FsServo_AngleFromPulse(uint16_t pulse)
{
    return (((int)pulse - 500) * 9 + 2) / 5 - 1800;
}

static uint8_t checksum(const uint8_t *data, size_t length)
{
    uint8_t sum = 0U;
    for (size_t i = 0; i < length; ++i) sum = (uint8_t)(sum + data[i]);
    return sum;
}

static size_t pack(uint8_t *out, size_t capacity, uint8_t op, const uint8_t *payload, size_t length)
{
    const size_t size = length + 5U;
    if (!out || size > capacity || payload[0] == 255U) return 0;
    out[0]=0x12; out[1]=0x4C; out[2]=op; out[3]=(uint8_t)length;
    memcpy(out + 4U, payload, length);
    out[size - 1U] = checksum(out, size - 1U);
    return size;
}

size_t FsServo_BuildQuery(uint8_t *frame, size_t capacity, uint8_t id)
{
    return pack(frame, capacity, FS_ANGLE, &id, 1U);
}

size_t FsServo_BuildMove(uint8_t *frame, size_t capacity, uint8_t id, uint16_t pulse,
                         uint16_t time_ms, uint16_t power_mw)
{
    if (pulse < 500U || pulse > 2500U) return 0;
    const uint16_t encoded = (uint16_t)FsServo_AngleFromPulse(pulse);
    const uint8_t payload[] = {id, (uint8_t)encoded, encoded >> 8, (uint8_t)time_ms, time_ms >> 8,
                               (uint8_t)power_mw, power_mw >> 8};
    return pack(frame, capacity, FS_MOVE, payload, sizeof(payload));
}

size_t FsServo_BuildHold(uint8_t *frame, size_t capacity, uint8_t id, uint16_t power_mw)
{
    const uint8_t payload[] = {id, 0x11U, (uint8_t)power_mw, power_mw >> 8};
    return pack(frame, capacity, FS_STOP, payload, sizeof(payload));
}

bool FsServo_ParsePosition(const uint8_t frame[FS_SERVO_ANGLE_REPLY_SIZE], uint8_t id, uint16_t *pulse)
{
    if (frame[0]!=0x05 || frame[1]!=0x1C || frame[2]!=FS_ANGLE || frame[3]!=3U ||
        frame[4]!=id || checksum(frame, FS_SERVO_ANGLE_REPLY_SIZE-1U)!=frame[7]) return false;
    const unsigned raw = (unsigned)frame[5] | ((unsigned)frame[6] << 8);
    const int angle = raw >= 32768U ? (int)raw - 65536 : (int)raw;
    if (angle < -1800 || angle > 1800) return false;
    *pulse = FsServo_PulseFromAngle(angle);
    return true;
}
