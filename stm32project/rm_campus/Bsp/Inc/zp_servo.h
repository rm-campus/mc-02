#ifndef ZP_SERVO_H
#define ZP_SERVO_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define ZP_FRAME_MAX 64U
#define ZP_VERSION_MAX 24U
typedef enum { ZP_MOVE, ZP_POSITION, ZP_ID, ZP_VERSION, ZP_MODE,
               ZP_STOP, ZP_PAUSE, ZP_RESUME, ZP_TORQUE_ON, ZP_TORQUE_OFF } ZpOperation;
typedef enum { ZP_REPLY_POSITION, ZP_REPLY_ID, ZP_REPLY_VERSION, ZP_REPLY_MODE, ZP_REPLY_OK } ZpReplyType;
typedef struct {
    ZpReplyType type;
    uint8_t id;
    uint16_t value;
    char version[ZP_VERSION_MAX];
} ZpReply;
size_t Zp_Build(char *out, size_t capacity, ZpOperation op, uint8_t id, uint16_t position, uint16_t time_ms);
bool Zp_Parse(const char *frame, size_t length, ZpReply *out);
bool Zp_ExpectsReply(ZpOperation op);
bool Zp_Matches(ZpOperation op, uint8_t id, const ZpReply *reply);
#endif
