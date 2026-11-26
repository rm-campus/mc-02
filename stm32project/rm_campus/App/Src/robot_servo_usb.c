#include "robot_control_config.h"
#if ROBOT_SERVO_TELEMETRY_ENABLE
#include "robot_servo_usb.h"
#include "robot_control.h"
#include "servo_app.h"
#include "usb_serial.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define TELEMETRY_PERIOD_MS 20U
#define INVALID_SPEED INT32_MIN

typedef struct {
    uint32_t feedback_ms;
    uint16_t position;
    int32_t speed; /* P/s */
    bool seen;
} ServoSample;

static ServoSample samples[SERVO_COUNT];
static uint32_t session, last_ms;
static bool active;

static int32_t update_speed(ServoSample *sample, const ServoState *state)
{
    const uint32_t elapsed = state->position_ms - sample->feedback_ms;
    if (!sample->seen || elapsed != 0U) {
        sample->speed = INVALID_SPEED;
        if (sample->seen && elapsed < SERVO_POSITION_STALE_MS) {
            const int32_t delta = (int32_t)state->position - sample->position;
            sample->speed = delta * 1000 / (int32_t)elapsed;
        }
        sample->seen = true;
        sample->position = state->position;
        sample->feedback_ms = state->position_ms;
    }
    return sample->speed;
}

void RobotServoUsb_Process(void)
{
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    if (!usb.ready) { active = false; return; }

    uint32_t now = HAL_GetTick();
    if (!active || session != usb.session_id) {
        memset(samples, 0, sizeof(samples));
        active = true;
        session = usb.session_id;
        last_ms = now - TELEMETRY_PERIOD_MS;
    }
    if (now - last_ms < TELEMETRY_PERIOD_MS) return;
    last_ms = now;

    ServoSnapshot snapshot;
    Servo_CopySnapshot(&snapshot);
    now = HAL_GetTick();
    long values[1U + 3U * SERVO_COUNT] = {0};
    values[0] = 2; /* 尚未接管。其余值与默认 CH9 档位一致。 */
    switch (RobotControl_Get(now).mode) {
    case ROBOT_DRIVE: values[0] = 1; break;
    case ROBOT_ARM: values[0] = 0; break;
    case ROBOT_WRIST: values[0] = -1; break;
    default: break;
    }
    for (unsigned i = 0; i < SERVO_COUNT; ++i) {
        values[1U + SERVO_COUNT + i] = INVALID_SPEED;
    }
    for (unsigned slot = 0; slot < SERVO_COUNT; ++slot) {
        const ServoState *state = &snapshot.device[slot];
        if (state->id == 0U || state->id > SERVO_COUNT) continue;
        const unsigned i = state->id - 1U;
        ServoSample *sample = &samples[i];
        if (!state->online || !state->position_seen ||
            now - state->position_ms >= SERVO_POSITION_STALE_MS) {
            sample->seen = false;
            continue;
        }
        values[1U + i] = state->position;
        values[1U + SERVO_COUNT + i] = update_speed(sample, state);
        values[1U + 2U * SERVO_COUNT + i] = state->target;
    }
    if (usb.tx_buffered != 0U) return; /* USB 忙时丢弃这一帧，不追赶补发。 */
    char frame[192];
    int written = snprintf(frame, sizeof(frame), "servo_wave:%lu", (unsigned long)now);
    if (written < 0 || (size_t)written >= sizeof(frame)) return;
    size_t used = (size_t)written;
    for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        written = snprintf(frame + used, sizeof(frame) - used, ",%ld", values[i]);
        if (written < 0 || (size_t)written >= sizeof(frame) - used) return;
        used += (size_t)written;
    }
    frame[used++] = '\n';
    /* 用会话检查保护刚重连的主机，发送仍然是非阻塞的。 */
    UsbSerial_GetStatus(&usb);
    if (usb.ready && usb.session_id == session && usb.tx_buffered == 0U) {
        (void)UsbSerial_Send(frame, (uint16_t)used);
    }
}
#endif
