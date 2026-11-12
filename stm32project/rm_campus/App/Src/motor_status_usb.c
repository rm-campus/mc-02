#include "motor_status_usb.h"
#include "motor_debug_config.h"
#include "campus_app.h"
#include "flash_tasks.h"
#include "usb_serial.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <string.h>

static bool s_pending, s_owned, s_sent;
static uint32_t s_session;
static UsbSerialTxTicket s_ticket;
static CampusMotorStatusSnapshot s_snapshot;
static char s_report[2048];
static unsigned s_length, s_offset, s_chunk;
static bool s_overflow;

static void append(const char *format, ...)
{
    if (s_overflow) return;
    va_list args;
    va_start(args, format);
    int n = vsnprintf(s_report + s_length, sizeof(s_report) - s_length, format, args);
    va_end(args);
    if (n < 0 || (unsigned)n >= sizeof(s_report) - s_length) s_overflow = true;
    else s_length += (unsigned)n;
}

static void decimal(float value)
{
    if (!isfinite(value) || fabsf(value) >= 1000000.0f) { append("NA"); return; }
    int v = (int)lroundf(value * 1000.0f);
    unsigned mag = (unsigned)(v < 0 ? -v : v);
    append("%s%u.%03u", v < 0 ? "-" : "", mag / 1000U, mag % 1000U);
}

static void field(const char *name, bool valid, uint32_t value)
{
    append(" %s=", name);
    if (valid) append("%lu", (unsigned long)value);
    else append("NA");
}

static void format_report(void)
{
    static const char *states[] = {"DISABLED", "WAIT_OFF", "WAIT_CENTER", "RUNNING"};
    static const char *reasons[] = {"OK", "BAD_CONFIG", "RC_OFFLINE", "MOTOR_OFFLINE", "CAN_FAULT",
        "TIMING_FAULT", "TEMPERATURE", "SPEED_FAULT"};
    static const char *errors[] = {"NONE", "STUFF", "FORM", "ACK", "BIT1", "BIT0", "CRC"};
    static const char *recovery_states[] = {"READY", "REQUESTED", "WAIT_BUS", "WAIT_FEEDBACK", "FAILED"};
    const CampusMotorStatusSnapshot *s = &s_snapshot;
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_SINGLE
    unsigned id = s->debug.motor_id;
    bool id_valid = id >= 1U && id <= 4U;
    unsigned slot = id_valid ? id - 1U : 0U;
    bool seen = id_valid && (s->motors.received_mask & (1U << slot)) != 0U;
    uint32_t age = s->now_ms - s->motors.last_feedback_ms[slot];
    bool online = seen && age < MOTOR_DEBUG_FEEDBACK_TIMEOUT_MS;
#endif
    bool rc_online = s->rc.received && s->rc.valid && (uint32_t)(s->now_ms - s->rc.last_valid_ms) < 100U;
    s_length = 0U; s_overflow = false;
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_SINGLE
    append("MOTOR_STATUS BEGIN\nSELECT can=FDCAN1 id=%u", id);
    if (id_valid) append(" feedback_id=0x%03X\n", 0x200U + id);
    else append(" feedback_id=NA\n");
#else
    append("MOTOR_STATUS BEGIN\nSELECT can=FDCAN1 mode=FOUR\n");
#endif
    append("SNAPSHOT"); field("time_ms", true, s->now_ms);
    field("can_sample_age_ms", true, s->now_ms - s->can.sample_ms);
    field("control_age_ms", true, s->now_ms - s->debug.input.now_ms);
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_SINGLE
    append("\nRX seen=%u online=%u", seen, online);
    field("age_ms", seen, age); field("valid_frames", id_valid, s->can.valid_frames[slot]);
#endif
    append("\nRX_CHECK");
    field("bad_format_bus", true, s->can.bad_format);
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_SINGLE
    field("bad_encoder", id_valid, s->can.bad_encoder[slot]);
#endif
    field("read_fail", true, s->can.read_fail); field("fifo_lost_events", true, s->can.fifo_lost_events);
    append("\nCAN init_ok=%u status_valid=%u counters_valid=%u", s->status.can_ready,
           s->can.status_valid, s->can.counters_valid);
    field("bus_off", s->can.status_valid, s->can.bus_off);
    field("passive", s->can.status_valid, s->can.passive);
    field("warning", s->can.status_valid, s->can.warning);
    field("tec", s->can.counters_valid, s->can.tec); field("rec", s->can.counters_valid, s->can.rec);
    append("\nCAN_ERROR last=%s", s->can.last_error < 7U ? errors[s->can.last_error] : "UNKNOWN");
    field("age_ms", s->can.last_error != 0U, s->now_ms - s->can.last_error_ms);
    field("tx_enqueue_fail", true, s->status.can_tx_failures);
    unsigned recovery_state = (unsigned)s->recovery.state;
    append("\nRECOVERY state=%s", recovery_state < 5U ? recovery_states[recovery_state] : "UNKNOWN");
    field("bus_off_count", true, s->recovery.bus_off_count);
    field("attempts", true, s->recovery.attempts);
    field("success", true, s->recovery.success);
    field("failures", true, s->recovery.failures);
    field("error", true, s->recovery.error);
    field("last_ms", s->recovery.success != 0U, s->recovery.last_duration_ms);
    field("feedback_mask", true, s->recovery.feedback_mask);
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_SINGLE
    append("\nMOTOR fresh=%u rpm=", online);
    if (seen) decimal(s->motors.velocity[slot] * (60.0f / 6.283185307179586f)); else append("NA");
    append(" current_a="); if (seen) decimal(s->motors.current[slot]); else append("NA");
    field("temperature_c", seen, s->motors.temperature[slot]); field("encoder", seen, s->motors.encoder[slot]);
    unsigned state = (unsigned)s->debug.control.state, reason = (unsigned)s->debug.control.reason;
    append("\nCONTROL state=%s reason=%s rc_online=%u switch_a=%d stick=%d",
        state < 4U ? states[state] : "UNKNOWN", reason < sizeof(reasons) / sizeof(reasons[0]) ? reasons[reason] : "UNKNOWN",
        rc_online, (int)s->rc.switch_a, (int)s->rc.rocker_l_v);
#else
    append("\nCONTROL running_mask=%u rc_online=%u switch_a=%d stick=%d",
        s->debug.running_mask, rc_online, (int)s->rc.switch_a, (int)s->rc.rocker_l_v);
    for (unsigned i = 0; i < 4U; ++i) {
        bool seen = (s->motors.received_mask & (1U << i)) != 0U;
        uint32_t age = s->now_ms - s->motors.last_feedback_ms[i];
        bool online = seen && age < MOTOR_DEBUG_FEEDBACK_TIMEOUT_MS;
        unsigned state = (unsigned)s->debug.wheel[i].control.state;
        unsigned reason = (unsigned)s->debug.wheel[i].control.reason;
        append("\nMOTOR id=%u seen=%u online=%u", i+1U, seen, online);
        field("age_ms", seen, age); field("valid_frames", true, s->can.valid_frames[i]);
        field("bad_encoder", true, s->can.bad_encoder[i]);
        append(" rpm="); if (seen) decimal(s->motors.velocity[i] * (60.0f/6.283185307179586f)); else append("NA");
        append(" current_a="); if (seen) decimal(s->motors.current[i]); else append("NA");
        field("temperature_c", seen, s->motors.temperature[i]); field("encoder", seen, s->motors.encoder[i]);
        append(" state=%s reason=%s", state < 4U ? states[state] : "UNKNOWN",
            reason < sizeof(reasons)/sizeof(reasons[0]) ? reasons[reason] : "UNKNOWN");
    }
#endif
    append("\nONLINE");
    for (unsigned i = 0; i < 4U; ++i) append(" id%u=%u", i + 1U,
        (unsigned)(((s->motors.received_mask & (1U << i)) != 0U) &&
        (uint32_t)(s->now_ms - s->motors.last_feedback_ms[i]) < MOTOR_DEBUG_FEEDBACK_TIMEOUT_MS));
    append("\nMOTOR_STATUS END\n");
    if (s_overflow) {
        strcpy(s_report, "MOTOR_STATUS ERROR report_too_large\n");
        s_length = (unsigned)strlen(s_report);
    }
}

bool MotorStatusUsb_Busy(void) { return s_pending; }

static void finish(void)
{
    if (s_owned) FlashTasks_ReleaseUsb(FLASH_USB_STATUS);
    s_owned = s_sent = s_pending = false;
}

void MotorStatusUsb_Request(void)
{
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    if (s_pending && (!usb.ready || usb.session_id != s_session)) finish();
    if (s_pending || !usb.ready) return;
    s_session = usb.session_id;
    Campus_CopyMotorStatus(&s_snapshot);
    format_report();
    s_offset = 0U;
    s_pending = true;
}

void MotorStatusUsb_Process(void)
{
    if (!s_pending) return;
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    if (!usb.ready || usb.session_id != s_session) { finish(); return; }
    if (s_sent) {
        UsbSerialTxState state = UsbSerial_GetTxState(&s_ticket);
        if (state == USB_TX_PENDING) return;
        if (state == USB_TX_DISCONNECTED) { finish(); return; }
        s_offset += s_chunk;
        s_sent = false;
        if (s_offset == s_length) { finish(); return; }
    }
    if (!s_owned) {
        if (!FlashTasks_AcquireUsb(FLASH_USB_STATUS)) return;
        s_owned = true;
    }
    /* 不让报告跨 USB 会话入队；完成前持有输出所有权，不阻塞控制任务。 */
    taskENTER_CRITICAL();
    UsbSerial_GetStatus(&usb);
    s_chunk = s_length - s_offset;
    if (s_chunk > USB_SERIAL_TX_CAPACITY) s_chunk = USB_SERIAL_TX_CAPACITY;
    if (usb.ready && usb.session_id == s_session &&
        UsbSerial_SendTracked(s_report + s_offset, (uint16_t)s_chunk, &s_ticket) == USB_SERIAL_OK) s_sent = true;
    taskEXIT_CRITICAL();
}
