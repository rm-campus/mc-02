#include "motor_debug_usb.h"
#include "motor_debug_config.h"
#include "flash_tasks_config.h"
#include "motor_status_usb.h"

/* 两种输出协议分时使用，防止重新开启 Flash 后调试帧夹入 Flash 回传。 */
#if MOTOR_DEBUG_TELEMETRY && FLASH_TASKS_ENABLE
#error "Disable MOTOR_DEBUG_TELEMETRY before enabling Flash tasks"
#endif

#if MOTOR_DEBUG_TELEMETRY
#include "campus_app.h"
#include "usb_serial.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>

static uint32_t s_run, s_session, s_last_ms;
static bool s_active, s_announced, s_pending;
static UsbSerialTxTicket s_ticket;

static bool streaming(const CampusMotorDebugSnapshot *d)
{
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_FOUR
    return d->running_mask != 0U;
#else
    return d->control.state == MOTOR_DEBUG_RUNNING;
#endif
}

/* USB IRQ 优先级为 5：短临界区内重新核对会话和运行编号后整帧入队。
 * 防止格式化期间 USB 重枚举，导致新会话先收到数据、后收到 ready。 */
static UsbSerialResult send_current(const void *data, uint16_t length)
{
    UsbSerialStatus usb;
    CampusMotorDebugSnapshot d;
    taskENTER_CRITICAL();
    UsbSerial_GetStatus(&usb);
    Campus_CopyMotorDebug(&d);
    UsbSerialResult result = USB_SERIAL_NOT_READY;
    if (usb.ready && usb.session_id == s_session &&
        streaming(&d) && d.run_id == s_run) {
        result = UsbSerial_SendTracked(data, length, &s_ticket);
    }
    taskEXIT_CRITICAL();
    return result;
}

static int milli(float value)
{
    return isfinite(value) && fabsf(value) < 1000000.0f ? (int)lroundf(value * 1000.0f) : 0;
}

void MotorDebugUsb_Process(void)
{
    if (MotorStatusUsb_Busy()) return;
    CampusMotorDebugSnapshot d;
    UsbSerialStatus usb;
    Campus_CopyMotorDebug(&d);
    UsbSerial_GetStatus(&usb);
    if (!usb.ready || !streaming(&d)) {
        s_active = false;
        return;
    }
    if (!s_active || s_run != d.run_id || s_session != usb.session_id) {
        s_active = true;
        s_run = d.run_id;
        s_session = usb.session_id;
        s_announced = false;
        s_pending = false;
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_FOUR
        s_announced = true; /* 四轮只发五通道数字，不发送 ready。 */
        s_last_ms = d.input.now_ms;
#endif
    }
    if (s_pending) {
        UsbSerialTxState state = UsbSerial_GetTxState(&s_ticket);
        if (state == USB_TX_DISCONNECTED) { s_active = false; return; }
        if (state != USB_TX_COMPLETE) return;
        s_pending = false;
        if (!s_announced) {
            s_announced = true;
            s_last_ms = d.input.now_ms;
            return;
        }
    }
    if (!s_announced) {
        /* 旧帧排空后再标记新一次运行，不撤销已经提交的 USB 传输。 */
        if (usb.tx_buffered != 0U) return;
        if (send_current("ready\n", 6U) == USB_SERIAL_OK) {
            s_pending = true;
            if (s_ticket.session_id != usb.session_id) s_active = false;
        }
        return;
    }
    if ((uint32_t)(d.input.now_ms - s_last_ms) < MOTOR_DEBUG_TELEMETRY_PERIOD_MS) return;
    s_last_ms = d.input.now_ms; /* 不追赶补发；下一帧使用最新快照。 */
    if (usb.tx_buffered != 0U) return;
#if MOTOR_DEBUG_MODE == MOTOR_DEBUG_MODE_FOUR
    int values[5] = {milli(d.common_target * (60.0f / 6.283185307179586f))};
    for (unsigned i = 0; i < 4U; ++i) {
        const MotorDebugInput *in = &d.wheel[i].input;
        bool online = in->motor_received && (uint32_t)(d.input.now_ms - in->feedback_ms) < MOTOR_DEBUG_FEEDBACK_TIMEOUT_MS;
        values[i+1U] = online ? milli(in->speed * (60.0f / 6.283185307179586f)) : 0;
    }
#else
    int values[4] = {
        milli(d.control.target * (60.0f / 6.283185307179586f)),
        milli(d.input.speed * (60.0f / 6.283185307179586f)),
        milli((float)d.control.command_raw * (20.0f / 16384.0f)),
        milli(d.input.measured_current)
    };
#endif
    const unsigned channels = sizeof(values) / sizeof(values[0]);
    char frame[96];
    unsigned used = 0U;
    for (unsigned i = 0; i < channels; ++i) {
        unsigned magnitude = (unsigned)(values[i] < 0 ? -values[i] : values[i]);
        int n = snprintf(frame + used, sizeof(frame) - used, "%s%u.%03u%s",
                         values[i] < 0 ? "-" : "", magnitude / 1000U, magnitude % 1000U,
                         i == channels-1U ? "\n" : ",");
        if (n <= 0 || (unsigned)n >= sizeof(frame) - used) return;
        used += (unsigned)n;
    }
    if (send_current(frame, (uint16_t)used) == USB_SERIAL_OK) {
        s_pending = true;
        if (s_ticket.session_id != usb.session_id) s_active = false;
    }
}
#else
void MotorDebugUsb_Process(void) {}
#endif
