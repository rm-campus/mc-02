#include "servo_mixed_probe.h"
#include "servo_uart.h"
#include "usb_serial.h"
#include "zp_servo.h"
#include "cmsis_os2.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>

enum { SHOULDER_ID = 3, QUERY_GAP_MS = 10, REPLY_MS = 120, RX_DUMP_LIMIT = 24 };
enum { FS_PING = 0x01, FS_ANGLE = 0x0A, FS_MONITOR = 0x16 };

typedef struct {
    uint8_t data[48];
    size_t length;
    bool truncated;
} Capture;

typedef enum { RX_DATA, RX_NONE, RX_INVALID, UART_FAIL, TX_TIMEOUT, USB_GONE } BusResult;
static const char *const result_name[] = {
    "OK", "NO_REPLY", "INVALID_REPLY", "UART_ERROR", "TX_TIMEOUT", "USB_DISCONNECTED"
};

typedef struct { char *data; size_t capacity, used; } Report;

static void append(Report *report, const char *format, ...)
{
    if (report->used >= report->capacity) return;
    const size_t room = report->capacity - report->used;
    va_list args;
    va_start(args, format);
    int n = vsnprintf(report->data + report->used, room, format, args);
    va_end(args);
    report->used = n < 0 || (size_t)n >= room ? report->capacity : report->used + (size_t)n;
}

static void append_rx(Report *report, const Capture *rx)
{
    append(report, " rx=");
    if (!rx->length) append(report, "-");
    for (size_t i = 0; i < rx->length && i < RX_DUMP_LIMIT; ++i)
        append(report, i == 0U ? "%02X" : "-%02X", (unsigned)rx->data[i]);
    if (rx->truncated || rx->length > RX_DUMP_LIMIT) append(report, "...");
    append(report, "\n");
}

/* 每次只发一条查询，等待 UART7 完整发送后再收回复。保留二进制 NUL 和噪声字节。 */
static BusResult exchange(const uint8_t *frame, uint16_t length, uint32_t session,
                          uint32_t reply_ms, Capture *rx)
{
    osDelay(QUERY_GAP_MS);
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    if (!usb.ready || usb.session_id != session) return USB_GONE;

    ServoUartStatus uart;
    ServoUart_CopyStatus(&uart);
    if ((!uart.ready && !ServoUart_Recover()) || !ServoUart_Send((const char *)frame, length))
        return UART_FAIL;

    const uint32_t started = HAL_GetTick();
    bool sent = false;
    for (;;) {
        const uint32_t events = ServoUart_TakeEvents();
        if (events & SERVO_UART_ERROR) {
            (void)ServoUart_Recover();
            return UART_FAIL;
        }
        if (events & SERVO_UART_TX_DONE) {
            ServoUart_CopyStatus(&uart);
            sent = true;
        }
        if (sent) {
            uint8_t bytes[64];
            uint16_t count = ServoUart_Read(bytes, sizeof(bytes));
            for (uint16_t i = 0; i < count; ++i) {
                if (rx->length < sizeof(rx->data)) rx->data[rx->length++] = bytes[i];
                else rx->truncated = true;
            }
            if ((uint32_t)(HAL_GetTick() - uart.tx_done_ms) >= reply_ms)
                return rx->length ? RX_DATA : RX_NONE;
        } else if ((uint32_t)(HAL_GetTick() - started) >= SERVO_TX_TIMEOUT_MS) {
            const uint32_t last_events = ServoUart_TakeEvents();
            if ((last_events & SERVO_UART_ERROR) || !(last_events & SERVO_UART_TX_DONE)) {
                (void)ServoUart_Recover();
                return (last_events & SERVO_UART_ERROR) ? UART_FAIL : TX_TIMEOUT;
            }
            ServoUart_CopyStatus(&uart);
            sent = true;
        }
        osDelay(1U);
    }
}

static uint8_t checksum(const uint8_t *data, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) sum = (uint8_t)(sum + data[i]);
    return sum;
}

static const uint8_t *fashionstar_reply(const Capture *rx, uint8_t op, uint8_t id, uint8_t payload_size)
{
    const size_t frame_size = 5U + payload_size;
    for (size_t i = 0; i + frame_size <= rx->length; ++i) {
        const uint8_t *frame = rx->data + i;
        if (frame[0] == 0x05 && frame[1] == 0x1C && frame[2] == op &&
            frame[3] == payload_size && frame[4] == id &&
            checksum(frame, frame_size - 1U) == frame[frame_size - 1U])
            return frame + 4U;
    }
    return NULL;
}

static unsigned le16(const uint8_t *bytes)
{
    return (unsigned)bytes[0] | ((unsigned)bytes[1] << 8);
}

static void query_fashionstar(Report *report, uint32_t session, uint8_t id,
                              uint8_t op, uint8_t payload_size, const char *label)
{
    uint8_t frame[] = {0x12, 0x4C, op, 1U, id, 0U};
    frame[5] = checksum(frame, 5U);
    Capture rx = {0};
    BusResult result = exchange(frame, sizeof(frame), session, REPLY_MS, &rx);
    const uint8_t *payload = fashionstar_reply(&rx, op, id, payload_size);
    if (result == RX_DATA && payload == NULL) result = RX_INVALID;

    append(report, "FS id=%03u query=%s result=%s", (unsigned)id, label, result_name[result]);
    if (result == RX_DATA && op == FS_ANGLE) {
        const unsigned raw = le16(payload + 1U);
        const int angle = raw >= 32768U ? (int)raw - 65536 : (int)raw;
        append(report, " angle_ddeg=%d", angle);
    } else if (result == RX_DATA && op == FS_MONITOR) {
        /* 0x16：温度字段是 ADC 原始值，status 是保护位掩码。 */
        append(report, " voltage_mV=%u current_mA=%u power_mW=%u temp_adc=%u status=0x%02X",
               le16(payload + 1U), le16(payload + 3U), le16(payload + 5U), le16(payload + 7U),
               (unsigned)payload[9]);
    }
    append_rx(report, &rx);
}

static bool shoulder_reply(const Capture *rx, ZpOperation op, ZpReply *reply)
{
    size_t start = rx->length;
    for (size_t i = 0; i < rx->length; ++i) {
        if (rx->data[i] == '#') start = i;
        if (rx->data[i] == '!' && start < i &&
            Zp_Parse((const char *)rx->data + start, i - start + 1U, reply) &&
            Zp_Matches(op, SHOULDER_ID, reply)) return true;
    }
    return false;
}

static void query_shoulder(Report *report, uint32_t session, ZpOperation op, const char *label)
{
    char frame[16];
    const uint16_t length = (uint16_t)Zp_Build(frame, sizeof(frame), op, SHOULDER_ID, 0U, 0U);
    Capture rx = {0};
    const uint32_t timeout = op == ZP_MODE ? SERVO_MODE_PROBE_TIMEOUT_MS : REPLY_MS;
    BusResult result = exchange((const uint8_t *)frame, length, session, timeout, &rx);
    ZpReply data;
    if (result == RX_DATA && !shoulder_reply(&rx, op, &data)) result = RX_INVALID;

    append(report, "ZP id=%03u query=%s result=%s", (unsigned)SHOULDER_ID, label, result_name[result]);
    if (result == RX_DATA && op == ZP_POSITION) append(report, " position_p=%u", (unsigned)data.value);
    if (result == RX_DATA && op == ZP_MODE) append(report, " mode=%u", (unsigned)data.value);
    append_rx(report, &rx);
}

void ServoMixedProbe_Run(uint8_t fashionstar_id, uint32_t session, char *reply, size_t capacity)
{
    Report report = {reply, capacity, 0U};
    append(&report, "MIXED_SERVO BEGIN uart7=115200/8N1\n");
    query_shoulder(&report, session, ZP_ID, "id");
    query_shoulder(&report, session, ZP_POSITION, "position");
    query_shoulder(&report, session, ZP_MODE, "mode");
    query_fashionstar(&report, session, fashionstar_id, FS_PING, 1U, "ping");
    query_fashionstar(&report, session, fashionstar_id, FS_ANGLE, 3U, "angle");
    query_fashionstar(&report, session, fashionstar_id, FS_MONITOR, 16U, "monitor");
    append(&report, "MIXED_SERVO END\n");
    if (report.used >= report.capacity) (void)snprintf(reply, capacity, "ERR REPORT_SIZE\n");
}
