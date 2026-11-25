#include "servo_id_setup.h"
#include "servo_config.h"
#include "servo_uart.h"
#include "usb_serial.h"
#include "zp_servo.h"
#include "cmsis_os2.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#define ID_REPLY_MS 100U
#define ID_WRITE_REPLY_MS 250U
#define ID_WRITE_SETTLE_MS 100U
#define ID_VERIFY_RETRY_MS 50U
#define ID_VERIFY_ATTEMPTS 3U

typedef enum {
    ID_BUS_OK, ID_BUS_NO_REPLY, ID_BUS_INVALID_REPLY, ID_BUS_UART_ERROR, ID_BUS_TX_TIMEOUT
} IdBusResult;

static const char *id_bus_cause(IdBusResult result)
{
    switch (result) {
    case ID_BUS_OK: return "OK";
    case ID_BUS_NO_REPLY: return "NO_REPLY";
    case ID_BUS_INVALID_REPLY: return "INVALID_REPLY";
    case ID_BUS_UART_ERROR: return "UART_ERROR";
    case ID_BUS_TX_TIMEOUT: return "TX_TIMEOUT";
    }
    return "UNKNOWN";
}

/* 只接受一到三位十进制设备地址。255 为广播地址，不允许写入或使用。 */
static bool parse_id(const uint8_t **cursor, const uint8_t *end, uint8_t *id)
{
    const uint8_t *p = *cursor;
    unsigned value = 0U;
    unsigned digits = 0U;
    while (p != end && *p >= '0' && *p <= '9') {
        value = value * 10U + (unsigned)(*p++ - '0');
        if (++digits > 3U) return false;
    }
    if (digits == 0U || value >= 255U) return false;
    *cursor = p;
    *id = (uint8_t)value;
    return true;
}

/* UART 驱动在 TX 完成中断里立即切回接收；这里只识别期望 ID 的应答。 */
static IdBusResult exchange(const char *command, uint8_t expected_id, uint32_t reply_ms)
{
    ServoUartStatus status;
    ServoUart_CopyStatus(&status);
    if ((!status.ready && !ServoUart_Recover()) ||
        !ServoUart_Send(command, (uint16_t)strlen(command))) return ID_BUS_UART_ERROR;

    const uint32_t sent_ms = HAL_GetTick();
    uint32_t tx_done_ms = 0U;
    bool sent = false;
    bool received = false;
    char frame[ZP_FRAME_MAX];
    size_t used = 0U;
    for (;;) {
        const uint32_t events = ServoUart_TakeEvents();
        if ((events & SERVO_UART_ERROR) != 0U) return ID_BUS_UART_ERROR;
        if ((events & SERVO_UART_TX_DONE) != 0U) {
            ServoUart_CopyStatus(&status);
            tx_done_ms = status.tx_done_ms;
            sent = true;
        }

        uint8_t bytes[32];
        /* 保留事件检查之后才抵达的应答，等下一轮取到 TX_DONE 再读。 */
        const uint16_t count = sent ? ServoUart_Read(bytes, sizeof(bytes)) : 0U;
        if (count != 0U) received = true;
        for (uint16_t i = 0; i < count; ++i) {
            const char byte = (char)bytes[i];
            if (byte == '#') used = 0U;
            else if (used == 0U) continue;
            frame[used++] = byte;
            if (byte == '!') {
                ZpReply reply;
                if (Zp_Parse(frame, used, &reply) &&
                    Zp_Matches(ZP_ID, expected_id, &reply)) return ID_BUS_OK;
                used = 0U;
            } else if (used == sizeof(frame)) {
                used = 0U;
            }
        }
        const uint32_t now = HAL_GetTick();
        if (sent ? (uint32_t)(now - tx_done_ms) >= reply_ms
                 : (uint32_t)(now - sent_ms) >= SERVO_TX_TIMEOUT_MS) {
            (void)ServoUart_Recover();
            if (!sent) return ID_BUS_TX_TIMEOUT;
            return received ? ID_BUS_INVALID_REPLY : ID_BUS_NO_REPLY;
        }
        osDelay(1U);
    }
}

static IdBusResult read_id(uint8_t id)
{
    char query[16];
    if (Zp_Build(query, sizeof(query), ZP_ID, id, 0U, 0U) == 0U) return ID_BUS_UART_ERROR;
    return exchange(query, id, ID_REPLY_MS);
}

static IdBusResult verify_new_id(uint8_t id)
{
    /* 改号回执只说明舵机已受理，留出切换和保存时间，再独立查询。
     * 重试只读新地址，不重发写命令。 */
    osDelay(ID_WRITE_SETTLE_MS);
    IdBusResult result = read_id(id);
    for (unsigned attempt = 1U; result != ID_BUS_OK && attempt < ID_VERIFY_ATTEMPTS; ++attempt) {
        osDelay(ID_VERIFY_RETRY_MS);
        result = read_id(id);
    }
    return result;
}

static void handle_command(const uint8_t *line, size_t length, uint32_t session,
                           char *reply, size_t capacity)
{
    static const char read_prefix[] = "servo_read id ";
    static const char set_prefix[] = "servo_set_id ";
    if (length != 0U && line[length - 1U] == '\r') --length;
    const uint8_t *end = line + length;
    if (length >= sizeof(read_prefix) - 1U &&
        memcmp(line, read_prefix, sizeof(read_prefix) - 1U) == 0) {
        const uint8_t *p = line + sizeof(read_prefix) - 1U;
        uint8_t id;
        if (!parse_id(&p, end, &id) || p != end) {
            (void)snprintf(reply, capacity, "ERR INVALID_ID (use 000..254)\n");
            return;
        }
        const IdBusResult result = read_id(id);
        if (result == ID_BUS_OK)
            (void)snprintf(reply, capacity, "OK ID id=%03u\n", (unsigned)id);
        else
            (void)snprintf(reply, capacity, "ERR ID_NO_REPLY id=%03u cause=%s\n",
                           (unsigned)id, id_bus_cause(result));
        return;
    }

    uint8_t old_id, new_id;
    if (length < sizeof(set_prefix) - 1U ||
        memcmp(line, set_prefix, sizeof(set_prefix) - 1U) != 0) {
        (void)snprintf(reply, capacity, "USAGE servo_read id ID | servo_set_id OLD NEW (000..254)\n");
        return;
    }
    const uint8_t *p = line + sizeof(set_prefix) - 1U;
    if (!parse_id(&p, end, &old_id) || p == end || *p++ != ' ' ||
        !parse_id(&p, end, &new_id) || p != end || old_id == new_id) {
        (void)snprintf(reply, capacity, "ERR INVALID_ID (use two different IDs, 000..254)\n");
        return;
    }
    IdBusResult result = read_id(old_id);
    if (result != ID_BUS_OK) {
        (void)snprintf(reply, capacity, "ERR OLD_ID_NO_REPLY id=%03u cause=%s\n",
                       (unsigned)old_id, id_bus_cause(result));
        return;
    }

    osDelay(SERVO_BUS_GAP_MS);
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    if (!usb.ready || usb.session_id != session) return;

    char command[16];
    (void)snprintf(command, sizeof(command), "#%03uPID%03u!", (unsigned)old_id, (unsigned)new_id);
    /* 写号只发一次。等待新号应答后，再独立读新号确认，避免把发送成功当作写入完成。 */
    result = exchange(command, new_id, ID_WRITE_REPLY_MS);
    if (result != ID_BUS_OK) {
        (void)snprintf(reply, capacity, "ERR WRITE_UNCONFIRMED old=%03u new=%03u cause=%s\n",
                       (unsigned)old_id, (unsigned)new_id, id_bus_cause(result));
        return;
    }
    result = verify_new_id(new_id);
    if (result != ID_BUS_OK) {
        (void)snprintf(reply, capacity, "ERR NEW_ID_NO_REPLY id=%03u cause=%s\n",
                       (unsigned)new_id, id_bus_cause(result));
        return;
    }
    (void)snprintf(reply, capacity, "OK ID %03u -> %03u\n", (unsigned)old_id, (unsigned)new_id);
}

void ServoIdSetup_Task(void)
{
    (void)ServoUart_Init();
    uint8_t line[32];
    size_t used = 0U;
    bool discard = false;
    uint32_t session = 0U;
    char reply[80] = "";
    for (;;) {
        UsbSerial_Process();
        UsbSerialStatus usb;
        UsbSerial_GetStatus(&usb);
        if (!usb.ready || session != usb.session_id) {
            session = usb.session_id;
            used = 0U;
            discard = false;
            reply[0] = '\0';
        }
        if (usb.ready && reply[0] != '\0') {
            if (UsbSerial_Send(reply, (uint16_t)strlen(reply)) == USB_SERIAL_OK) reply[0] = '\0';
        }
        if (usb.ready && reply[0] == '\0') {
            uint8_t byte;
            for (unsigned i = 0; i < sizeof(line) && UsbSerial_Read(&byte, 1U) != 0U; ++i) {
                if (byte == '\n') {
                    if (discard) (void)snprintf(reply, sizeof(reply), "ERR LINE_TOO_LONG\n");
                    else if (used != 0U) handle_command(line, used, session, reply, sizeof(reply));
                    used = 0U;
                    discard = false;
                    break;
                }
                if (used < sizeof(line)) line[used++] = byte;
                else discard = true;
            }
        }
        osDelay(1U);
    }
}
