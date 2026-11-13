#include "flash_tasks.h"
#include "flash_test.h"
#include "motor_speed_log.h"
#include "flash_service.h"
#include "usb_serial.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motor_status_usb.h"
#include <string.h>
#include <stdio.h>

static FlashUsbOwner s_owner;
#if FLASH_TASKS_ENABLE
static uint8_t s_line[W25Q64_SECTOR_SIZE];
#else
static uint8_t s_line[64]; /* motor_status 单行；Flash 开启时保持 4096 字节上限。 */
#endif
static uint16_t s_length;
static bool s_discard;
static uint32_t s_session;

void FlashTasks_Create(void)
{
#if FLASH_TASKS_ENABLE
    if (!FlashService_Create()) Error_Handler();
#if FLASH_TEST_TASK_ENABLE
    if (!FlashTest_Create()) Error_Handler();
#endif
#if MOTOR_SPEED_LOG_TASK_ENABLE
    if (!MotorLog_Create()) Error_Handler();
#endif
#endif
}

bool FlashTasks_AcquireUsb(FlashUsbOwner owner)
{
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    if (!usb.ready || usb.tx_buffered != 0U) return false;
    taskENTER_CRITICAL();
    bool acquired = s_owner == FLASH_USB_NONE;
    if (acquired) s_owner = owner;
    taskEXIT_CRITICAL();
    return acquired;
}

void FlashTasks_ReleaseUsb(FlashUsbOwner owner)
{
    taskENTER_CRITICAL();
    if (s_owner == owner) s_owner = FLASH_USB_NONE;
    taskEXIT_CRITICAL();
}

void FlashTasks_Reply(const char *text)
{
#if FLASH_TASKS_ENABLE
    /* 回放/测试回传期间不插入其它输出。此入口只供 UsbTask 的命令分发使用。 */
    taskENTER_CRITICAL();
    bool idle = s_owner == FLASH_USB_NONE;
    if (idle) (void)UsbSerial_Send(text, (uint16_t)strlen(text));
    taskEXIT_CRITICAL();
#else
    (void)text;
#endif
}

void FlashTasks_UsbSession(uint32_t session, bool ready)
{
    if (session != s_session || !ready) { s_length = 0; s_discard = false; }
    s_session = session;
}

static void dispatch_line(void)
{
    uint16_t length = s_length;
    while (length && (s_line[length - 1U] == '\r' || s_line[length - 1U] == '\n')) --length;
    if (length == 0U) return;
    if (length == 12U && memcmp(s_line, "motor_status", 12U) == 0) {
        MotorStatusUsb_Request();
        return;
    }
#if FLASH_TASKS_ENABLE
    if (length == 4U && memcmp(s_line, "read", 4) == 0)
    {
#if MOTOR_SPEED_LOG_TASK_ENABLE
        if (!MotorLog_RequestRead()) FlashTasks_Reply("LOG NOT READY OR USB BUSY\n");
#else
        FlashTasks_Reply("LOG DISABLED\n");
#endif
    }
    else if (length >= 6U && memcmp(s_line, "flash:", 6) == 0)
    {
#if FLASH_TEST_TASK_ENABLE
        if (!FlashTest_Accept(s_line, s_length)) FlashTasks_Reply("FLASH TEST BUSY\n");
#else
        FlashTasks_Reply("FLASH TEST DISABLED\n");
#endif
    }
    else if (length == 5U && memcmp(s_line, "clear", 5) == 0)
    {
#if MOTOR_SPEED_LOG_TASK_ENABLE
        if (!MotorLog_RequestClear()) FlashTasks_Reply("LOG BUSY\n");
#else
        FlashTasks_Reply("LOG DISABLED\n");
#endif
    }
    else if (length == 6U && memcmp(s_line, "status", 6) == 0)
    {
        MotorLogStatus log;
        MotorLog_GetStatus(&log);
        char message[192];
        int n = snprintf(message, sizeof(message),
            "FLASH state=%u error=%u; TEST state=%lu error=%lu; LOG state=%u records=%lu error=%lu stop=%lu target=%u reached=%u period_ms=%lu\n",
            (unsigned)FlashService_GetState(), (unsigned)FlashService_GetError(),
            (unsigned long)FlashTest_State(), (unsigned long)FlashTest_Error(),
            (unsigned)log.state, (unsigned long)log.records, (unsigned long)log.error,
            (unsigned long)log.stop_reason, (unsigned)log.target_valid, (unsigned)log.reached, (unsigned long)log.period_ms);
        if (n > 0 && (unsigned)n < sizeof(message)) FlashTasks_Reply(message);
    }
    else FlashTasks_Reply("COMMANDS flash:<data>, read, status, clear, motor_status; end with LF\n");
#endif
}

void FlashTasks_UsbInput(const uint8_t *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; ++i)
    {
        uint8_t c = data[i];
        if (!s_discard)
        {
            if (s_length < sizeof(s_line)) s_line[s_length++] = c;
            else { s_length = 0; s_discard = true; }
        }
        if (c == '\n')
        {
            if (s_discard) FlashTasks_Reply("INPUT TOO LONG\n");
            else dispatch_line();
            s_length = 0; s_discard = false;
        }
    }
}
