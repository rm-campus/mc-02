#include "flash_test.h"
#include "flash_tasks.h"
#include "flash_service.h"
#include "usb_serial.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stddef.h>

#if FLASH_TEST_TASK_ENABLE
enum { T_IDLE, T_LOADING, T_WAIT_SERVICE, T_ERASE_FIRST, T_WRITE, T_READ,
       T_SEND, T_WAIT_TX, T_ERASE_LAST, T_PAUSED, T_ERROR };
static volatile uint32_t s_state, s_error;
static uint8_t s_received[W25Q64_SECTOR_SIZE];
static uint16_t s_length, s_offset, s_chunk;
static FlashJob s_job;
static UsbSerialTxTicket s_ticket;
static uint32_t s_send_session;
static osThreadId_t s_thread;

static void fail(W25Q64Result result)
{
    s_error = (uint32_t)result;
    s_state = T_ERROR;
    FlashTasks_ReleaseUsb(FLASH_USB_TEST);
}

bool FlashTest_Accept(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0U || length > sizeof(s_received) ||
        (s_state != T_IDLE && s_state != T_ERROR) || !FlashTasks_AcquireUsb(FLASH_USB_TEST))
        return false;
    s_state = T_LOADING;
    memcpy(s_received, data, length);
    s_length = length; s_offset = 0; s_error = 0;
    s_state = T_WAIT_SERVICE;
    if (s_thread != NULL) (void)osThreadFlagsSet(s_thread, FLASH_APP_WAKE_FLAG);
    return true;
}

void FlashTest_Process(void)
{
    if (s_job.pending) return;
    W25Q64Result r;
    if (FlashService_TakeResult(&s_job, &r))
    {
        if (r != W25Q64_OK) { fail(r); return; }
        if (s_state == T_ERASE_FIRST) s_state = T_WRITE;
        else if (s_state == T_WRITE)
        {
            s_offset += s_chunk;
            if (s_offset == s_length)
            {
                /* 接收副本主动清空：后续只能把真正的 Flash 读回结果交给 USB。 */
                memset(s_received, 0, sizeof(s_received));
                s_offset = 0; s_state = T_READ;
            }
        }
        else if (s_state == T_READ) s_state = T_SEND;
        else if (s_state == T_ERASE_LAST)
        {
            s_state = T_IDLE;
            FlashTasks_ReleaseUsb(FLASH_USB_TEST);
            return;
        }
    }
    switch (s_state)
    {
    case T_WAIT_SERVICE:
        if (FlashService_GetState() == FLASH_SERVICE_READY) s_state = T_ERASE_FIRST;
        else if (FlashService_GetState() == FLASH_SERVICE_ERROR) fail(FlashService_GetError());
        break;
    case T_ERASE_FIRST:
    case T_ERASE_LAST:
        (void)FlashService_Submit(&s_job, FLASH_ERASE, FLASH_TEST_BASE, 0);
        break;
    case T_WRITE:
        s_chunk = s_length - s_offset;
        if (s_chunk > sizeof(s_job.data)) s_chunk = sizeof(s_job.data);
        memcpy(s_job.data, s_received + s_offset, s_chunk);
        (void)FlashService_Submit(&s_job, FLASH_WRITE, FLASH_TEST_BASE + s_offset, s_chunk);
        break;
    case T_READ:
        s_chunk = s_length - s_offset;
        if (s_chunk > sizeof(s_job.data)) s_chunk = sizeof(s_job.data);
        memset(s_job.data, 0xA5, sizeof(s_job.data));
        (void)FlashService_Submit(&s_job, FLASH_READ, FLASH_TEST_BASE + s_offset, s_chunk);
        break;
    case T_SEND:
        {
            UsbSerialStatus usb;
            UsbSerial_GetStatus(&usb);
            if (!usb.ready || (s_offset != 0U && usb.session_id != s_send_session))
            { s_state = T_PAUSED; break; }
            UsbSerialResult sent = UsbSerial_SendTracked(s_job.data, s_chunk, &s_ticket);
            if (sent == USB_SERIAL_OK)
            {
                /* 枚举可能恰好发生在状态查询与入队之间；跨会话后缀不能导致擦除。 */
                if (s_offset != 0U && s_ticket.session_id != s_send_session) s_state = T_PAUSED;
                else { s_send_session = s_ticket.session_id; s_state = T_WAIT_TX; }
            }
            else if (sent == USB_SERIAL_NOT_READY) s_state = T_PAUSED;
            else if (sent != USB_SERIAL_BUSY) fail(W25Q64_IO);
        }
        break;
    case T_WAIT_TX:
        {
            UsbSerialTxState tx = UsbSerial_GetTxState(&s_ticket);
            if (tx == USB_TX_DISCONNECTED) s_state = T_PAUSED;
            else if (tx == USB_TX_COMPLETE)
            {
                s_offset += s_chunk;
                s_state = s_offset == s_length ? T_ERASE_LAST : T_READ;
            }
        }
        break;
    case T_PAUSED:
        {
            UsbSerialStatus usb;
            UsbSerial_GetStatus(&usb);
            if (usb.ready && usb.tx_buffered == 0U)
            {
                s_offset = 0;
                s_state = T_READ; /* 重连后从 Flash 开头重新回传，绝不使用接收 RAM。 */
            }
        }
        break;
    default: break;
    }
}

static void test_task(void *argument)
{
    (void)argument;
    for (;;)
    {
        FlashTest_Process();
        (void)osThreadFlagsWait(FLASH_APP_WAKE_FLAG, osFlagsWaitAny,
                               (s_state == T_IDLE || s_state == T_ERROR) ? osWaitForever : 1U);
    }
}
bool FlashTest_Create(void)
{
    const osThreadAttr_t attrs = {.name = "FlashTestTask", .stack_size = 2048U, .priority = osPriorityLow};
    s_thread = osThreadNew(test_task, NULL, &attrs);
    return s_thread != NULL;
}
uint32_t FlashTest_State(void) { return s_state; }
uint32_t FlashTest_Error(void) { return s_error; }
#else
bool FlashTest_Create(void) { return true; }
bool FlashTest_Accept(const uint8_t *data, uint16_t length) { (void)data; (void)length; return false; }
void FlashTest_Process(void) {}
uint32_t FlashTest_State(void) { return 0; }
uint32_t FlashTest_Error(void) { return 0; }
#endif
