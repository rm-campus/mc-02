/** @file usb_serial.c
 * @brief 有界收发队列；USB IRQ 生产 RX/完成 TX，任务读 RX/提交 TX。
 * USB DMA 当前关闭。TX 队列中的在途区间直到完成回调（包括必要的 ZLP）才释放。
 */
#include "usb_serial.h"
#include "usb_serial_port.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

_Static_assert(USB_SERIAL_RX_CAPACITY >= CDC_DATA_HS_MAX_PACKET_SIZE &&
               USB_SERIAL_RX_CAPACITY <= UINT16_MAX, "Invalid USB RX capacity");
_Static_assert(USB_SERIAL_TX_CAPACITY > 0U &&
               USB_SERIAL_TX_CAPACITY <= UINT16_MAX, "Invalid USB TX capacity");
_Static_assert(USB_SERIAL_PRINTF_CAPACITY > 1U &&
               USB_SERIAL_PRINTF_CAPACITY <= USB_SERIAL_TX_CAPACITY,
               "Invalid USB printf capacity");

static uint8_t s_rx[USB_SERIAL_RX_CAPACITY];
static uint8_t s_tx[USB_SERIAL_TX_CAPACITY];
static uint16_t s_rx_head, s_rx_tail, s_rx_count;
static uint16_t s_tx_head, s_tx_tail, s_tx_count, s_tx_in_flight;
static uint16_t s_packet_size;
static bool s_configured, s_rx_armed;
static UsbSerialStatus s_status;

/* 最长临界区是一次容量受限的 memcpy；不等待端点、不延时、不格式化字符串。
 * 保存 PRIMASK，支持 USB IRQ、任务及应用短临界区内调用，不意外开启中断。 */
static uint32_t lock(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    return mask;
}

static bool ready(void)
{
    return s_configured && hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED &&
           hUsbDeviceHS.pClassData != NULL;
}

static void clear_queues(void)
{
    s_status.rx_discarded_bytes += s_rx_count;
    s_status.tx_discarded_bytes += s_tx_count;
    s_rx_head = s_rx_tail = s_rx_count = 0U;
    s_tx_head = s_tx_tail = s_tx_count = s_tx_in_flight = 0U;
    ++s_status.session_id;
}

/* 重新接收前预留完整最大包的空间，因此下一次回调不会覆盖尚未读取的数据。 */
static void rearm_rx(void)
{
    if (ready() && !s_rx_armed &&
        USB_SERIAL_RX_CAPACITY - s_rx_count >= s_packet_size)
    {
        if (USBD_CDC_ReceivePacket(&hUsbDeviceHS) == USBD_OK)
        {
            s_rx_armed = true;
        }
        else
        {
            ++s_status.io_error_count;
        }
    }
}

void UsbSerial_OnConfigured(uint16_t packet_size)
{
    uint32_t mask = lock();
    clear_queues();
    s_packet_size = packet_size;
    s_configured = packet_size != 0U && packet_size <= USB_SERIAL_RX_CAPACITY;
    /* ST 类驱动在 Init 返回后开放首次 OUT，这里不重复调用 ReceivePacket。 */
    s_rx_armed = s_configured;
    __set_PRIMASK(mask);
}

void UsbSerial_OnDeconfigured(void)
{
    uint32_t mask = lock();
    clear_queues();
    s_configured = false;
    s_rx_armed = false;
    s_packet_size = 0U;
    __set_PRIMASK(mask);
}

void UsbSerial_OnReceive(const uint8_t *data, uint32_t length)
{
    uint32_t mask = lock();
    s_rx_armed = false;
    if (!ready() || length > s_packet_size ||
        length > USB_SERIAL_RX_CAPACITY - s_rx_count || (data == NULL && length != 0U))
    {
        ++s_status.io_error_count;
        s_status.rx_discarded_bytes += length;
    }
    else if (length != 0U)
    {
        uint16_t first = (uint16_t)(USB_SERIAL_RX_CAPACITY - s_rx_head);
        if (first > length) first = (uint16_t)length;
        memcpy(&s_rx[s_rx_head], data, first);
        memcpy(s_rx, data + first, length - first);
        s_rx_head = (uint16_t)((s_rx_head + length) % USB_SERIAL_RX_CAPACITY);
        s_rx_count += (uint16_t)length;
        s_status.rx_bytes += length;
    }
    rearm_rx();
    if (ready() && !s_rx_armed) ++s_status.rx_pause_count;
    __set_PRIMASK(mask);
}

void UsbSerial_OnTransmitComplete(const uint8_t *data, uint32_t length)
{
    uint32_t mask = lock();
    if (s_configured && s_tx_in_flight != 0U &&
        data == &s_tx[s_tx_tail] && length == s_tx_in_flight)
    {
        s_tx_tail = (uint16_t)((s_tx_tail + length) % USB_SERIAL_TX_CAPACITY);
        s_tx_count -= s_tx_in_flight;
        s_status.tx_completed_bytes += s_tx_in_flight;
        s_tx_in_flight = 0U;
    }
    __set_PRIMASK(mask);
}

bool UsbSerial_IsReady(void)
{
    uint32_t mask = lock();
    bool result = ready();
    __set_PRIMASK(mask);
    return result;
}

UsbSerialResult UsbSerial_Send(const void *data, uint16_t length)
{
    if (data == NULL || length == 0U) return USB_SERIAL_INVALID_ARGUMENT;
    if (length > USB_SERIAL_TX_CAPACITY) return USB_SERIAL_TOO_LARGE;
    uint32_t mask = lock();
    UsbSerialResult result = USB_SERIAL_OK;
    if (!ready())
    {
        result = USB_SERIAL_NOT_READY;
    }
    else if (length > USB_SERIAL_TX_CAPACITY - s_tx_count)
    {
        ++s_status.tx_busy_count;
        result = USB_SERIAL_BUSY;
    }
    else
    {
        uint16_t first = (uint16_t)(USB_SERIAL_TX_CAPACITY - s_tx_head);
        if (first > length) first = length;
        memcpy(&s_tx[s_tx_head], data, first);
        memcpy(s_tx, (const uint8_t *)data + first, length - first);
        s_tx_head = (uint16_t)((s_tx_head + length) % USB_SERIAL_TX_CAPACITY);
        s_tx_count += length;
        s_status.tx_queued_bytes += length;
    }
    __set_PRIMASK(mask);
    return result;
}

uint16_t UsbSerial_Read(void *buffer, uint16_t capacity)
{
    if (buffer == NULL || capacity == 0U) return 0U;
    uint32_t mask = lock();
    uint16_t length = ready() ? s_rx_count : 0U;
    if (length > capacity) length = capacity;
    if (length != 0U)
    {
        uint16_t first = (uint16_t)(USB_SERIAL_RX_CAPACITY - s_rx_tail);
        if (first > length) first = length;
        memcpy(buffer, &s_rx[s_rx_tail], first);
        memcpy((uint8_t *)buffer + first, s_rx, length - first);
        s_rx_tail = (uint16_t)((s_rx_tail + length) % USB_SERIAL_RX_CAPACITY);
        s_rx_count -= length;
        rearm_rx();
    }
    __set_PRIMASK(mask);
    return length;
}

uint16_t UsbSerial_Available(void)
{
    uint32_t mask = lock();
    uint16_t length = ready() ? s_rx_count : 0U;
    __set_PRIMASK(mask);
    return length;
}

UsbSerialResult UsbSerial_Printf(const char *format, ...)
{
    if (format == NULL) return USB_SERIAL_INVALID_ARGUMENT;
    char buffer[USB_SERIAL_PRINTF_CAPACITY];
    va_list args;
    va_start(args, format);
    int length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if (length < 0) return USB_SERIAL_FORMAT_ERROR;
    if ((unsigned int)length >= sizeof(buffer)) return USB_SERIAL_TOO_LARGE;
    if (length == 0) return USB_SERIAL_OK;
    return UsbSerial_Send(buffer, (uint16_t)length);
}

void UsbSerial_GetStatus(UsbSerialStatus *status)
{
    if (status == NULL) return;
    uint32_t mask = lock();
    *status = s_status;
    status->ready = ready();
    status->rx_paused = status->ready && !s_rx_armed;
    status->tx_in_flight = s_tx_in_flight != 0U;
    status->rx_buffered = s_rx_count;
    status->tx_buffered = s_tx_count;
    __set_PRIMASK(mask);
}

void UsbSerial_Process(void)
{
    uint32_t mask = lock();
    if (ready())
    {
        rearm_rx();
        if (s_tx_in_flight == 0U && s_tx_count != 0U)
        {
            uint16_t length = (uint16_t)(USB_SERIAL_TX_CAPACITY - s_tx_tail);
            if (length > s_tx_count) length = s_tx_count;
            uint8_t result = CDC_Transmit_HS(&s_tx[s_tx_tail], length);
            if (result == USBD_OK)
            {
                s_tx_in_flight = length;
            }
            else if (result != USBD_BUSY)
            {
                ++s_status.io_error_count;
            }
            /* BUSY/FAIL 都保留队列，下一轮重试；绝不提前移动 tail。 */
        }
    }
    __set_PRIMASK(mask);
}
