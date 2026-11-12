#include "motor_speed_log.h"
#include "flash_tasks.h"
#include "flash_service.h"
#include "chassis_targets.h"
#include "campus_app.h"
#include "usb_serial.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>

#if MOTOR_SPEED_LOG_TASK_ENABLE
#define RECORD_BYTES 8U
#define HEADER_BYTES 32U
#define LOG_MAGIC 0x31474C4DUL /* 文件格式 MLG1，显式小端编码，没有逐条 CRC/时间戳。 */
#define MARK_RECORDING 0xFFFFFFFEUL
#define MARK_COMPLETE  0xFFFFFFFCUL
#define MARK_ERASING   0xFFFFFFF8UL
#define MAX_RECORDS ((FLASH_LOG_END - FLASH_LOG_DATA_BASE) / RECORD_BYTES)
_Static_assert(FLASH_LOG_END <= W25Q64_CAPACITY && FLASH_LOG_END > FLASH_LOG_DATA_BASE &&
               FLASH_LOG_END % W25Q64_SECTOR_SIZE == 0, "Invalid log partition");
enum { STOP_NONE, STOP_RELEASED, STOP_REACHED, STOP_RC_LOST, STOP_FULL, STOP_OVERFLOW, STOP_LATE };
enum { ERR_HEADER = 100, ERR_INCOMPLETE, ERR_DIRECTION_CHANGED };
typedef enum { A_NONE, A_BOOT, A_PREP_META, A_PREP_FIRST, A_START_HEADER,
               A_WRITE, A_ERASE_NEXT, A_FINISH, A_READ, A_MARK_ERASE, A_ERASE_DATA, A_ERASE_META } Action;
enum { CLEAN_MARK, CLEAN_DATA, CLEAN_META };

static volatile MotorLogStatus s_status;
static FlashJob s_job;
static Action s_action;
static osThreadId_t s_thread;
static int16_t s_fifo[MOTOR_LOG_FIFO_RECORDS][4];
static uint16_t s_head, s_tail, s_count, s_batch;
static uint32_t s_committed, s_accepted, s_prepared_until;
static uint32_t s_next_sample, s_hit_ms;
static uint8_t s_reversed;
static bool s_need_header;
static unsigned s_prep_phase, s_cleanup_phase;
static uint32_t s_erase_address, s_erase_end;
static volatile bool s_start_request, s_read_request, s_clear_request;
static volatile uint32_t s_rc_stop, s_rc_ms;
static volatile bool s_rc_valid;
static bool s_prev_valid;
static int16_t s_prev_x, s_prev_y;
static uint32_t s_play_index, s_next_send;
static uint32_t s_play_session;
static uint16_t s_play_position, s_play_count;
static bool s_play_ready, s_tx_pending;
static UsbSerialTxTicket s_ticket;
static char s_frame[128];

static void wake(void)
{
    if (s_thread != NULL) (void)osThreadFlagsSet(s_thread, FLASH_APP_WAKE_FLAG);
}

static void put32(uint8_t *out, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) out[i] = (uint8_t)(value >> (8U * i));
}
static uint32_t get32(const uint8_t *in)
{
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

static void header(uint32_t count, uint32_t marker)
{
    memset(s_job.data, 0xFF, HEADER_BYTES);
    put32(s_job.data, LOG_MAGIC);
    put32(s_job.data + 4, MOTOR_LOG_PERIOD_MS);
    put32(s_job.data + 8, count);
    put32(s_job.data + 12, s_reversed);
    put32(s_job.data + 16, marker);
    put32(s_job.data + 20, RECORD_BYTES);
}

static bool submit(Action action, FlashOperation operation, uint32_t address, uint16_t length)
{
    if (s_action != A_NONE || s_job.pending || s_job.done) return false;
    if (!FlashService_Submit(&s_job, operation, address, length)) return false;
    s_action = action;
    return true;
}

static void fail(uint32_t error)
{
    s_status.error = error;
    s_status.state = MOTOR_LOG_ERROR;
    FlashTasks_ReleaseUsb(FLASH_USB_LOG);
}

static void ready(void)
{
    s_committed = s_accepted = 0;
    s_head = s_tail = s_count = 0;
    s_prepared_until = FLASH_LOG_DATA_BASE + W25Q64_SECTOR_SIZE;
    s_status.records = 0; s_status.error = 0;
    s_status.reached = s_status.target_valid = false;
    s_status.state = MOTOR_LOG_READY;
    FlashTasks_ReleaseUsb(FLASH_USB_LOG);
}

static void begin_cleanup(uint32_t records, bool mark)
{
    s_status.state = MOTOR_LOG_ERASING;
    s_erase_address = FLASH_LOG_DATA_BASE;
    /* 即使没有记录也准备首扇区；绝不触碰 0x0000 的独立测试扇区。 */
    uint32_t bytes = records * RECORD_BYTES;
    if (bytes == 0U) bytes = 1U;
    s_erase_end = FLASH_LOG_DATA_BASE +
        ((bytes + W25Q64_SECTOR_SIZE - 1U) / W25Q64_SECTOR_SIZE) * W25Q64_SECTOR_SIZE;
    s_cleanup_phase = mark ? CLEAN_MARK : CLEAN_DATA;
}

static void load_header(void)
{
    bool blank = true;
    for (unsigned i = 0; i < HEADER_BYTES; ++i) blank = blank && s_job.data[i] == 0xFFU;
    if (blank)
    {
        s_status.state = MOTOR_LOG_PREPARING;
        s_prep_phase = 1; /* 信息页为空，只需准备首个数据扇区。 */
        return;
    }
    uint32_t n = get32(s_job.data + 8), direction = get32(s_job.data + 12);
    uint32_t marker = get32(s_job.data + 16);
    if (get32(s_job.data) != LOG_MAGIC || get32(s_job.data + 4) != MOTOR_LOG_PERIOD_MS ||
        get32(s_job.data + 20) != RECORD_BYTES || direction > 15U)
    { fail(ERR_HEADER); return; }
    if (marker == MARK_RECORDING) { fail(ERR_INCOMPLETE); return; }
    if (n > MAX_RECORDS || (marker != MARK_COMPLETE && marker != MARK_ERASING))
    { fail(ERR_HEADER); return; }
    s_reversed = (uint8_t)direction;
    s_committed = n;
    s_status.records = n;
    if (marker == MARK_ERASING) begin_cleanup(n, false);
    else s_status.state = MOTOR_LOG_WAIT_READ;
}

void MotorLog_OnRcFrame(const Fxi6x_Data *rc)
{
    if (rc == NULL) return;
    taskENTER_CRITICAL();
    bool valid = rc->received && rc->valid;
    int16_t x = rc->rocker_l_h, y = rc->rocker_l_v;
    s_rc_valid = valid;
    if (valid) s_rc_ms = rc->last_valid_ms;
    if (valid && s_prev_valid && s_status.state == MOTOR_LOG_READY && !s_clear_request &&
        ((s_prev_x == 0 && x != 0) || (s_prev_y == 0 && y != 0)))
        s_start_request = true;
    if (s_status.state == MOTOR_LOG_RECORDING || s_start_request)
    {
        if (!valid) s_rc_stop = STOP_RC_LOST;
        else if (x == 0 && y == 0) s_rc_stop = STOP_RELEASED;
    }
    s_prev_x = x; s_prev_y = y; s_prev_valid = valid;
    taskEXIT_CRITICAL();
    wake();
}

/* 覆盖 BSP 弱钩子，每个完整遥控帧只做事件更新，不访问 Flash。 */
void Fxi6x_OnFrame(const Fxi6x_Data *rc) { MotorLog_OnRcFrame(rc); }

bool MotorLog_RequestRead(void)
{
    if (s_status.state != MOTOR_LOG_WAIT_READ || s_read_request || s_clear_request ||
        s_job.pending || s_job.done || s_action != A_NONE || !FlashTasks_AcquireUsb(FLASH_USB_LOG))
        return false;
    taskENTER_CRITICAL();
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    s_play_session = usb.session_id;
    s_read_request = true;
    taskEXIT_CRITICAL();
    wake();
    return true;
}

bool MotorLog_RequestClear(void)
{
    taskENTER_CRITICAL();
    bool allowed = (s_status.state == MOTOR_LOG_WAIT_READ || s_status.state == MOTOR_LOG_READY ||
                    s_status.state == MOTOR_LOG_ERROR) && !s_job.pending && !s_job.done &&
                    !s_read_request && !s_start_request && !s_clear_request;
    if (allowed) s_clear_request = true;
    taskEXIT_CRITICAL();
    if (allowed) wake();
    return allowed;
}

void MotorLog_GetStatus(MotorLogStatus *out)
{
    if (out == NULL) return;
    taskENTER_CRITICAL(); *out = s_status; taskEXIT_CRITICAL();
}

static void stop_recording(uint32_t reason)
{
    s_status.stop_reason = reason;
    s_status.state = MOTOR_LOG_FLUSHING;
}

static void sample(uint32_t now)
{
    Motor3508 motors;
    Campus_CopyMotors(&motors);
    uint32_t sample_ms = HAL_GetTick();
    if (motors.reversed != s_reversed) { fail(ERR_DIRECTION_CHANGED); return; }
    if (s_accepted >= MAX_RECORDS) { stop_recording(STOP_FULL); return; }
    if (s_count == MOTOR_LOG_FIFO_RECORDS) { stop_recording(STOP_OVERFLOW); return; }
    uint8_t online = 0;
    float actual[4], target[4];
    for (unsigned i = 0; i < 4; ++i)
    {
        bool valid = (motors.received_mask & (1U << i)) &&
                     (uint32_t)(sample_ms - motors.last_feedback_ms[i]) < MOTOR_LOG_OFFLINE_MS;
        if (valid) online |= (uint8_t)(1U << i);
        s_fifo[s_head][i] = valid ? motors.rotor_rpm[i] : 0;
        actual[i] = (float)motors.rotor_rpm[i] * (187.0f / 3591.0f) *
                    ((s_reversed & (1U << i)) ? -1.0f : 1.0f);
    }
    s_head = (uint16_t)((s_head + 1U) % MOTOR_LOG_FIFO_RECORDS);
    ++s_count; ++s_accepted;
    bool target_valid = Chassis_CopyTargetWheelRpm(target);
    s_status.target_valid = target_valid;
    if (!s_status.reached && target_valid && online == 0x0FU)
    {
        bool reached = true, moving = false;
        for (unsigned i = 0; i < 4; ++i)
        {
            if (!isfinite(target[i])) { reached = false; continue; }
            float tolerance = fmaxf(MOTOR_LOG_REACHED_ABS_RPM, fabsf(target[i]) * MOTOR_LOG_REACHED_REL);
            if (target[i] != 0.0f)
            {
                moving = true;
                if (actual[i] * target[i] <= 0.0f) reached = false;
            }
            if (fabsf(actual[i] - target[i]) > tolerance) reached = false;
        }
        if (reached && moving) { s_hit_ms = now; s_status.reached = true; }
    }
}

static void start_recording(uint32_t now)
{
    Motor3508 motors;
    Campus_CopyMotors(&motors);
    s_reversed = motors.reversed & 0x0FU;
    s_head = s_tail = s_count = 0;
    s_committed = s_accepted = 0;
    s_status.records = s_status.error = s_status.stop_reason = 0;
    s_status.reached = s_status.target_valid = false;
    s_next_sample = now;
    s_need_header = true;
    s_status.state = MOTOR_LOG_RECORDING;
}

static void record_io(void)
{
    if (s_action != A_NONE) return;
    if (s_need_header)
    {
        header(UINT32_MAX, MARK_RECORDING);
        (void)submit(A_START_HEADER, FLASH_WRITE, FLASH_LOG_META_BASE, HEADER_BYTES);
        return;
    }
    if (s_count >= W25Q64_PAGE_SIZE / RECORD_BYTES ||
        (s_status.state == MOTOR_LOG_FLUSHING && s_count != 0U))
    {
        uint32_t address = FLASH_LOG_DATA_BASE + s_committed * RECORD_BYTES;
        if (address >= s_prepared_until)
        {
            (void)submit(A_ERASE_NEXT, FLASH_ERASE, address, 0);
            return;
        }
        s_batch = s_count;
        if (s_batch > W25Q64_PAGE_SIZE / RECORD_BYTES) s_batch = W25Q64_PAGE_SIZE / RECORD_BYTES;
        for (unsigned n = 0; n < s_batch; ++n)
            for (unsigned i = 0; i < 4; ++i)
            {
                uint16_t raw = (uint16_t)s_fifo[(s_tail + n) % MOTOR_LOG_FIFO_RECORDS][i];
                s_job.data[n * RECORD_BYTES + 2U * i] = (uint8_t)raw;
                s_job.data[n * RECORD_BYTES + 2U * i + 1U] = (uint8_t)(raw >> 8);
            }
        (void)submit(A_WRITE, FLASH_WRITE, address, s_batch * RECORD_BYTES);
    }
    else if (s_status.state == MOTOR_LOG_FLUSHING)
    {
        header(s_committed, MARK_COMPLETE);
        (void)submit(A_FINISH, FLASH_WRITE, FLASH_LOG_META_BASE, HEADER_BYTES);
    }
}

static void consume_result(uint32_t now)
{
    (void)now;
    W25Q64Result result;
    if (!FlashService_TakeResult(&s_job, &result)) return;
    Action action = s_action;
    s_action = A_NONE;
    if (result != W25Q64_OK) { fail(1000U + (uint32_t)result); return; }
    switch (action)
    {
    case A_BOOT: load_header(); break;
    case A_PREP_META: s_prep_phase = 1; break;
    case A_PREP_FIRST: ready(); break;
    case A_START_HEADER: s_need_header = false; break;
    case A_ERASE_NEXT: s_prepared_until = s_job.address + W25Q64_SECTOR_SIZE; break;
    case A_WRITE:
        s_tail = (uint16_t)((s_tail + s_batch) % MOTOR_LOG_FIFO_RECORDS);
        s_count -= s_batch; s_committed += s_batch;
        s_status.records = s_committed;
        break;
    case A_FINISH:
        if (s_committed == 0U) begin_cleanup(0, true);
        else s_status.state = MOTOR_LOG_WAIT_READ;
        break;
    case A_READ:
        s_play_position = 0; s_play_count = s_job.length / RECORD_BYTES;
        s_play_ready = true;
        break;
    case A_MARK_ERASE: s_cleanup_phase = CLEAN_DATA; break;
    case A_ERASE_DATA: s_erase_address += W25Q64_SECTOR_SIZE; break;
    case A_ERASE_META: ready(); break;
    default: break;
    }
}

static uint16_t format_frame(void)
{
    unsigned used = 0;
    memcpy(s_frame, "speed:", 6); used = 6;
    for (unsigned i = 0; i < 4; ++i)
    {
        const uint8_t *p = s_job.data + s_play_position * RECORD_BYTES + 2U * i;
        int16_t raw = (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
        /* 从 Flash 读出的转子 rpm 换算车轮 rpm，保留三位小数，避免依赖 printf 浮点库。 */
        int64_t value = (int64_t)raw * 187 * 1000;
        if (s_reversed & (1U << i)) value = -value;
        bool negative = value < 0;
        if (negative) value = -value;
        uint32_t milli_rpm = (uint32_t)((value + 3591 / 2) / 3591);
        int n = snprintf(s_frame + used, sizeof(s_frame) - used, "%s%s%lu.%03lu",
                         i ? "," : "", negative ? "-" : "",
                         (unsigned long)(milli_rpm / 1000U), (unsigned long)(milli_rpm % 1000U));
        if (n < 0 || (unsigned)n >= sizeof(s_frame) - used) return 0;
        used += (unsigned)n;
    }
    s_frame[used++] = '\n';
    return (uint16_t)used;
}

static void replay(uint32_t now)
{
    UsbSerialStatus usb;
    UsbSerial_GetStatus(&usb);
    if (!usb.ready || usb.session_id != s_play_session ||
        (s_tx_pending && UsbSerial_GetTxState(&s_ticket) == USB_TX_DISCONNECTED))
    {
        s_status.state = MOTOR_LOG_WAIT_READ;
        s_tx_pending = s_play_ready = false;
        FlashTasks_ReleaseUsb(FLASH_USB_LOG);
        return; /* 数据保留；重新 read 从头播放。 */
    }
    if (s_tx_pending)
    {
        if (UsbSerial_GetTxState(&s_ticket) != USB_TX_COMPLETE) return;
        s_tx_pending = false;
        ++s_play_index; ++s_play_position;
        if (s_play_index == s_committed) { begin_cleanup(s_committed, true); return; }
        if (s_play_position == s_play_count) s_play_ready = false;
    }
    if (s_committed == 0U) { begin_cleanup(0, true); return; }
    if (!s_play_ready)
    {
        if (s_action == A_NONE)
        {
            uint32_t bytes = (s_committed - s_play_index) * RECORD_BYTES;
            if (bytes > sizeof(s_job.data)) bytes = sizeof(s_job.data);
            memset(s_job.data, 0xA5, sizeof(s_job.data));
            (void)submit(A_READ, FLASH_READ, FLASH_LOG_DATA_BASE + s_play_index * RECORD_BYTES, (uint16_t)bytes);
        }
        return;
    }
    if ((int32_t)(now - s_next_send) < 0) return;
    uint16_t length = format_frame();
    if (length == 0U) { fail(W25Q64_ARGUMENT + 1000U); return; }
    UsbSerialResult sent = UsbSerial_SendTracked(s_frame, length, &s_ticket);
    if (sent == USB_SERIAL_OK)
    {
        if (s_ticket.session_id != s_play_session)
        {
            s_status.state = MOTOR_LOG_WAIT_READ;
            s_play_ready = false;
            FlashTasks_ReleaseUsb(FLASH_USB_LOG);
            return;
        }
        s_tx_pending = true;
        s_next_send = now + MOTOR_LOG_PERIOD_MS;
    }
    else if (sent != USB_SERIAL_BUSY && sent != USB_SERIAL_NOT_READY) fail(W25Q64_IO + 1000U);
}

void MotorLog_Process(uint32_t now)
{
    consume_result(now);
    taskENTER_CRITICAL();
    bool start = s_start_request, read = s_read_request, clear = s_clear_request;
    uint32_t rc_stop = s_rc_stop, rc_ms = s_rc_ms;
    bool rc_valid = s_rc_valid;
    s_start_request = s_read_request = s_clear_request = false;
    s_rc_stop = 0;
    taskEXIT_CRITICAL();
    if (clear && s_action == A_NONE)
    {
        s_status.error = 0;
        begin_cleanup(MAX_RECORDS, false); /* 显式 clear 才擦整个日志区，仍保留测试分区。 */
    }
    if (start && s_status.state == MOTOR_LOG_READY) start_recording(now);
    if (read && s_status.state == MOTOR_LOG_WAIT_READ)
    {
        s_play_index = 0; s_play_position = s_play_count = 0;
        s_tx_pending = s_play_ready = false; s_next_send = now;
        s_status.state = MOTOR_LOG_REPLAYING;
    }
    if (s_status.state == MOTOR_LOG_RECORDING)
    {
        /* 先处理回零/失联，再采样，避免写入回零后的制动过程。 */
        if (rc_stop) stop_recording(rc_stop);
        else if (!rc_valid || (int32_t)(now - rc_ms) >= (int32_t)MOTOR_LOG_RC_TIMEOUT_MS)
            stop_recording(STOP_RC_LOST);
        else if (s_status.reached && (uint32_t)(now - s_hit_ms) >= MOTOR_LOG_AFTER_REACHED_MS)
            stop_recording(STOP_REACHED);
        else if ((int32_t)(now - s_next_sample) >= 0)
        {
            if ((uint32_t)(now - s_next_sample) >= MOTOR_LOG_PERIOD_MS) stop_recording(STOP_LATE);
            else { sample(now); s_next_sample += MOTOR_LOG_PERIOD_MS; }
        }
    }
    if (s_status.state == MOTOR_LOG_RECORDING || s_status.state == MOTOR_LOG_FLUSHING)
        record_io();
    else if (s_status.state == MOTOR_LOG_REPLAYING) replay(now);
    else if (s_action == A_NONE)
    {
        if (s_status.state == MOTOR_LOG_BOOT)
        {
            if (FlashService_GetState() == FLASH_SERVICE_READY)
                (void)submit(A_BOOT, FLASH_READ, FLASH_LOG_META_BASE, HEADER_BYTES);
            else if (FlashService_GetState() == FLASH_SERVICE_ERROR) fail(1000U + FlashService_GetError());
        }
        else if (s_status.state == MOTOR_LOG_PREPARING)
        {
            if (s_prep_phase == 0U) (void)submit(A_PREP_META, FLASH_ERASE, FLASH_LOG_META_BASE, 0);
            else (void)submit(A_PREP_FIRST, FLASH_ERASE, FLASH_LOG_DATA_BASE, 0);
        }
        else if (s_status.state == MOTOR_LOG_ERASING)
        {
            if (s_cleanup_phase == CLEAN_MARK)
            {
                header(s_committed, MARK_ERASING);
                (void)submit(A_MARK_ERASE, FLASH_WRITE, FLASH_LOG_META_BASE, HEADER_BYTES);
            }
            else if (s_erase_address < s_erase_end)
                (void)submit(A_ERASE_DATA, FLASH_ERASE, s_erase_address, 0);
            else (void)submit(A_ERASE_META, FLASH_ERASE, FLASH_LOG_META_BASE, 0);
        }
    }
}

static void log_task(void *argument)
{
    (void)argument;
    for (;;)
    {
        MotorLog_Process(HAL_GetTick());
        MotorLogState state = s_status.state;
        (void)osThreadFlagsWait(FLASH_APP_WAKE_FLAG, osFlagsWaitAny,
            state == MOTOR_LOG_READY || state == MOTOR_LOG_WAIT_READ || state == MOTOR_LOG_ERROR
                ? osWaitForever : 1U);
    }
}

bool MotorLog_Create(void)
{
    const osThreadAttr_t attrs = {.name = "MotorSpeedLogTask", .stack_size = 2048U, .priority = osPriorityNormal};
    s_thread = osThreadNew(log_task, NULL, &attrs);
    return s_thread != NULL;
}
#else
bool MotorLog_Create(void) { return true; }
void MotorLog_OnRcFrame(const Fxi6x_Data *rc) { (void)rc; }
void MotorLog_Process(uint32_t now) { (void)now; }
bool MotorLog_RequestRead(void) { return false; }
bool MotorLog_RequestClear(void) { return false; }
void MotorLog_GetStatus(MotorLogStatus *out)
{
    if (out != NULL) { memset(out, 0, sizeof(*out)); out->state = MOTOR_LOG_READY; }
}
#endif
