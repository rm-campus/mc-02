#include "flash_tasks.h"
#include "flash_test.h"
#include "motor_speed_log.h"
#include "flash_service.h"
#include "campus_app.h"
#include "usb_serial.h"
#include <assert.h>
void MotorStatusUsb_Request(void) {} /* 协议诊断由 motor_status 集成套件覆盖。 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t tick;
static uint8_t nor[FLASH_LOG_END];
static unsigned erase_count[FLASH_LOG_END / 4096], thread_count, service_count;
static FlashJob *queue[8], *active;
static unsigned qhead, qtail, qcount;
static uint32_t finish_at;
static int mutate_echo;
static Motor3508 motors;
static CampusMotorDebugSnapshot debug;
static Fxi6x_Data rc;
static uint8_t online_mask = 15;
static int send_rc = 1, publish_target = 1;
static uint8_t running_mask = 15;
static bool service_hold;
static float target[4];
static bool usb_ready = true, usb_hold;
static uint32_t usb_session = 1, queued, completed, usb_due;
static uint8_t usb_pending[1024], sent[131072];
static uint16_t usb_length;
static size_t sent_length;
static uint32_t frame_times[4096];
static unsigned frame_count;
static int reconnect_during_send;

uint32_t HAL_GetTick(void) { return tick; }
void Error_Handler(void) { assert(!"Error_Handler"); }
void Campus_CopyMotors(Motor3508 *out) { *out = motors; }
void Campus_CopyMotorStatus(CampusMotorStatusSnapshot *out)
{
    memset(out, 0, sizeof(*out));
    out->now_ms = tick; out->motors = motors; out->debug = debug; out->rc = rc;
}
osThreadId_t osThreadNew(void (*fn)(void *), void *arg, const osThreadAttr_t *attrs)
{ (void)fn; (void)arg; assert(attrs->stack_size == 2048); return (void *)(uintptr_t)++thread_count; }
osThreadId_t osThreadGetId(void) { return (void *)1; }
uint32_t osThreadFlagsSet(osThreadId_t id, uint32_t flags) { (void)id; return flags; }
uint32_t osThreadFlagsWait(uint32_t f, uint32_t o, uint32_t t) { (void)f; (void)o; (void)t; return 0; }
int32_t osDelay(uint32_t n) { tick += n; return 0; }

bool FlashService_Create(void) { ++service_count; return true; }
FlashServiceState FlashService_GetState(void) { return FLASH_SERVICE_READY; }
W25Q64Result FlashService_GetError(void) { return W25Q64_OK; }
bool FlashService_Submit(FlashJob *job, FlashOperation op, uint32_t address, uint16_t length)
{
    assert(!job->pending && !job->done && length <= 256 && qcount < 8);
    assert(address < sizeof(nor) && length <= sizeof(nor) - address);
    if (op == FLASH_ERASE) assert(address % 4096 == 0);
    job->operation = op; job->address = address; job->length = length;
    job->pending = true;
    queue[qhead] = job; qhead = (qhead + 1) % 8; ++qcount;
    return true;
}
bool FlashService_TakeResult(FlashJob *job, W25Q64Result *result)
{
    if (!job->done) return false;
    *result = job->result; job->done = false; return true;
}
static void service_step(void)
{
    if (service_hold) return;
    if (active && (int32_t)(tick - finish_at) >= 0)
    {
        if (active->operation == FLASH_READ)
            memcpy(active->data, nor + active->address, active->length);
        else if (active->operation == FLASH_WRITE)
        {
            for (unsigned i = 0; i < active->length; ++i)
            {
                assert((nor[active->address + i] & active->data[i]) == active->data[i]);
                nor[active->address + i] &= active->data[i];
            }
            /* 在编程确认后改变 Flash 内容，验证业务回传来源确实是后续 Flash 读取。 */
            if (mutate_echo && active->address == FLASH_TEST_BASE) nor[6] = '9';
        }
        else
        {
            memset(nor + active->address, 0xFF, 4096);
            ++erase_count[active->address / 4096];
        }
        active->result = W25Q64_OK; active->pending = false; active->done = true; active = NULL;
    }
    if (!active && qcount)
    {
        active = queue[qtail]; qtail = (qtail + 1) % 8; --qcount;
        finish_at = tick + (active->operation == FLASH_ERASE ? 400 : 3);
    }
}

UsbSerialResult UsbSerial_SendTracked(const void *data, uint16_t length, UsbSerialTxTicket *ticket)
{
    if (!usb_ready) return USB_SERIAL_NOT_READY;
    if (usb_length) return USB_SERIAL_BUSY;
    if (reconnect_during_send)
    {
        reconnect_during_send = 0;
        ++usb_session; queued = completed = 0;
    }
    assert(length > 0 && length <= sizeof(usb_pending));
    memcpy(usb_pending, data, length); usb_length = length; queued += length; usb_due = tick + 2;
    if (ticket) { ticket->session_id = usb_session; ticket->end_bytes = queued; }
    const char *text = data;
    MotorLogStatus log; MotorLog_GetStatus(&log);
    if (log.state == MOTOR_LOG_REPLAYING) {
        unsigned commas = 0;
        for (unsigned i = 0; i < length; ++i) { if (text[i] == ',') ++commas; assert(text[i] != ':'); }
        assert(commas == 7 && text[length-1] == '\n');
        assert(frame_count < sizeof(frame_times) / sizeof(frame_times[0]));
        frame_times[frame_count++] = tick;
    }
    return USB_SERIAL_OK;
}
UsbSerialResult UsbSerial_Send(const void *data, uint16_t length)
{ return UsbSerial_SendTracked(data, length, NULL); }
UsbSerialTxState UsbSerial_GetTxState(const UsbSerialTxTicket *ticket)
{
    if (!usb_ready || ticket->session_id != usb_session) return USB_TX_DISCONNECTED;
    return (int32_t)(completed - ticket->end_bytes) >= 0 ? USB_TX_COMPLETE : USB_TX_PENDING;
}
void UsbSerial_GetStatus(UsbSerialStatus *out)
{
    memset(out, 0, sizeof(*out)); out->ready = usb_ready;
    out->session_id = usb_session; out->tx_buffered = usb_length;
    out->tx_in_flight = usb_length != 0; out->tx_queued_bytes = queued; out->tx_completed_bytes = completed;
}
static void usb_step(void)
{
    if (usb_ready && !usb_hold && usb_length && (int32_t)(tick - usb_due) >= 0)
    {
        assert(sent_length + usb_length + 1 < sizeof(sent));
        memcpy(sent + sent_length, usb_pending, usb_length); sent_length += usb_length;
        completed += usb_length; usb_length = 0; sent[sent_length] = 0;
    }
}
static void reconnect(void)
{
    ++usb_session; queued = completed = 0; usb_length = 0; usb_ready = true;
    FlashTasks_UsbSession(usb_session, true);
}
static void input(const char *s) { FlashTasks_UsbInput((const uint8_t *)s, (uint16_t)strlen(s)); }
static MotorLogStatus log_status(void) { MotorLogStatus s; MotorLog_GetStatus(&s); return s; }
static void step(void)
{
    ++tick;
    for (unsigned i = 0; i < 4; ++i)
        if (online_mask & (1U << i)) { motors.last_feedback_ms[i] = tick; motors.received_mask |= 1U << i; }
    if (send_rc) rc.last_valid_ms = tick;
    if (publish_target) {
        debug.input.now_ms = tick;
        debug.running_mask = running_mask;
        for (unsigned i = 0; i < 4; ++i) {
            debug.wheel[i].control.target = target[i] * (6.283185307179586f / 60.0f);
            debug.wheel[i].control.state = (running_mask & (1U << i)) ? MOTOR_DEBUG_RUNNING : MOTOR_DEBUG_WAIT_OFF;
        }
    }
    service_step();
    MotorLog_Process(tick); FlashTest_Process(); usb_step();
}
static void run(unsigned n) { while (n--) step(); }
static void until(MotorLogState state, unsigned limit)
{
    while (log_status().state != state && limit--) step();
    if (log_status().state != state)
    { fprintf(stderr, "state %u expected %u at %u err=%u\n", log_status().state, state, tick, log_status().error); abort(); }
}
static void boot(void)
{
    memset(nor, 0xFF, sizeof(nor));
    rc.received = rc.valid = true;
    rc.switch_a = -1;
    debug.active_mask = 15;
    FlashTasks_Create(); FlashTasks_UsbSession(usb_session, true);
}
static void put32(uint32_t address, uint32_t value)
{ for (unsigned i = 0; i < 4; ++i) nor[address + i] = (uint8_t)(value >> (i * 8)); }
static uint32_t get32(uint32_t address)
{ return (uint32_t)nor[address] | ((uint32_t)nor[address+1]<<8) | ((uint32_t)nor[address+2]<<16) | ((uint32_t)nor[address+3]<<24); }
static int16_t raw_at(uint32_t record, unsigned motor)
{ uint32_t a = FLASH_LOG_DATA_BASE + record * 32 + 16 + motor * 2; return (int16_t)((uint16_t)nor[a] | ((uint16_t)nor[a+1]<<8)); }
static int32_t target_at(uint32_t record, unsigned motor)
{ return (int32_t)get32(FLASH_LOG_DATA_BASE + record * 32 + motor * 4); }
static void seed_log(uint32_t marker)
{
    put32(FLASH_LOG_META_BASE, 0x32474C4D); put32(FLASH_LOG_META_BASE+4, MOTOR_LOG_PERIOD_MS);
    put32(FLASH_LOG_META_BASE+8, 2); put32(FLASH_LOG_META_BASE+12, 0);
    put32(FLASH_LOG_META_BASE+16, marker); put32(FLASH_LOG_META_BASE+20, 32);
    memset(nor + FLASH_LOG_DATA_BASE, 0, 64);
}
static void finish_test(void)
{
    unsigned limit = 5000;
    while (FlashTest_State() != 0 && limit--) step();
    assert(FlashTest_State() == 0 && FlashTest_Error() == 0);
}
static void capture(unsigned milliseconds)
{
    until(MOTOR_LOG_READY, 2000);
    target[0] = 100; target[1] = -200; target[2] = 300; target[3] = -400;
    run(milliseconds);
    assert(log_status().state == MOTOR_LOG_RECORDING);
    memset(target, 0, sizeof(target)); step();
    until(MOTOR_LOG_WAIT_READ, 2000);
}

int main(int argc, char **argv)
{
    assert(argc == 2); const char *scenario = argv[1]; boot();
    if (!strcmp(scenario, "switches"))
    {
        assert(service_count == (unsigned)FLASH_TASKS_ENABLE);
        assert(thread_count == (unsigned)(FLASH_TEST_TASK_ENABLE + MOTOR_SPEED_LOG_TASK_ENABLE));
        if (!FLASH_TEST_TASK_ENABLE) assert(!FlashTest_Accept((const uint8_t *)"flash:1\n", 8));
        if (!MOTOR_SPEED_LOG_TASK_ENABLE) assert(!MotorLog_RequestRead() && !MotorLog_RequestClear());
    }
    else if (!strcmp(scenario, "echo"))
    {
        uint8_t frame[] = "flash:1,2,3,4\n";
        usb_hold = true; mutate_echo = 1;
        FlashTasks_UsbInput(frame, 3); FlashTasks_UsbInput(frame+3, sizeof(frame)-4);
        memset(frame, 0xCC, sizeof(frame)); run(1200);
        assert(erase_count[0] == 1 && usb_length && nor[6] == '9'); /* 尚未完成发送，不得擦除。 */
        usb_hold = false; finish_test();
        assert(!strcmp((char *)sent, "flash:9,2,3,4\n") && erase_count[0] == 2);
        for (unsigned i = 0; i < 4096; ++i) assert(nor[i] == 0xFF);
    }
    else if (!strcmp(scenario, "echo_reconnect"))
    {
        char frame[602]; memset(frame, '1', sizeof(frame)); memcpy(frame, "flash:", 6);
        frame[600] = '\n'; frame[601] = 0; input(frame);
        while (sent_length < 256) step();
        usb_ready = false; run(20); assert(erase_count[0] == 1);
        reconnect(); sent_length = 0; finish_test();
        assert(sent_length == 601 && memcmp(sent, frame, 601) == 0 && erase_count[0] == 2);
    }
    else if (!strcmp(scenario, "parser"))
    {
        uint8_t huge[4100]; memset(huge, 'x', sizeof(huge)); huge[4099] = '\n';
        FlashTasks_UsbInput(huge, sizeof(huge)); run(5); assert(FlashTest_State() == 0);
        sent_length = 0; input("flash:7,");
        reconnect(); input("flash:1,2,3,4\r\n"); finish_test();
        assert(!strcmp((char *)sent, "flash:1,2,3,4\r\n"));
    }
    else if (!strcmp(scenario, "log") || !strcmp(scenario, "tick_wrap"))
    {
        if (!strcmp(scenario, "tick_wrap")) tick = UINT32_MAX - 1500;
        motors.rotor_rpm[0] = 3591; motors.rotor_rpm[1] = -3591;
        motors.rotor_rpm[2] = 7182; motors.rotor_rpm[3] = 3591;
        motors.reversed = 1; online_mask = 7;
        capture(5501);
        uint32_t n = log_status().records;
        assert(n == 551 && log_status().stop_reason == 1 && !log_status().reached);
        assert(raw_at(0, 0) == 3591 && raw_at(0, 1) == -3591 && raw_at(n-1, 3) == 0);
        assert(target_at(0, 0) == 100000 && target_at(0, 1) == -200000 && target_at(0, 3) == -400000);
        assert(get32(FLASH_LOG_META_BASE+8) == n);
        assert(get32(FLASH_LOG_META_BASE+4) == 10 && log_status().period_ms == 10);
        /* 修改 Flash 首条而不改采样 RAM；回放必须反映此修改。 */
        nor[FLASH_LOG_DATA_BASE+16] = 0x0E; nor[FLASH_LOG_DATA_BASE+17] = 0x1C; /* 7182 */
        put32(FLASH_LOG_DATA_BASE, (uint32_t)-250000); /* 历史目标也必须重新读 Flash。 */
        sent_length = frame_count = 0; usb_hold = true; input("re"); input("ad\n"); run(100);
        assert(usb_length && log_status().state == MOTOR_LOG_REPLAYING);
        assert(raw_at(0, 0) == 7182); usb_hold = false;
        while (frame_count < n) step();
        usb_hold = true; run(100);
        assert(log_status().state == MOTOR_LOG_REPLAYING && get32(FLASH_LOG_META_BASE+16) == 0xFFFFFFFC);
        assert(raw_at(0, 0) == 7182); /* 最后一帧已入队但未完成，仍必须保留 Flash。 */
        usb_hold = false; until(MOTOR_LOG_READY, 3000);
        const char *first = "-250.000,-374.000,-200.000,-187.000,300.000,374.000,-400.000,0.000\n";
        assert(frame_count == n && !strncmp((char *)sent, first, strlen(first)));
        for (unsigned i = 1; i < frame_count; ++i) assert((uint32_t)(frame_times[i]-frame_times[i-1]) >= 10);
        for (unsigned i = FLASH_LOG_META_BASE; i < FLASH_LOG_DATA_BASE + ((n*32+4095)/4096)*4096; ++i) assert(nor[i] == 0xFF);
    }
    else if (!strcmp(scenario, "dynamic"))
    {
        until(MOTOR_LOG_READY, 2000);
        const float first[4] = {187, -187, 374, -374}, next[4] = {374, -374, 187, -187};
        const int16_t first_raw[4] = {3591, -3591, 7182, -7182}, next_raw[4] = {7182, -7182, 3591, -3591};
        memcpy(target, first, sizeof(target));
        step(); uint32_t start = tick; run(99);
        for (unsigned i = 0; i < 4; ++i) { target[i] = next[i]; motors.rotor_rpm[i] = first_raw[i]; }
        run(120); assert(!log_status().reached); /* 达到首帧的旧目标不触发。 */
        online_mask = 7; motors.received_mask &= 7;
        memcpy(motors.rotor_rpm, next_raw, sizeof(next_raw));
        run(120); assert(!log_status().reached); /* 离线第四台绝不能参与达标。 */
        online_mask = 15;
        unsigned limit = 100; while (!log_status().reached && limit--) step();
        assert(log_status().reached);
        uint32_t hit = tick;
        for (unsigned i = 0; i < 4; ++i) target[i] = 50;
        while (tick - hit < 4999) step();
        assert(log_status().state == MOTOR_LOG_RECORDING);
        step(); assert(log_status().state == MOTOR_LOG_FLUSHING && tick - hit == 5000);
        assert(tick - start >= 5000);
        until(MOTOR_LOG_WAIT_READ, 1000); assert(log_status().stop_reason == 2);
        assert(target_at(0, 0) == 187000 && target_at(0, 1) == -187000 && target_at(0, 2) == 374000);
        assert(target_at(10, 0) == 374000 && target_at(10, 2) == 187000);
        input("read\n"); until(MOTOR_LOG_READY, 10000); run(200);
        assert(log_status().state == MOTOR_LOG_READY); /* 持续非零目标不能自动重录。 */
        memset(target, 0, sizeof(target)); step(); target[2] = -50; step();
        assert(log_status().state == MOTOR_LOG_RECORDING); /* 负目标重新触发。 */
    }
    else if (!strcmp(scenario, "start_conditions"))
    {
        until(MOTOR_LOG_READY, 2000); rc.rocker_l_h = rc.rocker_l_v = 300; run(30);
        assert(log_status().state == MOTOR_LOG_READY); /* 摇杆边沿不再触发。 */
        target[0] = NAN; run(25); assert(!log_status().target_valid && log_status().state == MOTOR_LOG_READY);
        target[0] = 0; target[3] = -100; running_mask = 7; run(25);
        assert(log_status().state == MOTOR_LOG_READY);
        running_mask = 15; debug.active_mask = 7; run(25); assert(log_status().state == MOTOR_LOG_READY);
        debug.active_mask = 15; rc.switch_a = 1; run(25); assert(log_status().state == MOTOR_LOG_READY);
        rc.switch_a = -1; rc.valid = false; run(25); assert(log_status().state == MOTOR_LOG_READY);
        rc.valid = true; rc.rocker_l_h = rc.rocker_l_v = 0; step();
        assert(log_status().state == MOTOR_LOG_RECORDING); /* 任意一台负目标；摇杆保持零。 */
        run(6100); assert(log_status().state == MOTOR_LOG_RECORDING && !log_status().reached);
        target[3] = 0; step(); until(MOTOR_LOG_WAIT_READ, 1000);
        assert(log_status().stop_reason == 1 && target_at(0, 3) == -100000);
    }
    else if (!strcmp(scenario, "reconnect"))
    {
        for (unsigned i = 0; i < 4; ++i) motors.rotor_rpm[i] = 3591;
        capture(1000); uint32_t n = log_status().records;
        input("read\n"); while (frame_count < 3) step(); run(3);
        /* 即使在两帧之间直接重新枚举，也不能继续半份回放后自动擦除。 */
        reconnect(); step(); assert(log_status().state == MOTOR_LOG_WAIT_READ);
        assert(get32(FLASH_LOG_META_BASE+8) == n);
        sent_length = frame_count = 0; input("read\n"); until(MOTOR_LOG_READY, 3000);
        assert(frame_count == n);
    }
    else if (!strcmp(scenario, "recovery"))
    {
        seed_log(0xFFFFFFFC); until(MOTOR_LOG_WAIT_READ, 100);
        assert(log_status().records == 2 && erase_count[2] == 0);
        input("read\n"); until(MOTOR_LOG_READY, 2000); assert(frame_count == 2);
    }
    else if (!strcmp(scenario, "enqueue_race"))
    {
        capture(1000); uint32_t n = log_status().records;
        reconnect_during_send = 1; input("read\n");
        unsigned limit = 100; while (reconnect_during_send && limit--) step();
        assert(!reconnect_during_send && log_status().state == MOTOR_LOG_WAIT_READ);
        assert(get32(FLASH_LOG_META_BASE+8) == n);
        run(5); sent_length = frame_count = 0;
        input("read\n"); until(MOTOR_LOG_READY, 3000); assert(frame_count == n);
    }
    else if (!strcmp(scenario, "incomplete"))
    {
        seed_log(0xFFFFFFFE); until(MOTOR_LOG_ERROR, 100);
        assert(log_status().error == 101 && erase_count[2] == 0);
        input("clear\n"); until(MOTOR_LOG_READY, 5000);
        for (unsigned i = 4096; i < sizeof(nor); ++i) assert(nor[i] == 0xFF);
    }
    else if (!strcmp(scenario, "full"))
    {
        until(MOTOR_LOG_READY, 2000); target[0] = 100;
        run(21000); until(MOTOR_LOG_WAIT_READ, 1000);
        assert(log_status().records == (FLASH_LOG_END-FLASH_LOG_DATA_BASE)/32 && log_status().stop_reason == 4);
    }
    else if (!strcmp(scenario, "rc_loss"))
    {
        until(MOTOR_LOG_READY, 2000); target[0] = 100; run(100);
        send_rc = 0; run(110); until(MOTOR_LOG_WAIT_READ, 1000);
        assert(log_status().stop_reason == 3);
    }
    else if (!strcmp(scenario, "late"))
    {
        until(MOTOR_LOG_READY, 2000); target[0] = 100; run(10);
        tick += 40; step(); until(MOTOR_LOG_WAIT_READ, 1000); assert(log_status().stop_reason == 6);
    }
    else if (!strcmp(scenario, "nonzero_boot"))
    {
        target[1] = -100; until(MOTOR_LOG_RECORDING, 2000);
        assert(rc.rocker_l_v == 0 && rc.rocker_l_h == 0);
    }
    else if (!strcmp(scenario, "local_fault"))
    {
        until(MOTOR_LOG_READY, 2000);
        for (unsigned i = 0; i < 4; ++i) { target[i] = 374; motors.rotor_rpm[i] = 3591; }
        step(); online_mask = running_mask = 7; target[3] = 0;
        run(99); assert(log_status().state == MOTOR_LOG_RECORDING);
        run(61); memset(target, 0, sizeof(target)); step(); until(MOTOR_LOG_WAIT_READ, 1000);
        assert(raw_at(9, 3) == 3591 && raw_at(10, 3) == 0); /* 90ms 有效，100ms 离线。 */
        assert(target_at(1, 3) == 0 && !log_status().reached && log_status().stop_reason == 1);
    }
    else if (!strcmp(scenario, "fault_not_reached"))
    {
        until(MOTOR_LOG_READY, 2000); for (unsigned i=0;i<4;++i) target[i]=374;
        step(); running_mask=7; target[3]=0;
        for(unsigned i=0;i<3;++i) { target[i]=187; motors.rotor_rpm[i]=3591; }
        run(6100); assert(log_status().state == MOTOR_LOG_RECORDING && !log_status().reached);
    }
    else if (!strcmp(scenario, "control_stale") || !strcmp(scenario, "switch_off") ||
             !strcmp(scenario, "all_stopped") || !strcmp(scenario, "invalid_target"))
    {
        until(MOTOR_LOG_READY, 2000); target[0]=100; run(100);
        unsigned reason;
        if (!strcmp(scenario,"control_stale")) { publish_target=0; reason=7; }
        else if (!strcmp(scenario,"switch_off")) { rc.switch_a=0; reason=8; }
        else if (!strcmp(scenario,"all_stopped")) { running_mask=0; reason=9; }
        else { target[2]=INFINITY; reason=10; }
        run(110); until(MOTOR_LOG_WAIT_READ,1000); assert(log_status().stop_reason==reason);
    }
    else if (!strcmp(scenario, "overflow"))
    {
        until(MOTOR_LOG_READY,2000); service_hold=true; target[0]=100; run(5200);
        assert(log_status().state==MOTOR_LOG_FLUSHING && log_status().stop_reason==5);
        service_hold=false; until(MOTOR_LOG_WAIT_READ,3000); assert(log_status().records==256);
    }
    else if (!strcmp(scenario, "legacy"))
    {
        seed_log(0xFFFFFFFC); put32(FLASH_LOG_META_BASE,0x31474C4D); put32(FLASH_LOG_META_BASE+20,8);
        until(MOTOR_LOG_ERROR,100); assert(log_status().error==103 && erase_count[2]==0);
        assert(!MotorLog_RequestRead()); input("clear\n"); until(MOTOR_LOG_READY,5000);
        for(unsigned i=4096;i<sizeof(nor);++i) assert(nor[i]==0xFF);
    }
    else if (!strcmp(scenario, "erase_recovery"))
    {
        seed_log(0xFFFFFFF8); until(MOTOR_LOG_READY,2000);
        assert(erase_count[1]==1 && erase_count[2]==1 && frame_count==0);
    }
    else if (!strcmp(scenario, "cadence100"))
    {
        capture(10000);
        assert(log_status().records == 1000 && log_status().period_ms == 10);
        input("status\n"); run(5); assert(strstr((char *)sent,"period_ms=10\n"));
        sent_length=frame_count=0; input("read\n"); until(MOTOR_LOG_READY,15000);
        assert(frame_count == 1000);
        for(unsigned i=1;i<frame_count;++i) assert(frame_times[i]-frame_times[i-1] == 10);
    }
    else if (!strcmp(scenario, "old_period") || !strcmp(scenario, "old_erase_recovery"))
    {
        bool erasing = !strcmp(scenario,"old_erase_recovery");
        seed_log(erasing ? 0xFFFFFFF8 : 0xFFFFFFFC); put32(FLASH_LOG_META_BASE+4,20);
        if(erasing) {
            until(MOTOR_LOG_ERASING,100); assert(log_status().period_ms==20);
        } else {
            until(MOTOR_LOG_WAIT_READ,100); assert(log_status().period_ms==20);
            input("status\n"); run(5); assert(strstr((char *)sent,"period_ms=20\n"));
            input("read\n"); until(MOTOR_LOG_ERASING,100);
            assert(frame_count==2 && frame_times[1]-frame_times[0]==20);
            run(10); assert(get32(FLASH_LOG_META_BASE+16)==0xFFFFFFF8);
            assert(get32(FLASH_LOG_META_BASE+4)==20); /* 标记擦除不会把 20 改写为 10。 */
        }
        until(MOTOR_LOG_READY,2000);
        assert(log_status().period_ms==10 && log_status().error==0);
        target[0]=-100; run(101); memset(target,0,sizeof(target)); step(); until(MOTOR_LOG_WAIT_READ,1000);
        assert(log_status().records==11 && get32(FLASH_LOG_META_BASE+4)==10);
    }
    else if (!strcmp(scenario, "bad_period"))
    {
        seed_log(0xFFFFFFFC); put32(FLASH_LOG_META_BASE+4,0);
        until(MOTOR_LOG_ERROR,100); assert(log_status().error==100 && erase_count[2]==0);
    }
    else assert(!"unknown scenario");
    printf("PASS: Flash app %s (test=%d log=%d)\n", scenario, FLASH_TEST_TASK_ENABLE, MOTOR_SPEED_LOG_TASK_ENABLE);
    return 0;
}
