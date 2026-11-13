#include "campus_app.h"
#include "usb_app.h"
#include "motor_status_usb.h"
#include "motor_debug_config.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static CampusMotorStatusSnapshot sample;
static UsbSerialStatus usb={.ready=true,.session_id=1};
static UsbSerialTxState completion;
static unsigned depth, frames, sends;
static bool busy, freeze_first;
static char last[1100], report[2400];
static unsigned report_len;
static const char *rx;
void test_enter(void) { ++depth; }
void test_exit(void) { assert(depth); --depth; }
void Campus_CopyMotorDebug(CampusMotorDebugSnapshot *out) { *out=sample.debug; }
void Campus_CopyMotorStatus(CampusMotorStatusSnapshot *out) { *out=sample; }
void UsbSerial_GetStatus(UsbSerialStatus *out) { *out=usb; }
void UsbSerial_Process(void) {}
int32_t osDelay(uint32_t n) { (void)n; return 0; }
uint16_t UsbSerial_Read(void *data,uint16_t cap)
{
    if(!rx || !*rx) return 0;
    unsigned n=(unsigned)strlen(rx); if(n>cap)n=cap;
    memcpy(data,rx,n); rx+=n; return (uint16_t)n;
}
UsbSerialTxState UsbSerial_GetTxState(const UsbSerialTxTicket *t)
{ return t->session_id==usb.session_id ? completion : USB_TX_DISCONNECTED; }
UsbSerialResult UsbSerial_SendTracked(const void *data,uint16_t n,UsbSerialTxTicket *ticket)
{
    if(busy)return USB_SERIAL_BUSY;
    assert(depth && !usb.tx_buffered && n<sizeof(last));
    memcpy(last,data,n); last[n]=0; assert(strcmp(last,"ready\n")!=0);
    if(MotorStatusUsb_Busy()) {
        assert(report_len+n<sizeof(report)); memcpy(report+report_len,data,n); report_len+=n; report[report_len]=0;
    } else {
        unsigned commas=0; for(unsigned i=0;i<n;++i)if(last[i]==',')++commas;
        assert(commas==(sample.debug.mecanum_mode?11U:8U) && last[n-1]=='\n' && !strchr(last,':') && !strchr(last,'=')); ++frames;
    }
    ++sends; ticket->session_id=usb.session_id; ticket->end_bytes=sends;
    usb.tx_buffered=n; completion=USB_TX_PENDING; return USB_SERIAL_OK;
}
static void complete(void) { usb.tx_buffered=0; completion=USB_TX_COMPLETE; }
static void process(uint32_t now)
{
    sample.now_ms=sample.debug.input.now_ms=now;
    for(unsigned i=0;i<3;++i) {
        if(i!=0 || !freeze_first) sample.debug.wheel[i].input.feedback_ms=now;
        sample.motors.last_feedback_ms[i]=sample.debug.wheel[i].input.feedback_ms;
    }
    Campus_UsbProcess();
}
int main(void)
{
    sample.debug.common_target=225.0f*6.283185307179586f/60;
    sample.debug.wheel[0].input.speed=-218.5f*6.283185307179586f/60;
    sample.debug.wheel[1].input.speed=220.2f*6.283185307179586f/60;
    sample.debug.wheel[2].input.speed=0;
    sample.debug.wheel[0].input.measured_current=0.398f;
    sample.debug.wheel[1].input.measured_current=-0.750f;
    sample.debug.wheel[2].input.measured_current=1.250f;
    sample.debug.wheel[3].input.measured_current=9.5f; /* 未收到反馈，不能回传陈旧电流。 */
    for(unsigned i=0;i<4;++i) sample.debug.wheel[i].control.command_raw=8192; /* 指令 10A 与反馈不同。 */
    for(unsigned i=0;i<3;++i)sample.debug.wheel[i].input.motor_received=true;
    sample.motors.received_mask=7;
    process(999); assert(!sends);
    sample.debug.running_mask=5; sample.debug.run_id=1;
    process(1000); assert(!sends); /* No ready, wait 5ms. */
    for(unsigned i=1001;i<=2000;++i){process(i);complete();}
    assert(frames==200);
    assert(!strcmp(last,"225.000,-218.500,220.200,0.000,0.000,0.398,-0.750,1.250,0.000\n"));
    freeze_first=true; process(2099); complete();
    assert(strstr(last,"-218.500"));
    process(2119); complete(); assert(!strcmp(last,"225.000,0.000,220.200,0.000,0.000,0.000,-0.750,1.250,0.000\n"));
    freeze_first=false;
    busy=true; process(2140); unsigned before=frames;
    busy=false; process(2141); assert(frames==before);
    process(2160); assert(frames==before+1);
    process(2500); assert(frames==before+1); complete(); process(2501); complete();
    before=frames; process(2502); assert(frames==before);

    /* Report must fit, span multiple USB transfers, retain ownership throughout. */
    sample.can.bad_format=sample.can.read_fail=sample.can.fifo_lost_events=UINT32_MAX;
    sample.recovery=(CanRecoveryStatus){.bus_off_count=UINT32_MAX,.attempts=UINT32_MAX,
        .success=UINT32_MAX,.failures=UINT32_MAX,.last_duration_ms=UINT32_MAX,.feedback_mask=5};
    for(unsigned i=0;i<4;++i){sample.can.valid_frames[i]=sample.can.bad_encoder[i]=UINT32_MAX;
        sample.debug.wheel[i].control.state=MOTOR_DEBUG_WAIT_OFF;sample.debug.wheel[i].control.reason=MOTOR_DEBUG_TEMPERATURE;}
    rx="motor_status\n"; process(2510); assert(MotorStatusUsb_Busy());
    before=frames; process(2600); assert(frames==before);
    complete(); process(2601); assert(MotorStatusUsb_Busy() && frames==before);
    complete(); process(2602);
    assert(!MotorStatusUsb_Busy() && strstr(report,"MOTOR_STATUS END\n") && !strstr(report,"report_too_large"));
    assert(strstr(report,"mode=FOUR") && strstr(report,"MOTOR id=1") && strstr(report,"MOTOR id=4"));
    assert(strstr(report,"reason=TEMPERATURE") && strstr(report,"feedback_mask=5")); complete();
    sample.debug.running_mask=0; before=frames; process(2700); assert(frames==before);
    sample.debug.running_mask=1; ++sample.debug.run_id; process(2800); process(2820); complete();
    assert(frames==before+1);
    usb.ready=false; process(2821); usb.ready=true; ++usb.session_id; process(3000);
    before=frames; process(3004); assert(frames==before); process(3005); complete(); assert(frames==before+1);
    sample.debug.wheel[3].input.motor_received=true;
    sample.debug.wheel[3].input.feedback_ms=3010;
    sample.debug.wheel[3].input.speed=-100.0f*6.283185307179586f/60;
    sample.debug.wheel[3].input.measured_current=-2.5f;
    process(3010); complete();
    assert(!strcmp(last,"225.000,-218.500,220.200,0.000,-100.000,0.398,-0.750,1.250,-2.500\n"));
    assert(strlen(last)>64); /* 完整一行可跨 USB FS 包。 */
    sample.debug.mecanum_mode=true;
    for(unsigned i=0;i<4;++i)
        sample.debug.wheel[i].input.target_rad_s=(i==0 || i==3 ? 40.0f : -40.0f)*6.283185307179586f/60;
    process(3015); complete();
    assert(!strcmp(last,"40.000,-40.000,-40.000,40.000,-218.500,220.200,0.000,-100.000,0.398,-0.750,1.250,-2.500\n"));
    sample.debug.mecanum_mode=false;
    sample.debug.common_target=-999999.0f*6.283185307179586f/60;
    for(unsigned i=0;i<4;++i) {
        sample.debug.wheel[i].input.speed=-999999.0f*6.283185307179586f/60;
        sample.debug.wheel[i].input.measured_current=-999999.0f;
    }
    sample.debug.wheel[3].input.feedback_ms=3015;
    before=frames; process(3020); complete();
    assert(frames==before+1 && strlen(last)>96 && strlen(last)<128); /* 格式器允许范围内的最长九列。 */
    puts("PASS: four-mode nine-column CSV, signed feedback currents, no ready, 200Hz, offline zero, long frames, backpressure/reconnect and complete diagnostics.");
    return 0;
}
