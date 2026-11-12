#include "campus_app.h"
#include "usb_app.h"
#include "motor_status_usb.h"
#include "flash_tasks.h"
#include "motor_debug_config.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>

static CampusMotorStatusSnapshot sample;
static UsbSerialStatus usb={.ready=true,.session_id=1};
static UsbSerialTxState completion;
static unsigned sends, snapshots, depth;
static bool busy, race;
static char last[1100], input[5000];
static unsigned input_size, input_offset;
void test_enter(void) { ++depth; }
void test_exit(void) { assert(depth); --depth; }
void Campus_CopyMotorStatus(CampusMotorStatusSnapshot *out) { ++snapshots; *out=sample; }
void Campus_CopyMotorDebug(CampusMotorDebugSnapshot *out) { *out=sample.debug; }
void UsbSerial_Process(void) {}
int32_t osDelay(uint32_t n) { (void)n; return 0; }
uint16_t UsbSerial_Read(void *out,uint16_t capacity)
{
    unsigned n=input_size-input_offset; if(n>capacity) n=capacity;
    memcpy(out,input+input_offset,n); input_offset+=n; return (uint16_t)n;
}
void UsbSerial_GetStatus(UsbSerialStatus *out)
{
    *out=usb;
    if(race && !depth) { ++usb.session_id; race=false; }
}
UsbSerialTxState UsbSerial_GetTxState(const UsbSerialTxTicket *t)
{ return t->session_id==usb.session_id ? completion : USB_TX_DISCONNECTED; }
UsbSerialResult UsbSerial_SendTracked(const void *data,uint16_t len,UsbSerialTxTicket *t)
{
    if(busy) return USB_SERIAL_BUSY;
    assert(depth && usb.tx_buffered==0 && len<sizeof(last));
    memcpy(last,data,len); last[len]=0; ++sends;
    t->session_id=usb.session_id; t->end_bytes=sends;
    completion=USB_TX_PENDING; usb.tx_buffered=len; return USB_SERIAL_OK;
}
static void complete(void) { completion=USB_TX_COMPLETE; usb.tx_buffered=0; }
static void process(void) { Campus_UsbProcess(); }
static void feed(const char *text)
{
    assert(strlen(text)<sizeof(input)); strcpy(input,text); input_size=(unsigned)strlen(text); input_offset=0;
    do { process(); } while(input_offset<input_size);
}
static void next_query(void) { complete(); process(); feed("motor_status\n"); }
int main(void)
{
    sample.debug.motor_id=1; sample.debug.control.state=MOTOR_DEBUG_WAIT_OFF;
    sample.now_ms=1000; sample.debug.input.now_ms=1000;
    process(); assert(!sends);
    feed("motor_"); assert(!sends);
    feed("status\r\n"); assert(sends==1 && MotorStatusUsb_Busy());
    assert(strstr(last,"SELECT can=FDCAN1 id=1 feedback_id=0x201"));
    assert(strstr(last,"RX seen=0 online=0 age_ms=NA valid_frames=0"));
    assert(strstr(last,"bus_off=NA passive=NA warning=NA tec=NA rec=NA"));
    assert(strstr(last,"MOTOR fresh=0 rpm=NA current_a=NA temperature_c=NA encoder=NA"));
    assert(strstr(last,"MOTOR_STATUS END\n"));
    feed("motor_status\nmotor_status\n"); assert(snapshots==1 && sends==1);
    complete(); process(); assert(!MotorStatusUsb_Busy());

    sample.motors.received_mask=1; sample.motors.last_feedback_ms[0]=999;
    sample.motors.velocity[0]=-225.0f*6.283185307179586f/60.0f;
    sample.motors.current[0]=0.398f; sample.motors.temperature[0]=28; sample.motors.encoder[0]=1234;
    sample.can.valid_frames[0]=12560; sample.can.bad_encoder[0]=2;
    sample.can.status_valid=sample.can.counters_valid=sample.status.can_ready=true;
    sample.can.bus_off=true; sample.can.tec=255; sample.can.rec=127;
    sample.can.last_error=3; sample.can.last_error_ms=900;
    sample.debug.control.reason=MOTOR_DEBUG_CAN_FAULT;
    feed("motor_status\n");
    assert(strstr(last,"online=1 age_ms=1 valid_frames=12560"));
    assert(strstr(last,"bus_off=1 passive=0 warning=0 tec=255 rec=127"));
    assert(strstr(last,"last=ACK age_ms=100"));
    assert(strstr(last,"MOTOR fresh=1 rpm=-225.000 current_a=0.398 temperature_c=28 encoder=1234"));
    assert(strstr(last,"reason=CAN_FAULT"));
    sample.now_ms=1098; next_query(); assert(strstr(last,"online=1 age_ms=99"));
    sample.now_ms=1099; next_query();
    assert(strstr(last,"online=0 age_ms=100") && strstr(last,"MOTOR fresh=0 rpm=-225.000"));

    complete(); process(); unsigned before=sends;
    feed("MOTOR_STATUS\n motor_status\nmotor_status \nmotor_statusX\n"); assert(sends==before);
    char overlong[100]; memset(overlong,'x',sizeof(overlong)-1); overlong[99]=0;
    feed(overlong); feed("motor_status\n"); assert(sends==before);
    feed("motor_status\n"); assert(sends==before+1);

    complete(); process(); before=sends;
    feed("motor_"); usb.ready=false; process(); usb.ready=true; ++usb.session_id;
    feed("status\n"); assert(sends==before); /* No cross-session half-line. */
    busy=true; feed("motor_status\n"); assert(sends==before && MotorStatusUsb_Busy());
    unsigned captured=snapshots; feed("motor_status\n"); assert(snapshots==captured);
    busy=false; process(); assert(sends==before+1);
    usb.ready=false; process(); assert(!MotorStatusUsb_Busy());
    complete(); usb.ready=true; ++usb.session_id; process();
    assert(FlashTasks_AcquireUsb(FLASH_USB_LOG));
    before=sends; feed("motor_status\n"); assert(sends==before);
    FlashTasks_ReleaseUsb(FLASH_USB_LOG); process(); assert(sends==before+1);
    assert(!FlashTasks_AcquireUsb(FLASH_USB_TEST));
    complete(); process(); assert(FlashTasks_AcquireUsb(FLASH_USB_TEST));
    FlashTasks_ReleaseUsb(FLASH_USB_TEST);

    before=sends; busy=true; feed("motor_status\n"); busy=false;
    race=true; process(); assert(sends==before); process(); assert(!MotorStatusUsb_Busy());
    complete();
    busy=true; feed("motor_status\n"); assert(MotorStatusUsb_Busy());
    ++usb.session_id; busy=false; before=sends;
    feed("motor_status\n"); assert(sends==before+1); /* First new-session query must not be lost. */
    complete(); process();
    sample.can.valid_frames[0]=sample.can.bad_format=sample.can.bad_encoder[0]=UINT32_MAX;
    sample.can.read_fail=sample.can.fifo_lost_events=sample.status.can_tx_failures=UINT32_MAX;
    sample.recovery=(CanRecoveryStatus){.state=CAN_RECOVERY_WAIT_FEEDBACK,
        .bus_off_count=UINT32_MAX,.attempts=UINT32_MAX,.success=UINT32_MAX,.failures=UINT32_MAX,
        .last_duration_ms=UINT32_MAX,.error=3};
    feed("motor_status\n"); assert(strstr(last,"MOTOR_STATUS END\n") && !strstr(last,"report_too_large"));
    assert(strstr(last,"RECOVERY state=WAIT_FEEDBACK bus_off_count=4294967295"));
    complete(); process();

#if MOTOR_DEBUG_TELEMETRY
    sample.debug.control.state=MOTOR_DEBUG_RUNNING; sample.debug.run_id=1;
    sample.debug.input.now_ms=2000; process(); assert(!strcmp(last,"ready\n"));
    feed("motor_status\n"); assert(!strcmp(last,"ready\n")); /* Wait for ready in flight. */
    complete(); process(); assert(strstr(last,"MOTOR_STATUS BEGIN"));
    before=sends;
    for(unsigned i=0;i<100;++i) { ++sample.debug.input.now_ms; process(); }
    assert(sends==before); /* Report remains whole, no curve interleaving. */
    complete(); process();
    sample.debug.input.now_ms+=20; process(); assert(strchr(last,',') && !strchr(last,':'));
#endif
    puts("PASS: motor_status parser, no motor/stale/bus fault, units, merge, busy, owner, session/race, telemetry coordination.");
    return 0;
}
