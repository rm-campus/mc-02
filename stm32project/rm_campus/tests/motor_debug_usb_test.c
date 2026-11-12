/* Real telemetry state machine with delayed USB completion and session changes. */
#include "motor_debug_usb.h"
#include "campus_app.h"
#include "usb_serial.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static CampusMotorDebugSnapshot snap;
static UsbSerialStatus usb = {.ready=true, .session_id=1};
static UsbSerialTxState completion;
static unsigned sends, ready_count, data_count;
static bool busy, race;
bool MotorStatusUsb_Busy(void) { return false; }
static unsigned depth;
void test_enter(void) { ++depth; }
void test_exit(void) { assert(depth); --depth; }
static char last[128];
void Campus_CopyMotorDebug(CampusMotorDebugSnapshot *out) { *out=snap; }
void UsbSerial_GetStatus(UsbSerialStatus *out)
{
    *out=usb;
    if(race && !depth) { ++usb.session_id; race=false; }
}
UsbSerialTxState UsbSerial_GetTxState(const UsbSerialTxTicket *t)
{ return t->session_id!=usb.session_id ? USB_TX_DISCONNECTED : completion; }
UsbSerialResult UsbSerial_SendTracked(const void *data,uint16_t length,UsbSerialTxTicket *t)
{
    if(busy) return USB_SERIAL_BUSY;
    assert(length<sizeof(last)); memcpy(last,data,length); last[length]=0;
    assert(depth);
    t->session_id=usb.session_id; t->end_bytes=++sends;
    usb.tx_buffered=length; completion=USB_TX_PENDING;
    if(strcmp(last,"ready\n")==0) ++ready_count;
    else { assert(strchr(last,':')==NULL); ++data_count; }
    return USB_SERIAL_OK;
}
static void process(uint32_t now)
{ snap.input.now_ms=now; MotorDebugUsb_Process(); }
static void complete(void) { completion=USB_TX_COMPLETE; usb.tx_buffered=0; }
static void start(uint32_t now)
{
    snap.control.state=MOTOR_DEBUG_RUNNING; ++snap.run_id;
    process(now); assert(strcmp(last,"ready\n")==0);
    complete(); process(now);
}
int main(void)
{
    snap.control.state=MOTOR_DEBUG_WAIT_OFF;
    for(unsigned i=0;i<1000;++i) process(i);
    assert(!sends);
    snap.control.state=MOTOR_DEBUG_RUNNING; snap.run_id=1;
    process(1000); assert(ready_count==1 && data_count==0);
    process(2000); assert(sends==1); /* Ready must actually complete first. */
    complete(); process(2000);
    snap.control.target=225.0f*(6.283185307179586f/60.0f);
    snap.input.speed=-218.5f*(6.283185307179586f/60.0f);
    snap.control.command_raw=344; snap.input.measured_current=-0.398f;
    for(unsigned i=2001;i<2020;++i) process(i);
    assert(data_count==0);
    process(2020);
    assert(strcmp(last,"225.000,-218.500,0.420,-0.398\n")==0);
    complete();
    for(unsigned i=2021;i<=3000;++i) { process(i); complete(); }
    assert(data_count==50); /* Exactly 50 samples in 1 second. */

    busy=true; process(3020); unsigned before=sends;
    busy=false; process(3021); assert(sends==before);
    process(3040); assert(sends==before+1);
    for(unsigned i=3041;i<4000;++i) process(i);
    assert(sends==before+1); /* Only one outstanding data frame. */
    complete(); process(4000); assert(sends==before+2);
    complete(); process(4001); assert(sends==before+2); /* No catch-up burst. */

    snap.control.state=MOTOR_DEBUG_WAIT_OFF; process(4002);
    before=sends; process(5000); assert(sends==before);
    start(5001); assert(ready_count==2);
    process(5021); complete();
    usb.ready=false; process(5022); before=sends;
    process(6000); assert(sends==before);
    usb.ready=true; ++usb.session_id; process(6001);
    assert(ready_count==3); complete(); process(6001);
    process(6021); complete();
    ++snap.run_id; process(6022); /* A short stop/rearm between USB task calls. */
    assert(ready_count==4); complete(); process(6022);
    before=sends; race=true; process(6042); assert(sends==before);
    complete(); process(6043);
    assert(ready_count==5); /* New session must get ready before further data. */
    snap.control.state=MOTOR_DEBUG_WAIT_OFF; process(6044); complete();
    before=sends; process(7000); assert(sends==before);

    start(UINT32_MAX-10U); before=sends;
    process(8U); assert(sends==before);
    process(9U); assert(sends==before+1); complete();
    puts("PASS: startup silence, ready completion, four units/signs, 50Hz, busy/no backlog, stop/rearm, reconnect/race and tick wrap.");
    return 0;
}
