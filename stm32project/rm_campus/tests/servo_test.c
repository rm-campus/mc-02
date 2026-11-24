#include "servo_app.h"
#include "servo_usb.h"
#include "flash_tasks.h"
#include "usb_app.h"
#include "campus_app.h"
#include "usb_serial.h"
#include "usart.h"
#include "cmsis_os2.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart7={7},huart5={5};
static uint32_t tick,mask; static unsigned depth;
static bool receive_mode,tx_pending,drop_tc,drop_move_tc,fail_tx;
static uint8_t *rx_byte;
static char tx[65]; static uint32_t tx_due;
static unsigned tx_count,moves,reply_mask=31;
static uint16_t positions[5]={1500,1500,1500,1500,1500};
static unsigned modes[5]={1,1,1,1,1};
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void Bsp_UnhandledUartError(UART_HandleTypeDef *);
uint32_t HAL_GetTick(void){return tick;}
uint32_t __get_PRIMASK(void){return mask;}
void __disable_irq(void){mask=1;}
void __set_PRIMASK(uint32_t p){mask=p;}
void test_enter(void){++depth;}
void test_exit(void){assert(depth);--depth;}
osThreadId_t osThreadGetId(void){return (void *)1;}
uint32_t osThreadFlagsSet(osThreadId_t t,uint32_t f){(void)t;return f;}
uint32_t osThreadFlagsWait(uint32_t f,uint32_t o,uint32_t n){(void)f;(void)o;(void)n;return 0;}
int32_t osDelay(uint32_t n){(void)n;return 0;}
int HAL_UART_Abort(UART_HandleTypeDef *h){assert(h==&huart7);rx_byte=NULL;tx_pending=false;return HAL_OK;}
int HAL_UART_AbortReceive(UART_HandleTypeDef *h){assert(h==&huart7);rx_byte=NULL;return HAL_OK;}
int HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *h){assert(h==&huart7);receive_mode=true;return HAL_OK;}
int HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *h){assert(h==&huart7&&!rx_byte);receive_mode=false;return HAL_OK;}
int HAL_UART_Receive_IT(UART_HandleTypeDef *h,uint8_t *p,uint16_t n){assert(h==&huart7&&receive_mode&&!rx_byte&&n==1);rx_byte=p;return HAL_OK;}
int HAL_UART_Transmit_IT(UART_HandleTypeDef *h,const uint8_t *p,uint16_t n)
{
    assert(h==&huart7&&!receive_mode&&!tx_pending&&n<sizeof(tx));
    if(fail_tx && n==15U && p[9]=='T'){fail_tx=false;return HAL_ERROR;} /* Fail the motion, not an intervening due query. */
    memcpy(tx,p,n);tx[n]=0;tx_pending=true;tx_due=tick+2;++tx_count;
    assert(!strstr(tx,"#255")&&!strstr(tx,"PBD")&&!strstr(tx,"PP"));
    return HAL_OK;
}
static void feed(const char *p)
{
    while(*p && rx_byte) { *rx_byte=(uint8_t)*p++;rx_byte=NULL;HAL_UART_RxCpltCallback(&huart7); }
}
static void bus_step(void)
{
    if(!tx_pending||drop_tc||(drop_move_tc && strlen(tx)==15U && tx[9]=='T')||(int32_t)(tick-tx_due)<0)return;
    tx_pending=false;HAL_UART_TxCpltCallback(&huart7);assert(receive_mode&&rx_byte);
    unsigned id=0,p=0,ms=0; assert(sscanf(tx,"#%3u",&id)==1&&id>=1&&id<=5);
    if(sscanf(tx,"#%3uP%4uT%4u!",&id,&p,&ms)==3) { positions[id-1]=(uint16_t)p;++moves;return; }
    if(!(reply_mask&(1U<<(id-1))))return;
    char reply[64];
    if(strstr(tx,"PRAD!"))snprintf(reply,sizeof(reply),"#%03uP%04u!",id,positions[id-1]);
    else if(strstr(tx,"PID!"))snprintf(reply,sizeof(reply),"#%03uP!",id);
    else if(strstr(tx,"PVER!"))snprintf(reply,sizeof(reply),"#%03uPV0.97!",id);
    else if(strstr(tx,"PMOD!"))snprintf(reply,sizeof(reply),"#%03uPMOD%u!",id,modes[id-1]);
    else strcpy(reply,"#OK!");
    feed(reply); /* Reply immediately after TC, before owner task runs. */
}

static UsbSerialStatus usb={.session_id=1};
static char usb_frame[1100],output[20000]; static unsigned usb_len,output_len;
static uint32_t usb_due,usb_seq,done_seq; static bool usb_hold;
void UsbSerial_GetStatus(UsbSerialStatus *out){*out=usb;out->tx_buffered=(uint16_t)usb_len;}
UsbSerialResult UsbSerial_Send(const void *p,uint16_t n){(void)p;(void)n;return USB_SERIAL_BUSY;}
UsbSerialResult UsbSerial_SendTracked(const void *p,uint16_t n,UsbSerialTxTicket *t)
{
    if(!usb.ready)return USB_SERIAL_NOT_READY;
    if(usb_len)return USB_SERIAL_BUSY;
    assert(n&&n<sizeof(usb_frame));memcpy(usb_frame,p,n);usb_len=n;usb_due=tick+2;
    t->session_id=usb.session_id;t->end_bytes=++usb_seq;return USB_SERIAL_OK;
}
UsbSerialTxState UsbSerial_GetTxState(const UsbSerialTxTicket *t)
{return t->session_id!=usb.session_id?USB_TX_DISCONNECTED:(t->end_bytes<=done_seq?USB_TX_COMPLETE:USB_TX_PENDING);}
void UsbSerial_Process(void)
{
    if(usb_len&&!usb_hold&&(int32_t)(tick-usb_due)>=0){
        assert(output_len+usb_len<sizeof(output));memcpy(output+output_len,usb_frame,usb_len);output_len+=usb_len;output[output_len]=0;
        usb_len=0;done_seq=usb_seq;
    }
}
uint16_t UsbSerial_Read(void *p,uint16_t n){(void)p;(void)n;return 0;}
void MotorStatusUsb_Request(void){}
void MotorStatusUsb_Process(void){}
bool MotorStatusUsb_Busy(void){return false;}
static bool mecanum_mode;
void Campus_CopyMotorDebug(CampusMotorDebugSnapshot *out)
{memset(out,0,sizeof(*out));out->running_mask=1;out->run_id=1;out->input.now_ms=tick;out->mecanum_mode=mecanum_mode;}
static void input(const char *s){Campus_UsbOnReceive((const uint8_t *)s,(uint16_t)strlen(s));}
static void run(unsigned n)
{while(n--){++tick;bus_step();Servo_Process(tick);Campus_UsbProcess();}}
static ServoState state(uint8_t id){ServoState s;assert(Servo_CopyState(id,&s));return s;}

static void protocol(void)
{
    char f[65];ZpReply r;
    assert(Zp_Build(f,sizeof(f),ZP_MOVE,1,500,20)==15&&!strcmp(f,"#001P0500T0020!"));
    assert(!Zp_Build(f,5,ZP_MOVE,1,1500,1000));
    assert(!Zp_Build(f,sizeof(f),ZP_MOVE,255,1500,1000));
    assert(!Zp_Build(f,sizeof(f),ZP_MOVE,1,499,1000));
    assert(Zp_Parse("#001P1500!",10,&r)&&r.value==1500&&Zp_Matches(ZP_POSITION,1,&r));
    assert(!Zp_Matches(ZP_POSITION,2,&r));
    assert(!Zp_Parse("#001P9999!",10,&r));
    assert(!Zp_Parse("#001P15x0!",10,&r));
    assert(Zp_Parse("#001PMOD1!",10,&r)&&r.type==ZP_REPLY_MODE);
    assert(Zp_Parse("#OK!",4,&r)&&Zp_Matches(ZP_STOP,1,&r)&&!Zp_Matches(ZP_POSITION,1,&r));
}
int main(void)
{
    protocol();assert(Servo_Init());uint32_t ticket;
    if(!SERVO_BUS_ENABLE){run(500);assert(!tx_count&&Servo_MovePulse(1,1500,1000,&ticket)==SERVO_DISABLED);puts("PASS: servo disabled, no bus traffic.");return 0;}
    assert(Servo_MovePulse(1,1500,1000,&ticket)==SERVO_OFFLINE);
    run(600);
    for(unsigned i=1;i<=5;++i){ServoState s=state((uint8_t)i);assert(s.online&&s.position==1500&&s.mode_valid&&s.mode==1&&s.version_valid);}
    assert(moves==0); /* Startup only queried, no movement/config writes. */
    assert(Servo_MovePulse(1,499,1000,&ticket)==SERVO_RANGE);
    assert(Servo_MovePulse(1,1500,10000,&ticket)==SERVO_RANGE);
    assert(Servo_RequestPosition(255,&ticket)==SERVO_INVALID);
    assert(Servo_MovePulse(1,1600,1000,&ticket)==SERVO_QUEUED);
    assert(Servo_MovePulse(1,1700,1000,&ticket)==SERVO_QUEUED);
    run(80);assert(moves==1&&state(1).position==1700&&state(1).last_result==SERVO_OK);
    assert(Servo_MovePulse(1,2000,1000,&ticket)==SERVO_QUEUED);
    assert(Servo_Stop(1,&ticket)==SERVO_QUEUED);run(80);assert(moves==1&&state(1).last_result==SERVO_OK);
    reply_mask&=~1U;run(600);assert(!state(1).online&&state(2).online);
    assert(Servo_MovePulse(1,1800,1000,&ticket)==SERVO_OFFLINE);
    reply_mask|=1U;run(1500);assert(state(1).online&&state(1).mode_valid&&moves==1);
    while(tx_pending)run(1);
    feed("noise#999P1500!#001Pxxx0!");run(2);
    ServoSnapshot ss;Servo_CopySnapshot(&ss);assert(ss.malformed>=2);
    char long_frame[300];memset(long_frame,'a',sizeof(long_frame));long_frame[0]='#';long_frame[299]=0;
    while(tx_pending)run(1);
    feed(long_frame);run(50);Servo_CopySnapshot(&ss);assert(ss.uart.overflows>0&&ss.uart.ready);
    fail_tx=true;assert(Servo_MovePulse(1,1750,1000,&ticket)==SERVO_QUEUED);run(80);
    assert(moves==1); /* No automatic replay of a failed motion request. */
    drop_tc=true;assert(Servo_RequestPosition(2,&ticket)==SERVO_QUEUED);run(100);drop_tc=false;run(300);
    Servo_CopySnapshot(&ss);assert(ss.device[1].timeouts>0&&ss.uart.ready);
    run(1000);
    usb.ready=true;
    assert(FlashTasks_AcquireUsb(FLASH_USB_LOG));
    input("servo_sta");input("tus\r\n");run(30);assert(ServoUsb_Busy()&&output_len==0);
    FlashTasks_ReleaseUsb(FLASH_USB_LOG);run(80);
    assert(strstr(output,"SERVO_STATUS BEGIN")&&strstr(output,"SERVO_STATUS END")&&strstr(output,"id=005"));
    assert(!strstr(output,"report_too_large"));
    /* Twelve-column motor telemetry must pause for a complete servo report. */
    mecanum_mode=true;run(20);output_len=0;output[0]=0;run(20);
    assert(strstr(output,"0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000\n"));
    input("servo_status\n");run(80);
    const char *report_begin=strstr(output,"SERVO_STATUS BEGIN");
    const char *report_end=strstr(output,"SERVO_STATUS END");
    assert(report_begin&&report_end&&report_end>report_begin);
    assert(!memchr(report_begin,',',(size_t)(report_end-report_begin)));
    assert(strstr(report_end,"0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000,0.000\n"));
    output_len=0;output[0]=0;
    input("servo_move 001 1500 1000\n");run(100);assert(strstr(output,"SERVO QUEUED")&&state(1).target==1500);
    unsigned before=moves;input("servo_move 001 1500 1000 extra\n");run(50);assert(moves==before&&strstr(output,"SERVO INVALID"));
    usb_hold=true;
    for(unsigned i=0;i<4;++i)input("servo_status\n");
    input("servo_move 001 1900 1000\n");run(5);usb_hold=false;run(100);
    assert(moves==before&&strstr(output,"SERVO BUSY rejected=1"));
    usb_hold=true;input("servo_status\n");run(5);assert(ServoUsb_Busy());
    usb.ready=false;run(1);usb_len=0;++usb.session_id;usb.ready=true;usb_hold=false;output_len=0;output[0]=0;run(20);
    assert(!ServoUsb_Busy()&&!strstr(output,"SERVO_STATUS"));
    usb.ready=false;run(1);
    /* Continuous updates cannot starve feedback or make healthy devices go offline. */
    for(unsigned n=0;n<2000;++n) {
        if(n%20U==0U)for(unsigned id=1;id<=5;++id)
            assert(Servo_MovePulse((uint8_t)id,(uint16_t)(1500+n%100U),100,&ticket)==SERVO_QUEUED);
        run(1);
        for(unsigned id=1;id<=5;++id)assert(state((uint8_t)id).online);
    }
    modes[0]=5;assert(Servo_Init());run(600);
    assert(state(1).online&&Servo_MovePulse(1,1500,1000,&ticket)==SERVO_BAD_MODE);
    modes[0]=1;
    /* The tick counter may wrap while queries and queue deadlines are active. */
    tick=UINT32_MAX-100;assert(Servo_Init());run(600);assert(state(5).online);
    puts("PASS: servo protocol, UART7 TC direction/fast replies, startup queries, limits/coalescing/stop, offline/reconnect, overflow/errors/timeouts, USB arbitration/session and tick wrap.");
    return 0;
}
