#include "servo_usb.h"
#if SERVO_USB_DEBUG_ENABLE
#include "servo_app.h"
#include "servo_rc.h"
#include "flash_tasks.h"
#include "usb_serial.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>
#define REPLIES 4U
#define REPORT_SIZE 2048U
static char replies[REPLIES][REPORT_SIZE];
static uint16_t lengths[REPLIES];
static unsigned head,tail,count,offset,chunk,rejected;
static bool owned,pending,session_known;
static uint32_t session;
static uint32_t mode_ticket;
static UsbSerialTxTicket ticket;
#if SERVO_RX_TRACE_ENABLE
/* 已取的块一直保留到 USB 确认发送完成；断线后可凭 seq 识别重发。 */
static ServoRxTraceBlock trace_block;
static UsbSerialTxTicket trace_ticket;
static bool trace_held, trace_in_flight;
static uint32_t trace_next_offset;

static void check_trace_completion(void)
{
    if(!trace_in_flight)return;
    UsbSerialTxState state=UsbSerial_GetTxState(&trace_ticket);
    if(state==USB_TX_PENDING)return;
    trace_in_flight=false;
    if(state==USB_TX_COMPLETE) {
        trace_next_offset=trace_block.offset+trace_block.length;
        trace_held=false;
    }
}

static bool send_trace(const UsbSerialStatus *usb)
{
    if(trace_in_flight || usb->tx_buffered)return false;
    if(!trace_held)trace_held=ServoUart_TakeTrace(&trace_block);
    if(!trace_held)return false;

    char timing[48]="tc_ms=NA dt_ms=NA";
    if(trace_block.last_tx[0])
        snprintf(timing,sizeof(timing),"tc_ms=%lu dt_ms=%lu",(unsigned long)trace_block.tx_done_ms,
                 (unsigned long)(trace_block.first_ms-trace_block.tx_done_ms));
    char line[192U+ZP_FRAME_MAX+4U*SERVO_TRACE_BYTES];
    int n=snprintf(line,sizeof(line),"SERVO_RX seq=%lu n=%u first_ms=%lu last_ms=%lu tx=%s %s lost=%lu HEX=",
        (unsigned long)trace_block.offset,(unsigned)trace_block.length,(unsigned long)trace_block.first_ms,
        (unsigned long)trace_block.last_ms,trace_block.last_tx[0]?trace_block.last_tx:"NA",timing,
        (unsigned long)(trace_block.offset-trace_next_offset));
    if(n<0 || (size_t)n+4U*trace_block.length+9U>sizeof(line))return false;
    static const char digits[]="0123456789ABCDEF";
    size_t used=(size_t)n;
    for(unsigned i=0;i<trace_block.length;++i) {
        uint8_t b=trace_block.bytes[i];
        if(i)line[used++]=' ';
        line[used++]=digits[b>>4];line[used++]=digits[b&15U];
    }
    memcpy(line+used," ASCII=",7U);used+=7U;
    for(unsigned i=0;i<trace_block.length;++i) {
        uint8_t b=trace_block.bytes[i];
        line[used++]=(b>=32U && b<=126U)?(char)b:'.';
    }
    line[used++]='\n';
    taskENTER_CRITICAL();
    UsbSerialStatus current;
    UsbSerial_GetStatus(&current);
    trace_in_flight=current.ready && current.session_id==usb->session_id &&
        UsbSerial_SendTracked(line,(uint16_t)used,&trace_ticket)==USB_SERIAL_OK;
    taskEXIT_CRITICAL();
    return trace_in_flight;
}
#endif
#if SERVO_JOG_TEST_ENABLE
/* USB 任务独占 FIFO，队头的点动收到实测结果后才出队。 */
#define JOG_QUEUE_SIZE 8U
typedef struct { uint8_t id, direction; uint32_t ticket; } JogCommand;
static JogCommand jog_queue[JOG_QUEUE_SIZE];
static unsigned jog_first, jog_count;
static uint32_t jog_sequence;
static bool jog_active;
#endif

static void reset(void)
{
    if(owned)FlashTasks_ReleaseUsb(FLASH_USB_SERVO);
    owned=pending=false; head=tail=count=offset=chunk=rejected=0;
    mode_ticket=0;
#if SERVO_JOG_TEST_ENABLE
    jog_first=jog_count=0;
    jog_active=false;
    ServoJog_Cancel();
#endif
}
static bool update_session(UsbSerialStatus *s)
{
    UsbSerial_GetStatus(s);
    if(!session_known || s->session_id!=session || !s->ready) {
        reset(); session=s->session_id; session_known=true;
    }
    return s->ready;
}
bool ServoUsb_Busy(void)
{
#if SERVO_JOG_TEST_ENABLE
    if(jog_count)return true;
#endif
    return count!=0U || rejected!=0U || mode_ticket!=0U;
}
static void commit(unsigned length)
{
    lengths[head]=(uint16_t)length; head=(head+1U)%REPLIES; ++count;
}
#if SERVO_JOG_TEST_ENABLE
static void jog_error(uint8_t id,uint32_t request,const char *reason)
{
    int n=snprintf(replies[head],REPORT_SIZE,"SERVO_TEST ERR id=%03u ticket=%lu reason=%s\n",
                   (unsigned)id,(unsigned long)request,reason);
    commit((unsigned)n);
}

static void submit_jog(const uint8_t *bytes,uint16_t length)
{
    /* servo_test_001_1 恰好 16 字节，禁止后缀、空白、嵌入 NUL 和广播地址。 */
    if(length!=16U || bytes[11]!='0' || bytes[12]!='0' || bytes[13]<'1' || bytes[13]>'5' ||
       bytes[14]!='_' || (bytes[15]!='1' && bytes[15]!='2')) {
        jog_error(0,0,"INVALID");
        return;
    }
    const uint8_t id=(uint8_t)(bytes[13]-'0');
    ServoState state;
    if(!Servo_CopyState(id,&state) || !state.position_seen) { jog_error(id,0,"OFFLINE"); return; }
    if(jog_count==JOG_QUEUE_SIZE) { jog_error(id,0,"BUSY"); return; }
    if(++jog_sequence==0U)++jog_sequence;
    JogCommand cmd={id,(uint8_t)(bytes[15]-'0'),jog_sequence};
    jog_queue[(jog_first+jog_count)%JOG_QUEUE_SIZE]=cmd;
    ++jog_count;
    int n=snprintf(replies[head],REPORT_SIZE,"SERVO_TEST QUEUED id=%03u ticket=%lu\n",
                   (unsigned)id,(unsigned long)cmd.ticket);
    commit((unsigned)n);
}

static void process_jog(void)
{
    if(!jog_count || count==REPLIES)return;
    const JogCommand *cmd=&jog_queue[jog_first];
    if(!jog_active) {
        ServoResult rc=ServoJog_Start(cmd->id,cmd->direction,cmd->ticket,session);
        if(rc==SERVO_QUEUED) { jog_active=true; return; }
        jog_error(cmd->id,cmd->ticket,Servo_ResultName(rc));
    } else {
        ServoJogReport r;
        if(!ServoJog_TakeResult(cmd->ticket,&r))return;
        if(strcmp(r.result,"DISCONNECTED")==0) {
            /* USB 任务可能错过短暂挂起；总线任务观察到断开时同样丢弃旧动作。 */
            jog_error(r.id,r.ticket,r.result);
            jog_count=jog_first=0;
            jog_active=false;
            return;
        }
        bool done=strcmp(r.result,"DONE")==0;
        int n=snprintf(replies[head],REPORT_SIZE,"SERVO_TEST %s id=%03u ticket=%lu%s%s start=%u target=%u actual=%u\n",
                       done?"DONE":"ERR",(unsigned)r.id,(unsigned long)r.ticket,
                       done?"":" reason=",done?"":r.result,(unsigned)r.start,(unsigned)r.target,(unsigned)r.actual);
        commit((unsigned)n);
    }
    jog_active=false;
    jog_first=(jog_first+1U)%JOG_QUEUE_SIZE;
    --jog_count;
}
#endif
static unsigned report(char *out, const ServoModeReport *modes)
{
    ServoSnapshot s; Servo_CopySnapshot(&s);
    int n=snprintf(out,REPORT_SIZE,"SERVO_STATUS BEGIN\nenabled=%u init=%u uart_ready=%u queued=%u errors=%lu overflow=%lu malformed=%lu unexpected=%lu coalesced=%lu cancelled=%lu\n",
        s.enabled,s.initialized,s.uart.ready,s.queued,(unsigned long)s.uart.errors,(unsigned long)s.uart.overflows,
        (unsigned long)s.malformed,(unsigned long)s.unexpected,(unsigned long)s.coalesced,(unsigned long)s.cancelled);
    if(n<0 || n>= (int)REPORT_SIZE)return 0;
    unsigned used=(unsigned)n;
    ServoRcStatus rc;ServoRc_CopyStatus(&rc);
    n=snprintf(out+used,REPORT_SIZE-used,"RC enabled=%u config_valid=%u off_seen=%u selected=%u armed=%u fault=%u\n",
        SERVO_RC_ENABLE && SERVO_BUS_ENABLE,rc.config_valid,rc.off_seen,rc.selected_mask,rc.armed_mask,rc.fault_mask);
    if(n<0 || (unsigned)n>=REPORT_SIZE-used)return 0;
    used+=(unsigned)n;
    if(modes && modes->requested_mode) {
        n=snprintf(out+used,REPORT_SIZE-used,"requested_mode=%u\n",modes->requested_mode);
        if(n<0 || (unsigned)n>=REPORT_SIZE-used)return 0;
        used+=(unsigned)n;
    }
    for(unsigned i=0;i<SERVO_COUNT;++i) {
        const ServoState *d=&s.device[i]; const ServoConfig *c=Servo_GetConfig(i);
        char position[12]="NA",age[12]="NA";
        if(d->position_seen) { (void)snprintf(position,sizeof(position),"%u",d->position); (void)snprintf(age,sizeof(age),"%lu",(unsigned long)(s.now_ms-d->position_ms)); }
        bool fresh=d->online && d->position_seen && s.now_ms-d->position_ms<SERVO_POSITION_STALE_MS;
        unsigned mode=modes ? modes->mode[i] : d->mode;
        bool mode_valid=modes ? mode!=0U : d->mode_valid;
        const char *check=modes ? Servo_ResultName(modes->result[i]) : "CACHED";
        n=snprintf(out+used,REPORT_SIZE-used,"id=%03u model=%s online=%u position=%s age_ms=%s mode=%u mode_valid=%u mode_check=%s version=%s range=%u..%u target=%u ticket=%lu result=%s timeouts=%lu\n",
            c->id,c->model,fresh,position,age,mode,mode_valid,check,d->version_valid?d->version:"NA",c->minimum,c->maximum,d->target,
            (unsigned long)d->last_ticket,Servo_ResultName(d->last_result),(unsigned long)d->timeouts);
        if(n<0 || (unsigned)n>=REPORT_SIZE-used)return 0;
        used+=(unsigned)n;
    }
    n=snprintf(out+used,REPORT_SIZE-used,"SERVO_STATUS END\n");
    return n>0 && (unsigned)n<REPORT_SIZE-used ? used+(unsigned)n : 0;
}
#if !SERVO_STATUS_ONLY
static bool numbers(const char *p,unsigned *args,unsigned wanted)
{
    for(unsigned i=0;i<wanted;++i) {
        if(*p!=' ')return false;
        while(*p==' ')++p;
        if(*p<'0'||*p>'9')return false;
        unsigned value=0,digits=0;
        do { if(++digits>5U)return false; value=value*10U+(unsigned)(*p++-'0'); } while(*p>='0'&&*p<='9');
        args[i]=value;
    }
    return *p==0;
}
#endif
static void commit_report(const ServoModeReport *modes)
{
    unsigned n=report(replies[head],modes);
    if(!n) { strcpy(replies[head],"SERVO ERROR report_too_large\n"); n=(unsigned)strlen(replies[head]); }
    commit(n);
}
#if !SERVO_STATUS_ONLY
static void commit_probe_report(const ServoModeReport *modes)
{
    const unsigned slot=modes->probe_slot;
    const unsigned id=Servo_GetConfig(slot)->id;
    const uint32_t limit=SERVO_MODE_PROBE_TIMEOUT_MS;
    size_t captured=modes->probe_rx_bytes;
    if(captured>sizeof(modes->probe_raw))captured=sizeof(modes->probe_raw);
    char ascii[ZP_FRAME_MAX+1U], hex[ZP_FRAME_MAX*3U+1U], first[16]="NA";
    static const char digits[]="0123456789ABCDEF";
    for(size_t i=0;i<captured;++i) {
        uint8_t c=modes->probe_raw[i];
        ascii[i]=c>=32U && c<=126U ? (char)c : '.';
        hex[i*3U]=digits[c>>4];
        hex[i*3U+1U]=digits[c&15U];
        hex[i*3U+2U]=' ';
    }
    ascii[captured]=0;
    hex[captured ? captured*3U-1U : 0U]=0;
    if(captured)snprintf(first,sizeof(first),"%lu",(unsigned long)modes->probe_first_ms);
    int n=snprintf(replies[head],REPORT_SIZE,
        "SERVO_MODE_PROBE id=%03u result=%s mode=%u window_ms=%lu rx_bytes=%u first_ms=%s truncated=%u\nASCII %s\nHEX %s\n",
        id,Servo_ResultName(modes->result[slot]),(unsigned)modes->mode[slot],(unsigned long)limit,
        (unsigned)modes->probe_rx_bytes,first,(unsigned)(captured<modes->probe_rx_bytes),
        captured ? ascii : "NA",captured ? hex : "NA");
    if(n>0 && n<(int)REPORT_SIZE)commit((unsigned)n);
}
#endif
bool ServoUsb_Dispatch(const uint8_t *bytes,uint16_t length)
{
#if SERVO_STATUS_ONLY
    static const char command[]="servo_status";
    const bool status_line=length==sizeof(command)-1U && memcmp(bytes,command,sizeof(command)-1U)==0;
#if SERVO_JOG_TEST_ENABLE
    static const char prefix[]="servo_test_";
    const bool jog_line=length>=sizeof(prefix)-1U && memcmp(bytes,prefix,sizeof(prefix)-1U)==0;
    if(!status_line && !jog_line)return false;
#else
    if(!status_line)return false;
#endif
#else
    if(length<6U || memcmp(bytes,"servo_",6)!=0)return false;
#endif
    UsbSerialStatus usb; if(!update_session(&usb))return true;
#if SERVO_JOG_TEST_ENABLE
    /* 高频状态查询应给已接收的运动留出结果回复空间。 */
    if(status_line && jog_count && count>=REPLIES-1U) { ++rejected; return true; }
#endif
    if(count+(mode_ticket!=0U)>=REPLIES) { ++rejected; return true; }
#if SERVO_STATUS_ONLY
#if SERVO_JOG_TEST_ENABLE
    if(jog_line) { submit_jog(bytes,length); return true; }
#endif
    commit_report(NULL);
#else
    char input[64];
    if(length>=sizeof(input)) { strcpy(replies[head],"SERVO INVALID length\n"); commit((unsigned)strlen(replies[head])); return true; }
    memcpy(input,bytes,length); input[length]=0;
    if(strlen(input)!=length) { strcpy(replies[head],"SERVO INVALID character\n"); commit((unsigned)strlen(replies[head])); return true; }
    if(!strcmp(input,"servo_status")) { commit_report(NULL); return true; }
    static const char mode_command[]="servo_position_mode";
    static const char probe_command[]="servo_mode_probe";
    const bool probe=!strncmp(input,probe_command,sizeof(probe_command)-1U);
    if(probe || !strncmp(input,mode_command,sizeof(mode_command)-1U)) {
        const size_t prefix=probe ? sizeof(probe_command)-1U : sizeof(mode_command)-1U;
        unsigned value=0;
        ServoResult result;
        if(!numbers(input+prefix,&value,1U) || (probe ? value>254U : value<1U || value>4U))
            result=SERVO_INVALID;
        else if(mode_ticket || ServoRc_Selected(HAL_GetTick()))result=SERVO_BUSY;
        else if(probe)result=Servo_RequestModeProbe((uint8_t)value,&mode_ticket);
        else result=Servo_RequestModes((uint8_t)value,&mode_ticket);
        if(result==SERVO_QUEUED)return true;
        int n=snprintf(replies[head],REPORT_SIZE,"SERVO %s mode_query\n",Servo_ResultName(result));
        if(n>0 && n<(int)REPORT_SIZE)commit((unsigned)n);
        return true;
    }
    char *space=strchr(input,' '); unsigned args[3]={0},wanted=1; uint32_t id=0;
    ServoResult result=SERVO_INVALID;
    if(space) {
        size_t name_len=(size_t)(space-input);
        if(name_len==10U && !memcmp(input,"servo_move",10))wanted=3;
        else if(name_len==12U && !memcmp(input,"servo_torque",12))wanted=2;
        if(numbers(space,args,wanted) && args[0]<=254U) {
            *space=0;
            if(!strcmp(input,"servo_move") && args[1]<=UINT16_MAX && args[2]<=9999U)
                result=Servo_MovePulse((uint8_t)args[0],(uint16_t)args[1],(uint16_t)args[2],&id);
            else if(!strcmp(input,"servo_read"))result=Servo_RequestPosition((uint8_t)args[0],&id);
            else if(!strcmp(input,"servo_stop"))result=Servo_Stop((uint8_t)args[0],&id);
            else if(!strcmp(input,"servo_pause"))result=Servo_Pause((uint8_t)args[0],&id);
            else if(!strcmp(input,"servo_resume"))result=Servo_Resume((uint8_t)args[0],&id);
            else if(!strcmp(input,"servo_torque") && args[1]<=1U)result=Servo_SetTorque((uint8_t)args[0],args[1]!=0U,&id);
        }
    }
    int n=snprintf(replies[head],REPORT_SIZE,"SERVO %s id=%u ticket=%lu; use servo_status for result\n",Servo_ResultName(result),args[0],(unsigned long)id);
    if(n>0 && n<(int)REPORT_SIZE)commit((unsigned)n);
#endif
    return true;
}
void ServoUsb_Process(void)
{
#if SERVO_RX_TRACE_ENABLE
    check_trace_completion();
    bool finished_reply=false;
#endif
    UsbSerialStatus usb; if(!update_session(&usb))return;
    if(pending) {
        UsbSerialTxState state=UsbSerial_GetTxState(&ticket);
        if(state==USB_TX_PENDING)return;
        if(state!=USB_TX_COMPLETE) { reset(); return; }
        pending=false; offset+=chunk;
        if(offset==lengths[tail]) {
            tail=(tail+1U)%REPLIES; --count; offset=0;
            FlashTasks_ReleaseUsb(FLASH_USB_SERVO); owned=false;
#if SERVO_RX_TRACE_ENABLE
            finished_reply=true;
#endif
        }
    }
#if !SERVO_STATUS_ONLY
    ServoModeReport modes;
    if(mode_ticket && Servo_CopyModeReport(mode_ticket,&modes)) {
        mode_ticket=0;
        if(modes.probe)commit_probe_report(&modes);
        else commit_report(&modes);
    }
#endif
#if SERVO_JOG_TEST_ENABLE
    process_jog();
#endif
    if(rejected && count+(mode_ticket!=0U)<REPLIES) {
        int n=snprintf(replies[head],REPORT_SIZE,"SERVO BUSY rejected=%u; commands not enqueued\n",rejected);
        rejected=0; if(n>0 && n<(int)REPORT_SIZE)commit((unsigned)n);
    }
#if SERVO_RX_TRACE_ENABLE
    /* 状态报告可跨多个 USB 块；只在完整命令回复之间穿插原始接收行。 */
    if((!count || finished_reply) && !owned && send_trace(&usb))return;
#endif
    if(!count)return;
    if(!owned) { if(!FlashTasks_AcquireUsb(FLASH_USB_SERVO))return; owned=true; }
    chunk=lengths[tail]-offset;
    if(chunk>USB_SERIAL_TX_CAPACITY)chunk=USB_SERIAL_TX_CAPACITY;
    taskENTER_CRITICAL();
    UsbSerial_GetStatus(&usb);
    if(usb.ready && usb.session_id==session && UsbSerial_SendTracked(replies[tail]+offset,(uint16_t)chunk,&ticket)==USB_SERIAL_OK)pending=true;
    taskEXIT_CRITICAL();
}
#endif
