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

static void reset(void)
{
    if(owned)FlashTasks_ReleaseUsb(FLASH_USB_SERVO);
    owned=pending=false; head=tail=count=offset=chunk=rejected=0;
    mode_ticket=0;
}
static bool update_session(UsbSerialStatus *s)
{
    UsbSerial_GetStatus(s);
    if(!session_known || s->session_id!=session || !s->ready) {
        reset(); session=s->session_id; session_known=true;
    }
    return s->ready;
}
bool ServoUsb_Busy(void) { return count!=0U || rejected!=0U || mode_ticket!=0U; }
static void commit(unsigned length)
{
    lengths[head]=(uint16_t)length; head=(head+1U)%REPLIES; ++count;
}
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
static void commit_report(const ServoModeReport *modes)
{
    unsigned n=report(replies[head],modes);
    if(!n) { strcpy(replies[head],"SERVO ERROR report_too_large\n"); n=(unsigned)strlen(replies[head]); }
    commit(n);
}
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
bool ServoUsb_Dispatch(const uint8_t *bytes,uint16_t length)
{
    if(length<6U || memcmp(bytes,"servo_",6)!=0)return false;
    UsbSerialStatus usb; if(!update_session(&usb))return true;
    if(count+(mode_ticket!=0U)>=REPLIES) { ++rejected; return true; }
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
    return true;
}
void ServoUsb_Process(void)
{
    UsbSerialStatus usb; if(!update_session(&usb))return;
    if(pending) {
        UsbSerialTxState state=UsbSerial_GetTxState(&ticket);
        if(state==USB_TX_PENDING)return;
        if(state!=USB_TX_COMPLETE) { reset(); return; }
        pending=false; offset+=chunk;
        if(offset==lengths[tail]) {
            tail=(tail+1U)%REPLIES; --count; offset=0;
            FlashTasks_ReleaseUsb(FLASH_USB_SERVO); owned=false;
        }
    }
    ServoModeReport modes;
    if(mode_ticket && Servo_CopyModeReport(mode_ticket,&modes)) {
        mode_ticket=0;
        if(modes.probe)commit_probe_report(&modes);
        else commit_report(&modes);
    }
    if(rejected && count+(mode_ticket!=0U)<REPLIES) {
        int n=snprintf(replies[head],REPORT_SIZE,"SERVO BUSY rejected=%u; commands not enqueued\n",rejected);
        rejected=0; if(n>0 && n<(int)REPORT_SIZE)commit((unsigned)n);
    }
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
