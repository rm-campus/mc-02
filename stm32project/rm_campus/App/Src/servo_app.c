#include "servo_app.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>

static const ServoConfig configs[SERVO_COUNT] = {SERVO_DEVICE_TABLE};
typedef struct { uint8_t slot; ZpOperation op; uint16_t position, time_ms; uint32_t ticket; } Request;
typedef enum { IDLE, TX, REPLY } Phase;
static Request queue[SERVO_QUEUE_SIZE], current;
static uint16_t queued;
static uint32_t sequence;
static ServoSnapshot work, published;
static uint32_t next_poll[SERVO_COUNT], next_move[SERVO_COUNT], next_slot, deadline, guard, request_started;
static uint8_t boot_phase[SERVO_COUNT], poll_slot;
static Phase phase;
static bool auto_request, hardware_ready;
static char line[ZP_FRAME_MAX];
static uint16_t line_length;
static osThreadId_t thread;

static bool due(uint32_t now, uint32_t when) { return (int32_t)(now-when)>=0; }
static int slot_for(uint8_t id)
{
    for(unsigned i=0;i<SERVO_COUNT;++i)if(configs[i].id==id)return (int)i;
    return -1;
}
const ServoConfig *Servo_GetConfig(unsigned i) { return i<SERVO_COUNT ? &configs[i] : NULL; }
const char *Servo_ResultName(ServoResult r)
{
    static const char *names[]={"OK","QUEUED","BUSY","INVALID","RANGE","OFFLINE","BAD_MODE",
        "NOT_READY","DISABLED","TX_ERROR","UART_ERROR","TIMEOUT"};
    return (unsigned)r<sizeof(names)/sizeof(names[0]) ? names[r] : "UNKNOWN";
}
void Servo_CopySnapshot(ServoSnapshot *out)
{
    if(out) { taskENTER_CRITICAL(); *out=published; taskEXIT_CRITICAL(); }
}
bool Servo_CopyState(uint8_t id, ServoState *out)
{
    int i=slot_for(id); if(i<0 || !out)return false;
    taskENTER_CRITICAL(); *out=published.device[i]; taskEXIT_CRITICAL(); return true;
}
static void publish(uint32_t now)
{
    work.now_ms=now; ServoUart_CopyStatus(&work.uart);
    taskENTER_CRITICAL(); work.queued=queued; published=work; taskEXIT_CRITICAL();
}
static void remove_at(unsigned i)
{
    for(unsigned j=i+1;j<queued;++j)queue[j-1]=queue[j];
    --queued;
}
static void cancel_moves(unsigned slot)
{
    taskENTER_CRITICAL();
    for(unsigned i=0;i<queued;) {
        if(queue[i].slot==slot && (queue[i].op==ZP_MOVE || queue[i].op==ZP_RESUME || queue[i].op==ZP_TORQUE_ON)) {
            remove_at(i); ++work.cancelled;
        } else ++i;
    }
    taskEXIT_CRITICAL();
}
static ServoResult enqueue(uint8_t id, ZpOperation op, uint16_t p, uint16_t ms, uint32_t *ticket)
{
    if(ticket)*ticket=0;
    if(!SERVO_BUS_ENABLE)return SERVO_DISABLED;
    int slot=slot_for(id); if(slot<0)return SERVO_INVALID;
    if(op==ZP_MOVE && (p<configs[slot].minimum || p>configs[slot].maximum || ms>9999U))return SERVO_RANGE;
    taskENTER_CRITICAL();
    ServoState *s=&published.device[slot];
    ServoResult result=SERVO_QUEUED;
    bool motion=op==ZP_MOVE || op==ZP_RESUME || op==ZP_TORQUE_ON;
    if(!published.initialized)result=SERVO_NOT_READY;
    else if(motion && (!s->online || (uint32_t)(HAL_GetTick()-s->position_ms)>=SERVO_POSITION_STALE_MS))result=SERVO_OFFLINE;
    else if(op==ZP_MOVE && (!s->mode_valid || s->mode<1U || s->mode>4U))result=SERVO_BAD_MODE;
    if(result!=SERVO_QUEUED) { taskEXIT_CRITICAL(); return result; }
    if(op==ZP_STOP || op==ZP_TORQUE_OFF)cancel_moves((unsigned)slot);
    int replace=-1;
    if(op==ZP_MOVE)for(unsigned i=0;i<queued;++i)if(queue[i].slot==(uint8_t)slot && queue[i].op==ZP_MOVE) { replace=(int)i; break; }
    if(replace<0 && queued==SERVO_QUEUE_SIZE) { taskEXIT_CRITICAL(); return SERVO_BUSY; }
    if(++sequence==0U)++sequence;
    Request r={(uint8_t)slot,op,p,ms,sequence};
    if(replace>=0) { queue[replace]=r; ++work.coalesced; }
    else queue[queued++]=r;
    if(ticket)*ticket=sequence;
    taskEXIT_CRITICAL();
    if(thread)(void)osThreadFlagsSet(thread,SERVO_TASK_FLAG);
    return SERVO_QUEUED;
}
ServoResult Servo_MovePulse(uint8_t id,uint16_t p,uint16_t ms,uint32_t *t) { return enqueue(id,ZP_MOVE,p,ms,t); }
ServoResult Servo_RequestPosition(uint8_t id,uint32_t *t) { return enqueue(id,ZP_POSITION,0,0,t); }
ServoResult Servo_Stop(uint8_t id,uint32_t *t) { return enqueue(id,ZP_STOP,0,0,t); }
ServoResult Servo_SetTorque(uint8_t id,bool on,uint32_t *t) { return enqueue(id,on?ZP_TORQUE_ON:ZP_TORQUE_OFF,0,0,t); }
ServoResult Servo_Pause(uint8_t id,uint32_t *t) { return enqueue(id,ZP_PAUSE,0,0,t); }
ServoResult Servo_Resume(uint8_t id,uint32_t *t) { return enqueue(id,ZP_RESUME,0,0,t); }

bool Servo_Init(void)
{
    thread=osThreadGetId();
    memset(&work,0,sizeof(work)); work.enabled=SERVO_BUS_ENABLE!=0;
    uint32_t now=HAL_GetTick();
    taskENTER_CRITICAL(); queued=0; sequence=0; taskEXIT_CRITICAL();
    phase=IDLE; line_length=0; poll_slot=0; next_slot=guard=now;
    for(unsigned i=0;i<SERVO_COUNT;++i) {
        work.device[i].id=configs[i].id;
        next_poll[i]=next_move[i]=now; boot_phase[i]=0;
        if(configs[i].id==255U || configs[i].minimum<500U || configs[i].maximum>2500U || configs[i].minimum>configs[i].maximum) {
            publish(now); return false;
        }
        for(unsigned j=0;j<i;++j)if(configs[i].id==configs[j].id) { publish(now); return false; }
    }
    work.initialized=SERVO_BUS_ENABLE!=0;
    hardware_ready=SERVO_BUS_ENABLE ? ServoUart_Init() : false;
    publish(now); return !SERVO_BUS_ENABLE || hardware_ready;
}

static void finish(ServoResult result,uint32_t now,const ZpReply *reply)
{
    ServoState *s=&work.device[current.slot];
    if(current.ticket) { s->last_ticket=current.ticket; s->last_result=result; }
    if(result==SERVO_OK) {
        if(current.op==ZP_MOVE)s->target=current.position;
        if(reply) {
            ++s->replies;
            switch(reply->type) {
            case ZP_REPLY_ID: s->id_seen=true; break;
            case ZP_REPLY_VERSION: memcpy(s->version,reply->version,sizeof(s->version)); s->version_valid=true; break;
            case ZP_REPLY_MODE: s->mode=(uint8_t)reply->value; s->mode_valid=true; break;
            case ZP_REPLY_POSITION:
                /* Recheck metadata after an offline device reappears. No stale moves are replayed. */
                if(s->position_seen && !s->online)boot_phase[current.slot]=0;
                s->position=reply->value; s->position_ms=now; s->position_seen=true; s->online=true; s->failures=0;
                break;
            default: break;
            }
        }
    } else {
        if(result==SERVO_TIMEOUT)++s->timeouts;
        if(current.op==ZP_POSITION && s->failures<255U)++s->failures;
    }
    if(auto_request && current.op!=ZP_POSITION && boot_phase[current.slot]<3U)++boot_phase[current.slot];
    /* Repeated movement commands must not postpone feedback polling indefinitely. */
    if(current.op>=ZP_POSITION && current.op<=ZP_MODE)
        next_poll[current.slot]=s->failures>=SERVO_FAIL_LIMIT ? now+SERVO_OFFLINE_RETRY_MS : request_started+SERVO_POLL_PERIOD_MS;
    if(result!=SERVO_OK) {
        hardware_ready=ServoUart_Recover();
        guard=now+SERVO_RECOVERY_QUIET_MS;
    } else guard=now+SERVO_BUS_GAP_MS;
    phase=IDLE; line_length=0;
}

static void input_byte(uint8_t c,uint32_t now)
{
    if(c=='#')line_length=0;
    else if(!line_length)return;
    if(line_length>=sizeof(line)) { ++work.malformed; line_length=0; return; }
    line[line_length++]=(char)c;
    if(c=='!') {
        ZpReply r;
        if(!Zp_Parse(line,line_length,&r))++work.malformed;
        else if(phase!=REPLY || !Zp_Matches(current.op,configs[current.slot].id,&r))++work.unexpected;
        else finish(SERVO_OK,now,&r);
        line_length=0;
    }
}

static bool take_request(uint32_t now)
{
    taskENTER_CRITICAL();
    int take=-1;
    for(unsigned i=0;i<queued;++i)if(queue[i].op==ZP_STOP || queue[i].op==ZP_TORQUE_OFF) { take=(int)i; break; }
    if(take<0)for(unsigned i=0;i<queued;++i)if(queue[i].op!=ZP_MOVE || due(now,next_move[queue[i].slot])) { take=(int)i; break; }
    if(take>=0) { current=queue[take]; remove_at((unsigned)take); auto_request=false; }
    taskEXIT_CRITICAL();
    if(take>=0)return true;
    if(!due(now,next_slot))return false;
    next_slot=now+SERVO_POLL_SLOT_MS;
    for(unsigned n=0;n<SERVO_COUNT;++n) {
        unsigned i=poll_slot; poll_slot=(uint8_t)((poll_slot+1U)%SERVO_COUNT);
        if(!due(now,next_poll[i]))continue;
        const ZpOperation ops[]={ZP_ID,ZP_VERSION,ZP_MODE,ZP_POSITION};
        ZpOperation op=ops[boot_phase[i]];
        /* Metadata discovery must not starve fresh position polling after reconnect. */
        if(work.device[i].position_seen && (uint32_t)(now-work.device[i].position_ms)>=SERVO_POSITION_STALE_MS/2U)
            op=ZP_POSITION;
        current=(Request){.slot=(uint8_t)i,.op=op}; auto_request=true; return true;
    }
    return false;
}

void Servo_Process(uint32_t now)
{
    if(!work.initialized || !SERVO_BUS_ENABLE)return;
    for(unsigned i=0;i<SERVO_COUNT;++i) {
        ServoState *s=&work.device[i];
        if(s->online && (s->failures>=SERVO_FAIL_LIMIT || (uint32_t)(now-s->position_ms)>=SERVO_POSITION_STALE_MS)) {
            s->online=false; s->mode_valid=false; boot_phase[i]=3;
            cancel_moves(i);
        }
    }
    uint32_t events=ServoUart_TakeEvents();
    if(events&SERVO_UART_ERROR) {
        if(phase!=IDLE)finish(SERVO_UART_ERROR_RESULT,now,NULL);
        else { hardware_ready=ServoUart_Recover(); guard=now+SERVO_RECOVERY_QUIET_MS; line_length=0; }
    }
    if(phase==TX && (events&SERVO_UART_TX_DONE)) {
        if(!Zp_ExpectsReply(current.op))finish(SERVO_OK,now,NULL);
        else { ServoUartStatus p; ServoUart_CopyStatus(&p); phase=REPLY; deadline=p.tx_done_ms+SERVO_REPLY_TIMEOUT_MS; }
    }
    uint8_t data[64];
    for(unsigned batch=0;batch<4;++batch) {
        uint16_t n=ServoUart_Read(data,sizeof(data));
        for(unsigned i=0;i<n;++i)input_byte(data[i],now);
        if(n<sizeof(data))break;
    }
    if(phase!=IDLE && due(now,deadline))finish(SERVO_TIMEOUT,now,NULL);
    if(phase==IDLE && due(now,guard)) {
        ServoUartStatus p; ServoUart_CopyStatus(&p);
        if(!hardware_ready || !p.ready) {
            hardware_ready=ServoUart_Recover(); guard=now+SERVO_RECOVERY_QUIET_MS;
        } else if((uint32_t)(now-p.last_rx_ms)>=SERVO_BUS_GAP_MS && take_request(now)) {
            request_started=now;
            ServoState *s=&work.device[current.slot];
            bool motion=current.op==ZP_MOVE || current.op==ZP_RESUME || current.op==ZP_TORQUE_ON;
            if(motion && !s->online)finish(SERVO_OFFLINE,now,NULL);
            else if(current.op==ZP_MOVE && (!s->mode_valid || s->mode>4U))finish(SERVO_BAD_MODE,now,NULL);
            else {
                char frame[ZP_FRAME_MAX];
                size_t n=Zp_Build(frame,sizeof(frame),current.op,s->id,current.position,current.time_ms);
                line_length=0;
                if(!n || !ServoUart_Send(frame,(uint16_t)n))finish(SERVO_TX_ERROR,now,NULL);
                else {
                    phase=TX; deadline=now+SERVO_TX_TIMEOUT_MS;
                    if(current.op==ZP_MOVE)next_move[current.slot]=now+SERVO_MOVE_INTERVAL_MS;
                    if(current.ticket) { s->last_ticket=current.ticket; s->last_result=SERVO_QUEUED; }
                }
            }
        }
    }
    publish(now);
}

void Campus_ServoTask(void *argument)
{
    (void)argument; (void)Servo_Init();
    for(;;) {
        Servo_Process(HAL_GetTick());
        (void)osThreadFlagsWait(SERVO_TASK_FLAG,osFlagsWaitAny, SERVO_BUS_ENABLE ? 1U : 1000U);
    }
}
