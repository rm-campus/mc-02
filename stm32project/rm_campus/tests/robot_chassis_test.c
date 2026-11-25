/* 复用四轮 CAN 仿真，把真正的 CH9 输入送入整机底盘任务和灯光任务。 */
#define main bench_fixture_main
#define vTaskDelayUntil bench_delay
#define Can_Send bench_send
#define osDelay bench_os_delay
#define Led_SetRGB bench_led
#include "four_motor_app_test.c"
#undef main
#undef vTaskDelayUntil
#undef Can_Send
#undef osDelay
#undef Led_SetRGB
#include "robot_control.h"

static void frames(void)
{
    Fxi6x_Data rc={.valid=true,.received=true,.last_valid_ms=now,.frame_count=now+1,
        .switch_a=1,.switch_c=now<100?ROBOT_SWITCH_UP:now<200?ROBOT_SWITCH_MIDDLE:ROBOT_SWITCH_DOWN,
        .rocker_l_v=1,.rocker_l_h=0,.rocker_r_h=now<200?0:15};
    if(now<=300 && now%10U==0) Fxi6x_OnFrame(&rc);
}

bool Can_Send(uint32_t id,const uint8_t *bytes,uint8_t size)
{
    assert(id==0x200 && size==8 && !depth);
    ++tx_count;
    for(unsigned i=0;i<4;++i) {
        int16_t current=(int16_t)((uint16_t)bytes[i*2]<<8|bytes[i*2+1]);
        if(now>=60 && now<100) assert((i%2==0)?current>0:current<0);
        else if(now>=230 && now<400) assert(current>0);
        else assert(current==0);
    }
    return true;
}
void vTaskDelayUntil(TickType_t *wake,TickType_t delta)
{
    *wake+=delta;now=*wake;
    if(now==420)longjmp(done,1);
    frames();
    for(rx_slot=0;rx_slot<4;++rx_slot){pending=1;HAL_FDCAN_RxFifo0Callback(&hfdcan1,0);}
}
static bool tone;
static unsigned beeps;
void Buzzer_Set(bool enabled)
{
    tone=enabled;
    if(enabled)++beeps;
}
bool Led_SetRGB(uint8_t r,uint8_t g,uint8_t b)
{
    RobotControlState s=RobotControl_Get(now);
    if(s.mode==ROBOT_WAIT)assert(r && !g && !b);
    if(s.mode==ROBOT_DRIVE)assert(!r && g && !b);
    if(s.mode==ROBOT_ARM)assert(!r && !g && b);
    if(s.mode==ROBOT_WRIST)assert(!r && g && b);
    return true;
}
int32_t osDelay(uint32_t n)
{
    if(now>=50 && now<150)assert(tone);
    if(now>=150)assert(!tone);
    now+=n;
    frames();
    if(now>=420)longjmp(done,1);
    return 0;
}
int main(void)
{
    Campus_Init();
    if(!setjmp(done))Campus_ChassisTask(NULL);
    assert(tx_count==420 && !depth);
    now=0; Fxi6x_OnStreamBreak(); frames();
    if(!setjmp(done))Campus_StatusTask(NULL);
    assert(beeps==1 && !tone && !depth);
    puts("PASS: one CAN frame per tick, CH7-independent startup, wrist turn, loss at 100ms, mode LEDs and buzzer");
    return 0;
}
