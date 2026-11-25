/* 复用确定性 UART7 从机、软中断和时钟，对实际应用/打包代码执行按毫秒的测试。 */
#define main legacy_servo_main
#include "servo_test.c"
#undef main
#include "robot_control.h"
#include "mecanum.h"
#include "motor_debug_config.h"
#include <math.h>

static Fxi6x_Data rc = {.valid=true, .received=true};
static unsigned grip_stops;
static void operate(unsigned duration)
{
    for (unsigned i=0; i<duration; ++i) {
        if (tick%10U==0) {
            rc.last_valid_ms=tick; ++rc.frame_count; Fxi6x_OnFrame(&rc);
        }
        if(tx_pending && strcmp(tx,"#005PDST!")==0) ++grip_stops;
        run(1);
    }
}
static void wheels(void)
{
    RobotControlState s={.mode=ROBOT_DRIVE,.ready=true};
    s.rc.rocker_r_h=660;
    float v[4], cap=MOTOR_DEBUG_RPM_TO_RAD_S(ROBOT_CHASSIS_MAX_WHEEL_RPM);
    Mecanum_RobotTargets(&s,cap,v);
    float upper=cap*ROBOT_UPPER_TURN_FRACTION,lower=cap*ROBOT_LOWER_TURN_FRACTION;
    if (ROBOT_WHEELBASE_M>0 && ROBOT_TRACK_M>0) {
        float conv=(3.14159265f/180.0f)*(ROBOT_WHEELBASE_M+ROBOT_TRACK_M)/(2.0f*ROBOT_WHEEL_RADIUS_M);
        upper=conv*ROBOT_UPPER_MAX_YAW_DEG_S;lower=conv*ROBOT_LOWER_MAX_YAW_DEG_S;
    }
    assert(fabsf(v[0]-upper)<0.001f && v[1]==-v[0] && v[2]==v[0] && v[3]==-v[0]);
    s.mode=ROBOT_WRIST; s.rc.rocker_l_v=660; s.rc.rocker_l_h=660;
    Mecanum_RobotTargets(&s,cap,v);
    assert(fabsf(v[0]-lower)<0.001f && v[1]==-v[0] && v[2]==v[0] && v[3]==-v[0]);
    s.mode=ROBOT_DRIVE; s.rc.rocker_l_h=s.rc.rocker_r_h=0; s.rc.rocker_l_v=1;
    Mecanum_RobotTargets(&s,cap,v); assert(v[0]>0 && v[0]==v[3]);
    s.mode=ROBOT_ARM; Mecanum_RobotTargets(&s,cap,v); assert(v[0]==0 && v[1]==0 && v[2]==0 && v[3]==0);
    MotorDebug d;MotorDebug_Init(&d);
    MotorDebugConfig config=MotorDebug_WheelConfig(1); config.enabled=true;
    MotorDebugInput in={.rc_received=true,.rc_valid=true,.motor_received=true,.robot_drive_enabled=true,
        .can_ok=true,.enable_switch=1,.temperature=30,.period_ms=1,.now_ms=1,.stick=600,.target_rad_s=cap};
    MotorDebug_Step(&d,&config,&in);assert(d.state==MOTOR_DEBUG_RUNNING && d.command_raw>0);
    in.now_ms=100;MotorDebug_Step(&d,&config,&in);assert(d.command_raw==0);
}
int main(void)
{
    wheels();
    rc.switch_c=ROBOT_SWITCH_DOWN;rc.rocker_r_v=660;
    assert(Servo_Init());operate(850);
    assert(positions[4]==1500 && moves==0);
    rc.rocker_r_v=0;operate(50); rc.rocker_r_v=330;operate(3500);
    assert(positions[4]==2000);
    /* 已上队列但还未发出的目标不能在切档后冒出来；也不能用测得的夹持位置覆盖目标。 */
    assert(Servo_RcMovePulse(5,2200,20,NULL)==SERVO_QUEUED);
    rc.switch_c=ROBOT_SWITCH_UP;rc.last_valid_ms=tick;++rc.frame_count;Fxi6x_OnFrame(&rc);
    rc.rocker_r_v=-660;operate(300);
    assert(positions[4]==2000 && grip_stops==0);
    positions[4]=1800; /* 模拟物体阻挡，协议反馈位置偏离已经发送的 P2000。 */
    rc.switch_c=ROBOT_SWITCH_DOWN;operate(300);
    assert(positions[4]==1800 && state(5).target==2000);
    rc.rocker_r_v=-332;operate(100);assert(state(5).target==2000);
    rc.rocker_r_v=-328;operate(100);rc.rocker_r_v=0;operate(3500);
    assert(positions[4]==1500);
    rc.rocker_r_v=-660;operate(7000);
    assert(positions[4]==2500);
    rc.rocker_l_v=1;rc.rocker_l_h=-1;operate(7000);
    assert(positions[0]>1500 && positions[3]<1500 && positions[1]==1500 && positions[2]==1500);
    rc.switch_c=ROBOT_SWITCH_MIDDLE;rc.rocker_l_v=-300;rc.rocker_l_h=300;operate(750);
    assert(positions[2]<1500 && positions[1]>1500 && positions[4]==2500 && !grip_stops);
    assert(!depth);
    puts("PASS: robot turn limits, four-wheel PI, wrist-only turn, mirrored gripper pickup/hold, zero deadband");
    return 0;
}
