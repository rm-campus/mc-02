/* 复用确定性 UART7 从机、软中断和时钟，对实际应用/打包代码执行按毫秒的测试。 */
#define main legacy_servo_main
#include "servo_test.c"
#undef main
#include "robot_control.h"
#include "mecanum.h"
#include "motor_debug_config.h"
#include <math.h>
#include <stdlib.h>

static Fxi6x_Data rc = {.valid=true, .received=true};
static unsigned grip_stops;

static void wrist_silence(void)
{
    assert(Servo_Init());
    uint32_t ticket=0;
    assert(Servo_RequestPosition(1,&ticket)==SERVO_DISABLED && ticket==0);
    assert(Servo_RequestModeProbe(1,&ticket)==SERVO_DISABLED && ticket==0);
    assert(Servo_RcMovePulse(1,1600,20,&ticket)==SERVO_DISABLED && ticket==0);
    assert(Servo_Stop(1,&ticket)==SERVO_DISABLED && ticket==0);
    assert(Servo_SetTorque(1,true,&ticket)==SERVO_DISABLED && ticket==0);
    const uint8_t ids[]={1};
    const uint16_t positions[]={1500}, times[]={20};
    assert(Servo_ControlMoveGroup(SERVO_OWNER_RC,ids,positions,times,1)==SERVO_DISABLED);
    assert(!Servo_ControlClaim(SERVO_OWNER_RC,1U));
    Servo_RcStopMask(1U);
    assert(Servo_RcStopPending()==0);
    assert(Servo_RequestModes(1,&ticket)==SERVO_QUEUED);
    run(2200);
    ServoModeReport report;
    assert(Servo_CopyModeReport(ticket,&report) && report.result[0]==SERVO_DISABLED);
    assert(!state(1).online && mode_writes[0]==0);
    assert(Servo_MovePulse(3,944,20,NULL)==SERVO_RANGE);
    assert(Servo_MovePulse(3,2056,20,NULL)==SERVO_RANGE);
    puts("PASS: no 001 query/motion/stop/torque/mode frames; shoulder rejects targets outside P945..P2055");
}
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

static void joint_lead_limit(void)
{
    motion_hold_mask=(1U<<2)|(1U<<3);
    rc.switch_c=ROBOT_SWITCH_MIDDLE;
    assert(Servo_Init());
    operate(850);
    assert(state(3).online && state(4).online);

    rc.rocker_l_v=660; rc.rocker_r_h=-660;
    operate(1700);
    assert(positions[2]==1500 && positions[3]==1500);
    assert(state(3).target==1800 && state(4).target==1200);

    rc.rocker_l_v=-660; rc.rocker_r_h=660;
    operate(120);
    assert(state(3).target<1800 && state(4).target>1200);
    operate(1700);
    assert(state(3).target==1200 && state(4).target==1800);

    positions[2]=1460; positions[3]=1540;
    operate(250);
    assert(state(3).target==1160 && state(4).target==1840);

    reply_mask&=~motion_hold_mask;
    operate(1600);
    assert(!state(3).online && !state(4).online);
    rc.rocker_l_v=660; rc.rocker_r_h=-660;
    operate(1600);
    assert(state(3).target==1760 && state(4).target==1240);
    assert(Servo_RcStopPending()==0U);

    positions[2]=1500; positions[3]=1500;
    reply_mask|=motion_hold_mask;
    operate(1600);
    assert(state(3).online && state(4).online);
    assert(state(3).target==1800 && state(4).target==1200);
    puts("PASS: 003 and 004 clamp lead to 300 P, reverse without hidden accumulation, and use cached position during feedback loss");
}

static void wheels(void)
{
    RobotControlState s={.mode=ROBOT_DRIVE,.ready=true};
    s.rc.rocker_r_h=660;
    float v[4], cap=MOTOR_DEBUG_RPM_TO_RAD_S(ROBOT_CHASSIS_MAX_WHEEL_RPM);
    Mecanum_RobotTargets(&s,cap,v);
    float upper=cap*ROBOT_UPPER_TURN_FRACTION;
    if (ROBOT_WHEELBASE_M>0 && ROBOT_TRACK_M>0) {
        float conv=(3.14159265f/180.0f)*(ROBOT_WHEELBASE_M+ROBOT_TRACK_M)/(2.0f*ROBOT_WHEEL_RADIUS_M);
        upper=conv*ROBOT_UPPER_MAX_YAW_DEG_S;
    }
    assert(fabsf(v[0]-upper)<0.001f && v[1]==-v[0] && v[2]==v[0] && v[3]==-v[0]);
    s.mode=ROBOT_WRIST; s.rc.rocker_l_v=660; s.rc.rocker_l_h=660;
    Mecanum_RobotTargets(&s,cap,v);
    assert(v[0]==0 && v[1]==0 && v[2]==0 && v[3]==0);
    const float turn_span=660.0f-MECANUM_TURN_DEADBAND;
    s.mode=ROBOT_DRIVE; s.rc.rocker_l_h=-1; s.rc.rocker_r_h=MECANUM_TURN_DEADBAND;
    s.rc.rocker_l_v=ROBOT_CHASSIS_DEADBAND;
    Mecanum_RobotTargets(&s,cap,v); assert(v[0]==0 && v[1]==0 && v[2]==0 && v[3]==0);
    s.rc.rocker_r_h=MECANUM_TURN_DEADBAND+1;
    Mecanum_RobotTargets(&s,cap,v);
    assert(fabsf(v[0]-upper/turn_span)<0.001f && v[0]>0 && v[1]==-v[0]);
    s.rc.rocker_r_h=0;
    Mecanum_RobotTargets(&s,cap,v); assert(v[0]==0 && v[1]==0 && v[2]==0 && v[3]==0);
    s.rc.rocker_l_v=ROBOT_CHASSIS_DEADBAND+1;
    Mecanum_RobotTargets(&s,cap,v); assert(v[0]>0 && v[0]==v[3]);
    const struct {
        int16_t vertical, horizontal;
        int wheel_direction[4];
    } translations[] = {
        { 660,    0, { 1,  1,  1,  1}}, /* 上推前进 */
        {-660,    0, {-1, -1, -1, -1}}, /* 下推后退 */
        {   0, -660, {-1,  1,  1, -1}}, /* 左推左移 */
        {   0,  660, { 1, -1, -1,  1}}, /* 右推右移 */
        { 660,  660, { 1,  0,  0,  1}}, /* 右上同时前进和右移 */
    };
    for (unsigned i=0; i<sizeof(translations)/sizeof(translations[0]); ++i) {
        s.rc.rocker_l_v=translations[i].vertical;
        s.rc.rocker_l_h=translations[i].horizontal;
        Mecanum_RobotTargets(&s,cap,v);
        for (unsigned j=0; j<4; ++j)
            assert(fabsf(v[j]-cap*translations[i].wheel_direction[j])<0.001f);
    }
    s.mode=ROBOT_ARM; Mecanum_RobotTargets(&s,cap,v); assert(v[0]==0 && v[1]==0 && v[2]==0 && v[3]==0);
    MotorDebug d;MotorDebug_Init(&d);
    MotorDebugConfig config=MotorDebug_WheelConfig(1); config.enabled=true;
    MotorDebugInput in={.rc_received=true,.rc_valid=true,.motor_received=true,.robot_drive_enabled=true,
        .can_ok=true,.enable_switch=1,.temperature=30,.period_ms=1,.now_ms=1,.stick=600,.target_rad_s=cap};
    MotorDebug_Step(&d,&config,&in);assert(d.state==MOTOR_DEBUG_RUNNING && d.command_raw>0);
    in.robot_drive_neutral=true; in.target_rad_s=0.0f;
    in.now_ms=2; in.speed=0.5f;
    MotorDebug_Step(&d,&config,&in);
    assert(d.reference==0.0f && d.ramped==0.0f && d.integral_a==0.0f && d.command_raw<0);
    in.now_ms=3; in.speed=0.0f;
    MotorDebug_Step(&d,&config,&in); assert(d.command_raw==0);
    in.now_ms=100;MotorDebug_Step(&d,&config,&in);assert(d.command_raw==0);
}
static void sparse_bus(void)
{
    /* Match the live setup: only ID1 and ID3 reply; ID1 doesn't reply to PMOD. */
    reply_mask=5U;
    mode_reply_mask&=~1U;
    rc.switch_c=ROBOT_SWITCH_MIDDLE;
    assert(Servo_Init());
    operate(2600);
    assert(state(3).online && !state(2).online);

    rc.rocker_l_v=165;
    for (unsigned i=0; positions[2]==1500 && i<250; ++i) operate(1);
    const uint16_t start=positions[2];
    assert(start>1500);
    uint16_t previous=start;
    uint32_t last_move_tick=tick, max_gap=0;
    for (unsigned i=0; i<3000U; ++i) {
        operate(1);
        if (positions[2]!=previous) {
            const uint32_t gap=tick-last_move_tick;
            if (gap>max_gap) max_gap=gap;
            previous=positions[2];
            last_move_tick=tick;
        }
        assert(state(3).online && tick-state(3).position_ms<100U);
    }
    assert(max_gap<=25U);
    /* Three seconds at quarter stick and 90°/s full scale, in 270° mode. */
    assert(positions[2]-start>=495 && positions[2]-start<=505);
    printf("Sparse bus: max shoulder frame gap %lu ms, advance %u P in 3 s\n",
           (unsigned long)max_gap,(unsigned)(positions[2]-start));
    rc.rocker_l_v=0;
    reply_mask|=2U;
    operate(1600);
    assert(state(2).online);
}

#if SERVO_FASHIONSTAR_GRIPPER
static void fashionstar_gripper(void)
{
    assert(FsServo_PulseFromAngle(-216)==1380U && FsServo_AngleFromPulse(1380)==-216);
    const uint16_t open=FsServo_PulseFromAngle(FS_GRIPPER_OPEN_DDEG);
    const uint16_t halfway=FsServo_PulseFromAngle(FS_GRIPPER_OPEN_DDEG+FS_GRIPPER_TRAVEL_DDEG/2);
    const uint16_t three_quarters=FsServo_PulseFromAngle(FS_GRIPPER_OPEN_DDEG+3*FS_GRIPPER_TRAVEL_DDEG/4);
    const uint16_t closed=FsServo_PulseFromAngle(FS_GRIPPER_OPEN_DDEG+FS_GRIPPER_TRAVEL_DDEG);
    assert(open==2180U && halfway==1890U && closed==1600U);
    /* 夹爪以 30°/s 推进，另给协议切换和反馈一点余量。 */
    const unsigned sweep_ms=(unsigned)abs(FS_GRIPPER_TRAVEL_DDEG)*1000U/300U;
    positions[4]=open;
    reply_mask=4U|8U|16U;
    fs_misses_after_zp=11;
    rc.switch_c=ROBOT_SWITCH_DOWN;
    rc.rocker_r_v=-660;
    assert(Servo_Init());
    operate(1600);
    assert(state(5).online && state(5).id_seen);
    assert(state(5).position==open && fs_queries>0 && fs_moves==0);
    rc.rocker_r_v=0; operate(sweep_ms/2U+600U);
    assert(positions[4]==halfway && state(5).target==halfway && fs_moves>0);
    rc.rocker_r_v=330; operate(sweep_ms/4U+600U);
    assert(positions[4]==three_quarters && state(5).target==three_quarters);
    rc.rocker_r_v=660; operate(sweep_ms/2U+600U);
    assert(positions[4]==closed && state(5).position==closed && state(5).target==closed);
    rc.switch_c=ROBOT_SWITCH_MIDDLE; operate(900);
    assert(positions[4]==closed && fs_holds==0);
    assert(state(3).online && positions[2]==1500 && state(4).online);
    const unsigned queries_before_wrist_move=fs_queries;
    rc.rocker_r_h=330; operate(400);
    const uint16_t wrist=positions[3];
    assert(wrist>1500 && positions[4]==closed && fs_queries==queries_before_wrist_move);
    reply_mask&=~16U;
    rc.switch_c=ROBOT_SWITCH_DOWN; rc.rocker_r_v=-660;
    operate(sweep_ms+900U);
    assert(positions[4]==open && !state(5).online && state(5).target==open);
    assert(positions[2]==1500 && positions[3]==wrist && !depth);
    reply_mask|=16U;
    operate(1700);
    assert(state(5).online && state(5).position==open);
    run(420);
    assert(fs_holds==1U);
    puts("PASS: V1 mapping and polling; gripper reopens after protocol switch without position replies, with RC link-loss hold");
}
#endif

int main(int argc,char **argv)
{
    if (argc==2 && !strcmp(argv[1],"joint-lead-limit")) {
        joint_lead_limit();
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"wrist-silence")) {
        wrist_silence();
        return 0;
    }
#if SERVO_FASHIONSTAR_GRIPPER
    if (argc==2 && !strcmp(argv[1],"fashionstar")) {
        fashionstar_gripper();
        return 0;
    }
#endif
    if (argc==2 && !strcmp(argv[1],"sparse")) {
        sparse_bus();
        return 0;
    }
    wheels();
    rc.switch_c=ROBOT_SWITCH_DOWN;rc.rocker_r_v=660;
    mode_reply_mask&=~2U; /* ID2 只回位置。 */
    modes[2]=5;           /* ID3 回报非定位模式，整机仍下发 P/T。 */
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
    assert(positions[0]==1500 && !state(1).online && positions[3]==1500 &&
           positions[1]==1500 && positions[2]==1500);
    rc.switch_c=ROBOT_SWITCH_MIDDLE;rc.rocker_l_v=-300;rc.rocker_l_h=300;rc.rocker_r_h=-300;operate(750);
    assert(positions[2]<1500 && positions[1]>1500 && positions[3]<1500 && positions[4]==2500 && !grip_stops);
    assert(!state(2).mode_valid && state(3).mode_valid && state(3).mode==5);
    /* 运动发送失败且停止无应答。同档等新位置再自动接管，从新位置继续。 */
    rc.rocker_l_v=0; rc.rocker_l_h=0; rc.rocker_r_h=0; operate(100);
    uint16_t shoulder_before=positions[2];
    fail_tx=true; rc.rocker_l_v=300; operate(500);
    assert(!fail_tx && positions[2]>shoulder_before && Servo_RcInhibited()==0U);
    fail_tx=true;
    for(unsigned elapsed=0;fail_tx && elapsed<200U;++elapsed) operate(1);
    assert(!fail_tx);
    reply_mask&=~4U;
    shoulder_before=positions[2];
    operate(450);
    assert(positions[2]==shoulder_before && !state(3).online && Servo_RcInhibited()==0U);
    const uint16_t shoulder_recovery=SERVO_SHOULDER_MIN_P+16U;
    positions[2]=shoulder_recovery;
    reply_mask|=4U;
    for(unsigned elapsed=0;positions[2]==shoulder_recovery && elapsed<1500U;++elapsed) operate(1);
    assert(positions[2]>shoulder_recovery && positions[2]<shoulder_recovery+20U && Servo_RcInhibited()==0U);
    /* 在中档持续打满肩关节摇杆，分别验证上下限与反向退出。 */
    rc.rocker_l_v=660; operate(7000);
    assert(positions[2]==2055 && state(3).target==2055);
    rc.rocker_l_v=-660; operate(8000);
    assert(positions[2]==945 && state(3).target==945);
    assert(!depth);
    puts("PASS: robot chassis neutral, gripper pickup/hold, mode-independent servo motion and same-gear recovery");
    return 0;
}
