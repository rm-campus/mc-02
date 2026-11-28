/* 复用确定性 UART7 从机、软中断和时钟，对实际应用/打包代码执行按毫秒的测试。 */
#define main legacy_servo_main
#define HAL_UART_Transmit_IT fixture_uart_transmit
#include "servo_test.c"
#undef HAL_UART_Transmit_IT
#undef main
#include "robot_control.h"
#include "mecanum.h"
#include "motor_debug_config.h"
#include <math.h>
#include <stdlib.h>

static Fxi6x_Data rc = {.valid=true, .received=true};
static unsigned grip_stops;

typedef struct {
    uint32_t started;
    RobotMode mode;
    char frame[sizeof(tx)];
} WireFrame;

static WireFrame wire[1024];
static unsigned wire_count;
static bool record_wire;

int HAL_UART_Transmit_IT(UART_HandleTypeDef *h, const uint8_t *p, uint16_t n)
{
    int result=fixture_uart_transmit(h,p,n);
    if (record_wire && result==HAL_OK) {
        assert(wire_count<sizeof(wire)/sizeof(wire[0]));
        WireFrame *entry=&wire[wire_count++];
        entry->started=tick;
        entry->mode=RobotControl_Get(tick).mode;
        memcpy(entry->frame,p,n);
        entry->frame[n]='\0';
    }
    return result;
}

#if SERVO_FASHIONSTAR_GRIPPER
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
    assert(Servo_MovePulse(3,833,20,NULL)==SERVO_RANGE);
    assert(Servo_MovePulse(3,2167,20,NULL)==SERVO_RANGE);
    puts("PASS: no 001 query/motion/stop/torque/mode frames; PMOD1 shoulder rejects targets outside P834..P2166");
}
#endif
static void publish_rc_frame(void)
{
    rc.last_valid_ms=tick;
    ++rc.frame_count;
    Fxi6x_OnFrame(&rc);
}

static void operate(unsigned duration)
{
    for (unsigned i=0; i<duration; ++i) {
        if (tick%10U==0) publish_rc_frame();
        if(tx_pending && strcmp(tx,"#005PDST!")==0) ++grip_stops;
        run(1);
    }
}

static void confirm_initial_gear(int gear, RobotMode mode)
{
    rc.switch_c=gear;
    for (unsigned elapsed=0; elapsed<=ROBOT_LINK_MIN_MS+ROBOT_SWITCH_STABLE_MS; elapsed+=10U) {
        publish_rc_frame();
        if (RobotControl_Get(tick).mode==mode) return;
        tick+=10U;
    }
    assert(false);
}

static unsigned metadata_frames(RobotMode mode, const char *command)
{
    unsigned count=0;
    for (unsigned i=0; i<wire_count; ++i) {
        const WireFrame *entry=&wire[i];
        if (entry->mode==mode && entry->frame[0]=='#' && strcmp(entry->frame+4,command)==0)
            ++count;
    }
    return count;
}

static void automatic_metadata_window(bool wrist_first)
{
    const RobotMode initial_mode=wrist_first ? ROBOT_WRIST : ROBOT_ARM;
    const RobotMode later_mode=wrist_first ? ROBOT_ARM : ROBOT_WRIST;
    confirm_initial_gear(wrist_first ? ROBOT_SWITCH_DOWN : ROBOT_SWITCH_MIDDLE,initial_mode);
#if SERVO_FASHIONSTAR_GRIPPER
    positions[4]=FsServo_PulseFromAngle(FS_GRIPPER_OPEN_DDEG+FS_GRIPPER_TRAVEL_DDEG/2);
#else
    positions[4]=(ROBOT_GRIPPER_OPEN_P+ROBOT_GRIPPER_CLOSED_P)/2U;
#endif
    mode_reply_mask&=~4U;
    record_wire=true;
    assert(Servo_Init());
    operate(2U*SERVO_METADATA_RETRY_MS);
    assert(state(3).online && !state(3).mode_valid && !state(3).version_valid && !moves);
    assert(!metadata_frames(initial_mode,"PMOD!") && !metadata_frames(initial_mode,"PVER!"));
#if !SERVO_FASHIONSTAR_GRIPPER
    assert(metadata_frames(initial_mode,"PID!"));
#endif

    rc.switch_c=ROBOT_SWITCH_UP;
    operate(2U*SERVO_METADATA_RETRY_MS);
    assert(metadata_frames(ROBOT_DRIVE,"PMOD!") && metadata_frames(ROBOT_DRIVE,"PVER!"));
    assert(state(3).version_valid && !state(3).mode_valid);

    rc.switch_c=wrist_first ? ROBOT_SWITCH_MIDDLE : ROBOT_SWITCH_DOWN;
    operate(2U*SERVO_METADATA_RETRY_MS);
    assert(state(5).online && !moves);
    assert(!metadata_frames(later_mode,"PMOD!") && !metadata_frames(later_mode,"PVER!"));
    assert(!state(3).mode_valid);

    mode_reply_mask|=4U;
#if SERVO_FASHIONSTAR_GRIPPER
    /* 混合隔离档在 WAIT 只轮询二进制夹爪，切回 DRIVE 后补读众灵。 */
    rc.switch_c=ROBOT_SWITCH_UP;
    operate(2U*SERVO_METADATA_RETRY_MS);
#else
    const unsigned wait_reads=metadata_frames(ROBOT_WAIT,"PMOD!");
    run(2U*SERVO_METADATA_RETRY_MS);
    assert(RobotControl_Get(tick).mode==ROBOT_WAIT);
    assert(metadata_frames(ROBOT_WAIT,"PMOD!")>wait_reads);
#endif
    assert(state(3).mode_valid);
    puts("PASS: automatic PMOD/PVER pause in centered ARM/WRIST; pending metadata resumes in permitted modes");
}

typedef enum { STICK_NEUTRAL, STICK_PUSH, STICK_RELEASED, STICK_GRIP_CENTER, STICK_SECOND_AXIS } StickCase;

static void first_stick_frame(StickCase test)
{
    const bool grip=test==STICK_GRIP_CENTER;
    const bool second_axis=test==STICK_SECOND_AXIS;
    const unsigned second_id=SERVO_FASHIONSTAR_GRIPPER ? 2U : 4U;
    if (grip) {
        rc.rocker_r_v=660;
        positions[4]=ROBOT_GRIPPER_CLOSED_P;
    }
    confirm_initial_gear(grip ? ROBOT_SWITCH_DOWN : ROBOT_SWITCH_MIDDLE,
                         grip ? ROBOT_WRIST : ROBOT_ARM);
    assert(Servo_Init());
    operate(500);
    assert(state(2).online && state(3).online && state(4).online);
    assert(!moves);
    if (second_axis) {
        rc.rocker_l_v=180;
        operate(240);
        assert(moves && positions[2]>1500U && positions[1]==1500U);
    }

    record_wire=true;
    const bool miss_wrist=second_axis && SERVO_FASHIONSTAR_GRIPPER;
    const unsigned missing_bit=miss_wrist ? 8U : 2U;
    const char *missing_query=miss_wrist ? "#004PRAD!" : "#002PRAD!";
    reply_mask&=~missing_bit;
    unsigned last=0;
    for (unsigned elapsed=0; elapsed<SERVO_POLL_PERIOD_MS+SERVO_POLL_SLOT_MS; ++elapsed) {
        operate(1);
        last=wire_count;
        if (last && !strcmp(wire[last-1U].frame,missing_query)) break;
    }
    assert(last && !strcmp(wire[last-1U].frame,missing_query) && tx_pending);
    assert(wire[last-1U].started==tick);

    const uint32_t first_allowed=tx_due+SERVO_REPLY_TIMEOUT_MS+SERVO_RECOVERY_QUIET_MS;
    if (grip) rc.rocker_r_v=0;
    else if (second_axis) {
        if (SERVO_FASHIONSTAR_GRIPPER) rc.rocker_l_h=660;
        else rc.rocker_r_h=660;
    }
    else if (test!=STICK_NEUTRAL) rc.rocker_l_v=660;
    if (test==STICK_RELEASED) {
        publish_rc_frame();
        operate(SERVO_RC_PERIOD_MS+1U);
        ServoSnapshot snapshot;
        Servo_CopySnapshot(&snapshot);
        assert(snapshot.queued && !moves);
        rc.rocker_l_v=0;
        publish_rc_frame();
    }
    while (tick+1U<first_allowed) operate(1);
    assert(wire_count==last);
    /* 003 的上次反馈已超过一轮，可和新动作争用恢复后的第一个发送机会。 */
    assert(tick-state(3).position_ms>=SERVO_POLL_PERIOD_MS);
    for (unsigned elapsed=0; wire_count==last && elapsed<5U; ++elapsed) operate(1);
    assert(wire_count==last+1U && wire[last].started>=first_allowed);
    printf("First frame at +%lu ms after %s starts: %s\n",
           (unsigned long)(wire[last].started-wire[last-1U].started),missing_query,wire[last].frame);
    fflush(stdout);
    if (test==STICK_NEUTRAL || test==STICK_RELEASED) {
        assert(!strcmp(wire[last].frame+4,"PRAD!"));
        puts(test==STICK_RELEASED ? "PASS: releasing before TX revokes priority for a queued MOVE" :
             "PASS: neutral control resumes automatic PRAD after the recovery guard");
        return;
    }

    unsigned target, duration;
    if (grip) {
        assert(sscanf(wire[last].frame,"#005P%4uT%4u!",&target,&duration)==2);
        assert(target<ROBOT_GRIPPER_CLOSED_P && target>ROBOT_GRIPPER_OPEN_P);
        puts("PASS: returning CH3 toward center from its settled closed position gets first-MOVE priority");
        return;
    }
    if (second_axis) {
        unsigned addressed;
        assert(sscanf(wire[last].frame,"#%3uP%4uT%4u!",&addressed,&target,&duration)==3 &&
               addressed==second_id && target>1500U);
        puts("PASS: a newly moved joint axis gets first-MOVE priority while CH2 remains held");
        return;
    }
    assert(sscanf(wire[last].frame,"#003P%4uT%4u!",&target,&duration)==2 && target>1500U);
    reply_mask|=missing_bit;
    operate(150);
    for (unsigned elapsed=0; elapsed<800U; ++elapsed) {
        operate(1);
        const ServoState shoulder=state(3);
        assert(shoulder.online && tick-shoulder.position_ms<100U);
    }
    assert(positions[2]>1500U && moves>2U);
    puts("PASS: first actual stick MOVE wins over due PRAD after the recovery guard; held stick retains feedback");
}

static void joint_lead_limit(void)
{
    motion_hold_mask=(1U<<2)|(1U<<3);
    rc.switch_c=ROBOT_SWITCH_MIDDLE;
    assert(Servo_Init());
    operate(850);
    assert(state(3).online && state(4).online);

    rc.rocker_l_v=660; rc.rocker_r_h=-660;
    operate(3000);
    assert(positions[2]==1500 && positions[3]==1500);
    assert(state(3).target==1800 && state(4).target==1200);

    rc.rocker_l_v=-660; rc.rocker_r_h=660;
    operate(120);
    assert(state(3).target<1800 && state(4).target>1200);
    operate(5500);
    assert(state(3).target==1200 && state(4).target==1800);

    positions[2]=1460; positions[3]=1540;
    operate(500);
    assert(state(3).target==1160 && state(4).target==1840);

    reply_mask&=~motion_hold_mask;
    operate(1600);
    assert(!state(3).online && !state(4).online);
    rc.rocker_l_v=660; rc.rocker_r_h=-660;
    operate(5500);
    assert(state(3).target==1760 && state(4).target==1240);
    assert(Servo_RcStopPending()==0U);

    positions[2]=1500; positions[3]=1500;
    reply_mask|=motion_hold_mask;
    operate(2000);
    assert(state(3).online && state(4).online);
    assert(state(3).target==1800 && state(4).target==1200);
    puts("PASS: 003 and 004 clamp lead to 300 P, reverse without hidden accumulation, and use cached position during feedback loss");
}

static void joint_speed_limit(void)
{
    modes[1]=3;
    modes[3]=4;
    modes[4]=4;
    confirm_initial_gear(ROBOT_SWITCH_UP,ROBOT_DRIVE);
    assert(Servo_Init());
    operate(2400);
    assert(state(1).online && state(1).mode_valid);
    assert(state(2).mode_valid && state(2).mode==3);
    assert(state(3).mode_valid && state(3).mode==1);
    assert(state(4).mode_valid && state(4).mode==4);
    assert(state(5).mode_valid && state(5).mode==4);

    rc.switch_c=ROBOT_SWITCH_MIDDLE;
    operate(80);
    rc.rocker_l_v=660; rc.rocker_r_h=660; rc.rocker_r_v=660;
    operate(1000);
    const int delta_270=(int)lroundf(2000.0f*SERVO_MAX_SPEED_DEG_S/270.0f);
    const int delta_180=(int)lroundf(2000.0f*SERVO_MAX_SPEED_DEG_S/180.0f);
    assert(abs((int)positions[0]-(1500+delta_270))<=10);
    assert(abs((int)positions[2]-(1500+delta_270))<=10);
    assert(abs((int)positions[3]-(1500+delta_180))<=12);
    assert(positions[1]==1500 && positions[4]==1500);

    rc.rocker_l_v=0; rc.rocker_r_h=0;
    rc.switch_c=ROBOT_SWITCH_DOWN;
    operate(120);
    rc.rocker_l_h=-660;
    operate(1000);
    assert(abs((int)positions[1]-(1500-delta_180))<=12 && positions[4]==1500);
    puts("PASS: the mapped 001/002/003/004 axes respect the shared speed limit in both position spans");

    rc.rocker_r_v=-660;
    operate(3700);
    assert(positions[1]==500 && state(2).target==500);
    assert(positions[4]==750 && state(5).target==750);
    rc.rocker_l_h=660; rc.rocker_r_v=660;
    operate(160);
    assert(positions[1]>500 && positions[4]>750);
    operate(6300);
    assert(positions[1]==2500 && state(2).target==2500);
    assert(positions[4]==1500 && state(5).target==1500);
    puts("PASS: confirmed PMOD3/4 retain the full joint and calibrated gripper travel and reverse from endpoints");
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
    const uint32_t feedback_before_motion=state(3).position_ms;
    for (unsigned i=0; i<3000U; ++i) {
        operate(1);
        if (positions[2]!=previous) {
            const uint32_t gap=tick-last_move_tick;
            if (gap>max_gap) max_gap=gap;
            previous=positions[2];
            last_move_tick=tick;
        }
        const ServoState shoulder=state(3);
        assert(shoulder.online);
        /* 首条动作可以先于上一个离线查询后恢复的位置回读。 */
        if(shoulder.position_ms!=feedback_before_motion)assert(tick-shoulder.position_ms<100U);
    }
    assert(max_gap<=45U);
    const int expected=(int)lroundf(3.0f*0.25f*2000.0f*SERVO_MAX_SPEED_DEG_S/270.0f);
    assert(abs((int)(positions[2]-start)-expected)<=6);
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
    const uint16_t open=2000U;
    const uint16_t halfway=FsServo_PulseFromAngle(FS_GRIPPER_OPEN_DDEG+FS_GRIPPER_TRAVEL_DDEG/2);
    const uint16_t three_quarters=FsServo_PulseFromAngle(FS_GRIPPER_OPEN_DDEG+3*FS_GRIPPER_TRAVEL_DDEG/4);
    const uint16_t closed=FsServo_PulseFromAngle(FS_GRIPPER_OPEN_DDEG+FS_GRIPPER_TRAVEL_DDEG);
    assert(FsServo_PulseFromAngle(FS_GRIPPER_OPEN_DDEG)==2180U && halfway==1890U && closed==1600U);
    const unsigned sweep_ms=(unsigned)abs(FS_GRIPPER_TRAVEL_DDEG)*100U/SERVO_MAX_SPEED_DEG_S;
    positions[4]=open;
    reply_mask=4U|8U|16U;
    fs_misses_after_zp=11;
    rc.switch_c=ROBOT_SWITCH_DOWN;
    rc.rocker_r_v=-660;
    assert(Servo_Init());
    operate(1600);
    assert(state(5).online && state(5).id_seen);
    assert(state(5).position==open && fs_queries>0 && fs_moves==0);
    rc.rocker_r_v=0; operate(400);
    const int expected=(int)lroundf(0.4f*2000.0f*SERVO_MAX_SPEED_DEG_S/360.0f);
    assert(abs((int)positions[4]-(open-expected))<=8);
    operate(sweep_ms/2U-400U);
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
    puts("PASS: V1 mapping and polling; gripper reopens to clipped P2000 after protocol switch without position replies, with RC link-loss hold");
}
#endif

int main(int argc,char **argv)
{
    if (argc==2 && !strcmp(argv[1],"automatic-metadata-window")) {
        automatic_metadata_window(false);
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"automatic-metadata-window-wrist-start")) {
        automatic_metadata_window(true);
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"first-stick-frame")) {
        first_stick_frame(STICK_PUSH);
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"neutral-frame")) {
        first_stick_frame(STICK_NEUTRAL);
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"released-stick-frame")) {
        first_stick_frame(STICK_RELEASED);
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"gripper-center-frame")) {
        first_stick_frame(STICK_GRIP_CENTER);
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"second-axis-frame")) {
        first_stick_frame(STICK_SECOND_AXIS);
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"joint-lead-limit")) {
        joint_lead_limit();
        return 0;
    }
    if (argc==2 && !strcmp(argv[1],"joint-speed-limit")) {
        joint_speed_limit();
        return 0;
    }
#if SERVO_FASHIONSTAR_GRIPPER
    if (argc==2 && !strcmp(argv[1],"wrist-silence")) {
        wrist_silence();
        return 0;
    }
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
    assert(Servo_RcMovePulse(5,833,20,NULL)==SERVO_RANGE);
    assert(Servo_RcMovePulse(5,1501,20,NULL)==SERVO_RANGE);
    rc.rocker_r_v=0;operate(1000);
    assert(abs((int)positions[4]-(1500-(int)lroundf(2000.0f*SERVO_MAX_SPEED_DEG_S/270.0f)))<=8);
    operate(3200);
    assert(positions[4]==1125 && state(5).target==1125);
    /* 切档取消未发送的目标，夹爪保持最后到达的位置。 */
    assert(Servo_RcMovePulse(5,1300,20,NULL)==SERVO_QUEUED);
    rc.switch_c=ROBOT_SWITCH_UP;rc.last_valid_ms=tick;++rc.frame_count;Fxi6x_OnFrame(&rc);
    rc.rocker_r_v=-660;operate(300);
    assert(positions[4]==1125 && state(5).target==1125 && grip_stops==0);
    rc.switch_c=ROBOT_SWITCH_DOWN;operate(4200);
    assert(positions[4]==834 && state(5).target==834);
    const unsigned grip_moves_at_limit=moves;
    const uint32_t grip_feedback_at_limit=state(5).position_ms;
    operate(250);
    assert(moves==grip_moves_at_limit && state(5).position_ms>grip_feedback_at_limit);
    rc.rocker_r_v=660;operate(7350);
    assert(positions[4]==1500 && state(5).target==1500);
    rc.rocker_l_v=1;rc.rocker_l_h=330;operate(800);
    assert(positions[0]==1500 && state(1).online && positions[3]==1500 &&
           positions[1]>1500 && positions[2]==1500);
    rc.switch_c=ROBOT_SWITCH_UP;rc.rocker_l_v=0;rc.rocker_l_h=0;operate(300);
    uint32_t mode_ticket;
    assert(Servo_RequestModeProbe(3,&mode_ticket)==SERVO_QUEUED);
    operate(2U*SERVO_MODE_PROBE_TIMEOUT_MS);
    assert(!state(2).mode_valid && state(3).mode_valid && state(3).mode==5);
    const uint16_t elbow=positions[1];
    rc.switch_c=ROBOT_SWITCH_MIDDLE;
    rc.rocker_l_v=-300;rc.rocker_l_h=300;rc.rocker_r_h=-300;rc.rocker_r_v=300;operate(750);
    assert(positions[0]>1500 && positions[2]<1500 && positions[1]==elbow &&
           positions[3]<1500 && positions[4]==1500 && !grip_stops);
    /* 运动发送失败且停止无应答。同档等新位置再自动接管，从新位置继续。 */
    rc.rocker_l_v=0; rc.rocker_l_h=0; rc.rocker_r_h=0; rc.rocker_r_v=0; operate(100);
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
    const uint16_t shoulder_recovery=850U;
    positions[2]=shoulder_recovery;
    reply_mask|=4U;
    for(unsigned elapsed=0;positions[2]==shoulder_recovery && elapsed<1500U;++elapsed) operate(1);
    assert(positions[2]>shoulder_recovery && positions[2]<shoulder_recovery+20U && Servo_RcInhibited()==0U);
    /* 在中档持续打满肩关节摇杆，分别验证上下限与反向退出。 */
    rc.rocker_l_v=660; operate(11000);
    assert(positions[2]==2166 && state(3).target==2166);
    rc.rocker_l_v=-660; operate(11000);
    assert(positions[2]==834 && state(3).target==834);

    record_wire=true;
    positions[2]=833;
    operate(140);
    assert(state(3).position==833 && metadata_frames(ROBOT_ARM,"PDST!")==1);
    const unsigned shoulder_moves_at_limit=moves;
    rc.rocker_l_v=660;
    operate(200);
    assert(positions[2]==833 && moves==shoulder_moves_at_limit);
    positions[2]=834;
    operate(220);
    assert(positions[2]>834 && moves>shoulder_moves_at_limit);
    assert(!depth);
    puts("PASS: ZP 005 follows signed CH3 at P834/P1125/P1500; unknown/non-position PMOD clips motion and declines out-of-range feedback");
    return 0;
}
