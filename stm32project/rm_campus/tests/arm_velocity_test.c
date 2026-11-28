/* 复用真实舵机/UART/USB路径的模拟环境；本文件中的标定仅用于测试，禁止用于实物。 */
#define main legacy_servo_main
#define MotorStatusUsb_Busy Fixture_MotorStatusUsb_Busy
#include "servo_test.c"
#undef MotorStatusUsb_Busy
#undef main
#include "arm_velocity.h"
#include "arm_usb.h"
#include "arm_kinematics.h"
#include <math.h>

static bool motor_report_busy;
bool MotorStatusUsb_Busy(void) { return motor_report_busy; }
static const ArmVelocityCommand zero={0};
static ArmStatus arm_state(void) { ArmStatus s;Arm_GetStatus(&s);return s; }
static void clear_output(void) { output_len=0;output[0]=0; }
static unsigned occurrences(const char *token)
{
    unsigned count=0;const char *p=output;
    while((p=strstr(p,token))!=NULL) { ++count;p+=strlen(token); }
    return count;
}
static void drive(ArmVelocityCommand command,unsigned milliseconds)
{
    for(unsigned n=0;n<milliseconds;++n) {
        if(n%20U==0U)assert(Arm_SetVelocity(&command));
        run(1);
    }
}
static void start_arm(void)
{
    assert(Arm_SetVelocity(&zero));assert(Arm_Enable());run(1);
    assert(arm_state().state==ARM_RUNNING);
}
static void recover(void)
{
    /* 停止事务排空、全组反馈新鲜后才请求清错；清错不等于重新使能。 */
    assert(!Arm_ResetRequired()); /* 本辅助流程只适用于仍允许软件恢复的故障。 */
    drive(zero,300);assert(Arm_ClearFault());run(1);
    assert(arm_state().state==ARM_IDLE);
    unsigned before=moves;drive(zero,100);assert(moves==before);
    start_arm();
}
static void same_reset_fault(const ArmStatus *original)
{
    ArmStatus s=arm_state();
    assert(Arm_ResetRequired() && s.state==ARM_FAULT);
    assert(s.error==original->error && s.fault_sequence==original->fault_sequence);
    assert(s.joint==original->joint && s.servo_id==original->servo_id);
    assert(s.requested==original->requested && s.limit==original->limit);
}
static void verify_reset_only(const ArmStatus *original)
{
    unsigned before=moves;
    drive(zero,300);same_reset_fault(original);
    assert(!Arm_ClearFault() && !Arm_Enable());
    Arm_Stop();run(100);same_reset_fault(original);
    ArmVelocityCommand legal={.vx_m_s=0.005f};drive(legal,300);
    assert(moves==before);same_reset_fault(original);
    /* 后续更大的指令不能覆盖首个故障的类型、轴号、请求值和序号。 */
    ArmVelocityCommand another={.vx_m_s=0.2f,.pitch_rad_s=10.0f};drive(another,100);
    same_reset_fault(original);assert(moves==before);
    /* 模块重新初始化和舵机驱动重新初始化都不是MCU复位。 */
    Arm_Init(tick);run(50);same_reset_fault(original);
    assert(!Arm_ClearFault() && !Arm_Enable());
    assert(Servo_Init());run(700);same_reset_fault(original);
    assert(Servo_MovePulse(1,1600,100,NULL)==SERVO_BUSY);
    assert(Servo_Resume(1,NULL)==SERVO_BUSY);
    assert(Servo_SetTorque(1,true,NULL)==SERVO_BUSY);
    assert(Servo_RequestPosition(1,NULL)==SERVO_QUEUED);run(100);
    assert(moves==before && !Arm_ClearFault() && !Arm_Enable());
    /* 锁存跨USB会话保持，重新初始化后也应持续5Hz报告原速度错误。 */
    char token[80];snprintf(token,sizeof(token),"ARM_ERROR code=%s",Arm_ErrorName(original->error));
    usb.ready=true;clear_output();run(1000);
    assert(occurrences(token)==5 && !strchr(output,','));
    usb.ready=false;run(5);usb_len=0;++usb.session_id;clear_output();usb.ready=true;run(1000);
    assert(occurrences(token)==5 && !strchr(output,','));same_reset_fault(original);
    assert(moves==before);
}
static void fresh_boot(void)
{
    /* 测试运行器启动新的进程模拟BSS复位；不提供任何清锁存的测试后门。 */
    assert(!Arm_ResetRequired() && arm_state().state==ARM_IDLE && !moves);
    drive(zero,200);assert(arm_state().state==ARM_IDLE && !moves);
    start_arm();assert(arm_state().state==ARM_RUNNING);
}
static void startup(void)
{
    assert(!moves && arm_state().state==ARM_IDLE);
    assert(Arm_Enable());run(1);
    assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_COMMAND_TIMEOUT);
    assert(Arm_ClearFault());run(1);assert(arm_state().state==ARM_FAULT);
    ArmVelocityCommand nonzero={.vx_m_s=0.01f};
    assert(Arm_SetVelocity(&nonzero));assert(Arm_ClearFault());run(1);assert(arm_state().state==ARM_FAULT);
    assert(Arm_SetVelocity(&zero));run(ARM_COMMAND_TIMEOUT_MS);
    assert(Arm_ClearFault());run(1);assert(arm_state().state==ARM_FAULT);
    recover();Arm_Stop();drive(zero,200);assert(arm_state().state==ARM_IDLE);
    assert(Arm_SetVelocity(&nonzero));assert(Arm_Enable());run(1);
    assert(arm_state().error==ARM_ERROR_INVALID_COMMAND && !moves);
}
static void normal(void)
{
    start_arm();ArmVelocityCommand command={.vx_m_s=0.01f};drive(command,600);
    ArmStatus s=arm_state();assert(s.state==ARM_RUNNING && s.error==ARM_ERROR_NONE);
    /* 测试姿态肩0/肘90°/腕-90°：+x运动要求肘ID2减小、腕ID1增大。 */
    assert(positions[1]<1490 && positions[0]>1510);
    assert(positions[3]==1500 && positions[4]==1500);
    assert(s.target_rad[1]<ARM_PI/2 && s.target_rad[2]>-ARM_PI/2);
    drive(zero,150);unsigned before=moves;uint16_t held[5];memcpy(held,positions,sizeof(held));
    drive(zero,1000);assert(moves==before && !memcmp(held,positions,sizeof(held)));
    assert(arm_state().state==ARM_RUNNING);Arm_Stop();drive(zero,200);
    assert(arm_state().state==ARM_IDLE && !Servo_ControlStopPending(SERVO_OWNER_ARM));
    assert(Servo_MovePulse(1,positions[0],100,NULL)==SERVO_QUEUED);
}
static void stop_drain(void)
{
    start_arm();ArmVelocityCommand command={.vx_m_s=0.005f};drive(command,160);
    Arm_Stop();assert(Servo_ControlStopPending(SERVO_OWNER_ARM));
    /* 同一批停止和使能请求以停止优先，不能越过尚未完成的UART事务。 */
    assert(Arm_SetVelocity(&zero));assert(Arm_Enable());run(1);
    assert(arm_state().state!=ARM_RUNNING && Servo_ControlStopPending(SERVO_OWNER_ARM));
    /* 下一周期再次请求使能，仍必须等待整组停止排空。 */
    assert(Arm_SetVelocity(&zero));assert(Arm_Enable());run(1);
    assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_CONTROL_BUSY);
    assert(Arm_ClearFault());run(1);assert(arm_state().state==ARM_FAULT);
    drive(zero,150);unsigned before=moves;drive(zero,150);assert(moves==before);
    assert(arm_state().state==ARM_FAULT && !Servo_ControlStopPending(SERVO_OWNER_ARM));
    recover();Arm_Stop();drive(zero,200);assert(arm_state().state==ARM_IDLE);
    /* 使能前的零指令若已被新非零指令替换，不得沿用旧零值通过检查。 */
    assert(Arm_SetVelocity(&zero));assert(Arm_Enable());assert(Arm_SetVelocity(&command));run(1);
    assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_INVALID_COMMAND);
}
static void linear_limit(void)
{
    start_arm();unsigned before=moves;
    /* 两个分量均小于50，但合速度56.6mm/s必须拒绝。 */
    ArmVelocityCommand command={.vx_m_s=0.04f,.vz_m_s=0.04f};drive(command,100);
    ArmStatus s=arm_state();assert(s.state==ARM_FAULT && s.error==ARM_ERROR_LINEAR_SPEED_LIMIT);
    assert(s.requested>56.0f && s.limit==ARM_MAX_LINEAR_SPEED_MM_S && moves==before);
    verify_reset_only(&s);
}
static ArmVelocityCommand command_for_axis(unsigned axis,float speed_deg_s)
{
    const float speed_rad_s=ARM_DEG_TO_RAD(speed_deg_s);
    if(axis==4U)return (ArmVelocityCommand){.gripper_rad_s=speed_rad_s};
    const ArmStatus s=arm_state();
    const ArmKinConfig geometry={.l1_m=ARM_L12_M,.l2_m=ARM_L23_M};
    float jacobian[4][4];
    assert(ArmKin_Jacobian(&geometry,s.q_rad,jacobian));
    return (ArmVelocityCommand){
        .vx_m_s=jacobian[0][axis]*speed_rad_s,.vz_m_s=jacobian[1][axis]*speed_rad_s,
        .pitch_rad_s=jacobian[2][axis]*speed_rad_s,.roll_rad_s=jacobian[3][axis]*speed_rad_s};
}
static void joint_limit(unsigned axis)
{
    static const uint8_t ids[5]={3U,2U,1U,4U,5U};
    if(axis==0U) {
        /* 屈肘至约120°，使肩轴限速附近的指令也符合50 mm/s腕交点限速。 */
        positions[1]=1722;run(150);
    }
    start_arm();
    /* DLS 解受浮点精度和阻尼影响，四个 IK 轴在上限下方预留少量裕量。 */
    const float allowed_deg_s=axis==4U ? 15.0f : 14.99f;
    drive(command_for_axis(axis,allowed_deg_s),ARM_PERIOD_MS+1U);
    ArmStatus s=arm_state();
    const float accepted_deg_s=ARM_RAD_TO_DEG(s.joint_velocity_rad_s[axis]);
    assert(s.state==ARM_RUNNING && accepted_deg_s>14.9f && accepted_deg_s<15.001f);
    drive(zero,150);
    assert(moves && arm_state().state==ARM_RUNNING);
    const unsigned before=moves;
    drive(command_for_axis(axis,-15.1f),50);
    s=arm_state();
    assert(s.state==ARM_FAULT && s.error==ARM_ERROR_JOINT_SPEED_LIMIT);
    assert(s.joint==axis+1U && s.servo_id==ids[axis] && s.limit==15.0f);
    assert(s.requested>15.0f && s.requested<15.2f && moves==before);
    verify_reset_only(&s);
}
static void unreachable(void)
{
    /* 舵机整数位置使肘接近伸直，径向任务应由残差检测拒绝。 */
    positions[1]=833;run(150);start_arm();
    ArmVelocityCommand command={.vx_m_s=0.008f};drive(command,100);
    ArmStatus s=arm_state();assert(s.state==ARM_FAULT && s.error==ARM_ERROR_VELOCITY_UNACHIEVABLE);
    assert(s.linear_residual_m_s>0.007f && !moves);
}
static void timeout(void)
{
    start_arm();ArmVelocityCommand command={.vx_m_s=0.005f};drive(command,200);
    run(150);assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_COMMAND_TIMEOUT);
    run(100);unsigned before=moves;drive(command,300);assert(moves==before && arm_state().state==ARM_FAULT);
}
static void feedback_loss(void)
{
    start_arm();ArmVelocityCommand command={.vx_m_s=0.005f};drive(command,150);
    reply_mask&=~2U;drive(command,500);
    ArmStatus s=arm_state();assert(s.state==ARM_FAULT);
    assert(s.error==ARM_ERROR_DRIVER || s.error==ARM_ERROR_FEEDBACK_STALE || s.error==ARM_ERROR_FEEDBACK_SKEW);
    unsigned before=moves;reply_mask|=2U;drive(command,1500);
    assert(moves==before && arm_state().state==ARM_FAULT);recover();
}
static void tx_failure(void)
{
    start_arm();unsigned before=moves;fail_tx=true;
    ArmVelocityCommand command={.vx_m_s=0.005f};drive(command,400);
    assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_DRIVER);
    assert(moves==before && !Servo_ControlStopPending(SERVO_OWNER_ARM));
    recover();
}
static void manual_halt(void)
{
    start_arm();ArmVelocityCommand command={.vx_m_s=0.005f};drive(command,200);
    assert(Servo_Pause(2,NULL)==SERVO_QUEUED);drive(command,150);
    assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_DRIVER);
    unsigned before=moves;drive(command,400);assert(moves==before);recover();
}
static void mode_change(void)
{
    /* 同样的P值在另一PMOD下不能复用此前标定。 */
    modes[0]=2;assert(Servo_Init());run(700);
    assert(Arm_SetVelocity(&zero));assert(Arm_Enable());run(1);
    assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_BAD_MODE && !moves);
}
static void tracking(void)
{
    start_arm();ArmVelocityCommand command={.vx_m_s=0.005f};
    for(unsigned n=0;n<2600;++n) {
        if(n%20U==0U)assert(Arm_SetVelocity(&command));
        run(1);
        /* 模拟有通信但机构无法跟随，目标不能无限向前积分。 */
        for(unsigned i=0;i<5;++i)positions[i]=1500;
    }
    assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_TRACKING);
}
static void mechanical_limit(void)
{
    const uint16_t shoulder_max=Servo_GetConfig(2)->maximum;
    positions[2]=shoulder_max;run(150);start_arm();
    float q1=(shoulder_max-1500.0f)*ARM_DEG_TO_RAD(270.0f)/2000.0f;
    ArmVelocityCommand command={
        .vx_m_s=(-ARM_L12_M*sinf(q1)-ARM_L23_M*sinf(q1+ARM_PI/2))*0.15f,
        .vz_m_s=(ARM_L12_M*cosf(q1)+ARM_L23_M*cosf(q1+ARM_PI/2))*0.15f,
        .pitch_rad_s=0.15f};
    drive(command,100);
    assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_JOINT_LIMIT && !moves);
}
static void invalid_and_timing(void)
{
    start_arm();ArmVelocityCommand invalid={.vx_m_s=NAN};assert(Arm_SetVelocity(&invalid));run(1);
    assert(arm_state().error==ARM_ERROR_INVALID_COMMAND && !moves);recover();
    tick+=80;assert(Arm_SetVelocity(&zero));run(1);
    assert(arm_state().error==ARM_ERROR_TIMING && !moves);
}
static void atomic_group(void)
{
    uint8_t ids[5]={3,2,1,4,5};uint16_t p[5]={1510,1520,1530,1540,499},t[5]={500,500,500,500,500};
    assert(Servo_ControlClaim(SERVO_OWNER_ARM,31));assert(!Servo_ControlClaim(SERVO_OWNER_RC,7));
    assert(Servo_ControlMoveGroup(SERVO_OWNER_ARM,ids,p,t,5)==SERVO_RANGE);run(100);assert(!moves);
    p[4]=1550;unsigned count=0;while(Servo_RequestPosition(1,NULL)==SERVO_QUEUED)++count;
    assert(count==SERVO_QUEUE_SIZE && Servo_ControlMoveGroup(SERVO_OWNER_ARM,ids,p,t,5)==SERVO_BUSY);
    run(300);assert(!moves);
    assert(Servo_ControlMoveGroup(SERVO_OWNER_ARM,ids,p,t,5)==SERVO_QUEUED);
    assert(!Servo_ControlRelease(SERVO_OWNER_ARM));
    assert(Servo_SetTorque(2,false,NULL)==SERVO_QUEUED);run(200);
    assert(!moves && Servo_ControlInhibited(SERVO_OWNER_ARM));
    assert(Servo_ControlRelease(SERVO_OWNER_ARM));
}
static void reports(void)
{
    /* 非速度故障继续支持旧清错流程；速度故障的5Hz持续输出在各限速场景验证。 */
    start_arm();ArmVelocityCommand command={.vx_m_s=NAN};drive(command,50);
    assert(arm_state().error==ARM_ERROR_INVALID_COMMAND && !Arm_ResetRequired());
    usb.ready=true;clear_output();run(1000);
    assert(occurrences("ARM_ERROR code=INVALID_COMMAND")==5);
    assert(!strchr(output,',') && strstr(output,"unit=none"));
    /* 真实手动舵机诊断必须完整，错误行不能插入BEGIN/END之间。 */
    clear_output();input("servo_status\n");run(500);
    const char *begin=strstr(output,"SERVO_STATUS BEGIN"),*end=strstr(output,"SERVO_STATUS END");
    assert(begin&&end&&end>begin);const char *error=strstr(begin,"ARM_ERROR");
    assert(!error || error>end);assert(occurrences("ARM_ERROR")>=1 && !strchr(output,','));
    /* 电机诊断优先权会让错误输出等待，但不会形成补发队列。 */
    run(10);motor_report_busy=true;clear_output();run(500);assert(!output_len);
    motor_report_busy=false;run(10);assert(occurrences("ARM_ERROR")==1);
    clear_output();usb_hold=true;run(1000);assert(usb_len && !output_len);
    usb_hold=false;run(10);assert(occurrences("ARM_ERROR")<=2);
    usb.ready=false;run(5);usb_len=0;++usb.session_id;clear_output();usb.ready=true;run(1000);
    assert(occurrences("ARM_ERROR")==5 && !strchr(output,','));
    clear_output();recover();run(100);assert(strchr(output,',')!=NULL); /* 清错后原电机CSV恢复。 */
}
int main(int argc,char **argv)
{
    const char *scenario=argc>1?argv[1]:"normal";
    if(!strcmp(scenario,"wrap"))tick=UINT32_MAX-100U;
    assert(Servo_Init());run(700);
    assert(!Arm_ResetRequired()); /* 每个测试进程都从真正的静态零初始化开始。 */
    if(!ARM_VELOCITY_IK_ENABLE) {
        assert(arm_state().state==ARM_DISABLED && !Arm_Enable() && !Arm_SetVelocity(&zero));
        assert(Servo_MovePulse(1,1510,100,NULL)==SERVO_QUEUED);run(100);assert(positions[0]==1510);
        puts("PASS: ARM disabled, existing manual servo function retained.");return 0;
    }
    if(!ARM_JOINT_CALIBRATION_VALID || !arm_state().calibrated) {
        assert(Arm_SetVelocity(&zero));assert(Arm_Enable());run(1);
        assert(arm_state().state==ARM_FAULT && arm_state().error==ARM_ERROR_UNCALIBRATED && !moves);
        puts("PASS: absent/invalid calibration blocks physical output.");return 0;
    }
    if(!ARM_VELOCITY_IK_OUTPUT_ENABLE) {
        start_arm();ArmVelocityCommand command={.vx_m_s=0.005f};drive(command,400);
        assert(arm_state().state==ARM_RUNNING && arm_state().target_rad[1]<ARM_PI/2 && !moves);
        Arm_Stop();run(10);assert(Servo_MovePulse(1,1510,100,NULL)==SERVO_QUEUED);run(100);
        assert(positions[0]==1510);
        start_arm();command=(ArmVelocityCommand){.pitch_rad_s=ARM_DEG_TO_RAD(45.0f)};drive(command,100);
        ArmStatus original=arm_state();assert(original.error==ARM_ERROR_JOINT_SPEED_LIMIT && Arm_ResetRequired());
        assert(!Arm_Enable() && !Arm_ClearFault());
        assert(Servo_MovePulse(1,1520,100,NULL)==SERVO_QUEUED);run(150);assert(positions[0]==1520);
        Arm_Init(tick);run(20);same_reset_fault(&original);
        assert(Servo_MovePulse(1,1530,100,NULL)==SERVO_QUEUED);run(150);assert(positions[0]==1530);
        puts("PASS: shadow reset latch retains diagnostics without blocking real manual motion.");return 0;
    }
    if(!strcmp(scenario,"startup"))startup();
    else if(!strcmp(scenario,"fresh_boot"))fresh_boot();
    else if(!strcmp(scenario,"normal") || !strcmp(scenario,"wrap"))normal();
    else if(!strcmp(scenario,"stop_drain"))stop_drain();
    else if(!strcmp(scenario,"linear"))linear_limit();
    else if(!strcmp(scenario,"shoulder"))joint_limit(0);
    else if(!strcmp(scenario,"elbow"))joint_limit(1);
    else if(!strcmp(scenario,"joint"))joint_limit(2);
    else if(!strcmp(scenario,"roll"))joint_limit(3);
    else if(!strcmp(scenario,"gripper"))joint_limit(4);
    else if(!strcmp(scenario,"unreachable"))unreachable();
    else if(!strcmp(scenario,"timeout"))timeout();
    else if(!strcmp(scenario,"feedback"))feedback_loss();
    else if(!strcmp(scenario,"tx_failure"))tx_failure();
    else if(!strcmp(scenario,"manual_halt"))manual_halt();
    else if(!strcmp(scenario,"mode"))mode_change();
    else if(!strcmp(scenario,"tracking"))tracking();
    else if(!strcmp(scenario,"mechanical_limit"))mechanical_limit();
    else if(!strcmp(scenario,"invalid_timing"))invalid_and_timing();
    else if(!strcmp(scenario,"atomic"))atomic_group();
    else if(!strcmp(scenario,"reports"))reports();
    else assert(!"unknown scenario");
    assert(!depth && !mask);printf("PASS: ARM integration %s.\n",scenario);return 0;
}
