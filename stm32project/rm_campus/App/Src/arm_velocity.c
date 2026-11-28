#include "arm_velocity.h"
#if ARM_VELOCITY_IK_ENABLE
#include "arm_kinematics.h"
#include "servo_app.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

typedef struct {
    uint8_t id; int direction; float ratio; uint16_t reference_p;
    float reference_rad, minimum_rad, maximum_rad;
    uint8_t expected_mode; /* 模式改变会改变比例或方向，不能复用旧标定。 */
} JointCalibration;
static const JointCalibration calibration[5]={ARM_JOINT_CALIBRATION_TABLE};
static const float servo_speed_limit_rad_s=ARM_DEG_TO_RAD(SERVO_MAX_SPEED_DEG_S);
typedef struct {
    ArmVelocityCommand command;
    uint32_t time_ms, sequence;
    bool received, enable, stop, clear;
} Mailbox;
static Mailbox mailbox;
static ArmStatus status, published;
static ArmKinConfig kinematics;
static uint8_t slots[5], modes[5], all_slots;
static uint16_t last_p[5];
static uint32_t previous_ms;
static bool config_valid, owns_servos;
/* 只由MCU复位后的C启动过程清零，不放入可被Arm_Init memset的状态结构。
 * 速度超限之后，改指令、清错或重复初始化均不能恢复运动。 */
static bool reset_required;
/* 仅ServoBusTask访问的工作缓冲，避免在512-word任务栈上放置大快照。 */
static ServoSnapshot feedback;

static void publish(uint32_t now)
{
    status.now_ms=now;
    taskENTER_CRITICAL();published=status;taskEXIT_CRITICAL();
}
void Arm_GetStatus(ArmStatus *out)
{ if(out) { taskENTER_CRITICAL();*out=published;taskEXIT_CRITICAL(); } }
const char *Arm_ErrorName(ArmError error)
{
    static const char *names[]={"NONE","BAD_CONFIG","UNCALIBRATED","NOT_READY","FEEDBACK_STALE",
        "FEEDBACK_SKEW","BAD_MODE","COMMAND_TIMEOUT","INVALID_COMMAND","TIMING",
        "LINEAR_SPEED_LIMIT","JOINT_SPEED_LIMIT","VELOCITY_UNACHIEVABLE","JOINT_LIMIT",
        "TRACKING","DRIVER","CONTROL_BUSY","NUMERIC"};
    return (unsigned)error<sizeof(names)/sizeof(names[0]) ? names[error] : "UNKNOWN";
}
bool Arm_SetVelocity(const ArmVelocityCommand *command)
{
    if(!command)return false;
    taskENTER_CRITICAL();mailbox.command=*command;mailbox.time_ms=HAL_GetTick();
    mailbox.received=true;++mailbox.sequence;taskEXIT_CRITICAL();return true;
}
bool Arm_ResetRequired(void)
{ taskENTER_CRITICAL();bool required=reset_required;taskEXIT_CRITICAL();return required; }
bool Arm_Enable(void)
{
    taskENTER_CRITICAL();
    if(reset_required) { taskEXIT_CRITICAL();return false; }
    mailbox.enable=true;++mailbox.sequence;taskEXIT_CRITICAL();return true;
}
void Arm_Stop(void)
{
    taskENTER_CRITICAL();mailbox.stop=true;++mailbox.sequence;
    /* 提交停止时立即撤销驱动中的待发运动，不等待下一次20ms解算。 */
#if ARM_VELOCITY_IK_OUTPUT_ENABLE
    Servo_ControlStopMask(SERVO_OWNER_ARM,all_slots);
#endif
    taskEXIT_CRITICAL();
}
bool Arm_ClearFault(void)
{
    taskENTER_CRITICAL();
    if(reset_required) { taskEXIT_CRITICAL();return false; }
    mailbox.clear=true;++mailbox.sequence;taskEXIT_CRITICAL();return true;
}
static bool finite_command(const ArmVelocityCommand *c)
{
    return isfinite(c->vx_m_s) && isfinite(c->vz_m_s) && isfinite(c->pitch_rad_s) &&
        isfinite(c->roll_rad_s) && isfinite(c->gripper_rad_s);
}
static bool zero_command(const ArmVelocityCommand *c)
{ return c->vx_m_s==0 && c->vz_m_s==0 && c->pitch_rad_s==0 && c->roll_rad_s==0 && c->gripper_rad_s==0; }
static void stop_outputs(void)
{
    if(owns_servos)Servo_ControlStopMask(SERVO_OWNER_ARM,all_slots);
    memset(status.joint_velocity_rad_s,0,sizeof(status.joint_velocity_rad_s));
}
static void fault(ArmError error,unsigned joint,float requested,float limit)
{
    if(status.state==ARM_FAULT)return; /* 锁存第一次原因，后续零速度不会自动清错。 */
    status.state=ARM_FAULT;status.error=error;status.joint=(uint8_t)joint;
    status.servo_id=joint>=1U && joint<=5U ? calibration[joint-1U].id : 0U;
    status.requested=requested;status.limit=limit;++status.fault_sequence;
    if(error==ARM_ERROR_LINEAR_SPEED_LIMIT || error==ARM_ERROR_JOINT_SPEED_LIMIT) {
        taskENTER_CRITICAL();reset_required=true;taskEXIT_CRITICAL();
    }
    stop_outputs();
}
void Arm_Init(uint32_t now)
{
    if(Arm_ResetRequired()) {
        /* 保留第一次超限的错误码、关节、请求值、上限和序号，USB继续报告同一原因。
         * Servo_Init重复调用时驱动控制权可能重置，驱动还会独立检查此复位锁存。 */
        taskENTER_CRITICAL();memset(&mailbox,0,sizeof(mailbox));taskEXIT_CRITICAL();
        stop_outputs();previous_ms=now;publish(now);return;
    }
    taskENTER_CRITICAL();memset(&mailbox,0,sizeof(mailbox));taskEXIT_CRITICAL();
    memset(&status,0,sizeof(status));memset(modes,0,sizeof(modes));memset(last_p,0,sizeof(last_p));
    status.state=ARM_IDLE;status.output_enabled=ARM_VELOCITY_IK_OUTPUT_ENABLE!=0;
    previous_ms=now;owns_servos=false;all_slots=0;
    kinematics=(ArmKinConfig){.l1_m=ARM_L12_M,.l2_m=ARM_L23_M,.damping=ARM_DLS_DAMPING,
        .max_linear_m_s=ARM_MAX_LINEAR_SPEED_MM_S*0.001f,
        .linear_tolerance_m_s=ARM_LINEAR_RESIDUAL_MM_S*0.001f,
        .angular_tolerance_rad_s=ARM_DEG_TO_RAD(ARM_ANGULAR_RESIDUAL_DEG_S)};
    config_valid=isfinite(kinematics.max_linear_m_s) && kinematics.max_linear_m_s>0 &&
        isfinite(ARM_MAX_TRACKING_ERROR_DEG) && ARM_MAX_TRACKING_ERROR_DEG>0 &&
        ARM_COMMAND_TIMEOUT_MS>0 && ARM_FEEDBACK_TIMEOUT_MS>0 && ARM_ERROR_PERIOD_MS>0 &&
        ARM_PERIOD_MS>0 && ARM_PERIOD_MS<=ARM_MAX_PERIOD_MS;
    bool calibrated=ARM_JOINT_CALIBRATION_VALID!=0;
    for(unsigned i=0;i<5;++i) {
        const JointCalibration *c=&calibration[i];slots[i]=SERVO_COUNT;
        for(unsigned j=0;j<SERVO_COUNT;++j)if(Servo_GetConfig(j)->id==c->id)slots[i]=(uint8_t)j;
        if(slots[i]>=SERVO_COUNT || c->id==255U || (c->direction!=1 && c->direction!=-1) ||
            !isfinite(c->ratio) || c->ratio<=0)config_valid=false;
        else all_slots|=(uint8_t)(1U<<slots[i]);
        for(unsigned j=0;j<i;++j)if(c->id==calibration[j].id)config_valid=false;
        if(c->reference_p<500 || c->reference_p>2500 || !isfinite(c->reference_rad) ||
            !isfinite(c->minimum_rad) || !isfinite(c->maximum_rad) || c->minimum_rad>=c->maximum_rad ||
            c->expected_mode<1U || c->expected_mode>4U)calibrated=false;
        if(i<4)kinematics.joint_limit_rad_s[i]=servo_speed_limit_rad_s/c->ratio;
    }
    const float zero[4]={0};ArmKinSolution check;
    if(ArmKin_SolveVelocity(&kinematics,zero,zero,&check)!=ARM_KIN_OK)config_valid=false;
    status.calibrated=calibrated;publish(now);
}
static float rad_per_p(uint8_t mode)
{ return ARM_DEG_TO_RAD(mode<=2U ? 270.0f : 180.0f)/2000.0f; }
/* 一次读取全组快照；时间戳各自保留，不能假装五台同步测量。 */
static ArmError read_feedback(uint32_t now,unsigned *bad,float *value)
{
    Servo_CopySnapshot(&feedback);
    if(!feedback.initialized || !feedback.enabled || !feedback.uart.ready)return ARM_ERROR_NOT_READY;
    uint32_t min_age=UINT32_MAX,max_age=0;
    for(unsigned i=0;i<5;++i) {
        const JointCalibration *c=&calibration[i];const ServoState *s=&feedback.device[slots[i]];
        uint32_t age=now-s->position_ms;*bad=i+1U;*value=(float)age;
        if(s->id!=c->id || !s->online || !s->position_seen || age>=ARM_FEEDBACK_TIMEOUT_MS)return ARM_ERROR_FEEDBACK_STALE;
        if(!s->mode_valid || s->mode!=c->expected_mode || (status.state==ARM_RUNNING && modes[i]!=s->mode)) { *value=s->mode;return ARM_ERROR_BAD_MODE; }
        float q=c->reference_rad+c->direction*((float)s->position-c->reference_p)*rad_per_p(s->mode)/c->ratio;
        status.q_rad[i]=q;
        const ServoConfig *limits=Servo_GetConfig(slots[i]);
        if(!isfinite(q) || q<c->minimum_rad || q>c->maximum_rad || s->position<limits->minimum || s->position>limits->maximum) {
            *value=ARM_RAD_TO_DEG(q);return ARM_ERROR_JOINT_LIMIT;
        }
        if(age<min_age)min_age=age;
        if(age>max_age)max_age=age;
    }
    *bad=0;*value=(float)(max_age-min_age);
    return max_age-min_age>ARM_MAX_FEEDBACK_SKEW_MS ? ARM_ERROR_FEEDBACK_SKEW : ARM_ERROR_NONE;
}
static bool live_zero(const Mailbox *m,uint32_t now)
{ return m->received && (uint32_t)(now-m->time_ms)<ARM_COMMAND_TIMEOUT_MS && finite_command(&m->command) && zero_command(&m->command); }
void Arm_Process(uint32_t now)
{
    Mailbox m;
    taskENTER_CRITICAL();m=mailbox;mailbox.enable=mailbox.stop=mailbox.clear=false;taskEXIT_CRITICAL();
    if(Arm_ResetRequired()) {
        /* 也拦截超限发生前已经提交的Enable/Clear，停止请求仍可执行但不会清错。 */
        if(m.stop)stop_outputs();
        previous_ms=now;publish(now);return;
    }
    if(m.stop) {
        stop_outputs();
        if(status.state!=ARM_FAULT)status.state=ARM_IDLE;
        m.enable=false;
    }
    if(status.state==ARM_IDLE && owns_servos && Servo_ControlRelease(SERVO_OWNER_ARM))owns_servos=false;
    if(status.state==ARM_FAULT) {
        if(m.clear && !m.stop && config_valid && status.calibrated && live_zero(&m,now)) {
            unsigned bad=0;float value=0;
            if(read_feedback(now,&bad,&value)==ARM_ERROR_NONE &&
                (!owns_servos || !Servo_ControlStopPending(SERVO_OWNER_ARM))) {
                taskENTER_CRITICAL();
                /* 清错只能使用仍为当前代的零速度，不能清掉新指令产生的保护条件。 */
                bool unchanged=mailbox.sequence==m.sequence && !mailbox.stop && !mailbox.enable && live_zero(&mailbox,HAL_GetTick());
                bool released=!owns_servos;
                if(unchanged && owns_servos) {
                    Servo_ControlClearInhibit(SERVO_OWNER_ARM);
                    released=Servo_ControlRelease(SERVO_OWNER_ARM);
                }
                if(unchanged && released) {
                    owns_servos=false;status.state=ARM_IDLE;status.error=ARM_ERROR_NONE;status.joint=status.servo_id=0;
                    status.requested=status.limit=0;
                }
                taskEXIT_CRITICAL();
            }
        }
        previous_ms=now;publish(now);return;
    }
    if(status.state==ARM_IDLE && !m.enable) { previous_ms=now;publish(now);return; }
    if(!config_valid) { fault(ARM_ERROR_BAD_CONFIG,0,0,0);publish(now);return; }
    if(!status.calibrated) { fault(ARM_ERROR_UNCALIBRATED,0,0,0);publish(now);return; }
    if(!m.received || (uint32_t)(now-m.time_ms)>=ARM_COMMAND_TIMEOUT_MS) {
        fault(ARM_ERROR_COMMAND_TIMEOUT,0,(float)(now-m.time_ms),ARM_COMMAND_TIMEOUT_MS);publish(now);return;
    }
    if(!finite_command(&m.command)) { fault(ARM_ERROR_INVALID_COMMAND,0,0,0);publish(now);return; }
    unsigned bad=0;float value=0;ArmError health=read_feedback(now,&bad,&value);
    if(health!=ARM_ERROR_NONE) {
        float limit=health==ARM_ERROR_FEEDBACK_STALE ? ARM_FEEDBACK_TIMEOUT_MS :
            health==ARM_ERROR_FEEDBACK_SKEW ? ARM_MAX_FEEDBACK_SKEW_MS : 0.0f;
        if(health==ARM_ERROR_BAD_MODE && bad>=1U && bad<=5U)limit=calibration[bad-1U].expected_mode;
        if(health==ARM_ERROR_JOINT_LIMIT && bad>=1U && bad<=5U) {
            const JointCalibration *c=&calibration[bad-1U];
            limit=ARM_RAD_TO_DEG(status.q_rad[bad-1U]<c->minimum_rad ? c->minimum_rad : c->maximum_rad);
        }
        fault(health,bad,value,limit);publish(now);return;
    }
    if(owns_servos && Servo_ControlInhibited(SERVO_OWNER_ARM)) { fault(ARM_ERROR_DRIVER,0,0,0);publish(now);return; }
    if(status.state==ARM_IDLE) {
        if(!live_zero(&m,now)) { fault(ARM_ERROR_INVALID_COMMAND,0,0,0);publish(now);return; }
        taskENTER_CRITICAL();
        bool unchanged=mailbox.sequence==m.sequence && !mailbox.stop && !mailbox.clear && live_zero(&mailbox,HAL_GetTick());
        if(!unchanged) { taskEXIT_CRITICAL();publish(now);return; }
        /* 同一owner也不能在上一轮停止尚未完成时提前重启。 */
        if(owns_servos || Servo_ControlStopPending(SERVO_OWNER_ARM)) {
            taskEXIT_CRITICAL();fault(ARM_ERROR_CONTROL_BUSY,0,0,0);publish(now);return;
        }
#if ARM_VELOCITY_IK_OUTPUT_ENABLE
        if(!Servo_ControlClaim(SERVO_OWNER_ARM,all_slots)) { taskEXIT_CRITICAL();fault(ARM_ERROR_CONTROL_BUSY,0,0,0);publish(now);return; }
        owns_servos=true;
#endif
        for(unsigned i=0;i<5;++i) {
            status.target_rad[i]=status.q_rad[i];last_p[i]=feedback.device[slots[i]].position;modes[i]=feedback.device[slots[i]].mode;
        }
        status.state=ARM_RUNNING;previous_ms=now;taskEXIT_CRITICAL();publish(now);return;
    }
    float speed=hypotf(m.command.vx_m_s,m.command.vz_m_s);
    if(!isfinite(speed) || speed>kinematics.max_linear_m_s) {
        fault(ARM_ERROR_LINEAR_SPEED_LIMIT,0,speed*1000.0f,ARM_MAX_LINEAR_SPEED_MM_S);publish(now);return;
    }
    if(fabsf(m.command.gripper_rad_s)>servo_speed_limit_rad_s) {
        fault(ARM_ERROR_JOINT_SPEED_LIMIT,5,ARM_RAD_TO_DEG(fabsf(m.command.gripper_rad_s)),SERVO_MAX_SPEED_DEG_S);publish(now);return;
    }
    for(unsigned i=0;i<5;++i)if(fabsf(status.target_rad[i]-status.q_rad[i])>ARM_DEG_TO_RAD(ARM_MAX_TRACKING_ERROR_DEG)) {
        fault(ARM_ERROR_TRACKING,i+1U,ARM_RAD_TO_DEG(fabsf(status.target_rad[i]-status.q_rad[i])),ARM_MAX_TRACKING_ERROR_DEG);publish(now);return;
    }
    uint32_t elapsed=now-previous_ms;
    if(elapsed<ARM_PERIOD_MS) { publish(now);return; }
    if(elapsed>ARM_MAX_PERIOD_MS) { fault(ARM_ERROR_TIMING,0,(float)elapsed,ARM_MAX_PERIOD_MS);publish(now);return; }
    const float desired[4]={m.command.vx_m_s,m.command.vz_m_s,m.command.pitch_rad_s,m.command.roll_rad_s};
    ArmKinSolution solution;
    ArmKinResult result=ArmKin_SolveVelocity(&kinematics,status.q_rad,desired,&solution);
    memcpy(status.achieved_velocity,solution.achieved_velocity,sizeof(status.achieved_velocity));
    status.linear_residual_m_s=solution.linear_residual_m_s;status.angular_residual_rad_s=solution.angular_residual_rad_s;
    if(result!=ARM_KIN_OK) {
        if(result==ARM_KIN_JOINT_SPEED_LIMIT && solution.offending_joint<4U) {
            unsigned i=solution.offending_joint;
            fault(ARM_ERROR_JOINT_SPEED_LIMIT,i+1U,ARM_RAD_TO_DEG(fabsf(solution.joint_velocity[i])*calibration[i].ratio),SERVO_MAX_SPEED_DEG_S);
        } else if(result==ARM_KIN_UNREACHABLE)fault(ARM_ERROR_VELOCITY_UNACHIEVABLE,0,solution.linear_residual_m_s*1000,ARM_LINEAR_RESIDUAL_MM_S);
        else if(result==ARM_KIN_NUMERIC)fault(ARM_ERROR_NUMERIC,0,0,0);
        else fault(ARM_ERROR_BAD_CONFIG,0,0,0);
        publish(now);return;
    }
    float next[5],dq[5];uint8_t ids[5];uint16_t positions[5],times[5];bool changed=false;
    memcpy(dq,solution.joint_velocity,sizeof(solution.joint_velocity));dq[4]=m.command.gripper_rad_s/calibration[4].ratio;
    for(unsigned i=0;i<5;++i) {
        const JointCalibration *c=&calibration[i];const ServoConfig *limits=Servo_GetConfig(slots[i]);
        next[i]=status.target_rad[i]+dq[i]*(float)elapsed*0.001f;
        if(!isfinite(next[i]) || next[i]<c->minimum_rad || next[i]>c->maximum_rad) {
            fault(ARM_ERROR_JOINT_LIMIT,i+1U,ARM_RAD_TO_DEG(next[i]),ARM_RAD_TO_DEG(next[i]<c->minimum_rad?c->minimum_rad:c->maximum_rad));publish(now);return;
        }
        float p=c->reference_p+c->direction*(next[i]-c->reference_rad)*c->ratio/rad_per_p(modes[i]);
        if(!isfinite(p) || p<limits->minimum || p>limits->maximum) { fault(ARM_ERROR_JOINT_LIMIT,i+1U,ARM_RAD_TO_DEG(next[i]),0);publish(now);return; }
        ids[i]=c->id;positions[i]=(uint16_t)lroundf(p);changed|=positions[i]!=last_p[i];
        float rounded_q=c->reference_rad+c->direction*((float)positions[i]-c->reference_p)*rad_per_p(modes[i])/c->ratio;
        if(rounded_q<c->minimum_rad || rounded_q>c->maximum_rad) {
            fault(ARM_ERROR_JOINT_LIMIT,i+1U,ARM_RAD_TO_DEG(rounded_q),ARM_RAD_TO_DEG(rounded_q<c->minimum_rad?c->minimum_rad:c->maximum_rad));publish(now);return;
        }
        if(fmaxf(fabsf(next[i]-status.q_rad[i]),fabsf(rounded_q-status.q_rad[i]))>ARM_DEG_TO_RAD(ARM_MAX_TRACKING_ERROR_DEG)) {
            fault(ARM_ERROR_TRACKING,i+1U,ARM_RAD_TO_DEG(fabsf(rounded_q-status.q_rad[i])),ARM_MAX_TRACKING_ERROR_DEG);publish(now);return;
        }
        /* 量化后的P指令也设置运动时间下限；依据最近回读位置，不能据此保证实际瞬时转速。 */
        float delta=fabsf((float)positions[i]-feedback.device[slots[i]].position)*rad_per_p(modes[i]);
        float ms=ceilf(delta/servo_speed_limit_rad_s*1000.0f);
        if(!isfinite(ms) || ms>9999.0f) { fault(ARM_ERROR_TRACKING,i+1U,ARM_RAD_TO_DEG(delta),ARM_MAX_TRACKING_ERROR_DEG);publish(now);return; }
        times[i]=(uint16_t)(ms<ARM_PERIOD_MS ? ARM_PERIOD_MS : ms);
    }
    /* 计算期间上层可能提交了停止或新命令，提交前再次核对代数，避免旧动作越过停止请求。 */
    taskENTER_CRITICAL();
    bool current=mailbox.sequence==m.sequence && !mailbox.stop && !mailbox.clear;
    ServoResult sent=SERVO_QUEUED;
    if(current && changed && owns_servos)sent=Servo_ControlMoveGroup(SERVO_OWNER_ARM,ids,positions,times,5U);
    if(current && sent==SERVO_QUEUED) {
        memcpy(status.target_rad,next,sizeof(next));memcpy(status.joint_velocity_rad_s,dq,sizeof(dq));memcpy(last_p,positions,sizeof(last_p));previous_ms=now;
    }
    taskEXIT_CRITICAL();
    if(current && sent!=SERVO_QUEUED)fault(ARM_ERROR_DRIVER,0,(float)sent,0);
    publish(now);
}
#endif
