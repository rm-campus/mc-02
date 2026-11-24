#ifndef ARM_VELOCITY_H
#define ARM_VELOCITY_H
#include "arm_config.h"
#include <stdbool.h>
#include <stdint.h>
typedef enum { ARM_DISABLED, ARM_IDLE, ARM_RUNNING, ARM_FAULT } ArmState;
typedef enum {
    ARM_ERROR_NONE, ARM_ERROR_BAD_CONFIG, ARM_ERROR_UNCALIBRATED,
    ARM_ERROR_NOT_READY, ARM_ERROR_FEEDBACK_STALE, ARM_ERROR_FEEDBACK_SKEW,
    ARM_ERROR_BAD_MODE, ARM_ERROR_COMMAND_TIMEOUT, ARM_ERROR_INVALID_COMMAND,
    ARM_ERROR_TIMING, ARM_ERROR_LINEAR_SPEED_LIMIT, ARM_ERROR_JOINT_SPEED_LIMIT,
    ARM_ERROR_VELOCITY_UNACHIEVABLE, ARM_ERROR_JOINT_LIMIT,
    ARM_ERROR_TRACKING, ARM_ERROR_DRIVER, ARM_ERROR_CONTROL_BUSY, ARM_ERROR_NUMERIC
} ArmError;
/* 一次提交所有速度，避免只刷新夹爪时意外延续过期的非零平移速度。
 * vx/vz是q3/q4交点在固定基座主运动平面的速度；pitch/roll不是任意世界角速度。
 * gripper_rad_s幅值为夹爪舵机轴角速度，正负按标定的机械正向；未定义夹口宽度或TCP。 */
typedef struct {
    float vx_m_s, vz_m_s, pitch_rad_s, roll_rad_s, gripper_rad_s;
} ArmVelocityCommand;
typedef struct {
    uint32_t now_ms, fault_sequence;
    ArmState state;
    ArmError error;
    bool output_enabled, calibrated;
    uint8_t joint, servo_id; /* joint=1..5，0表示整组错误。 */
    float requested, limit; /* 单位见error：速度超限用deg/s或mm/s，角度限制用deg。 */
    float q_rad[5], target_rad[5], joint_velocity_rad_s[5];
    float achieved_velocity[4], linear_residual_m_s, angular_residual_rad_s;
} ArmStatus;
#if ARM_VELOCITY_IK_ENABLE
/* 由ServoBusTask调用Init/Process；速度超限后重复Init也不能清错，必须复位MCU。 */
void Arm_Init(uint32_t now);
void Arm_Process(uint32_t now);
/* 返回true表示已提交请求，不表示已经通过条件检查或实际开始运动。 */
bool Arm_SetVelocity(const ArmVelocityCommand *command);
bool Arm_Enable(void); /* 速度超限复位锁存期间返回false。 */
void Arm_Stop(void);
/* 仅处理可恢复错误：先提交新鲜零速度，清错后仍需显式Enable。
 * LINEAR_SPEED_LIMIT/JOINT_SPEED_LIMIT只能复位MCU清除，本接口返回false。 */
bool Arm_ClearFault(void);
bool Arm_ResetRequired(void); /* 只读查询复位锁存，不提供软件清除此标志的接口。 */
void Arm_GetStatus(ArmStatus *out);
const char *Arm_ErrorName(ArmError error);
#else
static inline void Arm_Init(uint32_t now) { (void)now; }
static inline void Arm_Process(uint32_t now) { (void)now; }
static inline bool Arm_SetVelocity(const ArmVelocityCommand *c) { (void)c;return false; }
static inline bool Arm_Enable(void) { return false; }
static inline void Arm_Stop(void) {}
static inline bool Arm_ClearFault(void) { return false; }
static inline bool Arm_ResetRequired(void) { return false; }
static inline void Arm_GetStatus(ArmStatus *out) { if(out)*out=(ArmStatus){.state=ARM_DISABLED}; }
static inline const char *Arm_ErrorName(ArmError error) { (void)error;return "DISABLED"; }
#endif
#endif
