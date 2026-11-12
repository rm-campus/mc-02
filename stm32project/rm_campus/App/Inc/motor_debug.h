/** 与 HAL/RTOS 无关的单电机速度 PI 与启停状态机。单位：rad/s、A、s。 */
#ifndef MOTOR_DEBUG_H
#define MOTOR_DEBUG_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MOTOR_DEBUG_DISABLED = 0,
    MOTOR_DEBUG_WAIT_OFF,
    MOTOR_DEBUG_WAIT_CENTER,
    MOTOR_DEBUG_RUNNING
} MotorDebugState;

typedef enum {
    MOTOR_DEBUG_OK = 0,
    MOTOR_DEBUG_BAD_CONFIG,
    MOTOR_DEBUG_RC_OFFLINE,
    MOTOR_DEBUG_MOTOR_OFFLINE,
    MOTOR_DEBUG_CAN_FAULT,
    MOTOR_DEBUG_TIMING_FAULT,
    MOTOR_DEBUG_TEMPERATURE,
    MOTOR_DEBUG_SPEED_FAULT,
    MOTOR_DEBUG_RUN_TIMEOUT
} MotorDebugReason;

typedef struct {
    bool enabled;
    uint8_t motor_id;
    float kp, ki, max_speed, accel, current_limit, integral_limit;
    float overspeed;
    int16_t deadband;
    uint8_t max_temperature;
    uint32_t arm_hold_ms, max_run_ms, rc_timeout_ms, feedback_timeout_ms, max_period_ms;
} MotorDebugConfig;

typedef struct {
    uint32_t now_ms, period_ms;
    bool rc_valid, rc_received, motor_received, can_ok;
    uint32_t rc_ms, feedback_ms;
    int8_t enable_switch; /* -1 关闭，0 中档，1 启用。 */
    int16_t stick;        /* -660～660。 */
    float speed, measured_current;
    uint8_t temperature;
} MotorDebugInput;

typedef struct {
    MotorDebugState state;
    MotorDebugReason reason;
    float target, reference, integral_a, output_a;
    int16_t command_raw;
    uint32_t center_ms, run_ms;
    bool off_seen, centering;
} MotorDebug;

MotorDebugConfig MotorDebug_DefaultConfig(void);
void MotorDebug_Init(MotorDebug *debug);
/* 每周期调用一次；任何不满足的安全条件都会清 PI/斜坡并输出零。
 * 故障后必须重新观察关闭档，正常状态还需启用档摇杆回中保持 arm_hold_ms。 */
void MotorDebug_Step(MotorDebug *debug, const MotorDebugConfig *config,
                     const MotorDebugInput *input);
/* 本周期 CAN 入队失败时立即清状态；下一周期发送四路零电流。 */
void MotorDebug_TxFailed(MotorDebug *debug);
#endif
