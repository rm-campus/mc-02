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
    MOTOR_DEBUG_SPEED_FAULT
} MotorDebugReason;

typedef struct {
    bool enabled;
    uint8_t motor_id;
    float kp, ki, max_speed, current_limit, integral_limit;
    float overspeed;
    int16_t deadband;
    uint8_t max_temperature;
    uint32_t arm_hold_ms, rc_timeout_ms, feedback_timeout_ms, max_period_ms;
} MotorDebugConfig;

typedef struct {
    uint32_t now_ms, period_ms;
    bool rc_valid, rc_received, motor_received, can_ok;
    uint32_t rc_ms, feedback_ms;
    int8_t enable_switch; /* +1 关闭，0 中档，-1 启用。 */
    int16_t stick;        /* -660～660。 */
    float speed, measured_current;
    uint8_t temperature;
} MotorDebugInput;

typedef struct {
    MotorDebugState state;
    MotorDebugReason reason;
    float target, reference, integral_a, output_a;
    int16_t command_raw;
    uint32_t center_ms;
    bool off_seen, centering;
} MotorDebug;

MotorDebugConfig MotorDebug_DefaultConfig(void);
/* 四轮模式 ID1~4 的配置；公共参数与单电机一致，仅独立覆盖 ID/Kp/Ki。 */
MotorDebugConfig MotorDebug_WheelConfig(uint8_t motor_id);
void MotorDebug_Init(MotorDebug *debug);
/* 每周期调用一次；任何不满足的安全条件都会清 PI 状态并输出零。
 * 故障后必须重新观察关闭档，正常状态还需启用档摇杆回中保持 arm_hold_ms。 */
void MotorDebug_Step(MotorDebug *debug, const MotorDebugConfig *config,
                     const MotorDebugInput *input);
/* CAN_FAULT 保护开启时，发送入队失败清状态；关闭时此函数不修改控制状态。 */
void MotorDebug_TxFailed(MotorDebug *debug);
#endif
