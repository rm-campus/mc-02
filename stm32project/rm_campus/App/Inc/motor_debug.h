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
    int16_t lateral_stick; /* 平移模式的 CH4，用于启用前回中；其他模式填 0。 */
    float target_rad_s;   /* 底盘任务给本轮的输出轴角速度。 */
    float speed, measured_current; /* C620 反馈：减速后输出轴角速度 rad/s、电流 A。 */
    uint8_t temperature;
} MotorDebugInput;

typedef struct {
    MotorDebugState state;
    MotorDebugReason reason;
    float target, reference, integral_a, output_a;
    int16_t command_raw; /* 本轮计算出的 C620 指令，16384 对应 20 A；由底盘任务统一发送。 */
    uint32_t center_ms;
    bool off_seen, centering;
} MotorDebug;

/** 返回按编译配置生成的参数副本，含单电机模式所选 ID 的 PI 系数。 */
MotorDebugConfig MotorDebug_DefaultConfig(void);
/* 四轮模式 ID1~4 的配置；公共参数与单电机一致，仅独立覆盖 ID/Kp/Ki。 */
MotorDebugConfig MotorDebug_WheelConfig(uint8_t motor_id);
/** 清除指定轮的积分、输出和拨杆确认记录，初始状态为 WAIT_OFF。 */
void MotorDebug_Init(MotorDebug *debug);
/**
 * @brief 用一个控制周期的输入推进一台电机的启停状态和速度 PI。
 * @param[in,out] debug 该轮跨周期保存的状态；计算结果写回 output_a 和 command_raw。
 * @param[in] config 该轮的 PI 参数、速度/电流上限和启用条件。
 * @param[in] input 本次摇杆、拨杆、轮速反馈及时间戳。period_ms 是两次控制的实际间隔。
 * 参数指针为空时直接返回。输入故障会清积分和指令；恢复后需重新确认关闭档和回中。
 */
void MotorDebug_Step(MotorDebug *debug, const MotorDebugConfig *config,
                     const MotorDebugInput *input);
/* CAN_FAULT 保护开启时，发送入队失败清状态；关闭时此函数不修改控制状态。 */
void MotorDebug_TxFailed(MotorDebug *debug);
#endif
