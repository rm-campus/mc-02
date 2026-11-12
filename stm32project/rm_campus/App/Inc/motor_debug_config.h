/** 单电机台架配置。先阅读 docs/单电机速度环调试.md，再启用。 */
#ifndef MOTOR_DEBUG_CONFIG_H
#define MOTOR_DEBUG_CONFIG_H

/* 默认只观测、四路零电流。改为 1 并重新编译才允许启用速度环。 */
#ifndef MOTOR_DEBUG_ENABLE
#define MOTOR_DEBUG_ENABLE 0
#endif
#ifndef MOTOR_DEBUG_MOTOR_ID
#define MOTOR_DEBUG_MOTOR_ID 1U       /* C620 编号 1～4，不是轮子位置。 */
#endif
#ifndef MOTOR_DEBUG_KP
#define MOTOR_DEBUG_KP 0.0f          /* A / (rad/s)，初始为 0，需台架整定。 */
#endif
#ifndef MOTOR_DEBUG_KI
#define MOTOR_DEBUG_KI 0.0f          /* A / rad；实现中已乘 dt，不要重复乘。 */
#endif
#define MOTOR_DEBUG_MAX_SPEED 3.0f   /* 减速箱输出轴 rad/s，约 28.6 rpm。 */
#define MOTOR_DEBUG_ACCEL 2.0f       /* 目标速度斜坡，rad/s^2。 */
#define MOTOR_DEBUG_CURRENT_LIMIT 1.0f /* A，初次空载调试限流。 */
#define MOTOR_DEBUG_INTEGRAL_LIMIT 0.3f /* A，积分项限幅。 */
#define MOTOR_DEBUG_TELEMETRY 1      /* USB CDC 文本，每 100 ms 一行。 */

/* 本机实测 CH7/switch_a 为面板最左拨杆，LOW=-1 关闭，HIGH=1 启用。
 * 右摇杆上下使用 CH3/rocker_r_v。首次使用时通过 USB 核对开关和摇杆。 */
#define MOTOR_DEBUG_STICK_DEADBAND 40
#define MOTOR_DEBUG_ARM_HOLD_MS 300U
#define MOTOR_DEBUG_MAX_RUN_MS 10000U /* 每次最多运行 10 s，重新关闭/回中启用。 */
#define MOTOR_DEBUG_RC_TIMEOUT_MS 100U
#define MOTOR_DEBUG_FEEDBACK_TIMEOUT_MS 50U
#define MOTOR_DEBUG_MAX_PERIOD_MS 5U
#define MOTOR_DEBUG_MAX_TEMPERATURE 70U /* C620 反馈温度的保守调试阈值。 */
#define MOTOR_DEBUG_OVERSPEED 8.0f   /* 实测输出轴速度超过此值撤掉电流。 */

#endif
