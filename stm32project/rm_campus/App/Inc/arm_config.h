/** @file arm_config.h
 * @brief 速度逆运动学预备模块配置；长度统一为米，内部角度统一为弧度。
 * 最大速度是软件允许值，不是7.4V下尚未确认的舵机硬件极限。
 */
#ifndef ARM_CONFIG_H
#define ARM_CONFIG_H
#include "servo_config.h"
#ifndef ARM_VELOCITY_IK_ENABLE
#define ARM_VELOCITY_IK_ENABLE 0
#endif
#ifndef ARM_VELOCITY_IK_OUTPUT_ENABLE
#define ARM_VELOCITY_IK_OUTPUT_ENABLE 0
#endif
#ifndef ARM_JOINT_CALIBRATION_VALID
#define ARM_JOINT_CALIBRATION_VALID 0
#endif
/* 用户主要调整这两项：舵机输出轴角速度、腕交点的平移合速度。 */
#ifndef SERVO_MAX_SPEED_DEG_S
#define SERVO_MAX_SPEED_DEG_S 30.0f
#endif
/* 腕参考点平移合速度上限，单位mm/s；控制点为q3/q4交点，不是尚未定义的TCP。
 * 当前几何：L1=175.965097 mm，L2=111.162186 mm；关节传动比均为1:1。
 * 当前舵机软件限速Ω=SERVO_MAX_SPEED_DEG_S=30°/s，ω=Ω×π/180 rad/s。
 * 允许末端俯仰变化时，跨所有姿态取最大值所得的理论上界：
 *   v_max=(L1+2×L2)×ω≈208.544 mm/s。
 * 保持末端俯仰不变时，腕俯仰还需补偿肩肘运动，其同样限速30°/s：
 *   v_max=(L1+L2)×ω≈150.339 mm/s。
 * 上界仅在特定姿态的切向运动中可达到，不保证低于此值的指令在任意姿态/方向可执行。
 * 实际仍逐周期检查未限幅关节速度、任务残差及机械限位；本参数目前保留50 mm/s。
 * 修改舵机限速、连杆长度或传动比后应重新计算，不能继续套用上述数值。
 * 这些是软件限速下的运动学上界，不是7.4V下舵机硬件极限。
 */
#ifndef ARM_MAX_LINEAR_SPEED_MM_S
#define ARM_MAX_LINEAR_SPEED_MM_S 50.0f
#endif
#define ARM_PI 3.14159265358979323846f
#define ARM_DEG_TO_RAD(x) ((x)*(ARM_PI/180.0f))
#define ARM_RAD_TO_DEG(x) ((x)*(180.0f/ARM_PI))
#define ARM_L12_M 0.175965097f
#define ARM_L23_M 0.111162186f
/* 已知腕轴偏置保留供后续工具模型使用，不把它假定为TCP距离。 */
#define ARM_WRIST_HORN_OFFSET_M 0.072f
#define ARM_PERIOD_MS 20U
#define ARM_MAX_PERIOD_MS 50U
#define ARM_COMMAND_TIMEOUT_MS 100U
#define ARM_FEEDBACK_TIMEOUT_MS SERVO_POSITION_STALE_MS
#define ARM_MAX_FEEDBACK_SKEW_MS 100U
#define ARM_ERROR_PERIOD_MS 200U
#define ARM_DLS_DAMPING 0.005f
#define ARM_LINEAR_RESIDUAL_MM_S 0.5f
#define ARM_ANGULAR_RESIDUAL_DEG_S 0.5f
#define ARM_MAX_TRACKING_ERROR_DEG 5.0f
/* 顺序：肩、肘、腕俯仰、腕滚转、夹爪。ID4/5同型号，可交换配置。
 * 字段：ID、P增加的机械正向(+1/-1)、舵机角/关节角传动比、参考P、
 * 参考P对应的数学关节角、机械最小角、机械最大角（角度字段均rad）、标定时PMOD(1..4)。
 * 参考P=0、上下限相同是“未标定”占位，不能只把VALID置1就上机。
 * q1/q2数学零位为两主连杆沿基座+x伸直；q3以腕轴俯仰定义；不是CAD当前姿态。
 */
#ifndef ARM_JOINT_CALIBRATION_TABLE
#define ARM_JOINT_CALIBRATION_TABLE \
    {3U, 1, 1.0f, 0U, 0.0f, 0.0f, 0.0f, 0U}, \
    {2U, 1, 1.0f, 0U, 0.0f, 0.0f, 0.0f, 0U}, \
    {1U, 1, 1.0f, 0U, 0.0f, 0.0f, 0.0f, 0U}, \
    {4U, 1, 1.0f, 0U, 0.0f, 0.0f, 0.0f, 0U}, \
    {5U, 1, 1.0f, 0U, 0.0f, 0.0f, 0.0f, 0U}
#endif
#if (ARM_VELOCITY_IK_ENABLE != 0 && ARM_VELOCITY_IK_ENABLE != 1) || \
    (ARM_VELOCITY_IK_OUTPUT_ENABLE != 0 && ARM_VELOCITY_IK_OUTPUT_ENABLE != 1) || \
    (ARM_JOINT_CALIBRATION_VALID != 0 && ARM_JOINT_CALIBRATION_VALID != 1)
#error "ARM switches must be 0 or 1"
#endif
#if ARM_VELOCITY_IK_OUTPUT_ENABLE && (!ARM_VELOCITY_IK_ENABLE || !SERVO_BUS_ENABLE || SERVO_RC_ENABLE)
#error "ARM output requires ARM enabled, servo bus enabled and SERVO_RC_ENABLE=0"
#endif
#endif
