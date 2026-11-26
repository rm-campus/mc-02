#ifndef ROBOT_CONTROL_CONFIG_H
#define ROBOT_CONTROL_CONFIG_H

/* Debug/Release 预设默认开启舵机监测；servo_config.h 单独控制点动测试。 */
#ifndef SERVO_STATUS_ONLY
#define SERVO_STATUS_ONLY 0
#endif
#if SERVO_STATUS_ONLY != 0 && SERVO_STATUS_ONLY != 1
#error "SERVO_STATUS_ONLY must be 0 or 1"
#endif

/* 置 1 选择整机遥控，置 0 使用原调试配置。RobotControl 构建预设单独置 1。 */
#ifndef ROBOT_CONTROL_ENABLE
#define ROBOT_CONTROL_ENABLE 0
#endif
#ifndef ROBOT_SERVO_TELEMETRY_ENABLE
#define ROBOT_SERVO_TELEMETRY_ENABLE 0
#endif
#if ROBOT_SERVO_TELEMETRY_ENABLE && !ROBOT_CONTROL_ENABLE
#error "Robot servo telemetry requires robot control"
#endif
#if SERVO_STATUS_ONLY && (ROBOT_CONTROL_ENABLE || defined(SERVO_ID_SETUP))
#error "Servo status monitor is exclusive with robot control and ID setup"
#endif
#if ROBOT_CONTROL_ENABLE != 0 && ROBOT_CONTROL_ENABLE != 1
#error "ROBOT_CONTROL_ENABLE must be 0 or 1"
#endif
#if ROBOT_CONTROL_ENABLE && defined(SERVO_ID_SETUP)
#error "Robot control and servo ID setup are exclusive"
#endif

/* CH9 的物理上、中、下档；若遥控器通道反向，交换上下档两个值。 */
#define ROBOT_SWITCH_UP       1
#define ROBOT_SWITCH_MIDDLE   0
#define ROBOT_SWITCH_DOWN   (-1)
#define ROBOT_LINK_MIN_FRAMES 3U
#define ROBOT_LINK_MIN_MS    50U
#define ROBOT_LINK_TIMEOUT_MS 100U
#define ROBOT_SWITCH_STABLE_MS 30U

/* 仅用于推杆时延后后台轮询，不改变关节的摇杆死区。 */
#define ROBOT_SERVO_DEFER_POLL_STICK 20

/* 整机底盘 CH2/CH4 平移死区，CH1 自转死区在 motor_debug_config.h 单独设置。 */
#ifndef ROBOT_CHASSIS_DEADBAND
#define ROBOT_CHASSIS_DEADBAND 60
#endif
#if ROBOT_CHASSIS_DEADBAND < 0 || ROBOT_CHASSIS_DEADBAND >= 660
#error "Robot chassis translation deadband must be in 0..659"
#endif

/* 轮端 rpm、车体角速度 °/s。上下档各自可调。尺寸单位 m，轮心到轮心的全距。 */
#ifndef ROBOT_CHASSIS_MAX_WHEEL_RPM
#define ROBOT_CHASSIS_MAX_WHEEL_RPM 400.0f
#endif
#ifndef ROBOT_UPPER_MAX_YAW_DEG_S
#define ROBOT_UPPER_MAX_YAW_DEG_S 90.0f
#endif
#ifndef ROBOT_LOWER_MAX_YAW_DEG_S
#define ROBOT_LOWER_MAX_YAW_DEG_S 30.0f
#endif
#define ROBOT_WHEEL_RADIUS_M 0.075f
/* 未填实车尺寸时按旧麦轮的轮端转速比例转向；上、下档可分别调整。 */
#ifndef ROBOT_UPPER_TURN_FRACTION
#define ROBOT_UPPER_TURN_FRACTION 1.0f
#endif
#ifndef ROBOT_LOWER_TURN_FRACTION
#define ROBOT_LOWER_TURN_FRACTION (1.0f / 3.0f)
#endif
#ifndef ROBOT_WHEELBASE_M
#define ROBOT_WHEELBASE_M 0.0f
#endif
#ifndef ROBOT_TRACK_M
#define ROBOT_TRACK_M 0.0f
#endif
/* 暂按现有 ID5 协议上限试配，尚未验证闭合方向/机械端点；标定后填实测 P。
 * 可置 0 单独停用夹爪控制。该值绝不用于改写舵机的内部参数。 */
#ifndef ROBOT_GRIPPER_CLOSED_P
#define ROBOT_GRIPPER_CLOSED_P 2500U
#endif
#endif
