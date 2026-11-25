#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H
#include "robot_control_config.h"
#ifndef SERVO_BUS_ENABLE
#define SERVO_BUS_ENABLE 1
#endif
#ifndef SERVO_USB_DEBUG_ENABLE
#define SERVO_USB_DEBUG_ENABLE (!ROBOT_CONTROL_ENABLE)
#endif
#ifndef SERVO_RC_ENABLE
#define SERVO_RC_ENABLE (!ROBOT_CONTROL_ENABLE)
#endif
#ifndef SERVO_RC_TELEMETRY_ENABLE
#define SERVO_RC_TELEMETRY_ENABLE (!ROBOT_CONTROL_ENABLE)
#endif
#if ROBOT_CONTROL_ENABLE && (!SERVO_BUS_ENABLE || SERVO_RC_ENABLE || SERVO_USB_DEBUG_ENABLE || SERVO_RC_TELEMETRY_ENABLE)
#error "Robot mode requires exclusive use of the servo bus"
#endif
#define SERVO_RC_PERIOD_MS 20U
#define SERVO_RC_TELEMETRY_MS 20U
#define SERVO_RC_DEADBAND 60
#define SERVO_RC_CENTER_HOLD_MS 300U
#define SERVO_RC_TIMEOUT_MS 100U
#define SERVO_RC_MAX_PERIOD_MS 50U
/* ID1..5 的摇杆正向对应 P 增加(+1)或减小(-1)，最大协议角速度 deg/s。
 * 不增加底盘 yaw 舵机。下方位置范围仍是协议范围，装配后必须逐轴收窄。 */
#define SERVO_RC_AXIS_TABLE \
    {1, 30.0f}, {1, 30.0f}, {1, 30.0f}, {1, 30.0f}, {1, 30.0f}
#define SERVO_COUNT 5U
/* 先逐台配置 ID，再并联。这里只描述设备，不向舵机写 ID/波特率/模式/PID。 */
#define SERVO_DEVICE_TABLE \
    {1U, "ZP30", 500U, 2500U}, \
    {2U, "ZP35", 500U, 2500U}, \
    {3U, "ZP45", 500U, 2500U}, \
    {4U, "ZP25S", 500U, 2500U}, \
    {5U, "ZP25S", 500U, 2500U}
/* 上表是协议限位，安装后的机械软限位必须按各关节标定后收窄。 */
#define SERVO_QUEUE_SIZE 16U
#define SERVO_RX_RING_SIZE 256U
#define SERVO_TX_TIMEOUT_MS 20U
#define SERVO_REPLY_TIMEOUT_MS 20U
#define SERVO_MODE_REPLY_TIMEOUT_MS 100U
#define SERVO_MODE_PROBE_TIMEOUT_MS 1000U
#define SERVO_POLL_SLOT_MS 10U
#define SERVO_POLL_PERIOD_MS 50U
#define SERVO_OFFLINE_RETRY_MS 1000U
#define SERVO_FAIL_LIMIT 3U
#define SERVO_POSITION_STALE_MS 200U
#define SERVO_MOVE_INTERVAL_MS 20U
#define SERVO_RECOVERY_QUIET_MS 30U
#define SERVO_BUS_GAP_MS 1U
#define SERVO_TASK_FLAG 1U
#if (SERVO_BUS_ENABLE != 0 && SERVO_BUS_ENABLE != 1) || (SERVO_USB_DEBUG_ENABLE != 0 && SERVO_USB_DEBUG_ENABLE != 1)
#error "Invalid servo enable switch"
#endif
#if (SERVO_RC_ENABLE != 0 && SERVO_RC_ENABLE != 1) || (SERVO_RC_TELEMETRY_ENABLE != 0 && SERVO_RC_TELEMETRY_ENABLE != 1)
#error "Invalid servo RC switch"
#endif
#endif
