#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H
#include "robot_control_config.h"
/* USB 监测固件默认镜像 UART7 实际收到的字节；设 0 可关闭自动输出。 */
#ifndef SERVO_RX_TRACE_ENABLE
#define SERVO_RX_TRACE_ENABLE SERVO_STATUS_ONLY
#endif
#if (SERVO_RX_TRACE_ENABLE != 0 && SERVO_RX_TRACE_ENABLE != 1) || (SERVO_RX_TRACE_ENABLE && !SERVO_STATUS_ONLY)
#error "Servo RX tracing requires the USB servo monitor profile"
#endif
/* 调试监测固件上的定向点动。置 0 后 USB 恢复只接受 servo_status。 */
#ifndef SERVO_JOG_TEST_ENABLE
#define SERVO_JOG_TEST_ENABLE SERVO_STATUS_ONLY
#endif
#if (SERVO_JOG_TEST_ENABLE != 0 && SERVO_JOG_TEST_ENABLE != 1) || (SERVO_JOG_TEST_ENABLE && !SERVO_STATUS_ONLY)
#error "Servo jog is only supported in the USB servo monitor profile"
#endif
/* 正对主动输出轴，方向 1 顺时针、2 逆时针。P 增减取决于舵机的位置模式。 */
#ifndef SERVO_JOG_STEP_DEG
#define SERVO_JOG_STEP_DEG 10.0f
#endif
#ifndef SERVO_JOG_MOVE_MS
#define SERVO_JOG_MOVE_MS 500U
#endif
#define SERVO_JOG_TOLERANCE_DEG 1.0f
#define SERVO_JOG_SETTLE_TIMEOUT_MS 1000U
#if SERVO_JOG_MOVE_MS < 1 || SERVO_JOG_MOVE_MS > 9999
#error "Servo jog duration must fit the T field (1..9999 ms)"
#endif
#ifndef SERVO_BUS_ENABLE
#define SERVO_BUS_ENABLE 1
#endif
#ifndef SERVO_USB_DEBUG_ENABLE
#define SERVO_USB_DEBUG_ENABLE (!ROBOT_CONTROL_ENABLE)
#endif
#ifndef SERVO_RC_ENABLE
#define SERVO_RC_ENABLE (!ROBOT_CONTROL_ENABLE && !SERVO_STATUS_ONLY)
#endif
#ifndef SERVO_RC_TELEMETRY_ENABLE
#define SERVO_RC_TELEMETRY_ENABLE (!ROBOT_CONTROL_ENABLE && !SERVO_STATUS_ONLY)
#endif
#if SERVO_STATUS_ONLY && (!SERVO_BUS_ENABLE || !SERVO_USB_DEBUG_ENABLE || SERVO_RC_ENABLE || SERVO_RC_TELEMETRY_ENABLE)
#error "Servo status monitor requires the servo bus and USB monitoring, with RC disabled"
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
#define SERVO_METADATA_RETRY_MS 1000U /* UART7 全总线的后台模式/版本补读间隔。 */
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
