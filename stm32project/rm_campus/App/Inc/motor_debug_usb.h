#ifndef MOTOR_DEBUG_USB_H
#define MOTOR_DEBUG_USB_H
/** UsbTask 周期调用，以 MOTOR_DEBUG_TELEMETRY_PERIOD_MS 为最小发送间隔（默认 5 ms）。
 * 四轮至少一台 RUNNING 时回传同速九列或平移十二列（四个目标、四个实测、四个电流）。
 * 单轮模式先发 ready，再发目标 rpm、实测 rpm、指令 A、反馈 A；USB 忙时跳过本次采样。 */
void MotorDebugUsb_Process(void);
#endif
