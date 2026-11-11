# 校赛基础驱动接入

本次使用《26校内赛电控通用驱动代码文档 (1).md》的 CAN、M3508
公共数据结构和通信实现；SBUS 接收改为 UART5 循环 DMA；LED 保留
0x60/0x78 编码，改用 HAL 带超时发送。没有加入蜂鸣器、M2006 或 PWM 舵机。

## 当前行为

- C620 编号 1～4；FDCAN3 为标准经典帧、1 Mbps。
- ChassisTask 每 1 ms 发送一帧 0x200，四路电流均为零。
- RcTask 用 UART5 接收 FS-i6X/iA6B 的 25 字节 SBUS，超时/失控判无效。
- MonitorTask 每 10 ms 更新状态，StatusTask 每 50 ms 更新 LED。
- ServoBusTask 保留占位，不发送未知 UART 舵机协议。
- ImuTask 每 1 ms 轮询 BMI088 并运行 C 版四元数 EKF；上电需保持静止完成 1000 样本校准。
- UsbTask 服务 USB 收发队列并调用业务接收入口，发送由应用层显式调用。
  IMU 接线、应用接口和错误码见 [IMU_INTEGRATION.md](IMU_INTEGRATION.md)。本次未接入恒温。

这不是完整麦轮控制程序：尚未加入 PID、轮序/方向标定和运动学。
六个模拟通道按文档由 -784～783 映射为 -660～660 整数；开关按原始值负/零/正输出 1/0/-1，不设中位容差。需在实际遥控器上校准端点和开关方向。

## LED

|颜色|含义|
|---|---|
|紫色|CAN 初始化失败或 Bus-Off|
|红色|尚未收到有效遥控、失控或超过 100 ms 未收到有效遥控|
|黄色|遥控正常，但四个电机未全部在线（反馈超时 50 ms）|
|绿色|遥控和四电机在线；依然只发送零电流|

可在调试器调用 `Campus_CopyStatus()`、`Campus_CopyMotors()`、
`Fxi6x_CopyData()` 获取任务侧的一致性快照。电流 SetCurrent 参数是协议原始值，
不是安培。累计角度沿用参考代码，不保证上电首帧为零。

## 工程维护

- 自定义源码位于 Bsp/、App/；顶层 CMakeLists.txt 显式加入编译。
- main.c、freertos.c 接入点位于 USER CODE 中。
- 链接脚本新增 .dma_rx 段放入 RAM_D2。重新生成或替换链接脚本后须保留此段；
  UART DMA 缓冲区不能位于 DTCM。
- DMA 缓冲区独占 32 字节缓存行，已包含未来启用 DCache 时的缓存维护。
- UART5 使用 100000 baud、9 位字长含偶校验、2 停止位；板载 Q1 已反相，RX 不再反相。
- 当前不做 Bus-Off 自动恢复；状态可见，后续结合整车恢复策略实现。
- 下一步：上板核对遥控数据、四台电机反馈和 LED，再实现闭环控制。

## USB CDC 应用接口

应用层包含 `Bsp/Inc/usb_serial.h`，无需操作 HAL/USB 句柄：

| 接口 | 行为 |
|---|---|
| `UsbSerial_IsReady()` | 已枚举且未挂起；不代表电脑打开了串口或正在读取 |
| `UsbSerial_Send(data, length)` | 非阻塞、整段复制入队；返回后可复用调用方缓冲区 |
| `UsbSerial_Read(buffer, capacity)` | 非阻塞，返回实际读取长度；支持二进制和 `00` |
| `UsbSerial_Available()` | 当前可读字节数 |
| `UsbSerial_Printf(format, ...)` | 格式化文本后入队，不自动追加换行 |
| `UsbSerial_GetStatus(&status)` | 获取队列长度、在途状态、会话编号、累计收发及丢弃计数 |

公开接口只从任务调用。`Send/Printf` 支持多个任务，`Read` 由单一任务消费，默认由
`UsbTask` 负责。字节流不保留 USB 包边界，协议帧头、长度、校验与组帧由应用层处理。
不要从业务层直接调用 `CDC_Transmit_HS`，它是 BSP 专用的异步底层接口。

```c
#include "usb_serial.h"

/* 可放在任意业务任务中；OK 表示已复制入队，不表示电脑已收到。 */
uint8_t command[] = {0xAA, 0x55, 0x00, 0x01};
UsbSerialResult result = UsbSerial_Send(command, sizeof(command));
if (result == USB_SERIAL_BUSY)
{
    /* 在业务层保留本条数据，稍后重试，不能在这里忙等。 */
}

/* 示例：发送文本。生产代码同样应检查返回值。 */
result = UsbSerial_Printf("speed=%d\r\n", speed);
```

- RX/TX 队列默认各 1024 字节，通过 `usb_serial.h` 的容量宏调整；队列满时整条
  `Send` 返回 `USB_SERIAL_BUSY`，不截断、不覆盖在途数据。单次超过 TX 容量返回
  `USB_SERIAL_TOO_LARGE`；未连接/挂起返回 `USB_SERIAL_NOT_READY`；NULL/零长度返回
  `USB_SERIAL_INVALID_ARGUMENT`。发送函数不等待 USB 端点。
- `Printf` 默认最多 255 字节，超出则整条拒绝。格式化在临界区外完成；当前
  newlib-nano 未开启 `%f` 支持，优先使用整数或定点值。空文本为无操作，返回 OK。
- 接收缓存不足一个最大 USB 包时暂停 OUT，主机通过 NAK 等待；读取后空间足够即恢复。
  TX 在完成回调（含必要的 ZLP）后才释放对应缓存，发送 BUSY/FAIL 时保留队列重试。
- USB 撤销配置、总线复位或断开回调时清空旧队列，重新枚举建立新会话，`session_id`
  改变；累计计数保留。应用层若缓存半帧或待重发消息，也应随会话变化清理。
  USB 挂起期间暂停收发，恢复同一会话后继续；当前没有 VBUS 感知，物理拔线不保证
  立即触发断开回调，重新枚举时一定清理旧队列。

### 应用接收入口

`App/Src/usb_app.c` 是明确的应用入口。`UsbTask` 持续服务收发队列，收到电脑数据后调用
`Campus_UsbOnReceive(data, length)`，当前函数体为空。需要发送时由业务显式调用
`UsbSerial_Send/Printf`。

后续实现正式协议时，在 `Campus_UsbOnReceive(data, length)` 中处理数据。
该函数运行于 UsbTask，参数仅在调用期间有效，默认不自动回复。
也可将接收消费转移到另一个任务，但此时应移除 UsbTask 中的对应读取，保持单消费者。

`UsbTask` 使用 BelowNormal 优先级、1024 words（4096 字节）栈，每 1 tick 做一次有界服务。
它负责初始化 USB 和调用 `Campus_UsbProcess()`；`RcTask` 仅负责遥控。`.ioc` 中 UsbTask
排在首位，因为本机 CubeMX 模板将 `MX_USB_DEVICE_Init()` 放在第一个任务入口。
不要随意更改这一任务顺序。接入代码保留在 USER CODE 区域，自定义源码在顶层 CMake
中显式列出；重新生成时应检查 USB 初始化仍只出现一次，并保留 UART 的 `.dma_rx` 段。

默认线路参数继续为 **115200、8N1、无流控**；设置仅保存/返回，不改变 UART 或 USB
传输速度。连接 MCU 原生 USB（PA11/PA12），电脑端可通过对应 COM 口与业务协议通信。
内部回归检查为
`python tests/run_usb_serial_tests.py`（无需板卡、无需 pyserial，需要主机 GCC，可用
`--cc <gcc路径>` 指定），覆盖队列、异步发送、重新枚举、格式化与应用收发，不替代实板验证。

## 遥控通道映射与应用接口

以下对应关系来自本机 2026-11-11 的实测，左右以正常手持、正对遥控器面板为准。
驱动已将 SBUS 通道映射为物理控件字段，应用层通过 `Fxi6x_CopyData()` 获取快照：

| SBUS 通道 | 实测控件 | `Fxi6x_Data` 字段 |
|---|---|---|
| CH1 | 右摇杆水平方向 | `rocker_r_h` |
| CH2 | 左摇杆竖直方向 | `rocker_l_v` |
| CH3 | 右摇杆竖直方向 | `rocker_r_v` |
| CH4 | 左摇杆水平方向 | `rocker_l_h` |
| CH5 | VRA 旋钮 | `knob_l` |
| CH6 | VRB 旋钮 | `knob_r` |
| CH7 | 从左至右第 1 个拨杆 | `switch_a` |
| CH8 | 从左至右第 2 个拨杆 | `switch_b` |
| CH9 | 从左至右第 3 个拨杆 | `switch_c` |
| CH10 | 从左至右第 4 个拨杆 | `switch_d` |

```c
#include "fxi6x.h"

/* 在应用任务中读取一次快照，然后使用表中的字段。 */
Fxi6x_Data rc;
Fxi6x_CopyData(&rc);
/* rc.rocker_l_v 为 CH2 左摇杆竖直值，rc.rocker_r_v 为 CH3 右摇杆竖直值。
 * rc.knob_l / rc.knob_r 对应 VRA / VRB；rc.switch_a～switch_d 对应从左至右四个拨杆。 */
```

摇杆和旋钮为 -660～660 的 `int16_t`；拨杆为 1/0/-1 的 `int8_t`。
`valid` 仅代表最近一帧的解析结果，使用控制值前须同时检查接收历史和最后有效帧时间；
应用层也可读取 `Campus_CopyStatus()` 中包含 100 ms 超时判断的 `rc_online`。
