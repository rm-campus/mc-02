# UART7 众灵 ZP 总线舵机

更新：2026-11-16。本模块面向 ZP30、ZP35、ZP45 和两台 ZP25S。仅移植 ASCII 通信和任务调度，
不复制原厂的 RCC/SysTick/NVIC 分组、空循环延时、上电往返测试、广播改 ID 或内部 PID 设置。
温度/电压查询在资料中存在 PRTE/PRTV 差异，首版不猜测命令和单位，暂不实现。

## 接线与 CubeMX

UART7 原生单线半双工，115200、8N1、无流控；PE8/AF7、GPIO_MODE_AF_OD、GPIO_NOPULL，IRQ 优先级 5。
不使用 DMA。MC02 原理图 P13：1=板载 5V、2=GND、3=PE7/RX、4=PE8/TX。
本方案 S 接 P13-4，P13-3 不接，不要短接 PE7 与 PE8。半双工的发送和接收都使用 PE8。

五台舵机使用符合型号要求的独立 5～8.4V 动力电源，与 MC02 共地，不从 P13 的 5V 给五台舵机供电。
接入前确认 S 线的上拉和电平兼容性，按兼容的逻辑电压配置外部上拉；不能将动力电源电压接到 MCU 信号线上。
软件无法确认外部电平。实际线序以板卡丝印/原理图引脚号为准，不能凭插头朝向判断。

rm_campus.ioc 与 Core 初始化/IRQ 同步维护。CubeMX 应识别 PE8 的 Half_duplex(single_wire_mode)，
生成 HAL_HalfDuplex_Init(&huart7)。不改变现有 UART5 SBUS 的 100000/8E2、DMA、CAN、USB、Flash 配置。

## 配置与初始化

App/Inc/servo_config.h：SERVO_BUS_ENABLE=1，SERVO_USB_DEBUG_ENABLE=1。前者关闭时不发送舵机请求；
后者只控制手动 USB 命令，不影响应用层 API。ServoBusTask 复用现有 AboveNormal、512 words 栈，没有增加任务。

SERVO_DEVICE_TABLE 默认：001=ZP30、002=ZP35、003=ZP45、004/005=两台 ZP25S。
先逐台连接并配置唯一 ID，再并到同一总线。出厂 ID 通常为 000，本程序不会自动给它们改 ID。
未改号时 001～005 查询可能全部离线。不要使用参考例程中的 #255PID000! 广播改号。

表中的 500～2500 是协议允许范围，不是经过装配验证的机械行程。每个关节独立的 minimum/maximum
应按实际安装收窄；本次不猜测各关节零位、方向和安全端点。命令中的 position 是 P 值，不是角度。
ID 必须唯一且不是 255；非法配置显示 init=0，驱动不开始发送。

上电仅逐台读取 ID、版本、工作模式、当前位置，不主动运动、释力或写入配置。
舵机自身已保存的上电回中行为仍然有效，不能将 MCU 不发动作理解成舵机一定不动。
位置命令要求近期位置反馈在线且读到模式 1～4；模式 5～8 为连续旋转模式，位置命令返回 BAD_MODE。
不提供设置 ID、波特率、模式、硬件限位或 PID 的运行时命令；这些应在单台维护阶段确认固件后配置。

## API 与调度

接口在 App/Inc/servo_app.h：

```c
uint32_t ticket;
ServoResult r = Servo_MovePulse(1, 1500, 1000, &ticket);
Servo_RequestPosition(1, &ticket);
Servo_Stop(1, &ticket);
Servo_Pause(1, &ticket);
Servo_Resume(1, &ticket);
Servo_SetTorque(1, false, &ticket); /* 显式释放扭力，不能用于自动离线处理。 */
ServoState state;
Servo_CopyState(1, &state);
```

调用只提交请求，返回 QUEUED 不等于已经执行或到位。position 为 500～2500 且须在该轴软限位内，
time_ms 为 0～9999，仅在位置模式下表示运动时间。当前没有角度 API，换算应在后续机械臂应用完成标定后加入。

16 条静态请求队列；同一 ID 尚未发送的位置更新合并为最新目标，计入 coalesced。
停止/释力请求优先于队列中的普通请求，并取消该轴尚未发出的运动/继续/恢复扭力请求。
不打断正在发送或等待应答的事务；Stop 不是硬件急停。每台运动发送最小间隔 20 ms。

每次只有一个待应答事务。位置运动命令不等待 ACK，其 OK 只表示串口发送完成；查询、停止、暂停、继续、
扭力命令按各自回复匹配。到位需要比较新的位置反馈与目标，不能仅依据发送 OK 或 T 时间届满。
last_ticket/last_result 只描述该轴最近开始处理的用户请求，并非完整 ticket 历史；被合并或取消的请求不会补发。

正常位置轮询目标为每台 20 Hz，按 10 ms 槽错峰且不追赶补发；元信息查询与位置查询交错。
发送/应答超时均为 20 ms；位置连续 3 次失败或反馈年龄达到 200 ms 后离线，失败设备降至约 1 秒重试。
实际轮询速率受总线负载影响。运动请求不能推迟位置轮询，避免持续更新时误判离线。
离线取消未发送动作，不自动释力，不停止其他舵机；恢复通信重新确认模式，不补发旧动作。

超时或串口错误只恢复 UART7，等待 30 ms 恢复隔离窗口并检查接收静默后再发送。
协议没有序列号，无法绝对区分超长延迟的同 ID 同类型旧回复；现场应验证应答延迟。
默认控制器为总线上唯一主机，不允许同时用其他设备修改模式/ID/波特率。

## UART7 与中断

TX 使用 HAL_UART_Transmit_IT；HAL 在 TC（完整最后停止位已发送）回调时立即转接收，避免任务切换漏掉快速回复。
RX 使用单字节中断，进入 256 字节环形缓冲（有效容量 255），每帧最多 64 字节。ISR 不解析字符串、不延时、不打印。
任务按 # 开始、! 结束组帧，验证长度、ASCII、ID、类型和数值；超长帧丢弃，环形缓冲溢出触发 UART7 恢复。
UART7 没有 DMA，所以缓冲不需要 .dma_rx 段或缓存维护；后续若改 DMA 需重新设计。

fxi6x.c 保留唯一 HAL_UART_ErrorCallback；UART5 仍走原恢复逻辑，其余句柄通过 Bsp_UnhandledUartError
弱扩展点分发。servo_uart.c 提供该扩展点的 UART7 处理以及 HAL 的 TxCplt/RxCplt 回调。新增串口时不能重复定义全局回调。

## USB 手动命令

命令区发送 ASCII 并追加真实 LF，也接受 CRLF。不会自动周期输出舵机数据；电机实时回传为同速九列或麦轮十二列，Flash 日志仍为八列。

| 命令 | 行为 |
|---|---|
| servo_status | 返回五台的通信与缓存状态，不驱动运动 |
| servo_read 001 | 提交一次位置查询，随后用 servo_status 查看位置和结果 |
| servo_move 001 1500 1000 | 提交目标位置和运动时间 |
| servo_stop 001 | 终止该轴当前动作，不能据此保证通信中断的设备已停止 |
| servo_pause 001 | 暂停动作 |
| servo_resume 001 | 继续动作，要求在线 |
| servo_torque 001 0 | 显式释放扭力 |
| servo_torque 001 1 | 在当前位置恢复扭力，要求在线 |

数字必须为非负十进制，拒绝多余参数；示例使用三位 ID，USB 输入也接受 1～5，底层固定补齐三位。
返回例如：SERVO QUEUED id=1 ticket=7; use servo_status for result。
QUEUED 仅为入队确认，真实结果看该 ID 的 last ticket/result 和新位置反馈；不要将命令入队当成到位确认。
可能结果：OK、QUEUED、BUSY、INVALID、RANGE、OFFLINE、BAD_MODE、NOT_READY、DISABLED、TX_ERROR、UART_ERROR、TIMEOUT。

servo_status 用 SERVO_STATUS BEGIN/END 包围，包含 enabled/init、uart_ready、queued、错误/溢出/畸形/非预期帧计数，
各 ID 的 model、online、position、age_ms、mode/mode_valid、version、range、target、ticket/result、timeouts。
未收到位置显示 NA；离线保留最后位置并显示 online=0。target=0 代表尚未成功发出位置命令，不是有效位置目标。
mode_valid 表示确实收到模式回读；未知模式不猜测为 270 度。版本返回值只存字符串，不据此猜测扩展协议。

USB 回复采用四个有界缓冲槽，槽满的新命令不执行，随后给出 BUSY rejected=N 提示。
回复期间暂停实时 CSV；Flash 事务占用 USB 时回复等待，不夹入回放。串口断开时丢弃该会话未发完的回复；
已受理的舵机动作不会因 USB 断开自动撤销。不要把多条跨会话回复拼成一次报告。

## 验证与范围

tests/run_servo_tests.py 验证实际协议、UART7 端口、调度器及 USB 分发：开关、TC 后立即回复、上电只查询、
格式/ID/限位、动作合并与停止优先、离线恢复、持续更新不饿死查询、非法模式拒绝、溢出/错误/超时、
USB 占用/队列满/重连、麦轮十二列 CSV 与舵机报告互斥和 tick 回绕。其他回归保持旧模块隔离，新增集成测试使用真实舵机模块。
构建通过不等于硬件验收。上板先核对 S 线电平/上拉和 ID，再查询单台、测试小幅运动、接入五台，
最后与底盘/IMU/USB/Flash 同时运行检查通信时序及栈高水位。未烧录，不已知实际关节限位。
