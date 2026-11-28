# UART7 众灵 ZP 总线舵机

更新：2026-11-16。本模块面向 ZP30、ZP35、ZP45 和两台 ZP25S。仅移植 ASCII 通信和任务调度，
不复制原厂的 RCC/SysTick/NVIC 分组、空循环延时、上电往返测试、广播改 ID 或内部 PID 设置。
温度/电压查询在资料中存在 PRTE/PRTV 差异，首版不猜测命令和单位，暂不实现。

## 接线与 CubeMX

UART7 单线半双工，115200、8N1、无流控；启用 TX/RX Swap，使用 PE7/AF7 开漏，IRQ 优先级 5。
正常固件和 ServoId 改号固件均启用 PE7 内部弱上拉（GPIO_PULLUP）。五台上线后需实测反馈是否稳定。
不使用 DMA。MC02 原理图 P13：1=板载 5V、2=GND、3=PE7/RX、4=PE8/TX。
本方案 S 接 P13-3（PE7/RX），P13-4（PE8/TX）不接。自制板 U2 的 2 脚为 S，3 脚为 GND。

五台舵机使用符合型号要求的独立 5～8.4V 动力电源，与 MC02 共地，不从 P13 的 5V 给五台舵机供电。
接入前确认 S 线的逻辑电平兼容性；舵机动力电源与信号线分别接线。
软件无法确认外部电平。实际线序以板卡丝印/原理图引脚号为准，不能凭插头朝向判断。

重新生成 Core 代码后要检查 UART7 同时保留 HAL_HalfDuplex_Init(&huart7)、TX/RX Swap 和 PE7 的 AF7 开漏与内部上拉配置。

## U25-M 与肩舵机共线检测

`ServoMixedProbe` 预设只运行 USB 维护任务，独占 UART7。配置并构建后，烧录
`build/ServoMixedProbe/rm_campus_servo_mixed_probe.elf`。开串口助手，选择 MC02 的 USB 虚拟串口，
用 115200、8N1、ASCII 和 LF/CRLF，发送一次 `mixed_servo_probe`。华馨京默认地址按 0 查询；
如果已经改过地址，在命令后加实际地址，如 `mixed_servo_probe 5`。众灵侧固定查询肩关节 003。

报告用 `MIXED_SERVO BEGIN` 和 `MIXED_SERVO END` 分隔。`ZP` 行报告 ID、位置 P 值和模式；
`FS` 行报告通信检测、单圈位置（`angle_ddeg` 每单位为 0.1°）、电压/电流/功率、温度 ADC 和保护状态位。
`rx` 保留每次查询最多 24 字节的十六进制回包。两类回包通过各自的格式、地址和校验检查；
`NO_REPLY` 表示未收到字节，`INVALID_REPLY` 表示收到字节但未找到匹配回包。

UART7 固定 115200/8N1。先确认两台的有效电压交集、线序和信号电平。U25-M 要求 6.0～8.4 V，
舵机正极接外部舵机电源，GND 与 MC02 P13-2 共地，S 线接 P13-3。每次只保留一个发送主站。
如果用华馨京上位机调整 ID 或波特率，先把 U25-M 单独接到厂家转接板；上位机做单轴点动时选**单圈角度控制**。
此预设只在收到 USB 探测命令后读取状态，日常遥控使用 `RobotControl` 或本机的 `RobotControlTelemetry`。

## 用 U25-M 替换夹爪 005

`RobotControlFashionStar` 是 U25-M 夹爪预设，保留整机遥控和 USB 波形。U25-M 的总线地址为
005，与众灵舵机共用 115200/8N1 的 UART7。CH9 中档由右杆左右控制腕滚转 004，下档由
右杆上下（CH3）控制夹爪 005。

默认 `FS_GRIPPER_BENCH_ISOLATION=1`。待机和下档只轮询 U25-M，上、中档轮询众灵。
这台 U25-M 实测在众灵帧之后会漏回多条查询，分档轮询可减少两种协议的切换。

CH3 的 −660、0、+660 分别对应开端、行程中点和闭合端，按有符号杆量线性换算。
`FS_GRIPPER_OPEN_DDEG=1224` 对应实测开端 P2180，`FS_GRIPPER_TRAVEL_DDEG=-1044`
对应反向 104.4° 行程，闭合端为 P1600。进入下档后目标从回读位置限速移向当前杆位对应的角度。
USB 第五路为「005 夹爪」，保留实测、发送目标和速度字段。
这一轴把 −180°～+180° 映射到 P500～P2500，因此 1 P 约为 0.18°，速度用相同的映射比例换算。
启动示波器可用 `python tools/servo_scope.py --port COM5 --joint 5`，串口号按现场枚举选择。

## 单舵机读号与改号

`ServoId` 只启动 USB 改号任务，独占 UART7。断电换线，单次只接一台舵机。
电脑选 MC02 的 USB 虚拟串口，以 ASCII 发送，追加 LF（CRLF 也可）。

| 命令 | 行为 |
|---|---|
| `servo_read id 001` | 定向发送 `#001PID!`，收到匹配的 `#001P!` 时回复 `OK ID id=001`。只读。 |
| `servo_set_id 000 001` | 先读旧号，收到旧号回复才发送一次 `#000PID001!`。收到带新号的回执后等待 100 ms，再用新号查询，最多三次，失败时只重试查询。 |

地址范围 000～254，改号要求新旧不同。写入成功并独立回读后回复 `OK ID 000 -> 001`。
要确认掉电保留，给舵机断电重启后再用 `servo_read id 001` 查询。
若曾出现回读失败，先读目标新号，再根据需要读旧号。

`OLD_ID_NO_REPLY` 发生在写入之前；`WRITE_UNCONFIRMED` 表示写入回执未确认；
`NEW_ID_NO_REPLY` 表示收到改号回执，但新地址回读失败。失败回复的 `cause` 字段区分无回复
（`NO_REPLY`）、有字节但格式或地址不匹配（`INVALID_REPLY`）、UART 错误（`UART_ERROR`）
和未等到发送完成中断（`TX_TIMEOUT`）。

## 配置与初始化

App/Inc/servo_config.h：SERVO_BUS_ENABLE=1，SERVO_USB_DEBUG_ENABLE=1。前者关闭时不发送舵机请求；
后者只控制手动 USB 命令，不影响应用层 API。ServoBusTask 复用现有 AboveNormal、512 words 栈，没有增加任务。

SERVO_DEVICE_TABLE 默认：001=ZP30、002=ZP35、003=ZP45、004/005=两台 ZP25S。
众灵整机预设将 005 的软件行程设为 P750～P1500。CH9 下档用 CH3 线性控制，回中目标为 P1125。
先逐台连接并配置唯一 ID，再并到同一总线。出厂 ID 通常为 000，本程序不会自动给它们改 ID。
未改号时 001～005 查询可能全部离线。不要使用参考例程中的 #255PID000! 广播改号。

表中的 500～2500 是协议允许范围，不是经过装配验证的机械行程。每个关节独立的 minimum/maximum
应按实际安装收窄；本次不猜测各关节零位、方向和安全端点。命令中的 position 是 P 值，不是角度。
ID 必须唯一且不是 255；非法配置显示 init=0，驱动不开始发送。

上电仅逐台读取 ID、版本、工作模式、当前位置，不主动运动、释力或写入配置。
舵机自身已保存的上电回中行为仍然有效，不能将 MCU 不发动作理解成舵机一定不动。
台架位置命令要求近期位置反馈在线且读到模式 1～4；模式 5～8 返回 BAD_MODE。整机遥控沿用位置回读接管：即使模式未知或报告 5～8，也会发送按位置限速的 P/T 指令。连续旋转模式的实际轴速无法由这个限速保证。
正常固件可通过 USB 批量设置位置模式；ID、波特率、硬件限位和 PID 不由此命令设置。

### 五台模式设置与确认

将 CH7、CH9 置于关闭档，确保关节有安全行程。在正常固件的 USB 串口发送
`servo_position_mode 1` 加 LF。参数可选 1～4；1、2 对应 270°，3、4 对应 180°，
相邻模式的方向相反。该命令将当前设备表中五台统一设置为指定模式。
存在运动控制权或未完成的运动/停止命令时返回 BUSY。

UART7 按 ID 分别发送一次模式设置，留出 100 ms 后独立查询，每台最多查询两次。
单次模式查询从发送完成起等待最多 100 ms，普通位置查询仍等待最多 20 ms。
报告逐台列出回读模式与 `mode_check`。只有回读等于所选模式才显示 OK。
若某台不匹配则显示 BAD_MODE；未拿到模式时显示相应错误，例如 TIMEOUT。
任何单台失败均不撤销已确认的其他舵机，也不重发写入帧。

发送 `servo_status` 加 LF 会立即输出驱动快照。模式字段标为 `mode_check=CACHED`。
在 CH9 退出控制后，可发送 `servo_mode_probe 001` 加 LF，只查询一次 001 的模式。
此诊断从 UART 发送完成起最多等待 1000 ms，报告结果、原始 ASCII/HEX 字节以及首字节经过的时间。
批量操作期间若位置回读超过 200 ms 未更新，状态会暂时显示 `online=0`。
下一次有效位置回复会刷新在线状态；位置在线与模式回读成功分别检查。
给舵机断电重启后对同一个 ID 发 `servo_mode_probe`，可确认模式是否掉电保留。

## CH9 遥控与五台角度回传

在 servo_config.h 中设置 SERVO_RC_ENABLE=1 开启遥控；置 0 后保留原手动 USB 命令。
SERVO_RC_TELEMETRY_ENABLE 控制角度自动回传，SERVO_BUS_ENABLE 仍为总驱动开关。

| CH9 | CH2 左竖直 | CH4 左水平 | CH1 右水平 |
|---|---|---|---|
| +1 | 不控制舵机 | 不控制舵机 | 不控制舵机 |
| 0 | ID1 | ID2 | ID3 |
| -1 | ID4 | ID5 | 不控制舵机 |

ID1～5 对应原五台舵机，不增加底盘 yaw 舵机。底盘控制保持独立，因此已启用的底盘会同时响应同一摇杆。
每 20 ms 更新一次增量目标：角速度=死区后的摇杆比例×每轴最大角速度。
±60 为死区，SERVO_RC_AXIS_TABLE 按 ID1～5 配置正向(+1/-1)与最大 deg/s，五轴默认全部 +1，共用 SERVO_MAX_SPEED_DEG_S（30°/s）。
每轴软限位使用 SERVO_DEVICE_TABLE 的 minimum/maximum（P 值）。500～2500 只是协议范围，
必须按关节装配标定后收窄。配置非法时 config_valid=0，不启动遥控；不修改舵机内部 PID。

上电先观察 CH9=+1，再选择组。该组涉及的摇杆轴回中 300 ms，停止事务处理完且对应舵机位置新鲜、
模式为 1～4、当前位置在软限位内，才从实际回读位置建立目标。离线轴不阻止其他在线轴启用。
摇杆回中保持最后目标，不回 P1500；切组/关闭先清掉待发运动，发送 PDST 停止旧组，不自动释力。
切组会等待停止事务处理后重新回中，新的目标不沿用上一组历史值。
遥控超过 100 ms 未更新、失控或控制更新间隔超过 50 ms 时，停止遥控运动并要求重新观察 +1。
反馈过期、运动发送失败或工作模式变化会锁住对应轴；故障解除也须 CH9 回 +1 才能重新启用。
已在 UART 线上发送的字节不能撤销，停止需等待当前事务结束；总线故障时不能保证物理停止。

CH9=0/-1 时，手动 servo_move、servo_resume、servo_torque ... 1 返回 BUSY；servo_status/servo_read 可用。
手动 servo_stop、servo_pause、servo_torque ... 0 保留并锁住该轴，避免下一帧遥控指令覆盖。
手动释力后，应在 CH9=+1 且机构安全时显式恢复扭矩，再重新进入遥控组。
servo_status 新增 RC 行：config_valid、off_seen、selected、armed、fault。三个掩码 bit0～4 对应 ID1～5。

遥控有效且 CH9=0/-1 时，USB 每 20 ms 输出五列：angle1,angle2,angle3,angle4,angle5（线上无表头）。
270°模式：angle=(回读P-500)×0.135；180°模式：angle=(回读P-500)×0.09。P1500 对应135°或90°，
不是机械零位。未收到位置、离线或模式未知时输出 -1.000。不使用目标 P 代替反馈。
舵机位置轮询仍为每台约 20 Hz，50 Hz 是 USB 输出频率，允许重复最新测量；总线查询不与五台同步采样。
CH9=+1 恢复电机原有 200 Hz 数据；CH9=0/-1 暂停电机自动数据，仅发舵机五列，底盘控制不暂停。
手动诊断、USB 背压或掉线时自动 CSV 让行，不累计补发；切换时先排空已经入队的上一种帧。
启用 Flash 时需设置 SERVO_RC_TELEMETRY_ENABLE=0 和 MOTOR_DEBUG_TELEMETRY=0，编译期检查避免混流。

原有 ServoBusTask 内处理遥控，UsbTask 内格式化回传，没有新增 FreeRTOS 任务或更改 UART7 配置。

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
众灵位置指令的 time_ms 为 0～9999 ms。固件按量化后的输出轴位移和公共速度上限向上取整计算最短时长，保留调用方给出的较长时长。在 30°/s 下，270° 的全行程至少需要 9000 ms。计算出的时长超过 9999 ms 时返回 RANGE。

16 条静态请求队列；同一 ID 尚未发送的位置更新合并为最新目标，计入 coalesced。
停止/暂停/释力请求优先于队列中的普通请求，并取消该轴尚未发出的运动/继续/恢复扭力请求。
不打断正在发送或等待应答的事务；Stop 不是硬件急停。每台运动发送最小间隔 20 ms。

每次只有一个待应答事务。位置运动命令不等待 ACK，其 OK 只表示串口发送完成；查询、停止、暂停、继续、
扭力命令按各自回复匹配。到位需要比较新的位置反馈与目标，不能仅依据发送 OK 或 T 时间届满。
last_ticket/last_result 只描述该轴最近开始处理的用户请求，并非完整 ticket 历史；被合并或取消的请求不会补发。

正常位置轮询目标为每台约 20 Hz，按 10 ms 槽错峰且不追赶补发；元信息查询与位置查询交错。
到期查询先于普通运动请求，遥控停止请求使用额外的五位保留槽，不受普通队列满影响。
UART7 命令以 `!` 结束；确认最后一个停止位已经发完，再从缓冲区消费回复。
发送/普通应答超时为 20 ms，常规模式查询应答窗口为 100 ms；位置连续 3 次失败或反馈年龄达到 200 ms 后离线，失败设备降至约 1 秒重试。
启动查询结束后，对已在线但缺模式或版本的舵机按 ID 和字段轮换补读。全总线最小间隔由
`SERVO_METADATA_RETRY_MS` 控制（默认 1000 ms）；优先刷新即将过期的位置反馈。
显式点动或模式诊断正在执行、或速度逆解持有总线控制权时，后台补读暂缓。
实际轮询速率受总线负载影响。整机 CH9 中档、下档暂缓启动序列和后台补读中的自动 PVER/PMOD，退出这两个档位后按原调度继续。推杆起动的首条可发送运动指令优先于普通轮询，后续位置反馈继续按原节奏调度。
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

命令区发送 ASCII 并追加真实 LF，也接受 CRLF。手动命令在原格式基础上增加上述遥控接管限制；自动输出按 CH9 在电机与舵机数据间切换，Flash 日志仍为八列。

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

tests/run_servo_rc_tests.py 覆盖两组轴映射、上电未确认、三轴回中、增量/回中保持、限位与小数积累、
切组、手动停止/暂停/释力锁定、队列满保留停止槽、发送超时旧目标清理、单轴掉线/重连、遥控超时、
调度迟到、五列实测角度/无效占位、9/12列与5列切换、诊断互斥、USB背压/会话及tick回绕。
分别验证功能关闭、总驱动关闭和仅回传关闭配置。

tests/run_servo_tests.py 验证实际协议、UART7 端口、调度器及 USB 分发：开关、TC 后立即回复、上电只查询、
格式/ID/限位、动作合并与停止优先、离线恢复、持续更新不饿死查询、非法模式拒绝、溢出/错误/超时、
USB 占用/队列满/重连、麦轮十二列 CSV 与舵机报告互斥和 tick 回绕。其他回归保持旧模块隔离，新增集成测试使用真实舵机模块。
构建通过不等于硬件验收。上板先核对 S 线电平/上拉和 ID，再查询单台、测试小幅运动、接入五台，
最后与底盘/IMU/USB/Flash 同时运行检查通信时序及栈高水位。未烧录，不已知实际关节限位。

## 速度IK自动控制源

已新增默认关闭的ARM速度IK预备模块，详见ARM_KINEMATICS.md。驱动使用ServoControlOwner区分RC与ARM，原Servo_Rc接口保留。
Servo_ControlMoveGroup先校验全部目标再整组入队，不允许队列不足时部分提交。ARM任一轴异常或手动halt立即撤销全组待发运动。
实际IK输出与CH9遥控编译期互斥；默认ARM关闭时原遥控、手动命令和回传规则保持。没有重设舵机ID或内部PID。
