# USB 通信协议与状态码

更新日期：2026-11-14。本文描述当前工程已经实现的 USB 应用层协议，不是未来接口设计。
实现入口：[flash_tasks.c](App/Src/flash_tasks.c)、[usb_app.c](App/Src/usb_app.c)。
任务、分区和底盘接口说明另见 [FLASH_TASKS.md](FLASH_TASKS.md)。

## RobotControlTelemetry 舵机波形

USB 数据口每 20 ms 尝试发一帧，USB 忙时跳过。帧以 `servo_wave:` 开始、以 LF 结束，
17 个整数按以下顺序用逗号分隔。只回传数据，USB 输入不会进入运动控制。

| 列（从 1 开始） | 含义 |
|---|---|
| 1 | MCU 时间戳（ms，32 位回绕） |
| 2 | CH9 已接管档位，1 上、0 中、−1 下，2 等待遥控 |
| 3–7 | ID1～5 的实测 P 值，0 表示反馈无效 |
| 8–12 | ID1～5 的估算速度（P/s），−2147483648 表示无有效的速度样本 |
| 13–17 | ID1～5 最近发送的目标 P 值，0 表示无有效目标 |

每轴速度用两次读到的新位置反馈及其时间差计算，同一回包不会重复计算。
普通遥控波形在反馈超过 200 ms 后断开曲线，U25-M 共线台架版使用 350 ms。
重新收到位置后，第二个有效位置样本起才有速度。
使用 `python tools/servo_scope.py` 可按 ID 看位置、已发目标和估算速度，程序会将无效数据画成空缺。

## 当前 Debug / Release：舵机监测与定向点动

默认预设开启 `SERVO_STATUS_ONLY`。USB CDC 接受 ASCII 命令，行尾 LF 或 CRLF。
MC02 在 UART7 发给舵机的命令以英文 `!` 结束。例如查询 001 的模式，线上为
`#001PMOD!`，十六进制为 `23 30 30 31 50 4D 4F 44 21`，发送结束后切回接收。
轮询、点动和 ServoId 读号/改号共用该发送格式。MC02 仍以 `!` 识别舵机回复的结束位置。
推荐电脑每 100 ms 发送一次。一次返回 ID 001～005 的最近快照，起止标志为
`SERVO_STATUS BEGIN` 和 `SERVO_STATUS END`。位置字段 `position` 是实际反馈 P 值；
在线判断结合 `online` 与 `age_ms`。无需遥控。
在线舵机缺少工作模式或版本时，板端在位置反馈有时间余量时自动轮换补读，全总线每秒最多一笔。
收到有效回包后，后续状态报告会更新对应字段；未取得回包的字段继续显示未知。
模式查询兼容手册回包 `#001PMOD1!` 和实测回包 `#001MOD1!`。两者均表示模式 1。
版本查询兼容 `#001PV0.97!` 和 `#001@ ZL-ZServo_AD_CBM V2.1.16STG!\r\n`；
在 `servo_status` 中分别显示 `version=0.97`、`version=2.1.16STG`。
型号描述及帧尾之后的字节可在 `SERVO_RX` 原始监视中查看。

### 自动镜像 UART7 原始接收

默认 Debug/Release 从上电开始在 UART7 的接收中断中复制每个已收到的字节。
电脑打开 MC02 的 USB 串口即可持续看到 `SERVO_RX` 行，无需发送命令。
同一行是一次接收片段，最长 48 字节，约 2 ms 的字节间隔、下一条命令或长度上限均可能分段。
一条协议帧可能分行，多个短帧也可能在同一行。示例：

```text
SERVO_RX seq=0 n=10 first_ms=104 last_ms=105 tx=#001PMOD! tc_ms=100 dt_ms=4 lost=0 HEX=23 30 30 31 50 4D 4F 44 31 21 ASCII=#001PMOD1!
```

- `seq` 是本段首字节自 MCU 启动以来的偏移（从 0 起，32 位自然回绕），`n` 是字节数。
  `HEX` 保留全部字节，包括错误帧、残片、NUL 和换行。ASCII 栏将不可打印字节显示为点。
- `first_ms/last_ms` 是 MCU 收到本段首末字节的毫秒时刻；`tx/tc_ms` 是接收时最近发送完成的命令及完成时刻。
  `dt_ms = first_ms − tc_ms`，整段到达耗时可看 `last_ms − tc_ms`；尚无发送参照时显示 NA。
  总线协议无事务序列号，迟到或意外数据可能属于更早的命令。
- USB 未就绪或电脑读取较慢时，板端保留最近 64 个片段；旧片段被覆盖时，`lost` 给出本行之前缺失的监听字节数。
  正在发送的片段等 USB 传输完成才确认，重连时可能再次看到相同 `seq`。
  硬件未接收到的字节无法镜像。`lost` 只统计监听缓存，不表示物理总线的误码数。
- `servo_status` 仍是快照查询，点动回复与镜像按整条 USB 消息交替发送。
  `App/Inc/servo_config.h` 内 `SERVO_RX_TRACE_ENABLE` 置 0 可关闭自动镜像。

### 时间参数与诊断

电脑请求 `servo_status` 后 MCU 不等待舵机，USB 可能因已有输出排队。后台 UART7 从开始发命令到发送完成
的超时为 20 ms；从最后的停止位发完起，位置/ID/版本等普通查询等 20 ms，常规及后台补读模式等 100 ms，
点动补查或显式模式探测等 1000 ms。位置年龄 200 ms 是在线阈值，不能作为单次应答耗时使用。

### 舵机点动

`servo_test_001_1` 使 001 顺时针转 10°，`servo_test_001_2` 逆时针转 10°，方向按正对主动输出轴端面观察。
三位 ID 仅接受 001～005，方向仅接受 1、2。点动一次发一行，循环发送会多次运动。
USB 按接收顺序保存已接受的请求，同一台的连续两步也各自重新读位置。模式未知时会向目标 ID 查询 PMOD，
不写模式。模式 1、2 每 10° 约 74P，模式 3、4 约 111P；模式 2、4 的顺时针对应 P 减小。
若整步超出设备配置的限位，拒绝该步。

以起点 P1868、模式 1 为例，成功路径的回包如下：

```text
SERVO_TEST QUEUED id=001 ticket=1
SERVO_TEST DONE id=001 ticket=1 start=1868 target=1942 actual=1942
```

QUEUED 表示已受理。上例从静止起点移动 74 P，即 9.990°，按 15°/s 计算的最短运动时间为 666 ms。计划运动时间届满后，新的位置回读在目标约 1° 内才报告 DONE。
从队首启动起有 2000 ms 准备窗口。已见过反馈但当前过期的舵机先读位置验通；
模式未知再查询模式，确认模式后再读一次位置算起点。运动本身没有应答帧，发送完成后另计
本次实际下发的运动时间和 1000 ms 到位验证窗口。队列中的等待时间独立计算。
失败回复 `SERVO_TEST ERR` 和 `reason`：OFFLINE 表示缺少所需反馈，MODE_UNKNOWN 表示没有读到模式，
BAD_MODE 表示模式不适合定位，LIMIT 表示整步越限，NOT_REACHED 表示有移动后回读但未按时到位。
UART 错误或超时不会自动重发运动。结果中的 start、target、actual 是 P 值，0 表示无对应数据。
BUSY 表示未受理；USB 回复队列满时可能以 `SERVO BUSY rejected=N; commands not enqueued` 汇总。
固件检测到 USB 断线或会话变化时清除未发的点动，已经发到舵机的动作可能继续完成。
点动开启/关闭、步长、最短时长（默认 500 ms）和公共速度上限在 `App/Inc/servo_config.h` 配置；`SERVO_JOG_TEST_ENABLE=0` 恢复仅接受 `servo_status`。
下列遥控、Flash、电机和通用舵机运动命令属于其他固件配置。


## CH9 舵机遥控自动回传（优先于下述电机实时输出）

默认 SERVO_RC_ENABLE=1、SERVO_RC_TELEMETRY_ENABLE=1。遥控有效且 CH9 为0/-1时，每20ms发送五列纯CSV：

```text
135.000,120.150,90.045,150.120,-1.000
```

依次为 ID1～5 的回读协议角度（°），不是目标角度；无表头、无 ready，LF结束。
270°模式以P500为0°、P1500为135°；180°模式以P1500为90°。未收到/过期/模式未知为-1.000。
50Hz指USB回传，每台位置查询仍约20Hz，允许重复最新测量。
CH9=+1恢复原电机200Hz输出；0/-1时仅替换自动输出，底盘仍按CH7/CH8运行。
手动诊断期间暂停CSV，USB繁忙不积压补发；切换时上一帧可先发送完成，再切换列数。
servo_status 新增 RC 行：enabled/config_valid/off_seen/selected/armed/fault，掩码bit0～4对应ID1～5。
遥控组内拒绝手动move/resume/恢复扭矩，手动停止/暂停/释力锁住对应轴，需CH9回+1再重新启用。
Flash模式需同时关闭 SERVO_RC_TELEMETRY_ENABLE 和 MOTOR_DEBUG_TELEMETRY；Flash回放八列保持不变。
详细启用流程、轴映射和限位见 SERVO_INTEGRATION.md。


## 当前调试模式：纯 CSV，默认四轮

模式由 motor_debug_config.h 中 MOTOR_DEBUG_MODE 选择：SINGLE=0、FOUR=1，默认 FOUR。
MOTOR_DEBUG_ENABLE=1、MOTOR_DEBUG_TELEMETRY=1，两个 Flash 开关均为 0。两种模式无运行时限。
上电等待使能，不主动回传；无需发送 read。read/status/clear/flash: 在 Flash 关闭时无回应。
motor_status 加 LF 始终可用。电机反馈在线阈值为 <100 ms，达到 100 ms 则只停止对应电机。
最左拨杆 +1 确认关闭、-1 启用，左竖直回中 ±60 保持 300 ms，本轮基本静止后独立进入 RUNNING。
四轮无需全部在线；单轮异常不使健康轮停机。共同遥控、调度和物理总线故障可能影响全部轮。

麦轮模式（CH8 = −1）启用前要求 CH2、CH4、CH1 均回中。每 5 ms 发送十二列，按 ID1～4 顺序依次发送四个解算目标 rpm、四个实测 rpm 和四个反馈电流 A。目标列包含左杆平移与右杆旋转的合成结果。接收端切换 CH8 时按每行实际列数解析。

四轮同速（CH8 = +1 或 0）每 5 ms 发送九列数字，前五列为输出轴 rpm，后四列为反馈电流 A：

```text
225.000,221.500,223.100,220.800,222.600,1.200,1.100,1.300,1.000
```

| 列 | 含义 |
|---|---|
| 1 | 左竖直 ×400/660 的公共原目标，未经斜坡 |
| 2～5 | ID1、ID2、ID3、ID4 的实测转速，已考虑减速比和方向 |
| 6～9 | ID1、ID2、ID3、ID4 的电调反馈电流，单位 A，有正负号，不是指令电流或电池电流 |

ASCII、三位小数、逗号分隔、真实 LF 结束，无前缀。至少一台 RUNNING 才回传；
四轮不发 ready、指令电流、温度或其他自动文字。离线轮的转速、电流列均置 0，仍在线的故障轮继续显示实际反馈。
公共目标不代表每轮均启用，0 也不能区分静止与离线；按需发送 motor_status。
检测到 USB 重连时，四轮模式仍在运行则直接恢复 CSV；全部退出运行后不产生新帧。

单电机模式保留 ready，然后发送无前缀的四列 CSV：

```text
ready
225.000,218.500,0.420,0.398
```

顺序为原目标 rpm、实测 rpm、CAN 量化后的指令 A、反馈 A。单电机再次运行/USB 会话恢复仍重新发送 ready。
原 motor: 前缀已删除。两个模式都以 200 Hz 回传，无时间戳，USB 忙时跳过样本、不积压、不追赶补发。
已经在途的末帧可能在退出运行后到达。USB 断开不停止控制；当前没有 VBUS 感知，拔线/仅重开端口不一定立即改变会话。
按截图支持的逗号数字方式使用新串口助手，接收格式选 ASCII，电脑应按 LF 分帧，不依赖单次 USB 读取边界。
新助手实机绘图效果仍待核验；手动诊断文字中的数字可能被通用绘图解析，曲线分析时应排除诊断段。

编号 1～后续章节描述 Flash 模式，当前关闭。日志已升级为四轮目标/反馈八列纯 CSV，100 Hz；不支持单轮日志。
启用任一 Flash 任务前须关闭 MOTOR_DEBUG_TELEMETRY 和 SERVO_RC_TELEMETRY_ENABLE。操作细节见 [SINGLE_MOTOR_DEBUG.md](SINGLE_MOTOR_DEBUG.md)。

## UART7 舵机手动命令

SERVO_BUS_ENABLE=1、SERVO_USB_DEBUG_ENABLE=1，Flash 开关关闭时也可使用。命令追加真实 LF，接受 CRLF。
上电只查询 001～005，不自动移动或改写 ID/模式/波特率/PID。硬件接线、ID 和软限位见 [SERVO_INTEGRATION.md](SERVO_INTEGRATION.md)。

| 命令 | 作用 |
|---|---|
| servo_status | 查询五台在线、P 位置值/年龄、模式/版本、限位、最近 ticket/result 和总线错误 |
| servo_read 001 | 提交位置查询，随后用 servo_status 查看结果 |
| servo_move 001 1500 1000 | ID1 运动到 P1500，请求时长 1000 ms；固件按 15°/s 延长过短的时长。要求在线、模式 1～4、通过该轴位置限位 |
| servo_stop 001 | 终止该轴动作 |
| servo_pause 001 / servo_resume 001 | 暂停 / 继续 |
| servo_torque 001 0 / servo_torque 001 1 | 释放 / 恢复扭力，需显式调用 |

数字为非负十进制，拒绝缺少/多余参数与广播 ID。P 不是角度，time 范围 0～9999 ms。
返回 SERVO QUEUED id=1 ticket=7; use servo_status for result 只表示入队，运动命令没有硬件 ACK，发送完成也不证明到位。
同 ID 未发送的位置请求合并为最新值；离线取消未发送动作。last ticket/result 不构成完整请求历史。
回复槽满时新命令不执行，随后给出 SERVO BUSY rejected=N。接口错误包括 INVALID、RANGE、OFFLINE、BAD_MODE、NOT_READY、DISABLED。
执行结果还可为 TX_ERROR、UART_ERROR、TIMEOUT。没有自动重试运动命令。

手动回复用 SERVO_STATUS BEGIN/END 包围或单行 SERVO 文本；回复时暂停周期电机 CSV，不插入 Flash 回放。
USB 断开丢弃未完成回复，已受理动作不会因此自动撤销。诊断文本会包含数字，分析曲线时应排除手动诊断段。
完整字段说明见 SERVO_INTEGRATION.md。舵机支持手动诊断和 CH9 控制的五列角度回传；电机同速九列或麦轮十二列与舵机五列互斥输出，日志仍为八列。诊断期间暂停实时 CSV，报告完成后恢复。

## motor_status：电机与 CAN 诊断

在串口助手发送框输入 `motor_status`，选择追加真实 `\n`（也接受 CRLF）。区分大小写，不去除前后空格。
无需运行使能或 ready；与两个 Flash 宏、MOTOR_DEBUG_ENABLE、MOTOR_DEBUG_TELEMETRY 无关。
默认模式单行上限 64 字节（包含换行）；启用 Flash 时沿用 4096 字节。超长行丢弃到下一个 LF。
默认模式未知/超长命令无回复，Flash 模式保留原有提示。断线或换会话清除半行。

每次请求固定一份快照。下面为单电机模式示例（不是实测），四轮模式的差异见后文：

```text
MOTOR_STATUS BEGIN
SELECT can=FDCAN1 id=1 feedback_id=0x201
SNAPSHOT time_ms=1000 can_sample_age_ms=1 control_age_ms=1
RX seen=1 online=1 age_ms=1 valid_frames=12560
RX_CHECK bad_format_bus=0 bad_encoder=0 read_fail=0 fifo_lost_events=0
CAN init_ok=1 status_valid=1 counters_valid=1 bus_off=0 passive=0 warning=0 tec=0 rec=0
CAN_ERROR last=NONE age_ms=NA tx_enqueue_fail=0
RECOVERY state=READY bus_off_count=0 attempts=0 success=0 failures=0 error=0 last_ms=NA feedback_mask=15
MOTOR fresh=1 rpm=0.000 current_a=0.000 temperature_c=28 encoder=1234
CONTROL state=WAIT_OFF reason=OK rc_online=1 switch_a=-1 stick=0
ONLINE id1=1 id2=0 id3=0 id4=0
MOTOR_STATUS END
```

| 行/字段 | 含义 |
|---|---|
| SELECT | 当前选定电调 ID、预期反馈 ID，CAN1 接口；配置 ID 非法时反馈 ID 为 NA |
| SNAPSHOT | 请求时刻、距离控制任务最近采集 CAN 状态/控制快照的时间，单位 ms |
| seen / online / age_ms | 曾收到合法反馈 / 最近合法反馈距快照时刻 <100 ms / 反馈年龄 |
| valid_frames | 选定电机成功解包累计帧数 |
| bad_format_bus | 到达软件的帧中，ID、类型、格式或长度不符合预期的累计数量（整条总线） |
| bad_encoder | 选定电机编码器 ≥8192 导致拒绝解包的累计数量 |
| read_fail / fifo_lost_events | HAL 从 FIFO 读消息失败次数 / 收到 FIFO0 丢失通知次数（不是精确丢帧数） |
| init_ok | 过滤器、启动和接收通知初始化结果，不保证总线正常或电机在线 |
| status_valid / counters_valid | 最近一次 HAL 状态 / 错误计数读取是否成功；对应无效字段输出 NA |
| bus_off / passive / warning | 最近采集到的总线关闭 / 错误被动 / 错误警告状态 |
| tec / rec | 控制器当前采集到的收发错误计数，不是上电累计错误总数 |
| CAN_ERROR last / age_ms | 最近捕获并锁存的协议错误名称 / 距捕获时刻毫秒数；查询不清除 |
| tx_enqueue_fail | 控制帧发送入队失败累计次数；入队成功不证明 C620 已执行指令 |
| RECOVERY | Bus-Off 自动恢复状态及统计，详见下表；不改变自动 CSV 的通道数 |
| MOTOR | 选定电机最后合法反馈：输出轴 rpm、A、℃、原始编码器；fresh 与 online 同判定 |
| CONTROL | 控制快照状态/原因，以及最新遥控在线、最左拨杆、左竖直摇杆值 |
| ONLINE | ID1～4 在线状态；单电机模式不要求其他三台在线 |

所有软件计数从上电开始，32 位无符号自然回绕，查询不清零。
四轮模式 SELECT 行为 `SELECT can=FDCAN1 mode=FOUR`，取消单电机 RX/MOTOR 摘要，改为逐轮 MOTOR 行：

```text
CONTROL running_mask=5 rc_online=1 switch_a=-1 stick=330
MOTOR id=1 seen=1 online=1 age_ms=1 valid_frames=200 bad_encoder=0 rpm=220.000 current_a=0.500 temperature_c=28 encoder=1234 state=RUNNING reason=OK
```

同样格式连续输出 id=1～4；running_mask 的 bit0～3 表示各轮是否处于 RUNNING，5 表示 ID1/ID3。
每轮有各自 state/reason、成功/异常计数、反馈年龄。四轮 MOTOR.online=0 时显示最后值或 NA，其含义与周期 CSV 离线填 0 不同。
从未收到反馈时 age_ms、rpm、current_a、temperature_c、encoder 为 NA；超时后仍显示最后值但 fresh=0。
CAN 采集集中在原有 1 ms ChassisTask，MonitorTask 和查询不再重复读取 PSR。sample age 较大表示状态可能已过时。
最近错误 last 可为 NONE、STUFF、FORM、ACK、BIT1、BIT0、CRC；无新错误时保留上一次捕获值及时间。
NONE 表示未捕获到错误，不证明从未发生错误。HAL 的 NO_CHANGE 不作为新的错误覆盖历史。
WAIT_BUS 期间硬件可能用 BIT0 表示恢复进度，代码不将其锁存为新发送故障，保留触发恢复前的错误线索。
state：DISABLED、WAIT_OFF、WAIT_CENTER、RUNNING；reason：OK、BAD_CONFIG、RC_OFFLINE、MOTOR_OFFLINE、
CAN_FAULT、TIMING_FAULT、TEMPERATURE、SPEED_FAULT。WAIT_OFF + OK 表示尚未完成关闭档确认。
RUNNING 无运行时长限制；不会因累计运行时间自动退出，运行超时状态码已删除。
当前 `MOTOR_DEBUG_CAN_FAULT_ENABLE=0`，软件暂不因 CAN 状态异常或发送入队失败产生 CAN_FAULT 停机；
CAN 行和错误计数仍是真实诊断，不受开关影响。因此 reason=OK 不能替代 CAN 状态判断。
反馈离线等其他停机条件继续生效，硬件 Bus-Off 也不会被该开关解除。置 1 并重新编译烧录可恢复原保护。

Bus-Off 恢复现已独立启用，沿用 USTC Stop/Start 机制，错误 ISR 请求、ChassisTask 执行。
恢复暂停期间不调用正常发送，因此不会每毫秒增加 tx_enqueue_fail；实际入队失败仍计数。

| RECOVERY 字段/状态 | 含义 |
|---|---|
| READY / 0 | 当前无恢复事务，或已完成恢复；上电默认也为此状态，需结合 init_ok/online 判断 |
| REQUESTED / 1 | 已观察到 Bus-Off，等待本次恢复尝试；连续尝试间隔至少 100 ms |
| WAIT_BUS / 2 | Stop/Start 成功，等待硬件退出 Bus-Off/INIT，不会反复打断硬件恢复序列 |
| WAIT_FEEDBACK / 3 | 硬件已恢复，等待任一活动电机本次 Start 后的新合法反馈；单电机模式仍只接受所选 ID |
| FAILED / 4 | HAL 调用或队列检查失败，禁止发送、停止自动重试；排查后复位 MCU |
| bus_off_count | 任务识别的 Bus-Off 故障次数，极短的多次事件可能合并，不是精确物理错误数 |
| attempts | Stop/Start 尝试次数；首次不增加人为等待 |
| success | 硬件退出 Bus-Off且至少一台活动电机新反馈确认后的成功次数，不表示四台均已恢复 |
| failures | Stop/Start 或队列检查失败次数；总线仍不空闲的等待不计入此值 |
| error | 0 无执行错误、1 Stop 失败、2 旧收发队列未清空、3 Start 失败 |
| last_ms | 上一次恢复耗时（任务发现故障至首个活动电机新反馈确认，包含重试等待）；从未成功为 NA |
| feedback_mask | 本次恢复后有新反馈的轮，bit0～3 对应 ID1～4；未有新反馈的轮保持零电流，不阻塞其他轮。上电无恢复事务时初始化为 15 |

当前没有手动 can_recover 命令。上述恢复不自动解除已经锁存的反馈离线等故障停止状态。
短暂故障未触发停止、且 CAN_FAULT 保护关闭时，原运行可继续；否则仍需重新拨杆启用。

接收过滤范围仍是 0x201～0x204：硬件拒绝的帧不进入软件异常计数，不能自动发现配置为 ID5 的电调。
合法帧说明当前解析规则通过，不能证明实际转速测量无误，也不能证明电机接线、机械方向或执行电流正确。

回复是手动诊断文本，不属于自动 CSV。请求待处理或发送时临时暂停新曲线帧，等待在途帧完成，
报告使用最大 2048 字节静态缓冲，超过 USB 队列容量时分段，全部传输完成后释放输出所有权，再恢复实时数据。
不补发缺失样本，控制任务不停；单电机首次曲线前保持 ready，四轮没有 ready。
Flash 事务占用 USB 时，诊断保留一份快照，等待事务释放，不插入 Flash 数据中；等待期间重复命令合并。
快照值及年龄对应请求时刻，不是延迟发送后的时刻。断线/挂起/会话改变丢弃未完成回复，重连后需重新查询。
电脑收到 BEGIN 但没有 END，应当作不完整报告。缓冲配置不足时返回 `MOTOR_STATUS ERROR report_too_large`。

## 1. 接口与报文规则

| 项目 | 当前约定 |
|---|---|
| 物理接口 | MC02 原生 USB，USB CDC 虚拟串口 |
| 电脑端 | 支持纯 CSV 的串口绘图工具，选择枚举出的 COM 口；八列日志按 status.period_ms 绘图，新日志 10 ms |
| 线路参数 | 115200、8 数据位、无校验、1 停止位、无流控 |
| 命令格式 | ASCII 命令，按行分帧 |
| 结束符 | 必须有真实 LF（字节 `0A`）；也接受 CRLF（`0D 0A`） |
| 大小写 | 区分大小写 |
| 空格 | 不自动去除命令前后的空格 |
| 最大长度 | 一整行最多 4096 字节，包括前缀、内容和换行 |
| 空行 | 忽略 |
| USB 包边界 | 不是命令边界；支持拆包、粘包 |
| 自定义包头/长度/CRC | 当前 USB 文本协议没有这些字段 |
| 通用 ACK | 没有统一的 OK、ACK、DONE 回复 |

115200 仅是 USB CDC 的线路设置，不限制 USB 总线实际传输速度。
在串口助手发送框输入命令内容，再选择追加 `\n`；不要输入两个字面字符反斜杠和 n。
`read` 有效，`READ`、` read`、`read `、`read 10` 都不是当前支持的命令。
仅发送 CR 不会执行命令。重新枚举或挂起时会清除尚未完成的输入行。

## 2. 命令清单

下表命令均须追加 LF 或 CRLF。

| 命令 | 用途 | 接受条件 | 成功行为 |
|---|---|---|---|
| `flash:<内容>` | 独立 Flash 写入、读回测试 | 测试功能启用，测试任务空闲或可重新尝试，USB 通道空闲 | 写入后从 Flash 读取并原样回传整行；实际发送完成后擦除测试扇区 |
| `read` | 回放电机日志 | 日志功能启用，`LOG state=5`，无待处理内部请求且 USB 空闲 | 按日志原采样周期输出八列，新日志 100 Hz；传输完成后自动擦除 |
| `status` | 查询服务与任务状态 | 至少一个 Flash 功能启用，且普通回复未被 USB 事务抑制 | 返回一行状态文本 |
| `clear` | 丢弃电机日志 | 日志功能启用，处于 READY、WAIT_READ 或 ERROR，且无冲突请求 | 后台擦除整个日志数据区和信息区；没有单独成功回复 |

`clear` 是删除操作，不是普通状态复位；它不擦除独立 Flash 测试扇区。
被拒绝的命令不会等待条件满足后自动执行，应待状态允许后重新发送。
当前没有通过 USB 设置目标转速、启动电机、修改 PID 或立即停止记录的命令。

## 3. Flash 测试

电脑发送：

```text
flash:1,2,3,4
```

MCU 返回同一行，含原来的换行。执行链路：

```text
接收完整行 → 准备测试扇区 → 写入 → 等待编程完成
→ 清空原接收缓存 → 从 Flash 读回 → USB 回传
→ 确认实际发送完成 → 擦除测试扇区 → 空闲
```

RAM 仅作通信中转，回传来源是后续 Flash 读取，不是原接收副本。
程序只匹配 `flash:` 前缀，不校验后面的内容是否为数字。要在 VOFA 画图，请发送 CSV 数值。
长度上限包括 `flash:` 和结束符。测试数据回传后自动擦除，不需要也不能再用 `read` 读取它。

USB 中断时不执行发送后的擦除。连接恢复且旧队列排空后，从 Flash 开头自动重新回传测试帧；
此前已收到的部分可能重复，不能将跨连接的接收片段直接视为一次完整测试结果。

## 4. 四轮日志、read 与八通道 CSV

### 4.1 记录触发与结束

日志仅支持 MOTOR_DEBUG_MODE_FOUR。开启 MOTOR_SPEED_LOG_TASK_ENABLE 前关闭 MOTOR_DEBUG_TELEMETRY 和 SERVO_RC_TELEMETRY_ENABLE。
在 READY 且控制快照有效、遥控有效、拨杆为 -1 时，四台均 RUNNING 且任意目标非零立即开始；
正负目标均触发，不读取摇杆边沿。首次就绪时已满足条件也可开始，不必重新经历零到非零。
待回放、回放、清理时不覆盖已有日志；清理后若条件持续成立不自动重录，需先解除再成立。

每 10 ms 从 Campus_CopyMotorStatus 获取各轮当前目标及最新反馈。目标来自 wheel[i].control.target，
内部 rad/s 换算输出轴 rpm，不能重复公共目标冒充四路历史目标。四轮目标合法且控制快照年龄 <100 ms 才有效。
从未收到反馈或反馈年龄 >=100 ms 的电机记录零；故障但反馈仍在线时记录实际速度。

结束规则：

1. 四台均在线且 RUNNING，同时达到各自当前目标（至少一路非零）后继续记录 5000 ms；计时不重置。
2. 四路目标全零，立即结束，不记录后续减速过程。摇杆回中本身不再是日志结束条件。
3. 控制快照超时/目标非法、遥控失联、拨杆退出 -1、四台全部退出 RUNNING 或存储/采样异常提前结束。

容差为 max(1 rpm, |各自目标| × 2%)；非零目标要求实际反馈非零且同方向。
单轮故障不立即结束整组日志，但尚未达标时必须重新满足四台在线且 RUNNING 才能开始达标计时。
结束后先刷新缓存与日志头，再进入 WAIT_READ。日志只观察控制，不发送电流或制动指令，没有电机运行时限。

### 4.2 Flash 中的数据

MLG2 版本每条 32 字节，小端编码：

| 字节偏移 | 内容 |
|---|---|
| 0～15 | 四个 int32 历史目标，ID1～4，单位 0.001 输出轴 rpm |
| 16～23 | 四个 int16 原始转子 rpm，ID1～4，未应用方向翻转 |
| 24～31 | 保留为 FF，供页对齐，不增加 CSV 列 |

没有逐条时间戳、反馈年龄、有效掩码、CRC32；离线零值与真实零速无法仅凭日志区分。
32 字节信息头保存版本、10 ms 周期、条数、方向、完成/擦除标记及记录长度。
256 条缓存占 8 KiB，日志分区约存 43.6 分钟。旧 MLG1 日志报 error=103 并保留，不能直接 read；
如需旧数据先使用旧固件导出，确认可丢弃后执行 clear。不会把缺失的目标补零当作有效历史记录。

### 4.3 回放

收到独立命令 read 后，重新通过 OCTOSPI 读取 Flash，新日志每 10 ms 发送一行八列纯 CSV。
已有 50 Hz MLG2 日志按其信息头的 20 ms 周期回放，旧 MLG1 仍报 error=103 并保留数据。

```text
225.000,221.500,-225.000,-223.100,100.000,98.800,-100.000,-99.600
```

| 列 | 含义 | 单位 |
|---|---|---|
| 1、2 | ID1 历史目标、实际转速 | 输出轴 rpm |
| 3、4 | ID2 历史目标、实际转速 | 输出轴 rpm |
| 5、6 | ID3 历史目标、实际转速 | 输出轴 rpm |
| 7、8 | ID4 历史目标、实际转速 | 输出轴 rpm |

三位小数、真实 LF 结束，无 speed: 前缀、表头、ready 或自动结束提示。
目标和反馈均来自本次 Flash 读取；实际值 = 原始转子 rpm ÷ (3591/187) × 本轮保存的方向。
目标已经是带方向的输出轴 rpm，不再除减速比或翻转。轮位须另外标定，不能把 ID 等同前后左右。
绘图样本间隔按 status 的 period_ms：新日志 10 ms，旧 50 Hz MLG2 为 20 ms。
录制 10 秒通常回放约 10 秒。USB 忙时等待，不丢条目，不突发补发。

### 4.4 回放后的清理

- 最后一帧实际发送完成后，标记本轮为擦除中，擦除使用的数据扇区，最后擦除信息区，回到 READY。
- 中途断线、挂起或重新枚举：停止回放并保留日志；重连且旧队列排空后重新 read，从头播放。
- 完整结束的 MLG2 日志可在 MCU 重启后读取；录制或信息写入中断电不承诺恢复。
- 正常自动清理可在重启后继续；手动 clear 中途断电不承诺恢复完整清理。
- USB 完成只指传输层完成，不证明电脑应用已保存文件；没有应用层 ACK。
- motor_status 可以手动请求，但等待日志释放 USB 后发送；不会夹入八列 CSV。

## 5. status 格式与字段

```text
FLASH state=1 error=0; TEST state=0 error=0; LOG state=2 records=0 error=0 stop=0 target=1 reached=0 period_ms=10
```

| 字段 | 含义 |
|---|---|
| `FLASH state/error` | Flash 服务初始化状态/结果，不是每次擦写的总结果 |
| `TEST state/error` | 当前或最近一次 Flash 测试事务的状态/错误 |
| `LOG state/error` | 电机日志状态/错误 |
| `records` | 已成功写入 Flash 的四电机记录条数，不包含待写 RAM 数据 |
| `period_ms` | 当前日志的采样/回放周期：新日志 10 ms，兼容旧 MLG2 为 20 ms；清理完成后恢复新记录默认值 |
| `stop` | 本轮或最近一轮停止原因；下一轮开始时清零 |
| `target` | 最近一次日志检查时，四轮目标快照是否有效，不代表达标 |
| `reached` | 本轮是否曾满足首次同时达标条件，不代表当前仍达标 |

`target`、`reached`、`stop` 不随日志持久化，重启加载日志后可能为 0。
自动清理完成后 `records/reached` 清零，`target` 按当前控制快照重新检查；`stop` 可能保留上一轮原因。
由于批量写页，刚开始记录时 `records=0` 不代表没有采样。

### 5.1 FLASH state

| 值 | 名称 | 含义 |
|---|---|---|
| 0 | STARTING | 初始化中 |
| 1 | READY | 初始化成功 |
| 2 | ERROR | 初始化失败 |

`FLASH state=1 error=0` 仍可能伴随后续 `TEST error` 或 `LOG error`，应分别检查。

### 5.2 TEST state

| 值 | 名称 | 含义 |
|---|---|---|
| 0 | IDLE | 空闲 |
| 1 | LOADING | 复制测试请求 |
| 2 | WAIT_SERVICE | 等待 Flash 初始化 |
| 3 | ERASE_FIRST | 写入前擦除测试扇区 |
| 4 | WRITE | 写入 Flash |
| 5 | READ | 从 Flash 读回 |
| 6 | SEND | 尝试发送读回内容 |
| 7 | WAIT_TX | 等待 USB 实际发送完成 |
| 8 | ERASE_LAST | 回传后擦除测试扇区 |
| 9 | PAUSED | USB 中断，等待恢复重传 |
| 10 | ERROR | 测试失败 |

### 5.3 LOG state

| 值 | 名称 | 含义 |
|---|---|---|
| 0 | BOOT | 读取已有日志信息 |
| 1 | PREPARING | 准备存储空间 |
| 2 | READY | 等待四台 RUNNING 且任一目标非零，无待回放日志 |
| 3 | RECORDING | 100 Hz 记录中 |
| 4 | FLUSHING | 停止后刷新缓存及信息头 |
| 5 | WAIT_READ | 日志已结束，可以请求 read |
| 6 | REPLAYING | 回放中 |
| 7 | ERASING | 清理日志 |
| 8 | ERROR | 日志故障 |

即使 state=5，若尚有内部请求或 USB 未空闲，read 仍可能被拒绝。

### 5.4 LOG stop

| 值 | 原因 |
|---|---|
| 0 | 尚无原因，或重启后没有该历史信息 |
| 1 | 四路控制目标全部为零 |
| 2 | 首次四轮同时达标后满 5 秒 |
| 3 | 遥控无效或超过 100 ms 未收到有效帧 |
| 4 | 日志空间满 |
| 5 | 采样队列满 |
| 6 | 错过完整的 10 ms 采样周期；不等同于 CAN 单帧丢失 |
| 7 | 控制快照达到 100 ms 未更新 |
| 8 | 拨杆退出 -1 启用档 |
| 9 | 四台全部退出 RUNNING |
| 10 | 目标含非有限/超出存储范围值，或快照 active_mask 不为四轮 |

### 5.5 错误码

| FLASH / TEST error | LOG error | 含义 |
|---|---|---|
| 0 | 0 | 无错误 |
| 1 | 1001 | 参数/地址范围错误 |
| 2 | 1002 | 通信或 I/O 错误，业务层也可能表示 USB 发送异常 |
| 3 | 1003 | Flash 忙状态超时 |
| 4 | 1004 | JEDEC ID 不符 |
| 5 | 1005 | 写保护或写使能失败 |
| 6 | 1006 | 写后读回比较失败，或擦除检查失败 |

日志专用错误：100 = 信息头非法；101 = 发现未完成记录；102 = 记录中方向配置改变；103 = 旧 MLG1 格式，数据保留。
初始化错误不应通过反复 clear 来处理；应先核对硬件/接口并重新初始化。

## 6. 普通提示与无回复行为

| 完整返回文本（另带 LF） | 触发原因 |
|---|---|
| `LOG NOT READY OR USB BUSY` | read 未被接受：不在 WAIT_READ、有内部请求或 USB 未空闲 |
| `LOG DISABLED` | read/clear 对应的日志功能关闭 |
| `FLASH TEST BUSY` | 测试事务或 USB 未空闲，新测试未启动 |
| `FLASH TEST DISABLED` | 测试功能关闭 |
| `LOG BUSY` | 当前不允许 clear |
| `INPUT TOO LONG` | 输入行超过 4096 字节，整行丢弃 |
| `COMMANDS flash:<data>, read, status, clear, motor_status; end with LF` | Flash 模式下无法识别命令 |

Flash 测试、日志回放或 motor_status 诊断回复占有 USB 时，普通提示和 status 回复被抑制。
普通提示采取尽力发送，队列满时可能无回复。因此无回复不等于成功，也不是固定故障码。
不要一次发送多个依赖前一个完成的命令，并假定 MCU 会自动等到条件满足。

两个编译开关见 [flash_tasks_config.h](App/Inc/flash_tasks_config.h)，当前默认均为 0（四轮实时模式）。
status 没有单独开关字段：关闭的测试模块可能显示 TEST state=0，关闭的日志模块可能显示 LOG state=2。
功能是否关闭应结合编译配置及 DISABLED 提示判断。两个开关均为 0 时不处理 Flash 命令，motor_status 仍有效。

## 7. 推荐操作及截图提示解释

先发 status。正常新启动示例为 FLASH=1/error=0、TEST=0/error=0、LOG=2/records=0。
这时 read 返回 `LOG NOT READY OR USB BUSY` 可以完全正常，因为尚无结束的电机日志。

完整记录步骤：

1. 编译为四轮离线配置，等待 LOG state=2，按现有拨杆与回中流程使四台均 RUNNING。
2. 任一轮目标变为非零（正负均可），LOG 进入 3；当前控制使用左竖直输入。
3. 四路目标全零或首次同时达标后 5 秒等条件结束，等待 LOG 从 4 进入 5。
4. 发 status 确认 period_ms，再发 read；接收八列纯 CSV，新日志按 10 ms、旧 50 Hz MLG2 按 20 ms 绘图。
5. 回放清理后回到 LOG state=2；启动条件需先解除再成立才能开始下一轮。

如果只做了 flash: 测试，测试数据已经自动回传并擦除，read 不会再次读出它。
右侧 I0=-1 是 VOFA 通道显示，不能当作固件 state/error 解读。

日志已改读现有四轮控制快照，不依赖 Chassis_SetTargetWheelRpm。target=1 只表示目标快照有效，
不表示四轮已运行或达到目标。后续底盘应维护四个实际控制目标与运行状态，日志模块不会使电机转动。

## 速度IK预备模块的错误回传

ARM_VELOCITY_IK_ENABLE默认0，无新增自动数据。开启后出现锁存故障时，USB可用且空闲时每200ms发一行ARM_ERROR，
包含code、joint、servo_id、requested、limit、unit及平移/角速度残差。故障暂停普通电机/舵机自动CSV，手动诊断和Flash完整事务仍优先。
错误是文本而非五列角度CSV。断开USB不会清错，重连继续报告。
LINEAR_SPEED_LIMIT和JOINT_SPEED_LIMIT只能通过MCU复位清除，保持原错误原因和5Hz回传；
改为零/合法速度、Arm_ClearFault、Arm_Enable、Arm_Stop及重复Arm_Init/Servo_Init均不能恢复。
实际IK输出时，新Move/Resume/TorqueOn也被拒绝，防止绕过复位锁存；复位后仍须按原流程显式使能。
当前不增加串口使能或运动命令；其他错误保留通过Arm API清错和重新使能的流程。
关节序号为肩到爪1～5，对应总线ID 3/2/1/4/5；requested不是实测转速。完整协议与恢复流程见ARM_KINEMATICS.md。
