# USB 通信协议与状态码

更新日期：2026-11-13。本文描述当前工程已经实现的 USB 应用层协议，不是未来接口设计。
实现入口：[flash_tasks.c](App/Src/flash_tasks.c)、[usb_app.c](App/Src/usb_app.c)。
任务、分区和底盘接口说明另见 [FLASH_TASKS.md](FLASH_TASKS.md)。

## 当前调试模式：纯 CSV，默认四轮

模式由 motor_debug_config.h 中 MOTOR_DEBUG_MODE 选择：SINGLE=0、FOUR=1，默认 FOUR。
MOTOR_DEBUG_ENABLE=1、MOTOR_DEBUG_TELEMETRY=1，两个 Flash 开关均为 0。两种模式无运行时限。
上电等待使能，不主动回传；无需发送 read。read/status/clear/flash: 在 Flash 关闭时无回应。
motor_status 加 LF 始终可用。电机反馈在线阈值为 <100 ms，达到 100 ms 则只停止对应电机。
最左拨杆 +1 确认关闭、-1 启用，左竖直回中 ±40 保持 300 ms，本轮基本静止后独立进入 RUNNING。
四轮无需全部在线；单轮异常不使健康轮停机。共同遥控、调度和物理总线故障可能影响全部轮。

四轮模式每 5 ms 发送九列数字，前五列为输出轴 rpm，后四列为反馈电流 A：

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
启用任一 Flash 任务前须关闭 MOTOR_DEBUG_TELEMETRY。操作细节见 [SINGLE_MOTOR_DEBUG.md](SINGLE_MOTOR_DEBUG.md)。

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

日志仅支持 MOTOR_DEBUG_MODE_FOUR。开启 MOTOR_SPEED_LOG_TASK_ENABLE 前关闭 MOTOR_DEBUG_TELEMETRY。
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
