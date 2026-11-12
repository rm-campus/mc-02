# 单电机与四轮联调（宏切换、50 Hz CSV）

更新：2026-11-12。当前默认四轮模式，已接入四路独立 PI 和故障隔离。
本次软件测试与编译不等同于上板验收；PI 初值仍需按实机整定。

## 配置入口

所有调试参数位于 [motor_debug_config.h](App/Inc/motor_debug_config.h)。

```c
#define MOTOR_DEBUG_MODE_SINGLE 0
#define MOTOR_DEBUG_MODE_FOUR   1
#define MOTOR_DEBUG_MODE MOTOR_DEBUG_MODE_FOUR
```

修改模式后重新编译烧录。MOTOR_DEBUG_ENABLE 是总开关；MOTOR_DEBUG_TELEMETRY 控制自动回传。
单电机模式由 MOTOR_DEBUG_MOTOR_ID 选择 ID1～4，使用 MOTOR_DEBUG_KP/KI，其余三路零电流。
四轮模式使用 MOTOR_DEBUG_M1_KP/KI 到 MOTOR_DEBUG_M4_KP/KI，各自独立积分、速度参考、输出与启停状态。
四组默认均为 Kp=0.05、Ki=0；单位分别是 A/(rad/s)、A/rad。改变某台参数不会改变其他台。

| 两种模式共用参数 | 当前值 |
|---|---|
| 控制周期 | 1 ms，原有 ChassisTask，无新增任务 |
| 控制输入 | CH2 / rocker_l_v，左摇杆竖直 |
| 原目标 | 摇杆 ×450/660 rpm，正负半区对应 ±450 rpm |
| 速度参考斜坡 | 300 rpm/s，零到满目标约 1.5 s |
| 指令电流 / 积分项限幅 | 每台 ±15 A / ±0.3 A |
| 温度 / 超速停止条件 | ≥110℃ / 输出轴速度绝对值 ≥540 rpm |
| 遥控 / 电机反馈超时 | 均为 100 ms；电机反馈年龄 <100 ms 才在线 |
| 周期异常 | 间隔为 0 或 >5 ms |
| 回中判定 | 启用前左竖直 ±40 连续 300 ms；运行时没有死区重映射 |
| 基本静止条件 | 启用前本电机绝对速度 ≤0.5 rad/s，约 4.77 rpm |
| 运行时间 | 无限制，没有运行计时字段或超时停机逻辑 |
| MOTOR_DEBUG_CAN_FAULT_ENABLE | 0，软件 CAN_FAULT 停机暂时关闭，诊断/总线恢复仍工作 |

PI 内部 rad/s，回传使用减速后输出轴 rpm，不能再次除减速比。四台原目标相同但输出电流独立。
沿用现有 reversed 方向掩码，默认 0；ID1～4 的实际轮位和安装方向尚未标定，本功能不做麦轮运动学。
四台同号目标不保证车体同方向行驶。上限为指令范围，不保证带负载实际能达到目标。

## 启用和停止

1. C620 编号设为 1～4，连接 MC02 CAN1（FDCAN1、PD0/RX、PD1/TX），核对 H/L 线序。
2. 用本次 build/Debug/rm_campus.elf 下载，USB 打开对应 CDC 串口。
3. 通信正常时，最左拨杆 CH7/switch_a 先置 +1，再经中档拨到 -1。
4. 每台电机独立检查自己的反馈、温度、转速，以及公共遥控回中 300 ms 条件，满足后进入 RUNNING。
5. 左竖直摇杆控制各台相同原目标。四轮模式无需等四台全部在线，健康电机可以先启动。

某台离线、过温、超速或本台参数非法时，仅该台清 PI 并置零指令，其他轮继续运行。
故障消失后该台需重新观察 +1、-1、回中才能运行；不会在拨杆保持 -1 时自行重新启用。
拨杆是公共输入：人为拨到 0 或 +1 会停止所有正在运行的轮。遥控失联、公共周期异常也会影响所有轮。
摇杆回到精确 0 只使参考按斜坡降到零，PI 仍工作，可产生制动电流；拨杆退出是撤流，电机可能滑行。
总线故障可能使整条 CAN 暂停；单轮软件隔离不等于共享物理总线故障隔离。USB 断开不触发停机。

## 自动回传格式

按新串口助手截图支持的逗号数字格式，ASCII、三位小数、真实 LF 换行，一行一帧，无 motor: 前缀。
使用普通接收/绘图模式，每 20 ms 一帧（50 Hz），不需要发送 read。

四轮默认模式只有以下五列，全部为 rpm：

```text
225.000,221.500,223.100,220.800,222.600
```

顺序：公共原目标、ID1 实测、ID2 实测、ID3 实测、ID4 实测。
公共目标未经速度斜坡，也不表示每一台都已经使能。离线轮的转速列填 0；
因过温等原因停机但反馈仍在线的轮，继续显示真实反馈转速。0 无法区分静止/离线，需手动诊断。
至少一台 RUNNING 才自动回传，全部停止后不产生新帧。四轮模式不发 ready、电流、温度或其他自动文字。

单电机模式先发送 ready，然后保留四列：原目标 rpm、实际 rpm、指令电流 A、反馈电流 A：

```text
ready
225.000,218.500,0.420,0.398
```

单电机每次重新运行/识别到 USB 会话恢复后重新发送 ready；四轮模式重连后直接恢复五列 CSV。
USB 忙时不积压、不追赶补发；断线不保存历史。已经提交的末帧可能在退出运行后才到达电脑。
USB CDC 包边界不作为帧边界；电脑应按 LF 组帧。新助手的实机收包和绘图效果仍需上板核验。

## 手动诊断与 CAN 恢复

motor_status 加 LF 在两种模式始终可用。四轮诊断包含 mode=FOUR、running_mask、四条 MOTOR id=1～4，
各自的在线、反馈年龄、成功/异常计数、转速、电流、温度、state/reason，另有 CAN 和 RECOVERY 公共信息。
回复快照存储于静态缓冲，超过一个 USB 队列容量时分段发送；持有输出所有权直到 END 所在传输完成，
期间不夹入自动 CSV，控制仍运行。手动文本含数字，排查时宜用数据/终端视图查看，曲线分析剔除诊断段。
完整定义见 [COMMUNICATION_PROTOCOL.md](COMMUNICATION_PROTOCOL.md)。

Bus-Off 采用 [USTC-RoboWalker Stop/Start](https://github.com/yssickjgd/damiao_mc02_bsp/blob/30ff2965d55afbcad500027b0e1e5031ea6f93b8/dm02_test/User_File/1_Middleware/Driver/CAN/drv_can.cpp#L260)：
ISR 请求，ChassisTask 检查旧队列清空后重启；首次立即尝试，反复故障尝试间隔至少 100 ms。
硬件退出 Bus-Off 后，活动电机任一新反馈即可确认总线恢复；每台的输出还要通过本次恢复后的新反馈掩码。
缺少某台反馈不会阻塞已恢复轮。后续该台新反馈到达时更新掩码，但不会解除已经锁存的本台离线停机。
RECOVERY.feedback_mask 的 bit0～3 对应 ID1～4；last_ms 统计到首个活动电机新反馈确认的时间。
HAL 失败或旧队列未清空进入 FAILED，禁止发送，排查后复位；不在中断中阻塞或无限重初始化。
短暂中断未触发保护可继续；反馈达到 100 ms 未更新会触发本轮 MOTOR_OFFLINE，需重新拨杆。

## 测试与维护

```powershell
python tests/run_motor_debug_tests.py --cc "E:/CLion/CLion 2026.2.2/bin/mingw/bin/gcc.exe"
python tests/run_can_recovery_tests.py
```

包含单电机/四轮、总开关、CAN_FAULT 开关、独立 Kp/Ki 和积分、逐轮故障、100 ms 边界、
缺失电机、重新启用、61 s 连续运行、CSV 列数/单位/50 Hz、断线、背压、完整分段诊断与恢复掩码测试。
Flash 功能保留关闭，旧日志协议与 50 ms 日志采样离线规则未改；本次调试不调用四轮目标日志接口。
没有新增 FreeRTOS 任务，未改 .ioc 或堆大小；实板需检查控制时序、栈高水位与机械方向。
