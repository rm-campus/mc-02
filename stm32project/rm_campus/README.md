# rm_campus：达妙 MC02 校赛电控工程

开发交接快照：**2026-11-25**。本目录是 STM32 固件工程根目录。

工程面向四轮麦克纳姆底盘与后续机械臂。主控为达妙 MC02（STM32H723VGT6），使用 HAL、
FreeRTOS/CMSIS-RTOS2、CMake/Ninja 和 GNU Arm 工具链。

## 当前默认固件：USB 舵机状态与 10° 点动

`Debug` 和 `Release` 预设启用 `SERVO_STATUS_ONLY=ON`。默认调试产物为
`build/Debug/rm_campus.elf`。上电只运行 USB CDC 和 UART7 舵机轮询两个应用任务。
板端依次查询 ID 001～005，电脑可以在遥控器关闭时查看全部舵机状态。
默认开启 UART7 接收监视，打开 USB 串口即可看到 `SERVO_RX` 行。HEX 栏保留 MCU 实际收到的全部字节，
ASCII 栏便于读指令；时间戳取自 MCU 接收中断。掉线或背压导致缓存覆盖时查看 `lost`。
可在 [servo_config.h](App/Inc/servo_config.h) 将 `SERVO_RX_TRACE_ENABLE` 设为 0 关闭这项输出。

串口助手选择 MC02 的 USB 串口，115200、8N1、ASCII、行尾 LF 或 CRLF。
循环发送 `servo_status`，建议间隔 100 ms。每次从 `SERVO_STATUS BEGIN` 到
`SERVO_STATUS END` 为一份报告；每台查看 `online`、反馈位置 `position`（P 值）和 `age_ms`。
未收到位置反馈时显示 NA。掉线后会显示最后的反馈值及其年龄。

手动发送 `servo_test_001_1` 使 001 从实测位置顺时针转 10°，`servo_test_001_2` 逆时针转 10°；
将 001 替换为 002～005 即可选择其余舵机。方向按正对主动输出轴端面观察。每步默认 500 ms，
每个被接受的命令执行一次。状态查询可循环发送，点动使用手动发送。运动前确认位置模式、读取当前位置；
发出后等位置反馈确认，再回复 DONE。未接的 005 会回复 OFFLINE。
[servo_config.h](App/Inc/servo_config.h) 中 `SERVO_JOG_TEST_ENABLE` 默认跟随监测预设开启，设 0 并重新编译即恢复纯只读。
同一处可调 `SERVO_JOG_STEP_DEG` 和 `SERVO_JOG_MOVE_MS`。软件限位沿用设备表，实际机械端点需按装配标定。
详见 [USB 点动协议](COMMUNICATION_PROTOCOL.md)。

## 整机遥控固件

用 `cmake --preset RobotControl` 和 `cmake --build --preset RobotControl` 构建，产物为
`build/RobotControl/rm_campus_robot.elf`。此预设自动关闭 USB 舵机监测与点动并启用
[ROBOT_CONTROL_ENABLE](App/Inc/robot_control_config.h)。`ServoId` 预设单独用于读号、改号。
整机仅运行遥控、底盘、舵机总线、监测和状态任务，独占四轮和五舵机。USB 调试、Flash 日志及速度 IK 属于其他固件配置。

| CH9 实体档位（默认读数） | CH2 左竖直 | CH4 左水平 | CH1 右水平 | CH3 右竖直 | LED |
|---|---|---|---|---|---|
| 上（+1） | 前后移动 | 横向移动 | 车身转向，可与平移叠加 | 空闲 | 绿 |
| 中（0） | ID3 肩抬降 | ID2 肘伸收 | 空闲 | 空闲 | 蓝 |
| 下（−1） | ID1 腕俯仰 | ID4 腕滚转 | 车身原地转向 | ID5 夹爪闭合程度 | 青 |

启动按真实 SBUS 新帧计数，至少 3 帧且首末相隔 50 ms。CH7/CH8 及回中操作不参与使能。
切档立即撤销旧对象的运动，新档连续稳定 30 ms 后接管。遥控发生失控、丢帧或距有效帧达到
100 ms 时撤销运动。等待/失联显示红色；每次链路就绪时，PB15/TIM12_CH2 的板载蜂鸣器短鸣约 100 ms。
底盘 CH2/CH4 平移保留 ±60 死区，CH1 自转从 0 线性映射。CH1 中位偏差会产生小幅转向给定。
全部底盘轴回中时，轮速参考和积分立即清零，保留零速比例制动。
底盘仍有给定斜坡，作用于推杆和非零目标之间的变化。舵机关节沿用直接的摇杆映射。

舵机运动或停止事务失败后，丢弃该轴旧目标并等待新的位置反馈，再从实测位置在原档接管。
杆未回中时，位置恢复后会按当前杆位继续运动。舵机模式仍可查询，整机遥控不以模式回读作为接管条件。
步长按已知的 180° 位置模式换算，其余情况按 270° 换算。实际执行取决于舵机自身模式。
ID3 肩关节的软件限位为 P834～P2166，在 270° 位置模式下对应 P1500 两侧各约 90°。
整机从新鲜反馈接管，若 ID3 上电时已在限位外，先断电检查并调整到允许范围内。

**底盘。** 轮端上限默认 400 rpm，沿用四路 PI 和电流、温度、反馈保护。几何尺寸未知时，
上/下档满杆纯转的轮端目标幅值分别为上限的 1 和 1/3（约 400、133.3 rpm），
可独立改 `ROBOT_UPPER_TURN_FRACTION`、`ROBOT_LOWER_TURN_FRACTION`。
填入前后/左右轮心全距 `ROBOT_WHEELBASE_M`、`ROBOT_TRACK_M` 后，自动采用
`ROBOT_UPPER_MAX_YAW_DEG_S`、`ROBOT_LOWER_MAX_YAW_DEG_S`（初值 90、30 °/s）。
轮半径 0.075 m。车体角速度是运动学指令，实际值还受轮胎打滑等影响。

**夹爪。** 上电第一次有效位置回读作为零闭合基准。下档接管后的目标为
`P_boot + abs(CH3)/660 × (P_closed − P_boot)`。离开下档时保留已发送的位置目标和扭矩，
其他档忽略 CH3。再次进入下档要从当前杆位经过上次保持的闭合比例才能接管，正负两侧都有效。
首次进入时基准为 0%；已在零位则下一次拨动即可操作。肩、肘和两个腕轴采用回中保持的增量控制。
当前 `ROBOT_GRIPPER_CLOSED_P=2500` 取自设备表中的协议上限，方向及端点待实测。在拆开夹爪联动
或确认机械行程后写入实测闭合值。仅凭协议上限无法判断实物是否已经闭合。CH9 上下档、各轴机械正向也需上板核对。

整机专项测试入口为 `tests/run_robot_link_tests.py`、`tests/run_robot_motion_tests.py`；
底盘 0x200 帧及 LED/蜂鸣时序集成回归在 `tests/run_motor_debug_tests.py` 中执行。

## 普通调试固件

下面是关闭 `SERVO_STATUS_ONLY` 后的台架配置。需要时使用
`cmake --preset Debug -DSERVO_STATUS_ONLY=OFF` 再构建；重新执行 `cmake --preset Debug` 即选回 USB 舵机监测预设。

**默认四轮模式，CH8（左起第二个拨杆）在运行中选择目标。+1 或 0 为 CH2 四轮同速，−1 为麦轮控制。左杆控制前后（CH2）和横移（CH4），右杆水平（CH1）控制旋转，右推对应俯视顺时针。CH2/CH4 保留 ±60 死区，CH1 自转无死区，可边走边转，合成后每轮最高 400 rpm。
最左拨杆先 +1 确认关闭，再 -1 启用，各电机独立检查回中 300 ms 与基本静止；健康电机不等待缺失电机。**
四轮模式以 200 Hz 发 CSV。同速 9 列，麦轮模式 12 列（ID1～4 目标 rpm、实测 rpm、反馈电流 A，各四列）。
单电机模式保留 ready，随后 200 Hz 纯 CSV 四通道：目标 rpm、实测 rpm、指令 A、反馈 A。
两种模式均无运行时限，共用 ±9.5 A、110℃ 与 100 ms 电机反馈超时。两个 Flash 任务开关均为 0。
当前电机总线已切换到板载 CAN1（FDCAN1，PD0/RX、PD1/TX）；电调接线需同步移到 CAN1。
上电后可发送 `motor_status` 加 LF 查询选定电机反馈、CAN 状态与控制故障，不需要先进入运行或收到 ready。
按用户要求，当前 `MOTOR_DEBUG_CAN_FAULT_ENABLE=0`：临时禁用 CAN 状态异常/发送入队失败触发的软件停机。
CAN 诊断和 LED 指示保留；反馈达到 100 ms 未更新时，仅对应电机触发 MOTOR_OFFLINE。恢复 CAN_FAULT 保护将宏置 1 后重新编译烧录。
已移植 USTC-RoboWalker 的 Bus-Off Stop/Start 恢复方法，由现有 ChassisTask 执行；无需整板复位即可尝试恢复。
硬件恢复后，活动电机任意一台新反馈可恢复总线发送；逐轮检查恢复反馈掩码，未收到新反馈的轮保持零电流，不阻塞其他轮。
若已触发反馈离线等故障停机，恢复通信不自动重新使能；HAL 恢复失败则停止重试，诊断显示 FAILED。
已实现 X 型麦轮平移、旋转混合与四轮速度闭环。轮位、安装方向和遥控轴正方向待实车标定。LED 绿色只表示通信在线。

## CH9 舵机遥控（当前本机已启用）

SERVO_RC_ENABLE=1、SERVO_RC_TELEMETRY_ENABLE=1，配置位于 App/Inc/servo_config.h。
CH9=+1 关闭舵机遥控；0 时 CH2/CH4/CH1 分别控制 ID1/2/3；-1 时 CH2/CH4 控制 ID4/5。
摇杆增量控制，±60 死区、每轴初始最大 30°/s；回中保持目标，不回中位。上电先见 +1，
进入组后相关轴回中 300 ms，在线且位置模式有效的舵机从回读位置独立启用。没有底盘 yaw 舵机。
切组/关闭/遥控失联取消待发运动并尽力发送停止，保留扭矩；单轴离线或手动停止/暂停/释力后锁住该轴，
需 CH9 回 +1 再重新启用。停止通过总线发送，通信中断时不能保证实际停止。机械限位和正方向尚待实机标定。
底盘仍按 CH7/CH8 运行，同一摇杆可能同时驱动底盘和舵机。

USB 自动输出按 CH9 切换：+1 使用原电机 200 Hz 协议，0/-1 仅输出五台回读角度的 50 Hz 五列 CSV。
位置查询仍为每台约 20 Hz，50 Hz 输出可能重复最新测量；无有效反馈填 -1.000。
270°模式按 (P-500)×0.135° 换算，180°模式按 (P-500)×0.09°；这是协议角度，不是机械零位角度。
手动诊断期间暂停 CSV。关闭 SERVO_RC_ENABLE 保留原 USB 手动功能；仅关闭 SERVO_RC_TELEMETRY_ENABLE
则舵机遥控继续工作，USB 恢复原电机回传。Flash 模式须同时关闭电机与舵机自动回传。
复用 ServoBusTask 和 UsbTask，未增加任务或修改 .ioc。详见 SERVO_INTEGRATION.md。

## 机械臂速度逆运动学预备模块

已加入默认关闭的速度IK模块，说明见 [ARM_KINEMATICS.md](ARM_KINEMATICS.md)。
App/Inc/arm_config.h 中 ARM_VELOCITY_IK_ENABLE=0、ARM_VELOCITY_IK_OUTPUT_ENABLE=0，机械标定也默认无效。
主要参数为 SERVO_MAX_SPEED_DEG_S=30 和 ARM_MAX_LINEAR_SPEED_MM_S=50；均为软件允许值，不是舵机硬件极限。
控制q3/q4轴交点的平面速度、俯仰、腕滚转，夹爪独立；TCP后续定义，不绑定摇杆或拨杆。
逻辑肩/肘/腕俯仰/腕滚转/爪默认映射总线ID 3/2/1/4/5，不改变已有CH9映射。
未限幅速度解超关节上限、任务残差过大或其它故障均停止整组并锁存，USB以5Hz持续报告ARM_ERROR。
LINEAR_SPEED_LIMIT和JOINT_SPEED_LIMIT只能通过MCU复位清除；重复模块/驱动初始化不能绕过，其他错误仍保留软件清错。
末端限速参数旁已有当前208.544/150.339 mm/s理论上界的公式与适用条件，实际配置仍为50 mm/s。
本模块未启用时原控制保持；启用实际IK输出必须关闭SERVO_RC_ENABLE并完成PMOD/零位/方向/传动比/限位标定。
速度 IK 复用现有任务。本版本包含速度 IK 预备模块，实板验证待完成。

## 新对话从这里开始

请新接手的开发者或助手先阅读本文件，以及：

- [SINGLE_MOTOR_DEBUG.md](SINGLE_MOTOR_DEBUG.md)：两种调试模式宏、独立 PI、启停和 CSV 通道说明。

1. [COMMUNICATION_PROTOCOL.md](COMMUNICATION_PROTOCOL.md)：当前 CSV、手动诊断及保留的 Flash 协议。
2. [FLASH_TASKS.md](FLASH_TASKS.md)：Flash 分区、两个功能开关、任务、日志生命周期和目标转速接口。
3. [DRIVER_INTEGRATION.md](DRIVER_INTEGRATION.md)：遥控、CAN、电机、LED 和 USB 基础接口。
4. [IMU_INTEGRATION.md](IMU_INTEGRATION.md)：BMI088、四元数 EKF、校准、坐标及限制。
5. [SERVO_INTEGRATION.md](SERVO_INTEGRATION.md)：UART7 众灵舵机接线、ID、API、USB 命令和验收。

继续开发前应重新核对实际源码和工作区改动；本文不是已经完成上板验收的声明。
Flash 日志已直接读取 `Campus_CopyMotorStatus()` 中的四轮真实控制目标、反馈和 RUNNING 状态，
不再依赖 `Chassis_SetTargetWheelRpm()`。该预留发布接口保留，后续底盘必须维护真实四轮目标快照；烧录和推送需另行指示。

## 硬件与外设

| 模块 | 当前连接与配置 |
|---|---|
| MCU | STM32H723VGT6，当前 SYSCLK 480 MHz，HCLK 240 MHz |
| 存储 | MCU 内部 Flash 1 MiB；内部 RAM 560 KiB＋4 KiB 备份 SRAM；板载 W25Q64JV 外部 Flash 8 MiB |
| 四电机 | M3508＋C620，FDCAN1，PD0/RX、PD1/TX，经典 CAN 1 Mbps，ID 1～4 |
| 遥控 | FS-i6X＋iA6B，UART5/PD2，100000 baud、8E2（HAL 9 位字长含偶校验），循环 DMA；板载电路已反相 |
| BMI088 | SPI2 Mode 3 / 3 MHz；PC1 MOSI、PC2_C MISO、PB13 SCK；PC0 ACC_CS、PC3_C GYRO_CS；PE10/PE12 普通输入 |
| RGB LED | SPI6 MOSI/PA7，6 MHz，WS2812 编码 |
| 舵机总线 | UART7/PE7，P13-3，TX/RX Swap，115200/8N1 单线半双工、开漏；PE8 留空 |
| USB | MCU 原生 USB，PA11/PA12，USB_OTG_HS 内置 FS PHY，CDC 虚拟串口 |
| W25Q64JV | OCTOSPI2→OCTOSPIM Port 1；PB2 CLK、PE11 CS、PD11 IO0、PB0 IO1、PA3 IO2、PA1 IO3；10 MHz、Mode 3 |

外部 Flash 通过驱动访问，不自动扩展内部 RAM/程序 Flash。当前不使用外部 Flash 内存映射或 DMA。
W25 驱动使用普通单线指令和 24 位地址，不能因 CubeMX 配了四个 IO 就认为所有命令都在四线模式运行。

## 已实现与尚未完成

| 项目 | 当前状态 |
|---|---|
| SBUS 遥控接收 | 已实现 DMA、组帧、十通道解析、失控处理；用户完成实机通道映射测试 |
| 遥控 5 Hz USB 临时回传 | 已按用户要求彻底删除，原 `CAMPUS_USB_RC_TEST_ENABLE` 宏也已删除 |
| CAN / M3508 | 1 ms 四路独立 PI，统一一帧 0x200 电流控制；已有 Bus-Off 恢复 |
| 调试模式 | 默认 FOUR，单电机 SINGLE 可选 ID1～4；四轮各有 Kp/Ki，Kp 均为 3.0，Ki 按 ID1～4 为 0.1/0.08/0.12/0.1，参数可逐轮独立修改；无运行时限 |
| USB CDC | 已实现有界收发队列、会话处理和实际发送完成凭据 |
| BMI088 / 姿态 | 已接入采集、静止校准和 C 版四元数 EKF；未接入恒温、绝对航向或位置估计 |
| 外部 Flash | `.ioc`、初始化、HAL OSPI 和 W25Q64 驱动均已接入正式工程 |
| Flash 测试任务 | 功能保留，当前关闭；USB 测试帧写 Flash→实际读回→USB 完成后擦除 |
| 电机日志 | 已适配四轮，当前关闭；100 Hz 保存四路目标和原始转速，read 回放八列 CSV 后清理 |
| 底盘目标接口 | 已提供线程安全发布/快照接口；当前底盘尚未调用、目标默认无效 |
| 底盘闭环 | X 型麦轮平移、旋转混合与四轮独立速度 PI；轮序与方向待实车确认 |
| UART 舵机/机械臂 | UART7 单线半双工、五台位置/查询/启停/扭力 API 已接入；上电只查询，不含内部 PID 控制或机械臂运动学 |

已有主机回归和 Debug 编译通过记录。Flash 功能曾收到用户上板截图中的 `LOG NOT READY OR USB BUSY`，
这只能说明 read 请求被解析并拒绝，不能作为 Flash 读写或四轮日志的完整实板验收。
Flash JEDEC ID、实际擦写、任务栈高水位、四轮方向和时序仍需实测确认。

## 遥控映射：以实测结果为准

已在 `Bsp/Inc/fxi6x.h` 与解包代码中落实，不要换回早期参考文档中的 CH2/CH3 顺序。

| 通道 | 控件 | Fxi6x_Data 字段 |
|---|---|---|
| CH1 | 右摇杆水平 | `rocker_r_h` |
| CH2 | 左摇杆竖直 | `rocker_l_v` |
| CH3 | 右摇杆竖直 | `rocker_r_v` |
| CH4 | 左摇杆水平 | `rocker_l_h` |
| CH5 | VRA | `knob_l` |
| CH6 | VRB | `knob_r` |
| CH7～CH10 | 正对遥控器面板，从左至右四个拨杆 | `switch_a`～`switch_d` |

摇杆、旋钮为 `int16_t` 的 -660～660；拨杆为 `int8_t` 的 1/0/-1。
通过 `Fxi6x_CopyData()` 获取快照；`valid` 只表示最近帧解析有效，在线判断还需接收历史和超时。
遥控端应设置 PPM、s.bus、10 通道；CH5～10 分别对应 VRA、VRB 和四个拨杆。

## 电机单位与日志规则

- `Motor3508.rotor_rpm[4]`：C620 原始转子 rpm，未除减速比、未应用方向翻转。
- `velocity[4]`：减速箱输出轴 rad/s，已除减速比并应用 `reversed`。
- `position[4]`：输出轴累计 rad；`current[4]`：A；`command[4]`：协议原始电流值。
- 减速比为 `3591/187 ≈ 19.2032`。车轮直接连接输出轴时，车轮 rpm = 转子 rpm ÷ 减速比 × 方向。
- 当前方向掩码为 0x0A（反转 ID2、ID4），对单轮、四轮同速和麦轮模式均生效。
- 配置轮位为左前 ID1、右前 ID2、左后 ID3、右后 ID4；这是程序约定，仍需与实车接线和安装方向核对。

Flash 离线日志仅支持 FOUR，默认关闭；开启日志前须将 MOTOR_DEBUG_TELEMETRY 和 SERVO_RC_TELEMETRY_ENABLE 均置 0。
日志直接读取现有控制快照：四台均 RUNNING 且任意轮目标非零即开始，正负均可，不依赖摇杆边沿。
四路目标全零立即结束；四台在线、RUNNING 且首次同时达到各自当前目标后 5 秒结束，计时不重置。
拨杆关闭、遥控失联、控制快照超时、全部退出 RUNNING 或存储异常也会结束；单轮故障不结束整组记录。
日志不控制电机，不增加电机运行时限。首次就绪可立即触发；回放清理后需启动条件先解除再成立。

MLG2 每条 32 字节：四个 int32 历史目标（0.001 rpm）、四个 int16 原始转子 rpm、8 字节预留。
不增加逐条时间戳、反馈年龄、有效掩码或 CRC32。离线电机写 0，反馈在线阈值与控制统一为 <100 ms。
256 条队列占 8 KiB，100 Hz 下日志分区约可存 43.6 分钟。旧 MLG1 日志报 error=103 并保留，需处理后显式 clear。
新日志按 100 Hz 记录，收到 read 后重新从 Flash 读取，以 100 Hz 回放八列纯 CSV：

```text
目标1,实际1,目标2,实际2,目标3,实际3,目标4,实际4
```

上行仅表示列含义，线上不发送表头。全部为输出轴 rpm，三位小数、LF 结束，无 speed: 前缀或 ready。
电脑按 status 的 period_ms 绘图，新日志为 10 ms；旧 50 Hz MLG2 保留 20 ms 回放周期。
最后一帧 USB 确认完成后才擦除，断线保留日志并可重新 read。
实时模式为单轮四列、四轮同速九列或麦轮十二列，均为 200 Hz；与离线八列日志通过宏切换。

## FreeRTOS 任务与开关

当前 tick 为 1 kHz，动态堆为 **48 KiB**。下表栈大小按 32 位 word 表示；CMSIS 属性中的 stack_size 为字节。

| 任务 | 优先级 | 栈 / words | 主要工作 |
|---|---|---:|---|
| ImuTask | High2 / 42 | 1024 | 约 1 ms 周期采集与 EKF |
| RcTask | High1 / 41 | 512 | 遥控 DMA 事件处理与解析 |
| ChassisTask | High / 40 | 1024 | 1 ms 单电机/四轮独立 PI、逐轮启停与 CAN 恢复 |
| ServoBusTask | AboveNormal / 32 | 512 | UART7 五台舵机，非阻塞请求、轮询和错误恢复 |
| MonitorTask | Normal / 24 | 256 | 10 ms 在线状态汇总 |
| MotorSpeedLogTask | Normal / 24 | 512 | 当前不创建；启用后 10 ms 采样或回放 |
| UsbTask | BelowNormal / 16 | 1024 | USB 服务、200 Hz CSV 及 motor_status 手动诊断 |
| FlashServiceTask | BelowNormal / 16 | 512 | 当前不创建；唯一实际操作 W25Q64 的任务 |
| StatusTask | Low / 8 | 256 | LED 状态 |
| FlashTestTask | Low / 8 | 512 | 当前不创建；USB→Flash→USB 测试事务 |

新三个 Flash 任务在 `freertos.c` 的 USER CODE 中调用 `FlashTasks_Create()` 条件创建，
并非 `.ioc` 原有 Tasks01 中的无条件任务。两个业务开关在 [flash_tasks_config.h](App/Inc/flash_tasks_config.h)，
默认均为 0；重新开启 Flash 功能前先将 MOTOR_DEBUG_TELEMETRY 和 SERVO_RC_TELEMETRY_ENABLE 均置 0，避免两种输出混用。
编译期会拒绝同时启用实时回传和 Flash 任务。Flash 关闭不会擦除已有数据。
本次没有新增任务；MonitorTask 保持 256 words，格式化和回传放在 UsbTask 中。

## 代码布局与入口

```text
Core/                    CubeMX 初始化、异常/中断和 RTOS 入口
Bsp/                     CAN、电机、遥控、LED、USB、BMI088、Flash 驱动
App/                     任务、协议分发、IMU、日志与目标发布接口
Algorithm/               四元数 EKF、Kalman、滤波适配
USB_DEVICE/              USB CDC 类接口及平台支持
Drivers/ Middlewares/    ST HAL、CMSIS、USB、FreeRTOS
tests/                   主机回归测试，不操作真实板卡
cmake/                   工具链和 CubeMX 构建描述
build/Debug/             当前有效构建输出
.codex-backups/           本地历史备份，不属于现行源码
```

关键入口：

- `App/Src/campus_app.c`：电机快照、CAN 回调、遥控/底盘/监测任务。
- `Bsp/Src/can.c`：Bus-Off ISR 请求、任务内 Stop/Start、旧队列清空检查及恢复发送门；首次立即尝试，重复尝试至少间隔 100 ms。
- `App/Src/mecanum.c` 合成平移和旋转轮速，`motor_debug.c` 执行速度 PI 与逐轮启停，`motor_debug_usb.c` 发送实时 CSV。
- `App/Src/motor_status_usb.c`：只读诊断回复及输出协调；CAN 错误由 ChassisTask 统一采集并锁存，查询不清零。
- `Bsp/Src/fxi6x.c`：保留每帧弱钩子；日志不再覆盖该钩子，改为读取现有控制快照。
- `App/Src/usb_app.c` → `flash_tasks.c`：唯一 USB 接收消费者、换行组帧、命令分发。
- `flash_test.c` / `motor_speed_log.c` → `flash_service.c` → `Bsp/Src/w25q64.c`：异步业务到 Flash 操作。
- `Bsp/Src/usb_serial.c`：发送队列、会话、ZLP 与发送完成凭据；入队成功不等于传输完成。
- `App/Src/chassis_targets.c`：保留的目标发布接口，当前日志不依赖它；不包含底盘运动学。
- `servo_config.h`：舵机开关、ID/位置限位；servo_app.c 管理请求；servo_uart.c 为 UART7 端口；zp_servo.c 编解码；servo_usb.c 处理手动命令。

## 编译与测试

需要 CMake ≥3.22、Ninja、arm-none-eabi-gcc；将工具 bin 目录加入当前终端 PATH 后：

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel 4
```

产物为 `build/Debug/rm_campus.elf`，链接脚本 `STM32H723XG_FLASH.ld`。不要误用历史的
`cmake-build-debug` 主机 MinGW 配置。`openocd_stlink.cfg` 是 H723/ST-Link/SWD 配置；烧录应单独明确执行。

本机已验证的工具位置（仅供定位，其他电脑需要调整）：

| 工具 | 路径 |
|---|---|
| CubeMX 6.15.0 | `E:/Cube MX/STM32CubeMX.exe` |
| Cube H7 固件包 | `C:/Users/34116/STM32Cube/Repository/STM32Cube_FW_H7_V1.12.1` |
| ARM GCC | `E:/DevEnv/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe` |
| CMake | `E:/CLion/CLion 2026.2.2/bin/cmake/win/x64/bin/cmake.exe` |
| Ninja | `E:/CLion/CLion 2026.2.2/bin/ninja/win/x64/ninja.exe` |
| 主机 GCC | `E:/CLion/CLion 2026.2.2/bin/mingw/bin/gcc.exe` |
| 可用 Python | `C:/Users/34116/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe` |

本机 `python`/`py` 未必指向可用解释器；必要时用上表 Python 的绝对路径代替下方 `python`：

```powershell
python tests/run_driver_tests.py
python tests/run_can_recovery_tests.py
python tests/run_usb_serial_tests.py
python tests/run_flash_tests.py
python tests/run_motor_debug_tests.py --cc "E:/CLion/CLion 2026.2.2/bin/mingw/bin/gcc.exe"
python tests/run_servo_tests.py
python tests/run_servo_rc_tests.py
python tests/run_arm_kinematics_tests.py
python tests/run_arm_velocity_tests.py
python tests/run_imu_filter_tests.py
python tests/run_imu_pipeline_tests.py
```

- driver：SBUS、实测通道映射、数值转换、原始电机反馈等。
- can_recovery：真实 CAN BSP 配合模拟硬件寄存器，验证 Stop/Start、旧帧清空、选定电机新反馈、重试间隔及 HAL 失败。
- usb_serial：队列、异步发送、会话、发送完成凭据。
- motor_debug：线性映射、拨杆启停、故障撤流、真实 CAN 帧、ready 完成顺序、200 Hz、断线重连；
  另覆盖 motor_status 未接电机/反馈超时/错误统计、命令分包、重复合并、USB 输出占用和会话竞态。
- flash：真实业务源文件＋模拟 NOR/USB/遥控/电机，覆盖四种开关组合、跨页、断线、当前目标、
  正负目标触发、目标全零结束、首次达标后 5 秒、八列回放、旧格式保护、发送完成后擦除和重启恢复。
- imu：滤波和采集/校准流水线。

USB/Flash 测试脚本支持 `--cc <主机gcc路径>`；driver 脚本目前使用本机 CLion GCC 固定路径，移机需调整。
测试是主机模拟，不等同实板擦写或总线时序验收。对所改模块运行对应测试，不需要为纯文档变化重复构建。

## CubeMX 再生成约束

1. 重新加载磁盘上的最新 `.ioc`，避免旧 GUI 状态覆盖修改。
2. UsbTask 保持 Tasks01 第一项，使 USB 初始化留在 `StartUsbTask()`，不要重复初始化 USB。
3. 保留 USER CODE 中应用入口及 `FlashTasks_Create()`；新 Flash 任务由宏条件创建。
4. 保留顶层 CMake 的自定义 App/Bsp/Algorithm 源码清单；检查生成的构建文件包含 `octospi.c` 和 HAL OSPI。
5. 保留链接脚本 `.dma_rx` 段位于 RAM_D2；UART DMA 缓冲不能挪到默认 DTCM。
6. USART/DMA/CAN 的 RTOS 兼容中断优先级继续使用 5；Flash 仍采用轮询，不擅自改成 DMA/内存映射。
7. FreeRTOS 堆在 `.ioc` 和 `FreeRTOSConfig.h` 中均为 49152 字节。业务 Flash 等待必须让出 CPU。

## 当前待办与接续开发建议

- 已完成用户反馈的四轮空载联调；新增麦轮混合控制需按 SINGLE_MOTOR_DEBUG.md 验证轮向、回传和启停。
- 实地记录阶段重新启用 Flash 后，再验证 `status`、`flash:`、`read` 全链路。
- 确认四电机 ID 对应轮位、实际方向及机械传动比。
- 在实车上验证 CH8 切换、逐轴死区和前进、横移、斜移、旋转方向。
- 上板验证非零四轮目标触发、八列历史目标/反馈、首次同时达标后 5 秒结束与最后一帧完成后擦除。
- 架空确认四轮轮位和正方向，再低速落地验证平移与旋转；日志模块不负责运动控制。
- 舵机移植后需上板核对电平/上拉、逐台设置 ID、标定机械软限位；机械臂动作逻辑和 IMU 加热仍待接入。
- 已知数据完整性边界：日志按用户要求没有 CRC；中途断电恢复不保证，状态说明见协议文件。

## GitHub 与工作区状态

2026-11-12 已选择性合入 feat/single-motor-speed-debug 的 `13dda7dc9b47fe0f398cd592e7947db9debb269b`，
后续已扩展为宏切换单电机/四轮独立 PI、左摇杆 ±400 rpm、200 Hz 纯 CSV 和逐轮故障隔离，
并移植 USTC Bus-Off 恢复。Flash 功能及其关闭配置完整保留。没有导入该分支的备用构建脚本（本机 CMake 正常）。

远端仓库：[rm-campus/mc-02](https://github.com/rm-campus/mc-02)，远端固件目录为
`stm32project/rm_campus/`，不是仓库根目录。

本机电机/Flash 功能此前发布为 main 的 `b21086d`；UART7 驱动及麦轮集成已发布为 `7704872`。
2026-11-24 发布版本包含 CH9 舵机遥控、五列角度回传及默认关闭的机械臂速度 IK。
2026-11-16 已合入远端 `1bf9e46`、`48dd9c2`：CH8 模式切换、左杆平移、CH1 旋转、
ID2/4 方向反转、±60 死区及 9/12 列实时回传。CH8 运行中直接切换，不重新要求回中；
单轮/同速模式运行时仍按 CH2 线性映射，±60 仅用于这两种模式的启用前回中。

原 Windows 工作目录的 `.git` 为无提交的 main，未关联 origin；同步使用
`build/publish-audit/checkout` 中的临时克隆。不要直接在工程根目录 `git add .` 并推送整个目录。
正确同步应基于远端已有历史，按 `stm32project/rm_campus/` 路径放入经过审查的改动，并保留远端其它文件。
历史操作中 GitHub 连接器写入返回过 403，本机 Git 凭据管理器则有可用登录；不要输出凭据或令牌。

2026-11-16 用户已确认将 UART7 舵机驱动及其集成改动一并纳入 main 发布版本，撤销此前暂不发布舵机部分的限制。
本版本保留麦轮控制、舵机 USB 诊断与实时 CSV 的输出协调；电机、舵机、Flash 和 CAN 恢复主机测试通过，
Debug 与 FlashLog 两种 ARM 固件均编译通过。尚未烧录，舵机和新增麦轮控制仍需实板验收。

不要提交 `.codex-backups/`、`build/`、`cmake-build-*/`、IDE 本机配置、测试产物、缓存和 `*.bak`。
不要把未上板验证的功能描述为已经通过硬件验收。
