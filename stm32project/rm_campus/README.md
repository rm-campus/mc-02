# rm_campus：达妙 MC02 校赛电控工程

开发交接快照：**2026-11-12**。本目录是 STM32 固件工程根目录。

工程面向四轮麦克纳姆底盘与后续机械臂。主控为达妙 MC02（STM32H723VGT6），使用 HAL、
FreeRTOS/CMSIS-RTOS2、CMake/Ninja 和 GNU Arm 工具链。

**当前默认四轮联调，宏可切换单电机模式。ID1～4 独立 PI、独立故障停机，统一使用左摇杆竖直 ±450 rpm 目标。
最左拨杆先 +1 确认关闭，再 -1 启用，各电机独立检查回中 300 ms 与基本静止；健康电机不等待缺失电机。**
四轮模式 50 Hz 纯 CSV 五通道：公共原目标 rpm、ID1/2/3/4 实测 rpm，不发送 ready。
单电机模式保留 ready，随后 50 Hz 纯 CSV 四通道：目标 rpm、实测 rpm、指令 A、反馈 A。
两种模式均无运行时限，共用 ±15 A、110℃ 与 100 ms 电机反馈超时。两个 Flash 任务开关均为 0。
当前电机总线已切换到板载 CAN1（FDCAN1，PD0/RX、PD1/TX）；电调接线需同步移到 CAN1。
上电后可发送 `motor_status` 加 LF 查询选定电机反馈、CAN 状态与控制故障，不需要先进入运行或收到 ready。
按用户要求，当前 `MOTOR_DEBUG_CAN_FAULT_ENABLE=0`：临时禁用 CAN 状态异常/发送入队失败触发的软件停机。
CAN 诊断和 LED 指示保留；反馈达到 100 ms 未更新时，仅对应电机触发 MOTOR_OFFLINE。恢复 CAN_FAULT 保护将宏置 1 后重新编译烧录。
已移植 USTC-RoboWalker 的 Bus-Off Stop/Start 恢复方法，由现有 ChassisTask 执行；无需整板复位即可尝试恢复。
硬件恢复后，活动电机任意一台新反馈可恢复总线发送；逐轮检查恢复反馈掩码，未收到新反馈的轮保持零电流，不阻塞其他轮。
若已触发反馈离线等故障停机，恢复通信不自动重新使能；HAL 恢复失败则停止重试，诊断显示 FAILED。
已实现四轮独立速度闭环，尚未实现底盘运动学和轮位/方向标定；LED 绿色只表示通信在线。

## 新对话从这里开始

请新接手的开发者或助手先阅读本文件，以及：

- [SINGLE_MOTOR_DEBUG.md](SINGLE_MOTOR_DEBUG.md)：两种调试模式宏、独立 PI、启停和 CSV 通道说明。

1. [COMMUNICATION_PROTOCOL.md](COMMUNICATION_PROTOCOL.md)：当前 CSV、手动诊断及保留的 Flash 协议。
2. [FLASH_TASKS.md](FLASH_TASKS.md)：Flash 分区、两个功能开关、任务、日志生命周期和目标转速接口。
3. [DRIVER_INTEGRATION.md](DRIVER_INTEGRATION.md)：遥控、CAN、电机、LED 和 USB 基础接口。
4. [IMU_INTEGRATION.md](IMU_INTEGRATION.md)：BMI088、四元数 EKF、校准、坐标及限制。

继续开发前应重新核对实际源码和工作区改动；本文不是已经完成上板验收的声明。
当前最重要的待接接口是 `Chassis_SetTargetWheelRpm()`：由未来底盘模块逐帧发布真实四轮车轮 rpm。
不要用占位零值或首次遥控帧的目标替代当前目标。单电机模块当前不发布四轮目标；烧录和推送需另行指示。

## 硬件与外设

| 模块 | 当前连接与配置 |
|---|---|
| MCU | STM32H723VGT6，当前 SYSCLK 480 MHz，HCLK 240 MHz |
| 存储 | MCU 内部 Flash 1 MiB；内部 RAM 560 KiB＋4 KiB 备份 SRAM；板载 W25Q64JV 外部 Flash 8 MiB |
| 四电机 | M3508＋C620，FDCAN1，PD0/RX、PD1/TX，经典 CAN 1 Mbps，ID 1～4 |
| 遥控 | FS-i6X＋iA6B，UART5/PD2，100000 baud、8E2（HAL 9 位字长含偶校验），循环 DMA；板载电路已反相 |
| BMI088 | SPI2 Mode 3 / 3 MHz；PC1 MOSI、PC2_C MISO、PB13 SCK；PC0 ACC_CS、PC3_C GYRO_CS；PE10/PE12 普通输入 |
| RGB LED | SPI6 MOSI/PA7，6 MHz，WS2812 编码 |
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
| 调试模式 | 默认 FOUR，单电机 SINGLE 可选 ID1～4；四轮各有 Kp/Ki，初始均为 0.05/0；无运行时限 |
| USB CDC | 已实现有界收发队列、会话处理和实际发送完成凭据 |
| BMI088 / 姿态 | 已接入采集、静止校准和 C 版四元数 EKF；未接入恒温、绝对航向或位置估计 |
| 外部 Flash | `.ioc`、初始化、HAL OSPI 和 W25Q64 驱动均已接入正式工程 |
| Flash 测试任务 | 功能保留，当前关闭；USB 测试帧写 Flash→实际读回→USB 完成后擦除 |
| 电机日志 | 功能保留，当前关闭；50 Hz 原始转速记录、read 后回放与自动清理 |
| 底盘目标接口 | 已提供线程安全发布/快照接口；当前底盘尚未调用、目标默认无效 |
| 底盘闭环 | 已有四轮独立速度 PI；运动学、轮序与方向标定尚未完成 |
| UART 舵机/机械臂 | ServoBusTask 仅占位，协议与控制尚未接入 |

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
- 当前方向掩码为 0，电调 ID 与前后左右轮位尚未实测标定，不能假设 ID 1 就是某个轮位。

以下是保留的 Flash 日志模式，当前关闭，实时单电机模式不执行这些记录规则。
日志保存四个小端 int16 原始 rpm，每条 8 字节，离线电机写 0；不增加反馈年龄、有效掩码或 CRC32。
回放必须重新从 Flash 读取，再换算车轮 rpm，以 `speed:a,b,c,d\n` 发送，不能用原采样 RAM 代替 Flash。
VOFA 的 Δt 设置为 20 ms。

开始：左水平或左竖直任一轴从精确 0 变非 0。结束：四轮均在线并首次同时达到各自当前有效目标后 5 秒，
或左摇杆双轴提前回零。目标必须来自后续底盘模块的当前帧，首次达标后不因目标波动重置计时。
回零立即停止采样，不包括后续制动过程；日志任务不发送制动指令。

当前目标接口未接入时，达标计时不会触发，但仍可用摇杆开始、回零结束验证日志。
发布接口见 [chassis_targets.h](App/Inc/chassis_targets.h)，目标单位为车轮 rpm，100 ms 内需刷新。

## FreeRTOS 任务与开关

当前 tick 为 1 kHz，动态堆为 **48 KiB**。下表栈大小按 32 位 word 表示；CMSIS 属性中的 stack_size 为字节。

| 任务 | 优先级 | 栈 / words | 主要工作 |
|---|---|---:|---|
| ImuTask | High2 / 42 | 1024 | 约 1 ms 周期采集与 EKF |
| RcTask | High1 / 41 | 512 | 遥控 DMA 事件处理、解析与边沿通知 |
| ChassisTask | High / 40 | 1024 | 1 ms 单电机/四轮独立 PI、逐轮启停与 CAN 恢复 |
| ServoBusTask | AboveNormal / 32 | 512 | 舵机占位 |
| MonitorTask | Normal / 24 | 256 | 10 ms 在线状态汇总 |
| MotorSpeedLogTask | Normal / 24 | 512 | 当前不创建；启用后 20 ms 采样或回放 |
| UsbTask | BelowNormal / 16 | 1024 | USB 服务、50 Hz CSV 及 motor_status 手动诊断 |
| FlashServiceTask | BelowNormal / 16 | 512 | 当前不创建；唯一实际操作 W25Q64 的任务 |
| StatusTask | Low / 8 | 256 | LED 状态 |
| FlashTestTask | Low / 8 | 512 | 当前不创建；USB→Flash→USB 测试事务 |

新三个 Flash 任务在 `freertos.c` 的 USER CODE 中调用 `FlashTasks_Create()` 条件创建，
并非 `.ioc` 原有 Tasks01 中的无条件任务。两个业务开关在 [flash_tasks_config.h](App/Inc/flash_tasks_config.h)，
默认均为 0；重新开启 Flash 功能前先将 MOTOR_DEBUG_TELEMETRY 置 0，避免两种输出混用。
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
- `App/Src/motor_debug.c`：PI、斜坡、逐轮启停；`motor_debug_usb.c`：单电机四通道/四轮五通道 CSV。
- `App/Src/motor_status_usb.c`：只读诊断回复及输出协调；CAN 错误由 ChassisTask 统一采集并锁存，查询不清零。
- `Bsp/Src/fxi6x.c`：每帧解析后调用弱钩子 `Fxi6x_OnFrame()`；日志模块启用时提供强实现。
- `App/Src/usb_app.c` → `flash_tasks.c`：唯一 USB 接收消费者、换行组帧、命令分发。
- `flash_test.c` / `motor_speed_log.c` → `flash_service.c` → `Bsp/Src/w25q64.c`：异步业务到 Flash 操作。
- `Bsp/Src/usb_serial.c`：发送队列、会话、ZLP 与发送完成凭据；入队成功不等于传输完成。
- `App/Src/chassis_targets.c`：目标发布与新鲜度判断；不包含遥控到轮速的换算。

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
python tests/run_imu_filter_tests.py
python tests/run_imu_pipeline_tests.py
```

- driver：SBUS、实测通道映射、数值转换、原始电机反馈等。
- can_recovery：真实 CAN BSP 配合模拟硬件寄存器，验证 Stop/Start、旧帧清空、选定电机新反馈、重试间隔及 HAL 失败。
- usb_serial：队列、异步发送、会话、发送完成凭据。
- motor_debug：线性映射、拨杆启停、故障撤流、真实 CAN 帧、ready 完成顺序、50 Hz、断线重连；
  另覆盖 motor_status 未接电机/反馈超时/错误统计、命令分包、重复合并、USB 输出占用和会话竞态。
- flash：真实业务源文件＋模拟 NOR/USB/遥控/电机，覆盖四种开关组合、跨页、断线、当前目标、
  松杆、首次达标后 5 秒、50 Hz 回放、发送完成后擦除、重启恢复和异常边界。
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

- 先按 SINGLE_MOTOR_DEBUG.md 架空验证单电机、回传和保护；初始 PI 未实机整定。
- 实地记录阶段重新启用 Flash 后，再验证 `status`、`flash:`、`read` 全链路。
- 确认四电机 ID 对应轮位、实际方向及机械传动比。
- 在已有 ChassisTask 中实现遥控到四轮目标的运动学、限幅和控制逻辑，再发布当前目标快照。
- 在完整控制条件下验证四轮首次同时达标后 5 秒结束；不要额外加入未约定的持续稳定时间。
- 分别整定四路 PI 并确认轮位/方向，再接底盘运动学；日志模块不负责运动控制。
- 机械臂舵机协议和 IMU 加热均待后续明确需求。
- 已知数据完整性边界：日志按用户要求没有 CRC；中途断电恢复不保证，状态说明见协议文件。

## GitHub 与工作区状态

2026-11-12 已选择性合入 feat/single-motor-speed-debug 的 `13dda7dc9b47fe0f398cd592e7947db9debb269b`，
后续已扩展为宏切换单电机/四轮独立 PI、左摇杆 ±450 rpm、50 Hz 纯 CSV 和逐轮故障隔离，
并移植 USTC Bus-Off 恢复。Flash 功能及其关闭配置完整保留。没有导入该分支的备用构建脚本（本机 CMake 正常）。

远端仓库：[rm-campus/mc-02](https://github.com/rm-campus/mc-02)，远端固件目录为
`stm32project/rm_campus/`，不是仓库根目录。

本次同步基于 main 的遥控适配提交 `542d653`，提交说明为
“加入了单个电机和四个电机同步的调试功能，完善了CAN通信的bus_off恢复”。
完整源码、Flash 支持、诊断、测试及文档一起同步；实际提交编号以仓库 main 历史为准。

原 Windows 工作目录的 `.git` 为无提交的 main，未关联 origin；同步使用
`build/publish-audit/checkout` 中的临时克隆。不要直接在工程根目录 `git add .` 并推送整个目录。
正确同步应基于远端已有历史，按 `stm32project/rm_campus/` 路径放入经过审查的改动，并保留远端其它文件。
历史操作中 GitHub 连接器写入返回过 403，本机 Git 凭据管理器则有可用登录；不要输出凭据或令牌。

不要提交 `.codex-backups/`、`build/`、`cmake-build-*/`、IDE 本机配置、测试产物、缓存和 `*.bak`。
不要把未上板验证的功能描述为已经通过硬件验收。
