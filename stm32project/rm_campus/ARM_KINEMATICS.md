# 机械臂速度逆运动学预备模块

更新：2026-11-28。本模块已接入构建和现有任务，但默认关闭，不读取遥杆/拨杆，不自动使能或回零。
此版本已通过主机测试和 ARM 编译。机械臂实板验证及 TCP 定义待完成。

## 控制对象和数学约定

机械结构为三个平行俯仰关节、一个腕部滚转和独立爪开合，不使用底座yaw舵机。
输入的线速度作用在 q3/q4 两轴交点 O_wrist，不是爪尖/TCP。
基座+x和+z位于主体运动平面，+z向上；数学角度正向统一按平面角度定义。
q1/q2零位对应两主连杆沿+x伸直，q3确定腕轴俯仰，q4为腕滚转；数学零位不是P1500或CAD当前姿态。

内部统一使用 m、rad、s：

```text
L12 = 0.175965097 m
L23 = 0.111162186 m
x = L12*cos(q1) + L23*cos(q1+q2)
z = L12*sin(q1) + L23*sin(q1+q2)
pitch = q1+q2+q3
roll = q4
```

72 mm只保留为腕交点到主舵盘中心的已知轴向偏置，不并入上述交点位置，也不假设TCP。
当前接口不接受任意六维末端速度；未来定义TCP后须增加工具变换与可达性检查。

Algorithm/arm_kinematics 是独立纯C数学核心：正运动学解析求导得到4×4雅可比，线速度行按
L12+L23归一化，使用阻尼最小二乘和Cholesky解方程，不显式求逆、无动态内存。
它不调用位置逆解再作差分，不裁剪关节速度，不偷偷降低请求速度后返回成功。

## 宏与当前默认值

逆运动学参数集中在 App/Inc/arm_config.h。舵机输出轴限速由 App/Inc/servo_config.h 定义。

| 配置 | 默认值 | 用途 |
|---|---:|---|
| ARM_VELOCITY_IK_ENABLE | 0 | 应用层控制和错误回传总开关；数学核心仍可独立调用/测试 |
| ARM_VELOCITY_IK_OUTPUT_ENABLE | 0 | 是否允许实际调用舵机整组输出接口 |
| ARM_JOINT_CALIBRATION_VALID | 0 | 是否已经完成下述五轴标定 |
| SERVO_MAX_SPEED_DEG_S | 30 | 各舵机输出轴共用的指令限速，°/s |
| ARM_MAX_LINEAR_SPEED_MM_S | 50 | 腕交点平移合速度上限，mm/s |
| ARM_PERIOD_MS | 20 | 速度计算和位置目标更新周期 |
| ARM_COMMAND_TIMEOUT_MS | 100 | 整组速度命令最大年龄，达到此值即停止 |
| ARM_FEEDBACK_TIMEOUT_MS | 200 | 每轴位置反馈最大年龄，沿用现有舵机阈值 |
| ARM_MAX_FEEDBACK_SKEW_MS | 100 | 五轴反馈年龄最大差 |
| ARM_MAX_TRACKING_ERROR_DEG | 5 | 位置目标最多领先实测机械角度的量 |
| ARM_DLS_DAMPING | 0.005 | 归一化雅可比的无量纲阻尼 |
| ARM_LINEAR_RESIDUAL_MM_S | 0.5 | 平移速度残差二范数容差 |
| ARM_ANGULAR_RESIDUAL_DEG_S | 0.5 | 俯仰/滚转速度残差二范数容差 |
| ARM_ERROR_PERIOD_MS | 200 | 故障重复报告周期，5 Hz |

30°/s 和 50 mm/s 分别是舵机输出轴与腕交点的软件限速。当前几何下，关节传动比均为 1:1，
允许俯仰变化时，跨姿态的平移速度理论上界为 (L12+2×L23)×(30π/180)×1000≈208.54 mm/s。
保持俯仰不变时，上界为 (L12+L23)×(30π/180)×1000≈150.34 mm/s。
理论上界仅在特定姿态和方向可达。腕交点的平移合速度仍独立限制为 50 mm/s，每周期同时检查关节限速、任务残差和限位。
CH9 直接遥控和逆运动学共用同一舵机输出轴限速。
没有持续运行时间限制。只要上层不断提供有效速度命令且反馈正常，可以持续运行。

## 关节映射与标定

按用户指定的从肩到爪扭矩递减顺序，逻辑关节映射为：

| 逻辑变量 | 机械作用 | 舵机 | 总线ID |
|---|---|---|---:|
| q1 | 肩俯仰 | ZP45 | 3 |
| q2 | 肘俯仰 | ZP35 | 2 |
| q3 | 腕俯仰 | ZP30 | 1 |
| q4 | 腕滚转 | ZP25S | 4（同型号两台可交换配置） |
| g | 爪开合 | ZP25S | 5（同型号两台可交换配置） |

不改总线ID，不改变现有CH9对ID1～5的直接控制映射。
ARM_JOINT_CALIBRATION_TABLE每行依次为：ID、P增加的机械正向(+1/-1)、舵机轴角/机械关节角传动比、
参考P、参考P对应的数学角、机械最小角、机械最大角、标定时PMOD模式；角字段均为rad。

```text
q = q_ref + direction*(P-P_ref)*rad_per_P/ratio
P = P_ref + direction*(q-q_ref)*ratio/rad_per_P
```

PMOD1/2使用270°量程，3/4使用180°量程，位置回读的模式必须与标定模式一致。
修改方向模式或量程后不能直接重用旧标定；本总线仍要求唯一主机，不允许其它设备私下更改模式。
默认参考P=0、PMOD=0、机械上下限相同，明确表示未标定。仅将VALID置1仍会拒绝使能。
测试脚本注入的P1500及角度范围是合成测试数据，不可复制为实机标定。

## 接入上层程序

接口在 App/Inc/arm_velocity.h，所有速度一次提交，避免只刷新夹爪时延续旧的非零平移命令。

```c
ArmVelocityCommand zero = {0};
Arm_SetVelocity(&zero);  /* 记录当前HAL时间，命令不是无限有效。 */
Arm_Enable();            /* 提交使能请求；需要读取状态确认ARM_RUNNING。 */

/* 以下仅演示API；由未来上层任务定期调用，不绑定任何遥控通道。 */
ArmVelocityCommand command = {
    .vx_m_s = 0.01f,
    .vz_m_s = 0.0f,
    .pitch_rad_s = 0.0f,
    .roll_rad_s = 0.0f,
    .gripper_rad_s = 0.0f
};
Arm_SetVelocity(&command);
ArmStatus status;
Arm_GetStatus(&status);
```

夹爪速度的幅值是舵机轴rad/s，正负按标定机械方向解释，经传动比转换后积分；它不是夹口线速度。
SetVelocity/Enable/ClearFault的true只表示请求已提交，不表示已执行或条件已满足。
速度超限复位锁存期间，Arm_Enable和Arm_ClearFault立即返回false；Arm_ResetRequired可只读查询是否必须复位。
上电保持ARM_IDLE。使能要求正确配置、有效标定、五轴在线且模式匹配、新鲜零速度及总线旧事务排空。
目标从回读位置初始化，不自动转到软件零位。正常运行保持独立的连续目标，不每20ms从同一旧反馈重算目标。

实际输出须同时设ENABLE=1、OUTPUT_ENABLE=1，并设置SERVO_RC_ENABLE=0，编译期拒绝两种物理控制源并行。
ENABLE=1、OUTPUT_ENABLE=0为影子计算：仍需要有效标定和反馈，但不占用舵机控制权、不发送运动或停止。
默认ENABLE=0时不改变现有CH9、底盘、USB和Flash业务行为。

## 严格拒绝与故障恢复

每周期先计算未限幅关节速度，再与每轴允许速度比较。传动比已经纳入舵机轴速度判断。
任一轴超速时不积分、不提交新位置、不降速继续运行。平移合速度超过配置同样拒绝。
即使关节速度未超限，如果雅可比实现的速度残差超过容差，也以VELOCITY_UNACHIEVABLE停止。
连续角度和量化P目标均检查机械限位。量化P目标还须通过设备配置与当前总线模式的有效位置指令限位，
见 [SERVO_INTEGRATION.md](SERVO_INTEGRATION.md)。四/五轴目标全量验证后在一个短临界区整组入队，失败无部分入队。
实际UART仍逐台发送，整组入队不代表所有舵机在同一物理时刻执行。

超限、过期、跟踪偏差、数值异常或任一受控轴故障进入ARM_FAULT，撤销全组待发动作并尽力发送停止，保留扭矩。
驱动发现发送失败、反馈离线或手动halt时立即撤销ARM整组，避免等待下一次20ms计算。
总线上的已发字节不能撤销，故障断线时无法保证物理停止；软件上限限制的是指令，非直接测速闭环。
P/T运动时间还根据量化目标与最近反馈设下限，实际瞬时速度仍需实测确认。

FAULT锁存第一原因，更新成合法速度不会自动恢复。恢复规则按错误类型区分：

- LINEAR_SPEED_LIMIT和JOINT_SPEED_LIMIT（包括夹爪舵机轴超速）只能通过真正的MCU复位清除。
  零速度、合法新速度、Arm_ClearFault、Arm_Enable、Arm_Stop和重复Arm_Init/Servo_Init均不能清除或替换原错误。
  锁存标志独立保存在普通.bss中，没有软件清除接口；上电或MCU复位后的C启动过程才会清零。
  实际IK输出配置下，驱动也拒绝新的Move/Resume/TorqueOn，避免初始化重置owner后绕过保护；
  查询、停止、暂停和释放扭矩仍可用。影子计算模式只锁住IK计算，不封锁真实手动舵机。
  复位后回到原启动流程，仍需标定有效、反馈正常、新鲜零速度和显式Enable，不自动运动。
- 其他错误（包括VELOCITY_UNACHIEVABLE）继续使用原软件恢复流程：解除故障并等待停止事务完成，
  提交新鲜零速度，调用Arm_ClearFault，确认ARM_IDLE，再显式Arm_Enable。

Arm_Stop可主动停止，但不清除已锁存故障。停止未完成时再次使能返回CONTROL_BUSY故障，不提前启动。
ARM接管时拒绝手动Move/Resume/恢复扭矩；手动查询和Stop/Pause/释力保留，其中halt使整组进入保护。

## USB错误输出

FAULT期间在USB可用且其它事务空闲时每200ms持续发送一行ASCII文本，例如：

```text
ARM_ERROR code=JOINT_SPEED_LIMIT joint=3 servo_id=1 requested=16.000 limit=15.000 unit=deg_s linear_residual_mm_s=0.000 angular_residual_deg_s=0.003
```

joint=1～5表示肩到爪的逻辑顺序，servo_id是实际总线ID；joint=0表示整组错误。
requested是发生错误时的请求/检查值，不是测得的真实舵机转速。
状态码包括BAD_CONFIG、UNCALIBRATED、NOT_READY、FEEDBACK_STALE、FEEDBACK_SKEW、BAD_MODE、COMMAND_TIMEOUT、
INVALID_COMMAND、TIMING、LINEAR_SPEED_LIMIT、JOINT_SPEED_LIMIT、VELOCITY_UNACHIEVABLE、JOINT_LIMIT、TRACKING、
DRIVER、CONTROL_BUSY、NUMERIC。超速单位deg_s，平移超限mm_s，角度限制deg，超时ms，模式不匹配的request/limit为PMOD编号。

故障期间暂停电机/舵机常规自动CSV，手动motor_status/servo_status按完整事务优先发送。
Flash回放占用USB时错误报告等待，不夹入日志；USB忙不补发积压，断线不清除硬件控制故障，重连恢复报告。
不增加串口运动命令，不修改正常CSV列数；可恢复错误的清错/使能由未来应用层API调用，两类速度超限始终等待MCU复位。

## 任务、验证与限制

复用ServoBusTask的1ms服务循环，控制计算每20ms一次，USB错误格式化放在UsbTask；不新增任务、不修改ioc。
tests/run_arm_kinematics_tests.py验证有限差分雅可比、FK小步速度、奇异位形、严格超速拒绝及无效数值。
tests/run_arm_velocity_tests.py使用真实调度、UART端口与USB协议，验证映射、原子入队、锁存、清错、超时、
模式/限位/跟踪异常、5Hz回传与原协议互斥，并覆盖关闭、影子、未标定和编译互斥配置。
速度超限测试另验证清错/使能拒绝、重复模块与驱动初始化无法绕过、USB重连保持同一错误，以及新进程模拟MCU复位后仍需显式使能。

反馈仍约每台20Hz，50Hz只是计算与目标更新频率；位置快照是异步测量，程序检查年龄与最大时差。
本版不含碰撞检测、避障规划、重力补偿、底盘协同或实物速度标定，也不修改舵机内部PID。
后续启用硬件前，应补齐真实标定和工具定义；现阶段可以直接调用纯数学接口进行仿真验证。
