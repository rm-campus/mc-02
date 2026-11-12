# BMI088 / EKF 应用层接口

## 当前行为

本工程使用达妙 MC02 官方例程的 BMI088 C 驱动方案，加上 EGA C_VERSION 的四元数 EKF。
代码分为 `Bsp/Src/bmi088.c`（采集）、`Algorithm/`（滤波）和
`App/Src/imu_app.c`（任务与校准）。IMU 持续运行，通过应用接口读取，默认不发送串口数据。
当前不启用 TIM3/PB1 或任何 IMU 加热输出。

- ImuTask 优先级 High2 / 42，栈 1024 words / 4096 字节，使用 1 ms 绝对周期等待。
- 若本轮处理已跨过周期，会重新对齐并休眠，让低优先级 USB/监测任务仍能运行；
  实际采样率可在调试器中通过内部 `sample_seq/sample_ms` 观察，不能仅凭任务配置断言达到 1 kHz。
- 采样后通过 DWT CYCCNT 测量实际 `dt`，额外用 HAL tick 检查超过 20 ms 的采样间隔。
- 加速度 ±3g、800 Hz；陀螺 ±2000°/s、1000 Hz、116 Hz 带宽。1 ms 轮询使用最新寄存器值，
  不代表两颗传感器严格同步或每次轮询都有全新的加速度；`sample_seq` 是软件接受样本计数。
- SPI 采用有界阻塞整包传输，不使用 DMA/EXTI；通信失败不发布混合的新旧轴数据。
- 启动先采集 1000 个静止样本，约一秒（不含初始化、运动导致的重试），估计三轴陀螺零偏，
  由重力均值初始化 Roll/Pitch，初始 Yaw 设为零。明显运动或方差过大会清空校准窗口重试。
- 校准期间必须把板子放稳；仅凭 IMU 不能可靠区分微小恒定转动与陀螺零偏。
- 陀螺先减静止偏置，再送入 EKF 估计额外 X/Y 残余偏置。没有小角速度置零死区。
- 失败后将状态置为无效，等待后重试传感器初始化和校准。近零加速度（例如自由落体）也会
  进入该失效恢复流程；本版不承诺跳跃/自由落体期间连续有效的姿态。

恒温控制的作用是把传感器稳定在选定温度，减小温度变化导致的偏置漂移。此次按要求不接入。
因此热机期间和环境温度变化时，零偏仍可能变化；可以调用 `Imu_RequestCalibration()` 重新
进行静止校准。六轴 EKF 不提供绝对航向参考，也不提供位置估计。

## CubeMX 与构建

`.ioc` 已同步：

| 项目 | 当前值 |
|---|---|
| SPI2 | 8-bit Full Duplex Master，Mode 3，软件 NSS，16 分频 / 3 MHz |
| PC0 / PC3_C | ACC_CS / GYRO_CS，推挽初始高电平，高速 |
| PE10 / PE12 | ACC_INT / GYRO_INT，普通无上下拉输入，不开启 EXTI |
| ImuTask | High2 / 42，Dynamic，1024 words |
| UsbTask | BelowNormal / 16，Dynamic，1024 words，仍在任务列表第一项 |
| 加热 | 不配置 TIM3/PB1 |

UsbTask 必须保持在列表首位，让 CubeMX 把 USB 初始化留在 `StartUsbTask()` 中。
自定义任务调用在 USER CODE 区域。重新生成后保留顶层 CMake 自定义源文件和 UART `.dma_rx`
链接段；CubeMX 若正在打开旧 `.ioc`，应重新加载磁盘上的最新文件再生成，避免旧界面覆盖改动。

EKF 编译使用现有 CMSIS-DSP V1.6.0 的六个浮点矩阵源文件及 `ARM_MATH_MATRIX_CHECK`。
Kalman 模块限制为单实例 6 状态、3 量测，使用 2048 字节静态 arena，实际占用 1604 字节，
不占用 RTOS/libc 堆，重复初始化复用空间。加入 Flash 任务后 RTOS 堆为 48 KiB，实板可检查各任务栈高水位。
IMU 任务不进行字符串格式化或 USB 输出；需要通信时由业务层独立调用 USB 收发接口。

## 应用接口

```c
#include "imu_app.h"

ImuData imu;
Imu_CopyData(&imu);  // 短临界区一致性快照，只在任务中调用
if (imu.state == IMU_RUNNING && (uint32_t)(HAL_GetTick() - imu.sample_ms) < 20U)
{
    // imu.gyro[0..2]：校准及EKF残余偏置修正后的角速度，rad/s
    // imu.euler_deg[0..2]：roll、pitch、yaw，deg
    // 接入控制前，按实际安装位置确认坐标轴和正方向。
}

// 将板子放稳后，可以从业务任务发起重新校准：
// Imu_RequestCalibration();
```

默认坐标系保持板载 BMI088 的传感器 XYZ，`IMU_SENSOR_TO_BODY_MATRIX` 初始为单位阵。
未知机器人装板方向，因此没有假定板上哪一端指向车头。需要转换时在 `imu_app.h` 中配置
正交右手旋转矩阵，校准偏置和滤波输入会统一使用转换后的坐标。
四元数是 `[w,x,y,z]`，采用机体系到导航系的旋转；欧拉角使用 ZYX 约定，数组顺序为 Roll/Pitch/Yaw。

## 应用接口与状态

- `Imu_CopyData(&data)`：获取一致性快照，包括加速度、原始及校正后角速度、温度、
  四元数、标准 ZYX 欧拉角、累计 Yaw、合计零偏、采样时间和运行状态。
- `Imu_RequestCalibration()`：向 ImuTask 请求重新静止校准。调用后应保持板子静止。
- `Campus_ImuTask()`：独立任务主体，由 CubeMX 生成的 `StartImuTask()` 调用。

`Imu_CopyData()` 保留原始测量及诊断状态，校准过程中也可能读到测量值；业务必须先判断
`state == IMU_RUNNING`，并检查 `sample_ms` 新鲜度。初始化四元数或旧快照不代表有效姿态。

| 状态 | 含义 |
|---|---|
| 0 / IMU_INITIALIZING | 正在初始化传感器 |
| 1 / IMU_CALIBRATING | 正在积累静止校准样本 |
| 2 / IMU_RUNNING | 校准完成，正常解算 |
| 3 / IMU_ERROR | 发生错误，等待重试 |

| 内部错误码 | 含义 |
|---|---|
| 0x100 + 1～5 | 初始化失败：1 SPI、2 加速度ID、3 陀螺ID、4 寄存器读回、5 参数 |
| 0x200 + 1～5 | 采集失败，子码同上 |
| 0x400 / 0x401 | 明显运动 / 校准窗口噪声过大，保持静止后自动重新积累 |
| 0x500 | dt 或采样间隔异常 |
| 0x600 | EKF 数值/矩阵异常 |
| 0x700 | DWT 计数器不可用 |
| 0x800 | 非有限测量或近零加速度 |

## USB 应用接入

USB CDC 收发基础功能保留在 `Bsp/Inc/usb_serial.h`，业务可以调用
`UsbSerial_Send()`、`UsbSerial_Printf()`、`UsbSerial_IsReady()` 等接口。
接收入口是 `Campus_UsbOnReceive()`，当前默认为空；`UsbTask` 负责队列服务。
线路参数默认 115200 / 8N1，原生 USB 的线路波特率不限制实际传输速度。

用于联调的 IMU 定时串口输出、CSV 编码、电脑接收脚本和回传示例已经移除。
IMU 解算与 USB 服务互相独立；连接串口后没有自动输出是预期行为。

## 内部回归检查

```powershell
python tests/run_imu_filter_tests.py
python tests/run_imu_pipeline_tests.py
python tests/run_usb_serial_tests.py
```

检查需要 Python 3 和主机 GCC，可用 `--cc <gcc路径>` 指定编译器，无需 pyserial 或板卡。
保留的用例覆盖真实 EKF/CMSIS 运算、SPI 协议、校准恢复、USB 队列和应用层默认不自动发送。
它们不会连接串口，也不构成固件测试输出功能，不能代替实板验证。

## 外部源码来源

- BMI088 寄存器序列和读数流程：用户提供的 `dm-mc02-master.zip`，
  `例程/CtrBoard-H7_ALL/Bsp/BMI088driver.c` 及其头文件；对应达妙官方
  [DM-MC02](https://github.com/dmBots/DM-MC02) 例程。此移植重新封装整包 SPI、错误返回及任务延时，
  关闭传感器中断输出；采样量程/ODR 与选定官方档位一致。
- EKF / Kalman：EGA 的
  [固定版本源码](https://github.com/JAHNAN00/EGAdapter_MC02/tree/890201ddc17f62cead0f302b9a7fef78714a76e0/User/Module/imu/bmi088)，
  commit `890201ddc17f62cead0f302b9a7fef78714a76e0`，保留原作者 Wang Hongxi 文件头。
  本次检查未见仓库级许可证，未自行为其添加 MIT 等许可声明。
- 主要算法移植调整：静态内存、错误传播、重复初始化恢复、规范平方根、四元数归一化、
  反三角函数范围保护、统一标准 ZYX 输出；不带 EGA 外层的小角速度死区。
- CMSIS-DSP：沿用工程自带 ARM/ST 分发版本与原文件许可声明，只编译所需六个矩阵源文件。
