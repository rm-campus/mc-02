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

这不是完整麦轮控制程序：尚未加入 PID、轮序/方向标定和运动学。
通道归一化沿用参考值 -784～783；需在实际遥控器上校准端点和开关方向。

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
