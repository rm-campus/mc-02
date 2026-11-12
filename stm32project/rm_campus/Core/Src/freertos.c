/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "campus_app.h"
#include "usb_app.h"
#include "imu_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for UsbTask */
osThreadId_t UsbTaskHandle;
const osThreadAttr_t UsbTask_attributes = {
  .name = "UsbTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ImuTask */
osThreadId_t ImuTaskHandle;
const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh2,
};
/* Definitions for RcTask */
osThreadId_t RcTaskHandle;
const osThreadAttr_t RcTask_attributes = {
  .name = "RcTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ServoBusTask */
osThreadId_t ServoBusTaskHandle;
const osThreadAttr_t ServoBusTask_attributes = {
  .name = "ServoBusTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for MonitorTask */
osThreadId_t MonitorTaskHandle;
const osThreadAttr_t MonitorTask_attributes = {
  .name = "MonitorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for StatusTask */
osThreadId_t StatusTaskHandle;
const osThreadAttr_t StatusTask_attributes = {
  .name = "StatusTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartUsbTask(void *argument);
void StartImuTask(void *argument);
void StartRcTask(void *argument);
void StartChassisTask(void *argument);
void StartServoBusTask(void *argument);
void StartMonitorTask(void *argument);
void StartStatusTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;

  /* Stop scheduling on stack overflow; set a debugger breakpoint here. */
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
  /* Stop scheduling on allocation failure; set a debugger breakpoint here. */
  taskDISABLE_INTERRUPTS();
  for (;;)
  {
  }
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UsbTask */
  UsbTaskHandle = osThreadNew(StartUsbTask, NULL, &UsbTask_attributes);

  /* creation of ImuTask */
  ImuTaskHandle = osThreadNew(StartImuTask, NULL, &ImuTask_attributes);

  /* creation of RcTask */
  RcTaskHandle = osThreadNew(StartRcTask, NULL, &RcTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(StartChassisTask, NULL, &ChassisTask_attributes);

  /* creation of ServoBusTask */
  ServoBusTaskHandle = osThreadNew(StartServoBusTask, NULL, &ServoBusTask_attributes);

  /* creation of MonitorTask */
  MonitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &MonitorTask_attributes);

  /* creation of StatusTask */
  StatusTaskHandle = osThreadNew(StartStatusTask, NULL, &StatusTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartUsbTask */
/**
  * @brief USB 初始化、收发队列服务和应用层接收入口。
  * @note UsbTask 在 .ioc 任务列表中排第一，使 CubeMX 把 USB 初始化放在此任务。
  */
/* USER CODE END Header_StartUsbTask */
void StartUsbTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartUsbTask */
  Campus_UsbTask(argument);
  /* USER CODE END StartUsbTask */
}

/* USER CODE BEGIN Header_StartImuTask */
/**
  * @brief BMI088 采样、静止校准和姿态解算任务；本版本不接入恒温加热。
  * @note 周期及重试逻辑由应用层管理，优先级高于底盘控制任务。
  */
/* USER CODE END Header_StartImuTask */
void StartImuTask(void *argument)
{
  /* USER CODE BEGIN StartImuTask */
  Campus_ImuTask(argument);
  /* USER CODE END StartImuTask */
}

/* USER CODE BEGIN Header_StartRcTask */
/**
  * @brief 遥控接收任务：UART5 循环 DMA、SBUS 解包及串口错误恢复。
  * @note 接收事件唤醒，10 ms 超时处理。
  * @param argument 未使用。
  */
/* USER CODE END Header_StartRcTask */
void StartRcTask(void *argument)
{
  /* USER CODE BEGIN StartRcTask */
  /* 进入应用层常驻循环：接收事件唤醒后组帧、解析，处理串口异常。该函数内部不会正常返回。 */
  Campus_RcTask(argument);
  /* USER CODE END StartRcTask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
  * @brief 底盘联调任务：默认零电流，可配置单电机速度 PI 台架测试。
  * @note 1 ms 周期；麦轮运动学尚未接入。
  * @param argument 未使用。
  */
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  /* 进入应用层 1 ms 循环；调试默认关闭，配置和遥控确认后仅运行一个电机。 */
Campus_ChassisTask(argument);
  /* USER CODE END StartChassisTask */
}

/* USER CODE BEGIN Header_StartServoBusTask */
/**
  * @brief UART 舵机任务占位：待确认五个舵机的协议后接入。
  * @note 当前只休眠，不发送舵机指令。
  * @param argument 未使用。
  */
/* USER CODE END Header_StartServoBusTask */
void StartServoBusTask(void *argument)
{
  /* USER CODE BEGIN StartServoBusTask */
(void)argument;
  /* 舵机型号和协议未确定，本次保留任务入口，不向总线发送指令。 */
  for (;;)
  {
    osDelay(20U);
  }
  /* USER CODE END StartServoBusTask */
}

/* USER CODE BEGIN Header_StartMonitorTask */
/**
  * @brief 系统监测任务：遥控、电机反馈超时及 CAN Bus-Off 状态。
  * @note 10 ms 固定周期。
  * @param argument 未使用。
  */
/* USER CODE END Header_StartMonitorTask */
void StartMonitorTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTask */
  /* 进入应用层 10 ms 循环：对遥控时间戳、电机反馈时间戳和 CAN 状态做监测。该函数内部不会正常返回。 */
Campus_MonitorTask(argument);
  /* USER CODE END StartMonitorTask */
}

/* USER CODE BEGIN Header_StartStatusTask */
/**
  * @brief LED 状态指示任务：显示遥控、电机与 CAN 状态。
  * @note 50 ms 周期；本次不接入蜂鸣器。
  * @param argument 未使用。
  */
/* USER CODE END Header_StartStatusTask */
void StartStatusTask(void *argument)
{
  /* USER CODE BEGIN StartStatusTask */
  /* 进入应用层 50 ms 循环：只更新 LED，不使用蜂鸣器。该函数内部不会正常返回。 */
Campus_StatusTask(argument);
  /* USER CODE END StartStatusTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

