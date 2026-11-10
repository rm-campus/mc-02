/** BMI088 / MC02 采集接口。单一 ImuTask 调用，SPI2 无 DMA，不配置加热。 */
#ifndef CAMPUS_BMI088_H
#define CAMPUS_BMI088_H
#include <stdint.h>

typedef enum
{
    BMI088_OK = 0,
    BMI088_IO_ERROR = 1,
    BMI088_ACCEL_ID_ERROR = 2,
    BMI088_GYRO_ID_ERROR = 3,
    BMI088_CONFIG_ERROR = 4,
    BMI088_ARGUMENT_ERROR = 5
} Bmi088Result;

typedef struct
{
    float accel[3];    /* 传感器坐标系，m/s^2，未做六面标定。 */
    float gyro[3];     /* 传感器坐标系，rad/s，尚未减零偏。 */
    float temperature_c;
} Bmi088Sample;

/* 只能在调度器启动后的任务调用；初始化等待使用 osDelay，失败由上层重试。 */
Bmi088Result Bmi088_Init(void);
/* 完整成功才更新 out，避免部分读取失败时混用新旧轴数据。 */
Bmi088Result Bmi088_Read(Bmi088Sample *out);
#endif
