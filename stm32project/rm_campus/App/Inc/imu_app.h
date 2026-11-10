/** IMU 应用接口。坐标默认与板载 BMI088 的 XYZ 一致，角速度/零偏 rad/s，角度 deg。 */
#ifndef CAMPUS_IMU_APP_H
#define CAMPUS_IMU_APP_H
#include <stdbool.h>
#include <stdint.h>
#include "bmi088.h"

#define IMU_CALIBRATION_SAMPLES 1000U
/* 将传感器 XYZ 转为期望机体系；修改时必须保持正交右手旋转矩阵。
 * 未获知机器人安装朝向，默认保留传感器轴，不擅自假定板子哪一端朝前。 */
#ifndef IMU_SENSOR_TO_BODY_MATRIX
#define IMU_SENSOR_TO_BODY_MATRIX {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
#endif

typedef enum
{
    IMU_INITIALIZING = 0,
    IMU_CALIBRATING = 1,
    IMU_RUNNING = 2,
    IMU_ERROR = 3
} ImuState;

enum
{
    IMU_ERROR_NONE = 0,
    IMU_ERROR_INIT_BASE = 0x100, /* 加 Bmi088Result */
    IMU_ERROR_READ_BASE = 0x200, /* 加 Bmi088Result */
    IMU_ERROR_MOVING = 0x400,
    IMU_ERROR_CALIBRATION_NOISE = 0x401,
    IMU_ERROR_TIMING = 0x500,
    IMU_ERROR_FILTER = 0x600,
    IMU_ERROR_CLOCK = 0x700,
    IMU_ERROR_SAMPLE = 0x800
};

typedef struct
{
    uint32_t sample_seq, sample_ms; /* 完整成功读取的次数和时刻，不能当作芯片内部时间戳。 */
    uint32_t state, error, cal_samples, read_errors;
    float dt_s;
    float accel[3], raw_gyro[3], gyro[3];
    float temperature_c;
    float quaternion[4]; /* w,x,y,z */
    float euler_deg[3];  /* roll,pitch,yaw */
    float yaw_total_deg;
    float gyro_bias[3];  /* 静止校准 + EKF 在线估计的合计零偏。 */
} ImuData;

/* 应用任务使用。只有 state==IMU_RUNNING 且 sample_ms 新鲜时才应用于控制。 */
void Imu_CopyData(ImuData *out);
void Imu_RequestCalibration(void);
void Campus_ImuTask(void *argument);

/* 以下是单生产者处理入口，仅 ImuTask 或主机单元测试调用。 */
void Imu_BeginCalibration(void);
bool Imu_ProcessSample(const Bmi088Sample *sample, float dt_s, uint32_t sample_ms);
void Imu_ReportFault(uint32_t error, bool read_error);
#endif
