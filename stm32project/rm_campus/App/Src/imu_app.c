#include "imu_app.h"
#include "imu_clock.h"
#include "imu_filter.h"
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static const float sensor_to_body[3][3] = IMU_SENSOR_TO_BODY_MATRIX;
static ImuData working = {.quaternion = {1.0f, 0.0f, 0.0f, 0.0f}};
static ImuData published = {.quaternion = {1.0f, 0.0f, 0.0f, 0.0f}};
static bool calibration_requested;
static float cal_mean[6], cal_m2[6], static_bias[3];

static void publish(void)
{
    taskENTER_CRITICAL();
    published = working;
    taskEXIT_CRITICAL();
}

void Imu_CopyData(ImuData *out)
{
    if (out != NULL)
    {
        taskENTER_CRITICAL();
        *out = published;
        taskEXIT_CRITICAL();
    }
}

void Imu_RequestCalibration(void)
{
    taskENTER_CRITICAL();
    calibration_requested = true;
    taskEXIT_CRITICAL();
}

static bool take_calibration_request(void)
{
    taskENTER_CRITICAL();
    bool requested = calibration_requested;
    calibration_requested = false;
    taskEXIT_CRITICAL();
    return requested;
}

static void clear_calibration_window(void)
{
    working.cal_samples = 0U;
    memset(cal_mean, 0, sizeof(cal_mean));
    memset(cal_m2, 0, sizeof(cal_m2));
}

void Imu_BeginCalibration(void)
{
    clear_calibration_window();
    memset(static_bias, 0, sizeof(static_bias));
    memset(working.gyro_bias, 0, sizeof(working.gyro_bias));
    memset(working.euler_deg, 0, sizeof(working.euler_deg));
    memset(working.quaternion, 0, sizeof(working.quaternion));
    working.quaternion[0] = 1.0f;
    working.yaw_total_deg = 0.0f;
    working.state = IMU_CALIBRATING;
    working.error = IMU_ERROR_NONE;
    publish();
}

void Imu_ReportFault(uint32_t error, bool read_error)
{
    working.state = IMU_ERROR;
    working.error = error;
    if (read_error) ++working.read_errors;
    /* 保留最近成功的测量，sample_ms 不刷新，电脑可据此识别陈旧数据。 */
    publish();
}

static float norm3(const float v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

static void rotate(const float input[3], float output[3])
{
    for (unsigned i = 0; i < 3U; ++i)
    {
        output[i] = 0.0f;
        for (unsigned j = 0; j < 3U; ++j) output[i] += sensor_to_body[i][j] * input[j];
    }
}

static bool calibration_step(void)
{
    float accel_norm = norm3(working.accel);
    /* 此检测只能拒绝明显运动，不能证明静止：上电校准期间仍须把板子放稳。 */
    if (fabsf(accel_norm - 9.80665f) > 1.0f || norm3(working.raw_gyro) > 0.15f)
    {
        clear_calibration_window();
        working.error = IMU_ERROR_MOVING;
        return true;
    }
    ++working.cal_samples;
    for (unsigned i = 0; i < 6U; ++i)
    {
        float value = i < 3U ? working.accel[i] : working.raw_gyro[i - 3U];
        float delta = value - cal_mean[i];
        cal_mean[i] += delta / (float)working.cal_samples;
        cal_m2[i] += delta * (value - cal_mean[i]);
    }
    working.error = IMU_ERROR_NONE;
    if (working.cal_samples < IMU_CALIBRATION_SAMPLES) return true;
    for (unsigned i = 0; i < 6U; ++i)
    {
        float variance = cal_m2[i] / (float)(IMU_CALIBRATION_SAMPLES - 1U);
        float limit = i < 3U ? 0.15f * 0.15f : 0.01f * 0.01f;
        if (!isfinite(variance) || variance > limit)
        {
            clear_calibration_window();
            working.error = IMU_ERROR_CALIBRATION_NOISE;
            return true;
        }
    }
    float roll = atan2f(cal_mean[1], cal_mean[2]);
    float pitch = atan2f(-cal_mean[0], sqrtf(cal_mean[1] * cal_mean[1] + cal_mean[2] * cal_mean[2]));
    float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
    float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
    float q[4] = {cr * cp, sr * cp, cr * sp, -sr * sp};
    if (!ImuFilter_Init(q)) return false;
    for (unsigned i = 0; i < 3U; ++i)
    {
        static_bias[i] = cal_mean[i + 3U];
        working.gyro_bias[i] = static_bias[i];
        working.gyro[i] = working.raw_gyro[i] - static_bias[i];
    }
    memcpy(working.quaternion, q, sizeof(q));
    working.euler_deg[0] = roll * 57.295779513f;
    working.euler_deg[1] = pitch * 57.295779513f;
    working.euler_deg[2] = 0.0f;
    working.state = IMU_RUNNING;
    return true;
}

bool Imu_ProcessSample(const Bmi088Sample *sample, float dt_s, uint32_t sample_ms)
{
    if (sample == NULL || !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.02f)
    {
        Imu_ReportFault(IMU_ERROR_TIMING, false);
        return false;
    }
    for (unsigned i = 0; i < 3U; ++i)
    {
        if (!isfinite(sample->accel[i]) || !isfinite(sample->gyro[i]))
        {
            Imu_ReportFault(IMU_ERROR_SAMPLE, false);
            return false;
        }
    }
    float a_norm = norm3(sample->accel);
    if (!isfinite(sample->temperature_c) || !isfinite(a_norm) || a_norm < 0.01f)
    {
        Imu_ReportFault(IMU_ERROR_SAMPLE, false);
        return false;
    }
    rotate(sample->accel, working.accel);
    rotate(sample->gyro, working.raw_gyro);
    working.temperature_c = sample->temperature_c;
    working.dt_s = dt_s;
    working.sample_ms = sample_ms;
    ++working.sample_seq;
    for (unsigned i = 0; i < 3U; ++i) working.gyro[i] = working.raw_gyro[i] - static_bias[i];
    if (working.state == IMU_CALIBRATING)
    {
        if (!calibration_step())
        {
            Imu_ReportFault(IMU_ERROR_FILTER, false);
            return false;
        }
    }
    else if (working.state == IMU_RUNNING)
    {
        ImuFilterData filtered;
        if (!ImuFilter_Update(working.gyro, working.accel, dt_s, &filtered))
        {
            Imu_ReportFault(IMU_ERROR_FILTER, false);
            return false;
        }
        memcpy(working.quaternion, filtered.quaternion, sizeof(working.quaternion));
        memcpy(working.euler_deg, filtered.euler_deg, sizeof(working.euler_deg));
        working.yaw_total_deg = filtered.yaw_total_deg;
        for (unsigned i = 0; i < 3U; ++i)
        {
            working.gyro_bias[i] = static_bias[i] + filtered.gyro_bias[i];
            working.gyro[i] = working.raw_gyro[i] - working.gyro_bias[i];
        }
        working.error = IMU_ERROR_NONE;
    }
    publish();
    return true;
}

void Campus_ImuTask(void *argument)
{
    (void)argument;
    while (!ImuClock_Init())
    {
        Imu_ReportFault(IMU_ERROR_CLOCK, false);
        osDelay(500U);
    }
    for (;;)
    {
        working.state = IMU_INITIALIZING;
        publish();
        Bmi088Result result = Bmi088_Init();
        if (result != BMI088_OK)
        {
            Imu_ReportFault(IMU_ERROR_INIT_BASE + result, false);
            osDelay(500U);
            continue;
        }
        (void)take_calibration_request();
        Imu_BeginCalibration();
        TickType_t wake = xTaskGetTickCount();
        uint32_t previous_cycles = ImuClock_Cycles();
        uint32_t previous_ms = HAL_GetTick();
        for (;;)
        {
            if (take_calibration_request()) Imu_BeginCalibration();
            Bmi088Sample sample;
            result = Bmi088_Read(&sample);
            uint32_t cycles = ImuClock_Cycles();
            uint32_t now = HAL_GetTick();
            float dt = ImuClock_Elapsed(previous_cycles, cycles);
            previous_cycles = cycles;
            if (result != BMI088_OK)
            {
                Imu_ReportFault(IMU_ERROR_READ_BASE + result, true);
                break;
            }
            if ((uint32_t)(now - previous_ms) > 20U)
            {
                Imu_ReportFault(IMU_ERROR_TIMING, false);
                break;
            }
            previous_ms = now;
            if (!Imu_ProcessSample(&sample, dt, now)) break;
            /* 单轮处理超期时重新对齐，保证高优先级任务仍会阻塞，
             * 避免绝对周期追赶循环连续占用 CPU、饿死 USB/监测任务。 */
            TickType_t now_tick = xTaskGetTickCount();
            if ((TickType_t)(now_tick - wake) >= pdMS_TO_TICKS(1U)) wake = now_tick;
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(1U));
        }
        /* 故障后让出 CPU，重建传感器和静止校准状态；不把旧姿态继续标为有效。 */
        osDelay(100U);
    }
}
