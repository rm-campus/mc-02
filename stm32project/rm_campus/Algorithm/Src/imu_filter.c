#include "imu_filter.h"
#include "QuaternionEKFc.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAD_TO_DEG 57.29577951308232f
static bool s_ready;

static float clamp_unit(float value)
{
    return fminf(1.0f, fmaxf(-1.0f, value));
}

static void to_euler(const float q[4], float angles[3])
{
    angles[0] = atan2f(2.0f * (q[0]*q[1] + q[2]*q[3]),
                       1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2])) * RAD_TO_DEG;
    angles[1] = asinf(clamp_unit(2.0f * (q[0]*q[2] - q[3]*q[1]))) * RAD_TO_DEG;
    angles[2] = atan2f(2.0f * (q[0]*q[3] + q[1]*q[2]),
                       1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3])) * RAD_TO_DEG;
}

bool ImuFilter_Init(const float initial_quaternion[4])
{
    s_ready = false;
    if (initial_quaternion == NULL) return false;
    float norm_squared = 0.0f;
    for (unsigned i = 0; i < 4U; ++i)
    {
        if (!isfinite(initial_quaternion[i])) return false;
        norm_squared += initial_quaternion[i] * initial_quaternion[i];
    }
    if (!isfinite(norm_squared) || norm_squared < 1e-12f) return false;
    /* 保留 EGA C_VERSION 懒初始化路径的参数；lambda=1 不做记忆渐消。 */
    if (!IMU_QuaternionEKF_Init(10.0f, 0.001f, 10000000.0f, 1.0f, 0.0f)) return false;
    float norm = sqrtf(norm_squared);
    for (unsigned i = 0; i < 4U; ++i)
    {
        float value = initial_quaternion[i] / norm;
        QEKF_INS.q[i] = value;
        QEKF_INS.IMU_QuaternionEKF.xhat_data[i] = value;
        QEKF_INS.IMU_QuaternionEKF.FilteredValue[i] = value;
    }
    float euler[3];
    to_euler(QEKF_INS.q, euler);
    QEKF_INS.YawAngleLast = euler[2];
    QEKF_INS.YawTotalAngle = euler[2];
    s_ready = true;
    return true;
}

bool ImuFilter_Update(const float gyro_rad_s[3], const float accel_m_s2[3],
                      float dt_s, ImuFilterData *out)
{
    if (!s_ready || gyro_rad_s == NULL || accel_m_s2 == NULL || out == NULL ||
        !isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.05f) return false;
    float accel_norm_squared = 0.0f, gyro_norm_squared = 0.0f;
    for (unsigned i = 0; i < 3U; ++i)
    {
        if (!isfinite(gyro_rad_s[i]) || !isfinite(accel_m_s2[i])) return false;
        accel_norm_squared += accel_m_s2[i] * accel_m_s2[i];
        gyro_norm_squared += gyro_rad_s[i] * gyro_rad_s[i];
    }
    if (!isfinite(accel_norm_squared) || !isfinite(gyro_norm_squared) ||
        accel_norm_squared < 1e-8f) return false;

    if (!IMU_QuaternionEKF_Update(gyro_rad_s[0], gyro_rad_s[1], gyro_rad_s[2],
                                 accel_m_s2[0], accel_m_s2[1], accel_m_s2[2], dt_s))
    {
        s_ready = false;
        return false;
    }
    /* 只在完整更新有效时发布；内部出错后的部分状态不能继续驱动控制。 */
    for (unsigned i = 0; i < 36U; ++i)
    {
        if (!isfinite(QEKF_INS.IMU_QuaternionEKF.P_data[i]))
        {
            s_ready = false;
            return false;
        }
    }
    ImuFilterData result;
    memcpy(result.quaternion, QEKF_INS.q, sizeof(result.quaternion));
    memcpy(result.gyro_bias, QEKF_INS.GyroBias, sizeof(result.gyro_bias));
    to_euler(result.quaternion, result.euler_deg);
    result.yaw_total_deg = QEKF_INS.YawTotalAngle;
    for (unsigned i = 0; i < 3U; ++i)
    {
        if (!isfinite(result.euler_deg[i]) || !isfinite(result.gyro_bias[i]))
        {
            s_ready = false;
            return false;
        }
    }
    if (!isfinite(result.yaw_total_deg))
    {
        s_ready = false;
        return false;
    }
    *out = result;
    return true;
}
