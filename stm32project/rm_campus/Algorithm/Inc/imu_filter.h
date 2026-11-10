/** @file imu_filter.h
 * @brief 单实例六轴姿态 EKF；只由 ImuTask 串行调用。
 * 输入为同一机体坐标系下 rad/s、m/s^2、秒；不施加小角速度死区。
 * 静止三轴零偏由上层先扣除；这里输出 EKF 的额外 X/Y 残余零偏估计。
 */
#ifndef IMU_FILTER_H
#define IMU_FILTER_H
#include <stdbool.h>

typedef struct
{
    float quaternion[4];  /* [w,x,y,z]，机体系到导航系的单位四元数。 */
    float euler_deg[3];   /* 标准 ZYX 约定的 roll,pitch,yaw，单位度。 */
    float yaw_total_deg;
    float gyro_bias[3];   /* rad/s；此六状态 EKF 不估计 Z 轴残余零偏。 */
} ImuFilterData;

/* 初始姿态由上层静止重力均值给出；会归一化。重复 Init 复用静态内存。 */
bool ImuFilter_Init(const float initial_quaternion[4]);

/* 有效 dt 范围 (0,0.05] 秒。非法输入/零加速度返回 false，且不修改输出。
 * 内部数值或矩阵异常会停止本实例，需重新 Init；调用方应发布失效状态。
 * 不提供位置估计，也不保证无外部航向参考时的长期 Yaw 稳定。 */
bool ImuFilter_Update(const float gyro_rad_s[3], const float accel_m_s2[3],
                      float dt_s, ImuFilterData *out);
#endif
