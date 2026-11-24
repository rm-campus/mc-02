#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <stdbool.h>

/* 数学核心不依赖 HAL、RTOS 或舵机 ID。
 * 任务顺序固定为 [腕交点 x, 腕交点 z, 末端俯仰, 腕部滚转]。
 * 长度使用 m，角度使用 rad，速度使用 m/s 和 rad/s。
 * q1/q2/q3 是同一数学正向的相对角；q=0 表示主臂伸直且腕轴朝前，
 * 不是 P1500，也不是未经转换的 CAD 参考姿态。TCP 尚未定义。 */
typedef struct {
    float l1_m, l2_m;
    float damping;                  /* 无量纲阻尼，必须大于 0。 */
    float max_linear_m_s;           /* x/z 合速度上限，不是逐分量限速。 */
    float joint_limit_rad_s[4];     /* 数学关节限速；调用层应考虑舵机传动比。 */
    float linear_tolerance_m_s;     /* x/z 速度残差的二范数容差。 */
    float angular_tolerance_rad_s;  /* 俯仰/滚转速度残差的二范数容差。 */
} ArmKinConfig;

typedef enum {
    ARM_KIN_OK,
    ARM_KIN_INVALID,
    ARM_KIN_LINEAR_LIMIT,
    ARM_KIN_JOINT_SPEED_LIMIT,
    ARM_KIN_UNREACHABLE,
    ARM_KIN_NUMERIC
} ArmKinResult;

typedef struct {
    /* 只有返回 ARM_KIN_OK 时才允许执行。
     * 超速/不可达时仍保留有限的未限幅解，供串口报告使用，绝不能当有效命令。 */
    float joint_velocity[4];
    float achieved_velocity[4];
    float linear_residual_m_s;
    float angular_residual_rad_s;
    unsigned offending_joint;      /* 0..3 表示超限关节；4 表示没有单一关节。 */
} ArmKinSolution;

/* 正运动学/雅可比只检查几何和 q；不要求求解器限速参数已经配置。 */
bool ArmKin_Forward(const ArmKinConfig *config, const float q[4], float pose[4]);
bool ArmKin_Jacobian(const ArmKinConfig *config, const float q[4], float jacobian[4][4]);

/* 解析雅可比 + 归一化阻尼最小二乘。先检查未限幅关节速度，再检查任务残差。
 * 不裁剪关节速度，不整体缩放任务，不静默降低请求速度后返回成功。 */
ArmKinResult ArmKin_SolveVelocity(const ArmKinConfig *config, const float q[4],
                                 const float task_velocity[4], ArmKinSolution *solution);

#endif
