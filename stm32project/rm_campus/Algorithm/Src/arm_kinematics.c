#include "arm_kinematics.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

static bool vector_finite(const float values[4])
{
    if(values==NULL)return false;
    for(unsigned i=0;i<4U;++i)if(!isfinite(values[i]))return false;
    return true;
}

static bool geometry_valid(const ArmKinConfig *c, const float q[4])
{
    return c!=NULL && vector_finite(q) && isfinite(c->l1_m) && c->l1_m>0.0f &&
        isfinite(c->l2_m) && c->l2_m>0.0f && isfinite(c->l1_m+c->l2_m) &&
        isfinite(q[0]+q[1]) && isfinite(q[0]+q[1]+q[2]);
}

static bool solver_valid(const ArmKinConfig *c)
{
    if(!isfinite(c->damping) || c->damping<=0.0f ||
        !isfinite(c->max_linear_m_s) || c->max_linear_m_s<=0.0f ||
        !isfinite(c->linear_tolerance_m_s) || c->linear_tolerance_m_s<0.0f ||
        !isfinite(c->angular_tolerance_rad_s) || c->angular_tolerance_rad_s<0.0f)return false;
    for(unsigned i=0;i<4U;++i)
        if(!isfinite(c->joint_limit_rad_s[i]) || c->joint_limit_rad_s[i]<=0.0f)return false;
    return true;
}

bool ArmKin_Forward(const ArmKinConfig *c, const float q[4], float pose[4])
{
    if(pose==NULL || !geometry_valid(c,q))return false;
    float angle12=q[0]+q[1];
    float result[4]={c->l1_m*cosf(q[0])+c->l2_m*cosf(angle12),
        c->l1_m*sinf(q[0])+c->l2_m*sinf(angle12),angle12+q[2],q[3]};
    if(!vector_finite(result))return false;
    memcpy(pose,result,sizeof(result));
    return true;
}

bool ArmKin_Jacobian(const ArmKinConfig *c, const float q[4], float j[4][4])
{
    if(j==NULL || !geometry_valid(c,q))return false;
    float s1=sinf(q[0]), c1=cosf(q[0]);
    float s12=sinf(q[0]+q[1]), c12=cosf(q[0]+q[1]);
    float result[4][4]={
        {-c->l1_m*s1-c->l2_m*s12, -c->l2_m*s12, 0.0f, 0.0f},
        { c->l1_m*c1+c->l2_m*c12,  c->l2_m*c12, 0.0f, 0.0f},
        {1.0f, 1.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };
    for(unsigned row=0;row<4U;++row)if(!vector_finite(result[row]))return false;
    memcpy(j,result,sizeof(result));
    return true;
}

/* 原位 Cholesky 分解及前后代入，避免显式矩阵求逆和动态内存。
 * A=J_s J_s^T+lambda^2 I 理论上正定；若浮点运算破坏正定性则拒绝输出，
 * 不通过偷偷修改负主元或加入未知速度限幅掩盖数值问题。 */
static bool solve_positive(float a[4][4], const float b[4], float x[4])
{
    for(unsigned row=0;row<4U;++row) {
        for(unsigned col=0;col<=row;++col) {
            float value=a[row][col];
            for(unsigned k=0;k<col;++k)value-=a[row][k]*a[col][k];
            if(!isfinite(value))return false;
            if(row==col) {
                if(value<=0.0f)return false;
                a[row][col]=sqrtf(value);
            } else a[row][col]=value/a[col][col];
            if(!isfinite(a[row][col]))return false;
        }
    }
    float y[4];
    for(unsigned row=0;row<4U;++row) {
        float value=b[row];
        for(unsigned k=0;k<row;++k)value-=a[row][k]*y[k];
        y[row]=value/a[row][row];
        if(!isfinite(y[row]))return false;
    }
    for(unsigned n=4U;n>0U;--n) {
        unsigned row=n-1U;
        float value=y[row];
        for(unsigned k=row+1U;k<4U;++k)value-=a[k][row]*x[k];
        x[row]=value/a[row][row];
        if(!isfinite(x[row]))return false;
    }
    return true;
}

ArmKinResult ArmKin_SolveVelocity(const ArmKinConfig *c, const float q[4],
                                 const float velocity[4], ArmKinSolution *out)
{
    if(out==NULL)return ARM_KIN_INVALID;
    memset(out,0,sizeof(*out));out->offending_joint=4U;
    if(!geometry_valid(c,q) || !solver_valid(c) || !vector_finite(velocity))return ARM_KIN_INVALID;
    if(hypotf(velocity[0],velocity[1])>c->max_linear_m_s)return ARM_KIN_LINEAR_LIMIT;

    float j[4][4], normalized[4][4], a[4][4], b[4], x[4]={0};
    if(!ArmKin_Jacobian(c,q,j))return ARM_KIN_NUMERIC;
    /* 将米尺度除以 L1+L2，再与角速度行共同求解，防止阻尼含义随 mm/m 改变。 */
    float length=c->l1_m+c->l2_m, lambda2=c->damping*c->damping;
    if(!isfinite(lambda2) || lambda2<=0.0f)return ARM_KIN_NUMERIC;
    for(unsigned row=0;row<4U;++row) {
        float scale=row<2U ? 1.0f/length : 1.0f;
        b[row]=velocity[row]*scale;
        for(unsigned col=0;col<4U;++col)normalized[row][col]=j[row][col]*scale;
        if(!isfinite(b[row]) || !vector_finite(normalized[row]))return ARM_KIN_NUMERIC;
    }
    for(unsigned row=0;row<4U;++row)for(unsigned col=0;col<4U;++col) {
        float sum=row==col ? lambda2 : 0.0f;
        for(unsigned k=0;k<4U;++k)sum+=normalized[row][k]*normalized[col][k];
        a[row][col]=sum;
    }
    if(!solve_positive(a,b,x))return ARM_KIN_NUMERIC;
    ArmKinSolution result={0};result.offending_joint=4U;
    for(unsigned col=0;col<4U;++col)
        for(unsigned row=0;row<4U;++row)result.joint_velocity[col]+=normalized[row][col]*x[row];
    if(!vector_finite(result.joint_velocity))return ARM_KIN_NUMERIC;
    for(unsigned row=0;row<4U;++row)
        for(unsigned col=0;col<4U;++col)result.achieved_velocity[row]+=j[row][col]*result.joint_velocity[col];
    if(!vector_finite(result.achieved_velocity))return ARM_KIN_NUMERIC;
    result.linear_residual_m_s=hypotf(result.achieved_velocity[0]-velocity[0],result.achieved_velocity[1]-velocity[1]);
    result.angular_residual_rad_s=hypotf(result.achieved_velocity[2]-velocity[2],result.achieved_velocity[3]-velocity[3]);
    if(!isfinite(result.linear_residual_m_s) || !isfinite(result.angular_residual_rad_s))return ARM_KIN_NUMERIC;
    *out=result;

    /* 先检查原始解；这里没有关节速度裁剪或整组缩放。失败结果仅供诊断。 */
    for(unsigned i=0;i<4U;++i)if(fabsf(result.joint_velocity[i])>c->joint_limit_rad_s[i]) {
        out->offending_joint=i;return ARM_KIN_JOINT_SPEED_LIMIT;
    }
    if(result.linear_residual_m_s>c->linear_tolerance_m_s ||
        result.angular_residual_rad_s>c->angular_tolerance_rad_s)return ARM_KIN_UNREACHABLE;
    return ARM_KIN_OK;
}
