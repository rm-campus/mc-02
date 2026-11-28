#include "arm_kinematics.h"
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static ArmKinConfig defaults(void)
{
    return (ArmKinConfig){.l1_m=0.175965097f,.l2_m=0.111162186f,.damping=0.005f,
        .max_linear_m_s=0.05f,.joint_limit_rad_s={0.261799388f,0.261799388f,0.261799388f,0.261799388f},
        .linear_tolerance_m_s=0.0001f,.angular_tolerance_rad_s=0.001f};
}

static void geometry_and_derivatives(void)
{
    ArmKinConfig c=defaults();float pose[4],q[4]={0};
    assert(ArmKin_Forward(&c,q,pose));
    assert(fabsf(pose[0]-0.287127283f)<1e-7f && pose[1]==0 && pose[2]==0 && pose[3]==0);
    q[0]=1.5707963268f;assert(ArmKin_Forward(&c,q,pose));
    assert(fabsf(pose[0])<3e-8f && fabsf(pose[1]-0.287127283f)<1e-7f);
    /* 中心差分来自正运动学，独立核对解析雅可比的全部行列。 */
    for(unsigned sample=0;sample<240U;++sample) {
        q[0]=-2.8f+0.023f*(float)sample;
        q[1]=-2.7f+0.031f*(float)(sample%171U);
        q[2]=-1.2f+0.017f*(float)(sample%113U);q[3]=0.003f*(float)sample;
        float j[4][4];assert(ArmKin_Jacobian(&c,q,j));
        for(unsigned col=0;col<4U;++col) {
            float plus[4],minus[4],qp[4],qm[4];memcpy(qp,q,sizeof(qp));memcpy(qm,q,sizeof(qm));
            qp[col]+=0.001f;qm[col]-=0.001f;
            assert(ArmKin_Forward(&c,qp,plus) && ArmKin_Forward(&c,qm,minus));
            for(unsigned row=0;row<4U;++row)
                assert(fabsf((plus[row]-minus[row])/0.002f-j[row][col])<(row<2U ? 0.00003f : 0.0006f));
        }
    }
}

static void normal_tracking(void)
{
    ArmKinConfig c=defaults();float q[4]={0.2f,0.7f,-0.3f,0.4f};
    const float wanted_dq[4]={0.08f,-0.05f,0.04f,0.10f};
    float qp[4],qm[4],pp[4],pm[4],v[4];
    for(unsigned i=0;i<4U;++i){qp[i]=q[i]+wanted_dq[i]*0.005f;qm[i]=q[i]-wanted_dq[i]*0.005f;}
    assert(ArmKin_Forward(&c,qp,pp) && ArmKin_Forward(&c,qm,pm));
    for(unsigned i=0;i<4U;++i)v[i]=(pp[i]-pm[i])/0.01f;
    ArmKinSolution s;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_OK);
    for(unsigned i=0;i<4U;++i)assert(fabsf(s.joint_velocity[i]-wanted_dq[i])<0.0005f);
    float before[4],after[4];assert(ArmKin_Forward(&c,q,before));
    for(unsigned i=0;i<4U;++i)q[i]+=s.joint_velocity[i]*0.001f;
    assert(ArmKin_Forward(&c,q,after));
    for(unsigned i=0;i<4U;++i)assert(fabsf((after[i]-before[i])/0.001f-v[i])<(i<2U?0.00004f:0.00015f));
    assert(s.offending_joint==4U);
}

static void rejection_and_singularities(void)
{
    ArmKinConfig c=defaults();ArmKinSolution s;
    float q[4]={0,0,0,0},v[4]={0.01f,0,0,0};
    assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_UNREACHABLE);
    assert(s.linear_residual_m_s>0.0099f); /* 伸直时径向速度不能伪装成可达。 */
    v[0]=0;v[1]=0.005f;
    assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_OK); /* 同一姿态仍可作小切向速度。 */
    memset(v,0,sizeof(v));assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_OK);
    for(unsigned i=0;i<4U;++i)assert(s.joint_velocity[i]==0.0f);
    q[1]=3.1415926536f;v[0]=0.01f;
    assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_UNREACHABLE); /* 折叠奇异同样受检查。 */

    q[0]=0.2f;q[1]=0.7f;v[0]=v[1]=0.04f;v[2]=v[3]=0;
    assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_LINEAR_LIMIT); /* 逐分量各小于0.05仍会合成超限。 */
    c.max_linear_m_s=1.0f;float j[4][4];assert(ArmKin_Jacobian(&c,q,j));
    for(unsigned i=0;i<4U;++i)v[i]=j[i][0]*2.0f;
    assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_JOINT_SPEED_LIMIT);
    assert(s.offending_joint==0U && s.joint_velocity[0]>1.9f); /* 保留原始解，证明没有暗中限幅。 */
    memset(v,0,sizeof(v));v[3]=2.0f;
    assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_JOINT_SPEED_LIMIT);
    assert(s.offending_joint==3U && s.joint_velocity[3]>1.9f);

    c=defaults();c.damping=2.0f;v[3]=0.1f;
    assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_UNREACHABLE);
    assert(s.joint_velocity[3]<c.joint_limit_rad_s[3] && s.angular_residual_rad_s>0.07f);
}

static void invalid_inputs(void)
{
    ArmKinConfig c=defaults();float q[4]={0,0.7f,0,0},v[4]={0},pose[4],j[4][4];ArmKinSolution s;
    assert(ArmKin_SolveVelocity(NULL,q,v,&s)==ARM_KIN_INVALID);
    assert(ArmKin_SolveVelocity(&c,NULL,v,&s)==ARM_KIN_INVALID);
    assert(ArmKin_SolveVelocity(&c,q,NULL,&s)==ARM_KIN_INVALID);
    assert(ArmKin_SolveVelocity(&c,q,v,NULL)==ARM_KIN_INVALID);
    assert(!ArmKin_Forward(&c,q,NULL) && !ArmKin_Jacobian(&c,q,NULL));
    for(unsigned i=0;i<4U;++i) {
        float original=q[i];q[i]=NAN;
        assert(!ArmKin_Forward(&c,q,pose) && !ArmKin_Jacobian(&c,q,j));
        assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_INVALID);q[i]=original;
        v[i]=INFINITY;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_INVALID);v[i]=0;
        c.joint_limit_rad_s[i]=0;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_INVALID);c=defaults();
    }
    c.l1_m=0;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_INVALID);c=defaults();
    c.l2_m=-1;assert(!ArmKin_Forward(&c,q,pose));c=defaults();
    c.damping=0;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_INVALID);c=defaults();
    c.damping=NAN;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_INVALID);c=defaults();
    c.max_linear_m_s=-1;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_INVALID);c=defaults();
    c.linear_tolerance_m_s=-1;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_INVALID);c=defaults();
    c.angular_tolerance_rad_s=NAN;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_INVALID);c=defaults();
    c.damping=FLT_MAX;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_NUMERIC);c=defaults();
    c.damping=FLT_MIN;assert(ArmKin_SolveVelocity(&c,q,v,&s)==ARM_KIN_NUMERIC);
    assert(s.offending_joint==4U);
    for(unsigned i=0;i<4U;++i)assert(isfinite(s.joint_velocity[i]) && s.joint_velocity[i]==0);
}

int main(void)
{
    geometry_and_derivatives();normal_tracking();rejection_and_singularities();invalid_inputs();
    puts("PASS: arm FK/analytic Jacobian finite differences, DLS tracking, singular residual rejection, strict unscaled speed limits, invalid data and numeric failure.");
    return 0;
}
