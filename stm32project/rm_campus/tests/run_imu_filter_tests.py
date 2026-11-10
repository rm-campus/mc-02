"""Compile the real IMU EKF and bundled CMSIS matrix C sources on the host.

Only the CMSIS declaration header is replaced (no Cortex register/intrinsic code).
Both scalar and ARM_MATH_DSP float-matrix paths are exercised without a board.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

ARM_MATH = r'''
#ifndef HOST_ARM_MATH_H
#define HOST_ARM_MATH_H
#include <stdint.h>
#include <math.h>
#include <string.h>
typedef float float32_t;
typedef enum { ARM_MATH_SUCCESS=0, ARM_MATH_ARGUMENT_ERROR=-1,
  ARM_MATH_LENGTH_ERROR=-2, ARM_MATH_SIZE_MISMATCH=-3,
  ARM_MATH_NANINF=-4, ARM_MATH_SINGULAR=-5, ARM_MATH_TEST_FAILURE=-6 } arm_status;
typedef struct { uint16_t numRows, numCols; float32_t *pData; } arm_matrix_instance_f32;
void arm_mat_init_f32(arm_matrix_instance_f32 *, uint16_t, uint16_t, float32_t *);
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *, const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *, arm_matrix_instance_f32 *);
#endif
'''

TEST = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "imu_filter.h"
#include "QuaternionEKFc.h"

#define PI 3.14159265358979323846f
#define D2R (PI / 180.0f)
#define R2D (180.0f / PI)
static const float identity[4] = {1,0,0,0};
static const float gravity[3] = {0,0,9.80665f};
static const float zero[3] = {0,0,0};
static ImuFilterData out;

static void assert_finite_unit(const ImuFilterData *data)
{
    float norm = 0;
    for (unsigned i=0; i<4; ++i) { assert(isfinite(data->quaternion[i])); norm += data->quaternion[i]*data->quaternion[i]; }
    assert(fabsf(norm - 1.0f) < 1e-5f);
    for (unsigned i=0; i<3; ++i) { assert(isfinite(data->euler_deg[i])); assert(isfinite(data->gyro_bias[i])); }
    assert(isfinite(data->yaw_total_deg));
}

static void quaternion_from_euler(float roll, float pitch, float yaw, float q[4])
{
    float cr=cosf(roll*D2R/2), sr=sinf(roll*D2R/2);
    float cp=cosf(pitch*D2R/2), sp=sinf(pitch*D2R/2);
    float cy=cosf(yaw*D2R/2), sy=sinf(yaw*D2R/2);
    q[0]=cr*cp*cy+sr*sp*sy; q[1]=sr*cp*cy-cr*sp*sy;
    q[2]=cr*sp*cy+sr*cp*sy; q[3]=cr*cp*sy-sr*sp*cy;
}

static void test_tilt(float roll, float pitch, float yaw)
{
    float q[4]; quaternion_from_euler(roll,pitch,yaw,q);
    float a[3] = {2*(q[1]*q[3]-q[0]*q[2])*9.80665f,
                  2*(q[0]*q[1]+q[2]*q[3])*9.80665f,
                  (q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3])*9.80665f};
    assert(ImuFilter_Init(q));
    for (unsigned i=0;i<500;++i) assert(ImuFilter_Update(zero,a,0.001f,&out));
    assert_finite_unit(&out);
    assert(fabsf(out.euler_deg[0]-roll)<0.1f);
    assert(fabsf(out.euler_deg[1]-pitch)<0.1f);
    assert(fabsf(out.euler_deg[2]-yaw)<0.1f);
}

int main(void)
{
    float bad_q[4] = {0};
    assert(!ImuFilter_Init(NULL));
    assert(!ImuFilter_Init(bad_q));
    bad_q[0]=NAN; assert(!ImuFilter_Init(bad_q));
    assert(ImuFilter_Init(identity));
    size_t arena_used=Kalman_Filter_ArenaUsed();
    assert(arena_used > 0 && arena_used <= KALMAN_FILTER_ARENA_BYTES);
    for (unsigned i=0;i<3000;++i) assert(ImuFilter_Update(zero,gravity,0.001f,&out));
    assert_finite_unit(&out);
    for (unsigned i=0;i<3;++i) assert(fabsf(out.euler_deg[i]) < 0.01f);

    /* A short translational-acceleration spike is rejected after convergence. */
    float disturbed[3]={20,0,9.80665f};
    for (unsigned i=0;i<20;++i) assert(ImuFilter_Update(zero,disturbed,0.001f,&out));
    assert(fabsf(out.euler_deg[0])<0.1f && fabsf(out.euler_deg[1])<0.1f);

    /* Real dt integration and unwrapped yaw across +/-180 degrees. */
    assert(ImuFilter_Init(identity));
    const float yaw_rate[3]={0,0,0.5f};
    double elapsed=0;
    for(unsigned i=0;i<10000;++i) {
        float dt=(i&1)?0.0014f:0.0006f;
        elapsed+=dt;
        assert(ImuFilter_Update(yaw_rate,gravity,dt,&out));
    }
    assert_finite_unit(&out);
    assert(fabs(out.yaw_total_deg-elapsed*0.5*(180.0/3.141592653589793))<0.1);

    /* Small true rotations must not be erased by an artificial gyro deadband. */
    assert(ImuFilter_Init(identity));
    const float slow[3]={0,0,0.005f};
    for(unsigned i=0;i<1000;++i) assert(ImuFilter_Update(slow,gravity,0.001f,&out));
    assert(fabsf(out.yaw_total_deg-0.005f*R2D)<0.01f);

    test_tilt(35,-20,0);
    test_tilt(-25,40,45);
    test_tilt(0,89,0);
    test_tilt(0,-89,0);

    ImuFilterData before=out;
    float invalid[3]={NAN,0,9.8f};
    assert(!ImuFilter_Update(zero,zero,0.001f,&out));
    assert(!ImuFilter_Update(zero,invalid,0.001f,&out));
    invalid[0]=INFINITY;
    assert(!ImuFilter_Update(invalid,gravity,0.001f,&out));
    assert(!ImuFilter_Update(zero,gravity,0,&out));
    assert(!ImuFilter_Update(zero,gravity,-0.001f,&out));
    assert(!ImuFilter_Update(zero,gravity,0.1f,&out));
    assert(!ImuFilter_Update(zero,gravity,NAN,&out));
    assert(!ImuFilter_Update(NULL,gravity,0.001f,&out));
    assert(!ImuFilter_Update(zero,gravity,0.001f,NULL));
    assert(memcmp(&out,&before,sizeof(out))==0);

    /* Reinitialization must reuse memory and restore pristine covariance. */
    for(unsigned reset=0;reset<1000;++reset) {
        assert(ImuFilter_Init(identity));
        assert(Kalman_Filter_ArenaUsed()==arena_used);
        assert(QEKF_INS.IMU_QuaternionEKF.P_data[0]==100000.0f);
        assert(ImuFilter_Update(zero,gravity,0.001f,&out));
        assert_finite_unit(&out);
    }
    before=out;
    /* Singular innovation covariance propagates failure; no garbage published. */
    QEKF_INS.Q1=QEKF_INS.Q2=QEKF_INS.R=0.0f;
    memset(QEKF_INS.IMU_QuaternionEKF.P_data,0,36*sizeof(float));
    assert(!ImuFilter_Update(zero,gravity,0.001f,&out));
    assert(memcmp(&out,&before,sizeof(out))==0);
    assert(!ImuFilter_Update(zero,gravity,0.001f,&out));
    assert(ImuFilter_Init(identity));
    assert(ImuFilter_Update(zero,gravity,0.001f,&out));
    before=out;
    QEKF_INS.IMU_QuaternionEKF.P_data[0]=NAN;
    assert(!ImuFilter_Update(zero,gravity,0.001f,&out));
    assert(memcmp(&out,&before,sizeof(out))==0);
    assert(ImuFilter_Init(identity));
    assert(ImuFilter_Update(zero,gravity,0.001f,&out));
    printf("PASS: real QEKF/CMSIS static, tilt, variable dt, yaw wrap, slow yaw, input/matrix faults, 1000 resets; arena=%zu/%u bytes.\n",arena_used,KALMAN_FILTER_ARENA_BYTES);
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=shutil.which("gcc") or
                        r"E:\CLion\CLion 2026.2.2\bin\mingw\bin\gcc.exe")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    build = root / "build"
    build.mkdir(exist_ok=True)
    matrices = [root / "Drivers/CMSIS/DSP/Source/MatrixFunctions" / f"arm_mat_{name}_f32.c"
                for name in ("init", "add", "sub", "mult", "trans", "inverse")]
    with tempfile.TemporaryDirectory(prefix="imu-filter-test-", dir=build) as tmp:
        folder = Path(tmp)
        (folder / "arm_math.h").write_text(ARM_MATH, encoding="utf-8")
        (folder / "test.c").write_text(TEST, encoding="utf-8")
        for dsp in (False, True):
            exe = folder / f"imu_filter_{int(dsp)}.exe"
            cmd = [args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
                   "-DARM_MATH_MATRIX_CHECK", "-I" + str(folder),
                   "-I" + str(root / "Algorithm/Inc")]
            if dsp:
                cmd.append("-DARM_MATH_DSP=1")
            cmd += [str(folder / "test.c"), str(root / "Algorithm/Src/imu_filter.c"),
                    str(root / "Algorithm/Src/QuaternionEKFc.c"),
                    str(root / "Algorithm/Src/kalman_filter.c")]
            cmd += [str(path) for path in matrices]
            cmd += ["-lm", "-o", str(exe)]
            subprocess.run(cmd, check=True, timeout=30)
            subprocess.run([str(exe)], check=True, timeout=20)


if __name__ == "__main__":
    main()
