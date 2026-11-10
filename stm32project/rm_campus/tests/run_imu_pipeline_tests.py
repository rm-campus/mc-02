"""Host integration: simulated BMI088 SPI -> real calibration -> real EKF."""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

from run_imu_filter_tests import ARM_MATH

MAIN_STUB = r'''
#ifndef HOST_MAIN_H
#define HOST_MAIN_H
#include <stdint.h>
typedef struct { int id; } GPIO_TypeDef;
typedef enum { GPIO_PIN_RESET=0, GPIO_PIN_SET=1 } GPIO_PinState;
extern GPIO_TypeDef port_c;
#define ACC_CS_GPIO_Port (&port_c)
#define GYRO_CS_GPIO_Port (&port_c)
#define ACC_CS_Pin 1U
#define GYRO_CS_Pin 8U
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
uint32_t HAL_GetTick(void);
#endif
'''
SPI_STUB = r'''
#include "main.h"
typedef struct { int unused; } SPI_HandleTypeDef;
typedef enum { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 } HAL_StatusTypeDef;
extern SPI_HandleTypeDef hspi2;
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
'''
TASK_STUB = r'''
#ifndef HOST_TASK_H
#define HOST_TASK_H
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(n) (n)
void test_enter(void);
void test_exit(void);
#define taskENTER_CRITICAL() test_enter()
#define taskEXIT_CRITICAL() test_exit()
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *, TickType_t);
#endif
'''

TEST = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "spi.h"
#include "task.h"
#include "imu_app.h"
#include "imu_clock.h"

GPIO_TypeDef port_c;
SPI_HandleTypeDef hspi2;
static uint8_t acc_regs[256], gyro_regs[256];
static int acc_selected, gyro_selected, accel_spi_mode, fail_transfer;
static unsigned critical_depth;
static uint32_t tick;
static const float accel_scale=3.0f*9.80665f/32768.0f;
static const float gyro_scale=2000.0f*0.0174532925199433f/32768.0f;

void test_enter(void) { ++critical_depth; }
void test_exit(void) { assert(critical_depth); --critical_depth; }
uint32_t HAL_GetTick(void) { return tick; }
int32_t osDelay(uint32_t delay) { tick+=delay; return 0; }
TickType_t xTaskGetTickCount(void) { return tick; }
void vTaskDelayUntil(TickType_t *wake, TickType_t delay) { *wake+=delay; tick=*wake; }
bool ImuClock_Init(void) { return true; }
uint32_t ImuClock_Cycles(void) { return tick*480000U; }
float ImuClock_Elapsed(uint32_t a, uint32_t b) { return (uint32_t)(b-a)/480000000.0f; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    assert(port==&port_c && (pin==ACC_CS_Pin || pin==GYRO_CS_Pin));
    if(pin==ACC_CS_Pin) {
        if(state==GPIO_PIN_SET && acc_selected) accel_spi_mode=1;
        acc_selected=state==GPIO_PIN_RESET;
    } else gyro_selected=state==GPIO_PIN_RESET;
    assert(!(acc_selected && gyro_selected));
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
                                         uint8_t *rx, uint16_t length, uint32_t timeout)
{
    assert(spi==&hspi2 && timeout==2 && (acc_selected!=gyro_selected));
    assert(!critical_depth);
    if(fail_transfer) { fail_transfer=0; return HAL_TIMEOUT; }
    memset(rx,0xA5,length); /* Dummy 字节故意不是零，验证偏移处理。 */
    uint8_t *regs=acc_selected?acc_regs:gyro_regs;
    unsigned reg=tx[0]&0x7F;
    if(tx[0]&0x80) {
        unsigned prefix=acc_selected?2:1;
        assert(length>prefix && length<=10);
        for(unsigned i=prefix;i<length;++i)
            rx[i]=(acc_selected && !accel_spi_mode)?0xFF:regs[reg+i-prefix];
    } else {
        assert(length==2);
        regs[reg]=tx[1];
        if(acc_selected && reg==0x7E && tx[1]==0xB6) accel_spi_mode=0;
    }
    return HAL_OK;
}

static void le16(uint8_t *p, int16_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)((uint16_t)v>>8); }
static void sensor(int16_t gx, int16_t gy, int16_t gz)
{
    le16(acc_regs+0x12,0); le16(acc_regs+0x14,0); le16(acc_regs+0x16,10923);
    le16(gyro_regs+2,gx); le16(gyro_regs+4,gy); le16(gyro_regs+6,gz);
    int raw_temp=-60; /* 15.5 degC，覆盖负的11位温度编码。 */
    unsigned bits=(unsigned)(raw_temp&0x7FF);
    acc_regs[0x22]=(uint8_t)(bits>>3); acc_regs[0x23]=(uint8_t)(bits<<5);
}
static ImuData snapshot(void) { ImuData d; Imu_CopyData(&d); return d; }
static void step(void)
{
    Bmi088Sample sample;
    assert(Bmi088_Read(&sample)==BMI088_OK);
    ++tick;
    assert(Imu_ProcessSample(&sample,0.001f,tick));
}

int main(void)
{
    acc_regs[0]=0x1E; gyro_regs[0]=0x0F;
    assert(Bmi088_Init()==BMI088_OK);
    assert(!acc_selected && !gyro_selected);
    assert(acc_regs[0x40]==0xAB && acc_regs[0x41]==0);
    assert(gyro_regs[0x10]==0x82 && gyro_regs[0x0F]==0);
    assert(acc_regs[0x53]==0 && acc_regs[0x58]==0 && gyro_regs[0x15]==0);
    sensor(5,-7,11);
    Bmi088Sample sample;
    assert(Bmi088_Read(&sample)==BMI088_OK);
    assert(fabsf(sample.accel[2]-10923*accel_scale)<1e-6f);
    assert(fabsf(sample.gyro[1]+7*gyro_scale)<1e-7f);
    assert(sample.temperature_c==15.5f);
    Bmi088Sample saved=sample;
    fail_transfer=1;
    assert(Bmi088_Read(&sample)==BMI088_IO_ERROR);
    assert(!acc_selected && !gyro_selected && memcmp(&sample,&saved,sizeof(sample))==0);
    gyro_regs[0]=0xFF;
    assert(Bmi088_Read(&sample)==BMI088_GYRO_ID_ERROR);
    assert(memcmp(&sample,&saved,sizeof(sample))==0);
    gyro_regs[0]=0x0F;

    Imu_BeginCalibration();
    for(unsigned i=0;i<500;++i) step();
    ImuData calibrating=snapshot();
    assert(calibrating.state==IMU_CALIBRATING && calibrating.cal_samples==500);
    sensor(5,-7,1000); step();
    assert(snapshot().cal_samples==0 && snapshot().error==IMU_ERROR_MOVING);
    sensor(5,-7,11);
    for(unsigned i=0;i<1000;++i) step();
    ImuData d=snapshot();
    assert(d.state==IMU_RUNNING && d.cal_samples==1000 && d.error==0);
    assert(fabsf(d.gyro_bias[2]-11*gyro_scale)<1e-6f);
    assert(fabsf(d.gyro[0])<1e-6f && fabsf(d.gyro[1])<1e-6f);
    sensor(5,-7,58); /* 在静止零偏基础上增加真实Z转速。 */
    for(unsigned i=0;i<1000;++i) step();
    d=snapshot();
    assert(fabsf(d.yaw_total_deg-47*gyro_scale*57.295779513f)<0.05f);
    assert(fabsf(d.gyro[2]-47*gyro_scale)<1e-6f);

    Imu_BeginCalibration();
    for(unsigned i=0;i<1000;++i) { sensor(5,-7,(i&1)?50:-50); step(); }
    assert(snapshot().state==IMU_CALIBRATING && snapshot().cal_samples==0);
    assert(snapshot().error==IMU_ERROR_CALIBRATION_NOISE);
    sensor(5,-7,11);
    for(unsigned i=0;i<1000;++i) step();
    assert(snapshot().state==IMU_RUNNING);

    d=snapshot();
    Bmi088Sample invalid=sample; invalid.accel[0]=NAN;
    assert(!Imu_ProcessSample(&invalid,0.001f,tick+1));
    assert(snapshot().state==IMU_ERROR && snapshot().sample_ms==d.sample_ms);
    Imu_ReportFault(IMU_ERROR_READ_BASE+BMI088_IO_ERROR,true);
    assert(snapshot().read_errors==1);
    Imu_BeginCalibration();
    for(unsigned i=0;i<1000;++i) step();

    assert(snapshot().state==IMU_RUNNING);
    assert(!critical_depth);
    puts("PASS: BMI088 SPI/scaling/errors, calibration motion/noise rejection, bias correction, real EKF yaw, fault recovery.");
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
    with tempfile.TemporaryDirectory(prefix="imu-pipeline-test-", dir=build) as tmp:
        folder = Path(tmp)
        stubs = {"main.h": MAIN_STUB, "spi.h": SPI_STUB, "task.h": TASK_STUB,
                 "FreeRTOS.h": "#pragma once\n", "arm_math.h": ARM_MATH,
                 "cmsis_os2.h": "#include <stdint.h>\nint32_t osDelay(uint32_t);\n"}
        for name, contents in stubs.items():
            (folder / name).write_text(contents, encoding="utf-8")
        (folder / "test.c").write_text(TEST, encoding="utf-8")
        sources = [root / path for path in (
            "Bsp/Src/bmi088.c", "App/Src/imu_app.c",
            "Algorithm/Src/imu_filter.c", "Algorithm/Src/QuaternionEKFc.c",
            "Algorithm/Src/kalman_filter.c")]
        sources += [root / "Drivers/CMSIS/DSP/Source/MatrixFunctions" / f"arm_mat_{name}_f32.c"
                    for name in ("init", "add", "sub", "mult", "trans", "inverse")]
        exe = folder / "pipeline.exe"
        cmd = [args.cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror",
               "-DARM_MATH_MATRIX_CHECK", "-I" + str(folder)]
        cmd += ["-I" + str(root / path) for path in ("Bsp/Inc", "App/Inc", "Algorithm/Inc")]
        cmd += [str(folder / "test.c"), *map(str, sources), "-lm", "-o", str(exe)]
        subprocess.run(cmd, check=True, timeout=30)
        subprocess.run([str(exe)], check=True, timeout=20)


if __name__ == "__main__":
    main()
