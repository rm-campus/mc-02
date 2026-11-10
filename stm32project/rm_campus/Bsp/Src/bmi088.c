/**
 * BMI088 C 驱动，按达妙 DM-MC02 官方 CtrBoard-H7_ALL 的寄存器配置/数据换算移植。
 * 来源：用户提供 dm-mc02-master.zip / 例程/CtrBoard-H7_ALL/Bsp/BMI088driver.c。
 * 改动：HAL 整包传输、错误返回、任务延时、关闭 DRDY 输出，不添加角速度死区。
 * 加速度读多一个 dummy 字节；每次传输期间始终保持对应片选为低。
 */
#include "bmi088.h"
#include "spi.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define SPI_TIMEOUT_MS 2U
#define ACC_ID_REG 0x00U
#define ACC_ID_VALUE 0x1EU
#define GYRO_ID_REG 0x00U
#define GYRO_ID_VALUE 0x0FU
#define ACC_DATA_REG 0x12U
#define ACC_TEMP_REG 0x22U
#define ACC_RESET_REG 0x7EU
#define GYRO_RESET_REG 0x14U
#define RESET_COMMAND 0xB6U

typedef struct { uint8_t reg, value; } RegisterValue;
/* 官方采样档位：加速度 ±3g / 800 Hz，陀螺 ±2000 deg/s / 1000 Hz、116 Hz 带宽。
 * 本轮 1 ms 轮询，使用最近一次加速度；两芯片并非严格同步采样。 */
static const RegisterValue accel_config[] = {
    {0x7DU, 0x04U}, /* ACC_PWR_CTRL: enable */
    {0x7CU, 0x00U}, /* ACC_PWR_CONF: active */
    {0x40U, 0xABU}, /* ACC_CONF: normal, 800 Hz */
    {0x41U, 0x00U}, /* ACC_RANGE: 3g */
    {0x53U, 0x00U}, /* INT1 output disabled */
    {0x58U, 0x00U}  /* no external DRDY mapping */
};
static const RegisterValue gyro_config[] = {
    {0x0FU, 0x00U}, /* GYRO_RANGE: 2000 deg/s */
    {0x10U, 0x82U}, /* GYRO_BANDWIDTH: 1000/116 Hz */
    {0x11U, 0x00U}, /* normal mode */
    {0x15U, 0x00U}, /* DRDY output disabled */
    {0x16U, 0x00U},
    {0x18U, 0x00U}
};

static void select_chip(bool accel, GPIO_PinState state)
{
    HAL_GPIO_WritePin(accel ? ACC_CS_GPIO_Port : GYRO_CS_GPIO_Port,
                      accel ? ACC_CS_Pin : GYRO_CS_Pin, state);
}

static Bmi088Result read_regs(bool accel, uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t tx[10] = {0}, rx[10] = {0};
    uint8_t prefix = accel ? 2U : 1U;
    if (data == NULL || length == 0U || length > 8U) return BMI088_ARGUMENT_ERROR;
    tx[0] = reg | 0x80U;
    select_chip(accel, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi2, tx, rx,
                                                      length + prefix, SPI_TIMEOUT_MS);
    select_chip(accel, GPIO_PIN_SET);
    if (result != HAL_OK) return BMI088_IO_ERROR;
    memcpy(data, rx + prefix, length);
    return BMI088_OK;
}

static Bmi088Result write_reg(bool accel, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value}, rx[2];
    select_chip(accel, GPIO_PIN_RESET);
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2U, SPI_TIMEOUT_MS);
    select_chip(accel, GPIO_PIN_SET);
    return result == HAL_OK ? BMI088_OK : BMI088_IO_ERROR;
}

static Bmi088Result configure(bool accel, const RegisterValue *table, size_t count)
{
    for (size_t i = 0; i < count; ++i)
    {
        uint8_t value;
        Bmi088Result result = write_reg(accel, table[i].reg, table[i].value);
        if (result != BMI088_OK) return result;
        /* 2 tick 至少跨过一个完整毫秒，满足模式切换和寄存器写入间隔。 */
        osDelay(2U);
        result = read_regs(accel, table[i].reg, &value, 1U);
        if (result != BMI088_OK) return result;
        if (value != table[i].value) return BMI088_CONFIG_ERROR;
    }
    return BMI088_OK;
}

Bmi088Result Bmi088_Init(void)
{
    uint8_t id;
    Bmi088Result result;
    select_chip(true, GPIO_PIN_SET);
    select_chip(false, GPIO_PIN_SET);
    osDelay(10U);
    /* 加速度计上电默认 I2C；第一次 dummy read 的 CS 上升沿切换到 SPI。 */
    result = read_regs(true, ACC_ID_REG, &id, 1U);
    if (result != BMI088_OK) return result;
    result = write_reg(true, ACC_RESET_REG, RESET_COMMAND);
    if (result != BMI088_OK) return result;
    osDelay(100U);
    result = read_regs(true, ACC_ID_REG, &id, 1U);
    if (result != BMI088_OK) return result;
    osDelay(2U);
    result = read_regs(true, ACC_ID_REG, &id, 1U);
    if (result != BMI088_OK) return result;
    if (id != ACC_ID_VALUE) return BMI088_ACCEL_ID_ERROR;

    result = write_reg(false, GYRO_RESET_REG, RESET_COMMAND);
    if (result != BMI088_OK) return result;
    osDelay(100U);
    result = read_regs(false, GYRO_ID_REG, &id, 1U);
    if (result != BMI088_OK) return result;
    if (id != GYRO_ID_VALUE) return BMI088_GYRO_ID_ERROR;

    result = configure(true, accel_config, sizeof(accel_config) / sizeof(accel_config[0]));
    if (result != BMI088_OK) return result;
    result = configure(false, gyro_config, sizeof(gyro_config) / sizeof(gyro_config[0]));
    if (result != BMI088_OK) return result;
    osDelay(10U);
    return BMI088_OK;
}

static int16_t signed_le16(const uint8_t *bytes)
{
    return (int16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
}

Bmi088Result Bmi088_Read(Bmi088Sample *out)
{
    if (out == NULL) return BMI088_ARGUMENT_ERROR;
    uint8_t acc[6], gyro[8], temp[2], id;
    Bmi088Sample sample;
    Bmi088Result result = read_regs(true, ACC_ID_REG, &id, 1U);
    if (result != BMI088_OK) return result;
    if (id != ACC_ID_VALUE) return BMI088_ACCEL_ID_ERROR;
    result = read_regs(true, ACC_DATA_REG, acc, sizeof(acc));
    if (result != BMI088_OK) return result;
    /* 与官方例程一致，从 ID 到 XYZ 连读 8 字节，同时验证陀螺通信。 */
    result = read_regs(false, GYRO_ID_REG, gyro, sizeof(gyro));
    if (result != BMI088_OK) return result;
    if (gyro[0] != GYRO_ID_VALUE) return BMI088_GYRO_ID_ERROR;
    result = read_regs(true, ACC_TEMP_REG, temp, sizeof(temp));
    if (result != BMI088_OK) return result;
    for (uint8_t i = 0; i < 3U; ++i)
    {
        sample.accel[i] = signed_le16(&acc[2U * i]) * (3.0f * 9.80665f / 32768.0f);
        sample.gyro[i] = signed_le16(&gyro[2U + 2U * i]) *
                         (2000.0f * 0.0174532925199433f / 32768.0f);
    }
    int16_t raw_temp = (int16_t)(((uint16_t)temp[0] << 3U) | (temp[1] >> 5U));
    if (raw_temp > 1023) raw_temp -= 2048;
    sample.temperature_c = raw_temp * 0.125f + 23.0f;
    *out = sample;
    return BMI088_OK;
}
