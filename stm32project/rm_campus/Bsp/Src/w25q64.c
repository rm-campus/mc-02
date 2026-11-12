#include "w25q64.h"
#include "octospi.h"
#include "cmsis_os2.h"
#include <stddef.h>
#include <string.h>

#define IO_TIMEOUT_MS 20U
static W25Q64Result command(uint8_t instruction, uint32_t address, uint32_t length, int has_address)
{
    OSPI_RegularCmdTypeDef cmd = {0};
    cmd.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
    cmd.FlashId = HAL_OSPI_FLASH_ID_1;
    cmd.Instruction = instruction;
    cmd.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
    cmd.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
    cmd.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
    cmd.AddressMode = has_address ? HAL_OSPI_ADDRESS_1_LINE : HAL_OSPI_ADDRESS_NONE;
    cmd.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
    cmd.Address = address;
    cmd.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = length ? HAL_OSPI_DATA_1_LINE : HAL_OSPI_DATA_NONE;
    cmd.NbData = length;
    cmd.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
    cmd.DQSMode = HAL_OSPI_DQS_DISABLE;
    cmd.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
    return HAL_OSPI_Command(&hospi2, &cmd, IO_TIMEOUT_MS) == HAL_OK ? W25Q64_OK : W25Q64_IO;
}

static W25Q64Result status(uint8_t *value)
{
    W25Q64Result r = command(0x05U, 0, 1, 0);
    if (r != W25Q64_OK) return r;
    return HAL_OSPI_Receive(&hospi2, value, IO_TIMEOUT_MS) == HAL_OK ? W25Q64_OK : W25Q64_IO;
}

static W25Q64Result wait_ready(uint32_t timeout)
{
    uint32_t start = HAL_GetTick();
    for (;;)
    {
        uint8_t value;
        W25Q64Result r = status(&value);
        if (r != W25Q64_OK) return r;
        if ((value & 1U) == 0U) return W25Q64_OK;
        if ((uint32_t)(HAL_GetTick() - start) >= timeout) return W25Q64_TIMEOUT;
        osDelay(1U); /* Flash 内部擦写时让出 CPU，不关中断或占用控制任务。 */
    }
}

static W25Q64Result write_enable(void)
{
    uint8_t value;
    W25Q64Result r = wait_ready(750U);
    if (r != W25Q64_OK) return r;
    r = status(&value);
    if (r != W25Q64_OK) return r;
    if ((value & 0x1CU) != 0U) return W25Q64_PROTECTED; /* 不擅自清除保护位。 */
    r = command(0x06U, 0, 0, 0);
    if (r != W25Q64_OK) return r;
    r = status(&value);
    if (r != W25Q64_OK) return r;
    return (value & 2U) ? W25Q64_OK : W25Q64_PROTECTED;
}

W25Q64Result W25Q64_Init(void)
{
    uint8_t id[3];
    W25Q64Result r = command(0xABU, 0, 0, 0); /* 释放可能的深度休眠。 */
    if (r != W25Q64_OK) return r;
    osDelay(1U);
    r = wait_ready(1000U);
    if (r != W25Q64_OK) return r;
    r = command(0x9FU, 0, sizeof(id), 0);
    if (r != W25Q64_OK) return r;
    if (HAL_OSPI_Receive(&hospi2, id, IO_TIMEOUT_MS) != HAL_OK) return W25Q64_IO;
    /* IQ/JQ 为 EF4017，IM/JM 为 EF7017。 */
    return id[0] == 0xEFU && (id[1] == 0x40U || id[1] == 0x70U) && id[2] == 0x17U
        ? W25Q64_OK : W25Q64_ID_ERROR;
}

static int range_ok(uint32_t address, const void *data, uint32_t length)
{
    return data != NULL && length != 0U && address < W25Q64_CAPACITY &&
           length <= W25Q64_CAPACITY - address;
}

W25Q64Result W25Q64_Read(uint32_t address, void *data, uint32_t length)
{
    if (!range_ok(address, data, length)) return W25Q64_ARGUMENT;
    W25Q64Result r = wait_ready(750U);
    if (r != W25Q64_OK) return r;
    uint8_t *out = data;
    while (length)
    {
        uint32_t chunk = length > W25Q64_SECTOR_SIZE ? W25Q64_SECTOR_SIZE : length;
        r = command(0x03U, address, chunk, 1);
        if (r != W25Q64_OK) return r;
        if (HAL_OSPI_Receive(&hospi2, out, IO_TIMEOUT_MS) != HAL_OK) return W25Q64_IO;
        out += chunk; address += chunk; length -= chunk;
    }
    return W25Q64_OK;
}

W25Q64Result W25Q64_Write(uint32_t address, const void *data, uint32_t length)
{
    if (!range_ok(address, data, length)) return W25Q64_ARGUMENT;
    const uint8_t *in = data;
    uint8_t verify[W25Q64_PAGE_SIZE];
    while (length)
    {
        uint32_t chunk = W25Q64_PAGE_SIZE - address % W25Q64_PAGE_SIZE;
        if (chunk > length) chunk = length;
        W25Q64Result r = write_enable();
        if (r != W25Q64_OK) return r;
        r = command(0x02U, address, chunk, 1);
        if (r != W25Q64_OK) return r;
        if (HAL_OSPI_Transmit(&hospi2, (uint8_t *)in, IO_TIMEOUT_MS) != HAL_OK) return W25Q64_IO;
        r = wait_ready(20U);
        if (r != W25Q64_OK) return r;
        r = W25Q64_Read(address, verify, chunk);
        if (r != W25Q64_OK) return r;
        if (memcmp(verify, in, chunk) != 0) return W25Q64_VERIFY_ERROR;
        address += chunk; in += chunk; length -= chunk;
    }
    return W25Q64_OK;
}

W25Q64Result W25Q64_EraseSector(uint32_t address)
{
    if (address >= W25Q64_CAPACITY || address % W25Q64_SECTOR_SIZE) return W25Q64_ARGUMENT;
    W25Q64Result r = write_enable();
    if (r != W25Q64_OK) return r;
    r = command(0x20U, address, 0, 1);
    if (r != W25Q64_OK) return r;
    r = wait_ready(750U);
    if (r != W25Q64_OK) return r;
    uint8_t verify[W25Q64_PAGE_SIZE];
    for (uint32_t offset = 0; offset < W25Q64_SECTOR_SIZE; offset += sizeof(verify))
    {
        r = W25Q64_Read(address + offset, verify, sizeof(verify));
        if (r != W25Q64_OK) return r;
        for (unsigned i = 0; i < sizeof(verify); ++i)
            if (verify[i] != 0xFFU) return W25Q64_VERIFY_ERROR;
    }
    return W25Q64_OK;
}
