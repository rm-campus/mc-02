#include "w25q64.h"
#include "octospi.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

OSPI_HandleTypeDef hospi2;
static uint8_t nor[W25Q64_CAPACITY], input[600], output[600];
static OSPI_RegularCmdTypeDef command;
static uint32_t tick, busy_until, programs, erases;
static uint8_t status_bits, device_id = 0x40;
static int fail_io, stuck, corrupt;
uint32_t HAL_GetTick(void) { return tick; }
int32_t osDelay(uint32_t ticks) { tick += ticks; return 0; }
void Error_Handler(void) { assert(0); }

int HAL_OSPI_Command(OSPI_HandleTypeDef *h, OSPI_RegularCmdTypeDef *c, uint32_t timeout)
{
    (void)h; (void)timeout;
    if (fail_io) return HAL_ERROR;
    assert(c->InstructionMode == 1 && c->DataDtrMode == 0);
    command = *c;
    if (c->Instruction == 0x06) status_bits |= 2;
    if (c->Instruction == 0x20)
    {
        assert(status_bits & 2); assert(c->Address % 4096 == 0);
        memset(nor + c->Address, 0xFF, 4096); ++erases;
        status_bits &= (uint8_t)~2U; busy_until = tick + 400;
    }
    return HAL_OK;
}
int HAL_OSPI_Receive(OSPI_HandleTypeDef *h, uint8_t *data, uint32_t timeout)
{
    (void)h; (void)timeout;
    if (command.Instruction == 0x9F) { data[0] = 0xEF; data[1] = device_id; data[2] = 0x17; }
    else if (command.Instruction == 0x05)
        data[0] = status_bits | ((stuck || (int32_t)(tick - busy_until) < 0) ? 1 : 0);
    else
    {
        assert(command.Instruction == 0x03 && command.AddressMode == 1 && command.AddressSize == 24);
        memcpy(data, nor + command.Address, command.NbData);
        if (corrupt) data[0] ^= 1;
    }
    return HAL_OK;
}
int HAL_OSPI_Transmit(OSPI_HandleTypeDef *h, uint8_t *data, uint32_t timeout)
{
    (void)h; (void)timeout;
    assert(command.Instruction == 0x02 && (status_bits & 2));
    assert(command.NbData <= 256 - command.Address % 256); /* 禁止页内回卷。 */
    for (unsigned i = 0; i < command.NbData; ++i) nor[command.Address + i] &= data[i];
    ++programs; status_bits &= (uint8_t)~2U; busy_until = tick + 3;
    return HAL_OK;
}

int main(void)
{
    memset(nor, 0xFF, sizeof(nor));
    assert(W25Q64_Init() == W25Q64_OK);
    device_id = 0x70; assert(W25Q64_Init() == W25Q64_OK);
    device_id = 0x33; assert(W25Q64_Init() == W25Q64_ID_ERROR); device_id = 0x40;
    for (unsigned i = 0; i < sizeof(input); ++i) input[i] = (uint8_t)(i * 37);
    assert(W25Q64_Write(250, input, sizeof(input)) == W25Q64_OK && programs == 4);
    assert(W25Q64_Read(250, output, sizeof(output)) == W25Q64_OK);
    assert(memcmp(input, output, sizeof(input)) == 0);
    assert(W25Q64_EraseSector(0) == W25Q64_OK && erases == 1);
    for (unsigned i = 0; i < 4096; ++i) assert(nor[i] == 0xFF);
    assert(W25Q64_EraseSector(1) == W25Q64_ARGUMENT);
    assert(W25Q64_Write(W25Q64_CAPACITY - 10, input, 11) == W25Q64_ARGUMENT);
    assert(W25Q64_Read(UINT32_MAX, output, 1) == W25Q64_ARGUMENT);
    assert(W25Q64_Write(0, NULL, 1) == W25Q64_ARGUMENT);
    status_bits = 4; assert(W25Q64_Write(0, input, 1) == W25Q64_PROTECTED); status_bits = 0;
    corrupt = 1; assert(W25Q64_Write(0, input, 1) == W25Q64_VERIFY_ERROR); corrupt = 0;
    stuck = 1; uint32_t before = tick;
    assert(W25Q64_Read(0, output, 1) == W25Q64_TIMEOUT && tick - before >= 750); stuck = 0;
    fail_io = 1; assert(W25Q64_Read(0, output, 1) == W25Q64_IO);
    puts("PASS: W25Q64 JEDEC IDs, real command readback, page splitting, erase verification, bounds, protection and timeouts.");
}
