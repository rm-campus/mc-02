#ifndef W25Q64_H
#define W25Q64_H
#include <stdint.h>
#define W25Q64_CAPACITY 0x800000UL
#define W25Q64_SECTOR_SIZE 4096U
#define W25Q64_PAGE_SIZE 256U
typedef enum {
    W25Q64_OK, W25Q64_ARGUMENT, W25Q64_IO, W25Q64_TIMEOUT,
    W25Q64_ID_ERROR, W25Q64_PROTECTED, W25Q64_VERIFY_ERROR
} W25Q64Result;
/* 仅 FlashServiceTask 调用；普通单线指令，24 位地址，不使用内存映射/DMA。 */
W25Q64Result W25Q64_Init(void);
W25Q64Result W25Q64_Read(uint32_t address, void *data, uint32_t length);
W25Q64Result W25Q64_Write(uint32_t address, const void *data, uint32_t length);
W25Q64Result W25Q64_EraseSector(uint32_t address);
#endif
