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
/* address 是芯片内字节偏移，data 至少可容纳 length 字节；长度为零或越界返回 ARGUMENT。 */
W25Q64Result W25Q64_Read(uint32_t address, void *data, uint32_t length);
/* 从 data 取 length 字节，跨页自动拆分，每段写后读回校验。调用方负责按需预擦除。
 * 中途出错返回对应错误码，之前已成功编程的字节保留。 */
W25Q64Result W25Q64_Write(uint32_t address, const void *data, uint32_t length);
/* address 必须按扇区大小对齐。成功返回 OK 表示已读回确认整扇区均为 0xFF。 */
W25Q64Result W25Q64_EraseSector(uint32_t address);
#endif
