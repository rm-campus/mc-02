#ifndef FLASH_TEST_H
#define FLASH_TEST_H
#include <stdbool.h>
#include <stdint.h>
bool FlashTest_Create(void);
bool FlashTest_Accept(const uint8_t *data, uint16_t length);
void FlashTest_Process(void);
uint32_t FlashTest_State(void);
uint32_t FlashTest_Error(void);
#endif
