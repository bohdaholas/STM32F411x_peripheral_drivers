#ifndef DRIVER_DEVELOPMENT_STM32F4_RCC_DRIVER_H
#define DRIVER_DEVELOPMENT_STM32F4_RCC_DRIVER_H

#include "stm32f411x.h"

uint32_t RCC_GetSYSCLKValue();
uint32_t RCC_GetAHBPrescaler();
uint32_t RCC_GetAPBxPrescaler(uint8_t PREx);
uint32_t RCC_GetPCLKxValue(uint8_t PREx);
uint32_t RCC_GetPLLOutputClock();

#endif //DRIVER_DEVELOPMENT_STM32F4_RCC_DRIVER_H
