#ifndef DRIVER_DEVELOPMENT_STM32F4_GPIO_DRIVER_H
#define DRIVER_DEVELOPMENT_STM32F4_GPIO_DRIVER_H

#include <stdint.h>
#include "stm32f411x.h"

#define GPIO_MODE_IN            0b00
#define GPIO_MODE_OUT           0b01
#define GPIO_MODE_ALT           0b10
#define GPIO_MODE_ANALOG        0b11
#define GPIO_MODE_IT_FT         0b100
#define GPIO_MODE_IT_RT         0b101
#define GPIO_MODE_IT_RFT        0b110

#define GPIO_OP_TYPE_PP         0
#define GPIO_OP_TYPE_OD         1

#define GPIO_SPEED_LOW          0b00
#define GPIO_SPEED_MED          0b01
#define GPIO_SPEED_HIGH         0b10
#define GPIO_SPEED_VERY_HIGH    0b11

#define GPIO_NO_PUPD            0b00
#define GPIO_PU                 0b01
#define GPIO_PD                 0b10

#define GPIO_PIN_NO_0           0
#define GPIO_PIN_NO_1           1
#define GPIO_PIN_NO_2           2
#define GPIO_PIN_NO_3           3
#define GPIO_PIN_NO_4           4
#define GPIO_PIN_NO_5           5
#define GPIO_PIN_NO_6           6
#define GPIO_PIN_NO_7           7
#define GPIO_PIN_NO_8           8
#define GPIO_PIN_NO_9           9
#define GPIO_PIN_NO_10          10
#define GPIO_PIN_NO_11          11
#define GPIO_PIN_NO_12          12
#define GPIO_PIN_NO_13          13
#define GPIO_PIN_NO_14          14
#define GPIO_PIN_NO_15          15

#define AF0                     0
#define AF1                     1
#define AF2                     2
#define AF3                     3
#define AF4                     4
#define AF5                     5
#define AF6                     6
#define AF7                     7
#define AF8                     8
#define AF9                     9
#define AF10                    10
#define AF11                    11
#define AF12                    12
#define AF13                    13
#define AF14                    14
#define AF15                    15

typedef struct {
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode;
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOpType;
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct {
    GPIO_RegDef_t *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

void GPIO_SetReg(uint8_t cfg_reg, volatile uint32_t *ll_reg, uint8_t pinNumber, uint8_t bitsPerPin);
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t en);

uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber);

void GPIO_IRQ_InterruptConfig(uint8_t IRQ_number, uint8_t en);
void GPIO_IRQ_PriorityConfig(uint8_t IRQ_number, uint32_t priority);
void GPIO_IRQ_Handling(uint8_t pinNumber);

#endif //DRIVER_DEVELOPMENT_STM32F4_GPIO_DRIVER_H
