#include "../../drivers/inc/stm32f411x.h"

void GPIO_SetReg(uint8_t cfg_reg, volatile uint32_t *ll_reg, uint8_t pinNumber, uint8_t bitsPerPin) {
    // set low level register using the value set by the user
    uint8_t clearMask;
    if (bitsPerPin == 1) clearMask = 0b1;
    if (bitsPerPin == 2) clearMask = 0b11;
    if (bitsPerPin == 4) clearMask = 0b1111;
    *ll_reg &= ~(clearMask << bitsPerPin * pinNumber);
    *ll_reg |= cfg_reg << (bitsPerPin * pinNumber);
}

uint8_t convertAddrToCode(GPIO_RegDef_t *pGPIOx) {
    if (pGPIOx == GPIOA) {
        return 0;
    } else if (pGPIOx == GPIOB) {
        return 1;
    } else if (pGPIOx == GPIOC) {
        return 2;
    } else if (pGPIOx == GPIOD) {
        return 3;
    } else if (pGPIOx == GPIOE) {
        return 4;
    } else {
        return 7;
    }
}

void GPIO_Init(GPIO_Handle_t *pGPIO_Handle) {
    GPIO_PCLK_Control(pGPIO_Handle->pGPIOx, ENABLE);

    uint8_t pinNumber = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber;
    // 1. configure mode
    if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        // IN, OUT, ALTERNATE, ANALOG
        GPIO_SetReg(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode, &pGPIO_Handle->pGPIOx->MODER, pinNumber, 2);
    } else {
        // Configure the type of interrupt trigger (falling trigger, rising trigger, both)
        if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
            EXTI->FTSR |= (1 << pinNumber);
            EXTI->RTSR &= ~(1 << pinNumber);
        } else if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
            EXTI->RTSR |= (1 << pinNumber);
            EXTI->FTSR &= ~(1 << pinNumber);
        } else if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
            EXTI->FTSR |= (1 << pinNumber);
            EXTI->RTSR |= (1 << pinNumber);
        }
        // EXTI line k is configured for pin k of GPIOx
        uint8_t EXTICRx = pinNumber / 4;
        uint8_t EXTIx = pinNumber % 4;
        uint8_t portcode = convertAddrToCode(pGPIO_Handle->pGPIOx);
        SYSCFG_PCLK_EN();
        SYSCFG->EXTICR[EXTICRx] |= portcode << (4 * EXTIx);

        // Enable delivery of the interrupt signal from EXTI block to the CPU
        EXTI->IMR |= 1 << pinNumber;
    }
    // 2. configure speed
    GPIO_SetReg(pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed, &pGPIO_Handle->pGPIOx->OSPEEDR, pinNumber, 2);
    // 3. configure pull up/down
    GPIO_SetReg(pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl, &pGPIO_Handle->pGPIOx->PUPDR, pinNumber, 2);
    // 4. configure output type
    GPIO_SetReg(pGPIO_Handle->GPIO_PinConfig.GPIO_PinOpType, &pGPIO_Handle->pGPIOx->OTYPER, pinNumber, 1);
    // 5. configure alternate functionality
    if (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT) {
        volatile uint32_t *AFR;
        if (GPIO_PIN_NO_0 <= pinNumber && pinNumber <= GPIO_PIN_NO_7) {
            AFR = &pGPIO_Handle->pGPIOx->AFR[0];
        } else {
            AFR = &pGPIO_Handle->pGPIOx->AFR[1];
        }
        GPIO_SetReg(pGPIO_Handle->GPIO_PinConfig.GPIO_PinAltFunMode, AFR, pinNumber % 8, 4);
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
    if (pGPIOx == GPIOA) {
        GPIOA_REG_RESET();
    } else if (pGPIOx == GPIOB) {
        GPIOB_REG_RESET();
    } else if (pGPIOx == GPIOC) {
        GPIOC_REG_RESET();
    } else if (pGPIOx == GPIOD) {
        GPIOD_REG_RESET();
    } else if (pGPIOx == GPIOE) {
        GPIOE_REG_RESET();
    } else {
        GPIOH_REG_RESET();
    }
}

void GPIO_PCLK_Control(GPIO_RegDef_t *pGPIOx, uint8_t en) {
    if (en == ENABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        } else {
            GPIOH_PCLK_EN();
        }
    } else {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_DI();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DI();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DI();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DI();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DI();
        } else {
            GPIOH_PCLK_DI();
        }
    }
}

uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
    return (pGPIOx->IDR >> pinNumber) & 1;
}

uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx) {
    return pGPIOx->IDR;
}

void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber, uint8_t value) {
    pGPIOx->ODR &= ~(1 << pinNumber);
    pGPIOx->ODR |= (value << pinNumber);
}

void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t value) {
    pGPIOx->ODR = value;
}

void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t pinNumber) {
    pGPIOx->ODR ^= (1 << pinNumber);
}

void GPIO_IRQ_InterruptConfig(uint8_t IRQ_number, uint8_t en) {
    if (en == ENABLE) {
        *NVIC_ISER |= (1 << IRQ_number);
    } else {
        *NVIC_ICER |= (1 << IRQ_number);
    }
}

void GPIO_IRQ_PriorityConfig(uint8_t IRQ_number, uint32_t priority) {
    uint8_t iprx = IRQ_number / 4;
    uint8_t prix = IRQ_number % 4;

    uint8_t shift_amount = (8 * prix) + (8 - NO_BITS_IMPLEMENTED);
    *(NVIC_PR_BASEADDR + 4 * iprx) |= (priority << shift_amount);
}

void GPIO_IRQ_Handling(uint8_t pinNumber) {
    if (EXTI->PR & (1 << pinNumber)) {
        EXTI->PR |= (1 << pinNumber);
    }
}

