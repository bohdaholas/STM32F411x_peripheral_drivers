#include "../inc/stm32f411x.h"

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baud_rate) {
    uint32_t PCLKx, usart_divider, mantissa, fraction, tmp;

    //Get the value of APB bus clock in to the variable PCLKx
    if (pUSARTx == USART1 || pUSARTx == USART6) {
        PCLKx = RCC_GetPCLKxValue(RCC_CFGR_PRE2);
    } else {
        PCLKx = RCC_GetPCLKxValue(RCC_CFGR_PRE1);
    }

    // usard_divider is multiplied by 100 actually
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
        usart_divider = (25 * PCLKx) / (2 * baud_rate);
    } else {
        usart_divider = (25 * PCLKx) / (4 * baud_rate);
    }

    mantissa = usart_divider / 100;

    // Place the Mantissa part in appropriate bit position
    tmp = mantissa << 4;

    fraction = usart_divider - 100 * mantissa;
    if (pUSARTx->CR1 & (1 << USART_CR1_OVER8)) {
        fraction = ((8 * fraction + 50) / 100) & ((uint8_t) 0x7);
    } else {
        fraction = ((16 * fraction + 50) / 100) & ((uint8_t) 0xF);
    }

    // Place the fractional part in appropriate bit position
    tmp |= fraction;

    pUSARTx->BRR = tmp;
}

void USART_Init(USART_Handle_t *pUSART_Handle) {
    USART_PCLK_Control(pUSART_Handle->pUSARTx, ENABLE);

    // program CR1
    uint32_t tmp = 0;
    if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_RX) {
        tmp |= (1 << USART_CR1_RE);
    } else if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_TX) {
        tmp |= (1 << USART_CR1_TE);
    } else if (pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TXRX) {
        tmp |= ((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
    }

    tmp |= pUSART_Handle->USART_Config.USART_WordLength << USART_CR1_M;

    if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
        tmp |= (1 << USART_CR1_PCE);
    } else if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) {
        tmp |= (1 << USART_CR1_PCE);
        tmp |= (1 << USART_CR1_PS);
    }
    pUSART_Handle->pUSARTx->CR1 = tmp;

    // program CR2
    tmp = pUSART_Handle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;
    pUSART_Handle->pUSARTx->CR2 = tmp;

    // program CR3
    if (pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
        tmp |= (1 << USART_CR3_CTSE);
    } else if (pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS) {
        tmp |= (1 << USART_CR3_RTSE);
    } else if (pUSART_Handle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
        tmp |= ((1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE));
    }
    pUSART_Handle->pUSARTx->CR3 = tmp;

    USART_SetBaudRate(pUSART_Handle->pUSARTx, pUSART_Handle->USART_Config.USART_BaudRate);
}

void USART_DeInit(USART_RegDef_t *pUSARTx) {
    if (pUSARTx == USART1) {
        USART1_REG_RESET();
    } else if (pUSARTx == USART2) {
        USART2_REG_RESET();
    } else if (pUSARTx == USART6) {
        USART6_REG_RESET();
    }
}

void USART_SendData(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint32_t len) {
    uint16_t *pData;
    for (size_t i = len; i > 0; i--) {
        while (USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TXE) == USART_FLAG_RESET);

        if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            // load the DR with 2 bytes and mask first 9 bits
            pData = (uint16_t *) pTxBuffer;
            pUSART_Handle->pUSARTx->DR = (*pData & (uint16_t) 0x01FF);

            if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                // 9 data bits
                pTxBuffer += 2;
            } else {
                // 8 data bits + 1 parity bit
                pTxBuffer++;
            }
        } else {
            // 7 data bits and parity bit OR 8 data bits and no parity bit
            pUSART_Handle->pUSARTx->DR = *pTxBuffer;
            pTxBuffer++;
        }
    }
    while (USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_TC) == USART_FLAG_RESET);
}

void USART_ReceiveData(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint32_t len) {
    for (size_t i = len; i > 0; i--) {
        while (USART_GetFlagStatus(pUSART_Handle->pUSARTx, USART_FLAG_RXNE) == USART_FLAG_RESET);

        if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
            if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                uint16_t *pData = (uint16_t *) pRxBuffer;
                *pData = pUSART_Handle->pUSARTx->DR & (uint16_t) 0x1FF;
                pRxBuffer += 2;
            } else {
                *pRxBuffer = pUSART_Handle->pUSARTx->DR & (uint8_t) 0xFF;
                pRxBuffer++;
            }
        } else {
            if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                *pRxBuffer = pUSART_Handle->pUSARTx->DR;
            } else {
                *pRxBuffer = pUSART_Handle->pUSARTx->DR & (uint8_t) 0x7F;
            }
            pRxBuffer++;
        }
    }
}

uint8_t USART_SendDataIT(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint32_t len) {
    uint8_t tx_state = pUSART_Handle->TxState;

    if (tx_state != USART_BUSY_IN_TX) {
        pUSART_Handle->TxLen = len;
        pUSART_Handle->pTxBuffer = pTxBuffer;
        pUSART_Handle->TxState = USART_BUSY_IN_TX;

        pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);
        pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
    }

    return tx_state;
}

uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint32_t len) {
    uint8_t rx_state = pUSART_Handle->RxState;

    if (rx_state != USART_BUSY_IN_RX) {
        pUSART_Handle->RxLen = len;
        pUSART_Handle->pRxBuffer = pRxBuffer;
        pUSART_Handle->RxState = USART_BUSY_IN_RX;

        pUSART_Handle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    }

    return rx_state;
}

void USART_PCLK_Control(USART_RegDef_t *pUSARTx, uint8_t en) {
    if (en == ENABLE) {
        if (pUSARTx == USART1) {
            USART1_PCLK_EN();
        } else if (pUSARTx == USART2) {
            USART2_PCLK_EN();
        } else if (pUSARTx == USART6) {
            USART6_PCLK_EN();
        }
    } else {
        if (pUSARTx == USART1) {
            USART1_PCLK_DI();
        } else if (pUSARTx == USART2) {
            USART2_PCLK_EN();
        } else if (pUSARTx == USART6) {
            USART6_PCLK_DI();
        }
    }
}

void USART_Peripheral_Control(USART_RegDef_t *pUSARTx, uint8_t en) {
    if (en == ENABLE) {
        pUSARTx->CR1 |= (1 << USART_CR1_UE);
    } else {
        pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
    }
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flagName) {
    uint8_t flagStatus = pUSARTx->SR & flagName;
    if (flagStatus) {
        return USART_FLAG_SET;
    }
    return USART_FLAG_RESET;
}

void USART_IRQ_InterruptConfig(uint8_t IRQ_number, uint8_t en) {
    if (en == ENABLE) {
        *NVIC_ISER |= (1 << IRQ_number);
    } else {
        *NVIC_ICER &= ~(1 << IRQ_number);
    }
}

void USART_IRQ_PriorityConfig(uint8_t IRQ_number, uint32_t priority) {
    uint8_t iprx = IRQ_number / 4;
    uint8_t prix = IRQ_number % 4;

    uint8_t shift_amount = (8 * prix) + (8 - NO_BITS_IMPLEMENTED);
    *(NVIC_PR_BASEADDR + 4 * iprx) |= (priority << shift_amount);
}

void USART_IRQHandling(USART_Handle_t *pUSART_Handle) {
    uint32_t tmp1, tmp2;

    tmp1 = pUSART_Handle->pUSARTx->SR & (1 << USART_SR_TC);
    tmp2 = pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_TCIE);
    if (tmp1 && tmp2) {
        USART_IRQ_Handle_TC(pUSART_Handle);
    }

    tmp1 = pUSART_Handle->pUSARTx->SR & (1 << USART_SR_TXE);
    tmp2 = pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_TXEIE);
    if (tmp1 && tmp2) {
        USART_IRQ_Handle_TXE(pUSART_Handle);
    }

    tmp1 = pUSART_Handle->pUSARTx->SR & (1 << USART_SR_RXNE);
    tmp2 = pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_RXNEIE);
    if (tmp1 && tmp2) {
        USART_IRQ_Handle_RXNE(pUSART_Handle);
    }

    tmp1 = pUSART_Handle->pUSARTx->SR & (1 << USART_SR_CTS);
    tmp2 = pUSART_Handle->pUSARTx->CR3 & (1 << USART_CR3_CTSE);
    if (tmp1 && tmp2) {
        USART_IRQ_Handle_CTS(pUSART_Handle);
    }

    tmp1 = pUSART_Handle->pUSARTx->SR & (1 << USART_SR_IDLE);
    tmp2 = pUSART_Handle->pUSARTx->CR1 & (1 << USART_CR1_IDLEIE);
    if (tmp1 && tmp2) {
        USART_IRQ_Handle_IDLE(pUSART_Handle);
    }

    tmp1 = pUSART_Handle->pUSARTx->SR & USART_SR_ORE;
    tmp2 = pUSART_Handle->pUSARTx->CR1 & USART_CR1_RXNEIE;
    if (tmp1 && tmp2) {
        USART_IRQ_Handle_ORE(pUSART_Handle);
    }

    // Handling noise flag, overrun error and framing Error in case of multibuffer communication
    uint32_t err_interrupt_enable = pUSART_Handle->pUSARTx->CR3 & (1 << USART_CR3_EIE);
    if (err_interrupt_enable) {
        tmp1 = pUSART_Handle->pUSARTx->SR;
        if (tmp1 & (1 << USART_SR_FE)) {
            USART_IRQ_Handle_FE(pUSART_Handle);
        }
        if (tmp1 & (1 << USART_SR_NE)) {
            USART_IRQ_Handle_NE(pUSART_Handle);
        }
        if (tmp1 & (1 << USART_SR_ORE)) {
            USART_IRQ_Handle_ORE(pUSART_Handle);
        }
    }
}

static void USART_IRQ_Handle_TC(USART_Handle_t *pUSART_Handle) {
    // close transmission and call application callback if TxLen is zero
    if (pUSART_Handle->TxState == USART_BUSY_IN_TX && pUSART_Handle->TxLen == 0) {
        pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);
        pUSART_Handle->pUSARTx->SR &= ~(1 << USART_SR_TC);

        pUSART_Handle->TxState = USART_READY;
        pUSART_Handle->pTxBuffer = NULL;
        USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_TX_CMPLT);
    }
}

static void USART_IRQ_Handle_TXE(USART_Handle_t *pUSART_Handle) {
    if (pUSART_Handle->TxState == USART_BUSY_IN_TX) {
        if (pUSART_Handle->TxLen > 0) {
            if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
                // load the DR with 2 bytes and mask first 9 bits
                uint16_t *pData = (uint16_t *) pUSART_Handle->pTxBuffer;
                pUSART_Handle->pUSARTx->DR = (*pData & (uint16_t) 0x01FF);

                if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                    // 9 data bits
                    pUSART_Handle->pTxBuffer += 2;
                    pUSART_Handle->TxLen -= 2;
                } else {
                    // 8 data bits + 1 parity bit
                    pUSART_Handle->pTxBuffer++;
                    pUSART_Handle->TxLen--;
                }
            } else {
                // 7 data bits and parity bit OR 8 data bits and no parity bit
                pUSART_Handle->pUSARTx->DR = (*pUSART_Handle->pTxBuffer & (uint8_t) 0xFF);
                pUSART_Handle->pTxBuffer++;
                pUSART_Handle->TxLen--;
            }
        }
        if (pUSART_Handle->TxLen == 0) {
            pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
        }
    }
}

static void USART_IRQ_Handle_RXNE(USART_Handle_t *pUSART_Handle) {
    if (pUSART_Handle->TxState == USART_BUSY_IN_RX) {
        if (pUSART_Handle->RxLen > 0) {
            if (pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
                if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                    uint16_t *pData = (uint16_t *) pUSART_Handle->pRxBuffer;
                    *pData = pUSART_Handle->pUSARTx->DR & (uint16_t) 0x1FF;
                    pUSART_Handle->pRxBuffer += 2;
                    pUSART_Handle->RxLen -= 2;
                } else {
                    *pUSART_Handle->pRxBuffer = pUSART_Handle->pUSARTx->DR & (uint8_t) 0xFF;
                    pUSART_Handle->pRxBuffer++;
                    pUSART_Handle->RxLen--;
                }
            } else {
                if (pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
                    *pUSART_Handle->pRxBuffer = pUSART_Handle->pUSARTx->DR;
                } else {
                    *pUSART_Handle->pRxBuffer = pUSART_Handle->pUSARTx->DR & (uint8_t) 0x7F;
                }
                pUSART_Handle->pRxBuffer++;
                pUSART_Handle->RxLen--;
            }
        }
        if (pUSART_Handle->RxLen == 0) {
            pUSART_Handle->pUSARTx->CR1 &= ~(1 << USART_CR1_RXNEIE);
            pUSART_Handle->RxState = USART_READY;
            USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_RX_CMPLT);
        }
    }
}

static void USART_IRQ_Handle_CTS(USART_Handle_t *pUSART_Handle) {
    pUSART_Handle->pUSARTx->SR &= ~(1 << USART_SR_CTS);
    USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_CTS);
}

static void USART_IRQ_Handle_IDLE(USART_Handle_t *pUSART_Handle) {
    pUSART_Handle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);
    USART_ApplicationEventCallback(pUSART_Handle, USART_EVENT_IDLE);
}

static void USART_IRQ_Handle_ORE(USART_Handle_t *pUSART_Handle) {
    // Need not clear the ORE flag here
    // Instead there's api for the application to clear the ORE flag
    USART_ApplicationEventCallback(pUSART_Handle, USART_ERR_ORE);
}

static void USART_IRQ_Handle_FE(USART_Handle_t *pUSART_Handle) {
    /* Frame error bit is set by hardware when a de-synchronization is detected. Cleared by a software
     * sequence (reading SR and DR registers) */
    uint32_t dummy_read;
    dummy_read = pUSART_Handle->pUSARTx->SR;
    dummy_read = pUSART_Handle->pUSARTx->DR;
    (void) dummy_read;
    USART_ApplicationEventCallback(pUSART_Handle, USART_ERR_FE);
}

static void USART_IRQ_Handle_NE(USART_Handle_t *pUSART_Handle) {
    /* Noise error is set by hardware when noise is detected on a received frame. Cleared by a
        software sequence (reading SR and DR registers). */
    uint32_t dummy_read;
    dummy_read = pUSART_Handle->pUSARTx->SR;
    dummy_read = pUSART_Handle->pUSARTx->DR;
    (void) dummy_read;
    USART_ApplicationEventCallback(pUSART_Handle, USART_ERR_NE);
}

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle, uint8_t event) {
    // should be defined in user level application
}