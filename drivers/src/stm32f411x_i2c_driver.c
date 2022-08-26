#include "../inc/stm32f411x.h"

void I2C_Init(I2C_Handle_t *pI2C_Handle) {
    I2C_PCLK_Control(pI2C_Handle->pI2Cx, ENABLE);

    I2C_Peripheral_Control(pI2C_Handle->pI2Cx, ENABLE);
    uint32_t tmp = pI2C_Handle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
    pI2C_Handle->pI2Cx->CR1 = tmp;
    I2C_Peripheral_Control(pI2C_Handle->pI2Cx, DISABLE);

    tmp = pI2C_Handle->I2C_Config.I2C_DeviceAddress << 1;
    tmp |= (1 << 14); // due to some requitement in reference manual
    pI2C_Handle->pI2Cx->OAR1 = tmp;

    tmp = RCC_GetPCLKxValue(RCC_CFGR_PRE1) / 1000000U;
    pI2C_Handle->pI2Cx->CR2 = tmp & 0x3F; // get first 6 bits and clear the rest

    uint16_t ccr;
    if (pI2C_Handle->I2C_Config.I2C_SCLSpeed == I2C_SCL_SPEED_SM_100kHz) {
        ccr = RCC_GetPCLKxValue(RCC_CFGR_PRE1) / (2 * pI2C_Handle->I2C_Config.I2C_SCLSpeed);
    } else {
        tmp = (1 << I2C_CCR_FS); // configure fast mode
        tmp |= (pI2C_Handle->I2C_Config.I2C_FMDutyCycle << 14); // configure duty cycle
        if (pI2C_Handle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
            ccr = RCC_GetPCLKxValue(RCC_CFGR_PRE1) / (3 * pI2C_Handle->I2C_Config.I2C_SCLSpeed);
        } else {
            ccr = RCC_GetPCLKxValue(RCC_CFGR_PRE1) / (25 * pI2C_Handle->I2C_Config.I2C_SCLSpeed);
        }
    }
    tmp |= (ccr & 0xFFF);
    pI2C_Handle->pI2Cx->CCR = tmp;

    if(pI2C_Handle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM_100kHz) {
        tmp = (RCC_GetPCLKxValue(RCC_CFGR_PRE1) /1000000U) + 1;
    } else {
        tmp = ((RCC_GetPCLKxValue(RCC_CFGR_PRE1) * 300) / 1000000000U) + 1;
    }
    pI2C_Handle->pI2Cx->TRISE = (tmp & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
    if (pI2Cx == I2C1) {
        I2C1_REG_RESET();
    } else if (pI2Cx == I2C2) {
        I2C2_REG_RESET();
    } else if (pI2Cx == I2C3) {
        I2C3_REG_RESET();
    }
}

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t en) {
    if (en == I2C_ACK_ENABLE) {
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    } else {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t rw_bit) {
    pI2Cx->DR = (slaveAddr << 1) | rw_bit;
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2C_Handle) {
    if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL) &&
        pI2C_Handle->TxRxState == I2C_BUSY_IN_RX &&
        pI2C_Handle->RxSize == 1) {
        // disable ack
        I2C_ManageAcking(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);
    }
    // clear addr
    uint32_t dummy_read;
    dummy_read = pI2C_Handle->pI2Cx->SR1;
    dummy_read = pI2C_Handle->pI2Cx->SR2;
    (void) dummy_read;
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx) {
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr) {
    // generate start condition
    I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

    // confirm the start generation is completed by checking the SB flag in SR1
    while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB) == I2C_FLAG_RESET);

    // send the address of the slave with r/w bit set to 0
    I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, slaveAddr, I2C_WRITE_OPERATION);

    // confirm the address phase is completed
    while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR) == SPI_FLAG_RESET);

    // clear addr flag
    I2C_ClearADDRFlag(pI2C_Handle);

    // send data
    while (len) {
        while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE) == I2C_FLAG_RESET);
        pI2C_Handle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        len--;
    }

    // wait for
    // 1. data register to become empty
    // 2. successful transmission of the last byte
    while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE) == I2C_FLAG_RESET);
    while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_BTF) == I2C_FLAG_RESET);

    // generate stop condition
    if (sr == I2C_DISABLE_SR) {
        I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
    }
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr) {
    // generate start condition
    I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

    // confirm the start generation is completed by checking the SB flag in SR1
    while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB) == I2C_FLAG_RESET);

    // send the address of the slave with r/w bit set to 1
    I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, slaveAddr, I2C_READ_OPERATION);

    // confirm the address phase is completed
    while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR) == SPI_FLAG_RESET);

    if (len == 1) {
        // disable acking
        I2C_ManageAcking(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);

        // clear addr flag
        I2C_ClearADDRFlag(pI2C_Handle);

        // wait until rx buffer is non-empty
        while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE) == SPI_FLAG_RESET);

        *pRxBuffer = pI2C_Handle->pI2Cx->DR;

        // generate stop condition
        if (sr == I2C_DISABLE_SR) {
            I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
        }
    } else {
        // clear addr flag
        I2C_ClearADDRFlag(pI2C_Handle);

        while (len != 0) {
            // wait until rx buffer is non-empty
            while (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE) == SPI_FLAG_RESET);

            if (len == 2) {
                // disable acking
                I2C_ManageAcking(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);

                // generate stop condition
                if (sr == I2C_DISABLE_SR) {
                    I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
                }
            }
            *pRxBuffer = pI2C_Handle->pI2Cx->DR;
            pRxBuffer++;
            len--;
        }
    }

    // re-enable ack
    if (pI2C_Handle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2C_Handle->pI2Cx, I2C_ACK_ENABLE);
    }
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr) {
    uint8_t busy_state = pI2C_Handle->TxRxState;

    if (busy_state != I2C_BUSY_IN_TX && busy_state != I2C_BUSY_IN_RX) {
        pI2C_Handle->pTxBuffer = pTxBuffer;
        pI2C_Handle->TxLen = len;
        pI2C_Handle->TxRxState = I2C_BUSY_IN_TX;
        pI2C_Handle->DevAddr = slaveAddr;
        pI2C_Handle->Sr = Sr;

        I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busy_state;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr) {
    uint8_t busystate = pI2C_Handle->TxRxState;

    if (busystate != I2C_BUSY_IN_TX && busystate != I2C_BUSY_IN_RX) {
        pI2C_Handle->pRxBuffer = pRxBuffer;
        pI2C_Handle->RxLen = len;
        pI2C_Handle->TxRxState = I2C_BUSY_IN_RX;
        pI2C_Handle->RxSize = len; //Rxsize is used in the ISR code to manage the data reception
        pI2C_Handle->DevAddr = slaveAddr;
        pI2C_Handle->Sr = Sr;

        I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2C_Handle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }
    return busystate;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data) {
    pI2Cx->DR = data;
}

uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx) {
    return pI2Cx->DR;
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t en) {
    if (en == ENABLE) {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    } else {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
    }
}

void I2C_PCLK_Control(I2C_RegDef_t *pI2Cx, uint8_t en) {
    if (en == ENABLE) {
        if (pI2Cx == I2C1) {
            I2C1_PCLK_EN();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_EN();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_EN();
        }
    } else {
        if (pI2Cx == I2C1) {
            I2C1_PCLK_DI();
        } else if (pI2Cx == I2C2) {
            I2C2_PCLK_DI();
        } else if (pI2Cx == I2C3) {
            I2C3_PCLK_DI();
        }
    }
}

void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t en) {
    if (en == ENABLE) {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    } else {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName) {
    uint8_t flagStatus = pI2Cx->SR1 & flagName;
    if (flagStatus) {
        return I2C_FLAG_SET;
    }
    return I2C_FLAG_RESET;
}

void I2C_IRQ_InterruptConfig(uint8_t IRQ_number, uint8_t en) {
    if (en == ENABLE) {
        *NVIC_ISER |= (1 << IRQ_number);
    } else {
        *NVIC_ICER &= ~(1 << IRQ_number);
    }
}

void I2C_IRQ_PriorityConfig(uint8_t IRQ_number, uint32_t priority) {
    uint8_t iprx = IRQ_number / 4;
    uint8_t prix = IRQ_number % 4;

    uint8_t shift_amount = (8 * prix) + (8 - NO_BITS_IMPLEMENTED);
    *(NVIC_PR_BASEADDR + 4 * iprx) |= (priority << shift_amount);
}

void I2C_IRQ_EventHandling(I2C_Handle_t *pI2C_Handle) {
    uint32_t itevten = pI2C_Handle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    uint32_t itbufen = pI2C_Handle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

    if (itevten) {
        if (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_SB) == I2C_FLAG_SET) {
            I2C_IRQ_Handle_SB(pI2C_Handle);
        }

        if (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_ADDR) == I2C_FLAG_SET) {
            I2C_ClearADDRFlag(pI2C_Handle);
        }

        if (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_BTF) == I2C_FLAG_SET) {
            I2C_IRQ_Handle_BTF(pI2C_Handle);
        }

        if (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_STOPF) == I2C_FLAG_SET) {
            I2C_IRQ_Handle_STOPF(pI2C_Handle);
        }

        if (itbufen) {
            if (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE) == I2C_FLAG_SET) {
                I2C_IRQ_Handle_TXE(pI2C_Handle);
            }

            if (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_RXNE) == I2C_FLAG_SET) {
                I2C_IRQ_Handle_RXNE(pI2C_Handle);
            }
        }
    }
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle) {
    // Disable interrupts
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    // Reset handle structure state
    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    // Re-enable acking
    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE) {
        I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
    }
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) {
    // Disable interrupts
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    // Reset handle structure state
    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
}

static void I2C_IRQ_Handle_SB(I2C_Handle_t *pI2C_Handle) {
    if (pI2C_Handle->TxRxState == I2C_BUSY_IN_TX) {
        I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr, I2C_ADDRESS_PHASE_WRITE);
    } else {
        I2C_ExecuteAddressPhase(pI2C_Handle->pI2Cx, pI2C_Handle->DevAddr, I2C_ADDRESS_PHASE_READ);
    }
}

static void I2C_IRQ_Handle_BTF(I2C_Handle_t *pI2C_Handle) {
    if (I2C_GetFlagStatus(pI2C_Handle->pI2Cx, I2C_FLAG_TXE) &&
        pI2C_Handle->TxLen == 0 && pI2C_Handle->TxRxState == I2C_BUSY_IN_TX) {
        // 1. generate stop condition
        if (pI2C_Handle->Sr == I2C_DISABLE_SR) {
            I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
        }

        // 2. reset all member elements of handle structure
        I2C_CloseSendData(pI2C_Handle);

        // 3. inform the application about complete transmission
        I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_TX_CMPLT);
    }
}

static void I2C_IRQ_Handle_STOPF(I2C_Handle_t *pI2C_Handle) {
    // clearing STOPF bit involves reading SR1 and writing to CR1
    pI2C_Handle->pI2Cx->CR1 |= 0;
    I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_STOP);
}

static void I2C_IRQ_Handle_TXE(I2C_Handle_t *pI2C_Handle) {
    if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL) &&
        pI2C_Handle->TxRxState == I2C_BUSY_IN_TX &&
        pI2C_Handle->TxLen > 0) {

        pI2C_Handle->pI2Cx->DR = *pI2C_Handle->pTxBuffer;
        pI2C_Handle->pTxBuffer++;
        pI2C_Handle->TxLen--;
    } else {
        if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)) {
            I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_DATA_REQ);
        }
    }
}

static void I2C_IRQ_Handle_RXNE(I2C_Handle_t *pI2C_Handle) {
    if (pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_MSL) && pI2C_Handle->TxRxState == I2C_BUSY_IN_RX) {
        if (pI2C_Handle->RxSize == 1) {
            *pI2C_Handle->pRxBuffer = pI2C_Handle->pI2Cx->DR;
            pI2C_Handle->RxLen--;
        }
        if (pI2C_Handle->RxSize > 1) {
            if (pI2C_Handle->RxLen == 2) {
                I2C_ManageAcking(pI2C_Handle->pI2Cx, I2C_ACK_DISABLE);
            }
            *pI2C_Handle->pRxBuffer = pI2C_Handle->pI2Cx->DR;
            pI2C_Handle->pRxBuffer++;
            pI2C_Handle->RxLen--;
        }
        if (pI2C_Handle->RxLen == 0) {
            // close data reception and notify application

            // generate stop condition
            if (pI2C_Handle->Sr == I2C_DISABLE_SR) {
                I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
            }

            // close I2C rx
            I2C_CloseReceiveData(pI2C_Handle);

            // notify application
            I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_RX_CMPLT);
        }
    } else {
        if (!(pI2C_Handle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))) {
            I2C_ApplicationEventCallback(pI2C_Handle, I2C_EVENT_DATA_RCV);
        }
    }
}

void I2C_IRQ_ErrorHandling(I2C_Handle_t *pI2C_Handle) {
    // TODO rewrite
    uint32_t tmp, iterren;

    iterren = (pI2C_Handle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

    if (iterren) {
        tmp = (pI2C_Handle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
        if(tmp) {
            // Bus error
            // Clear flag and notify application
            pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);
            I2C_ApplicationEventCallback(pI2C_Handle,I2C_ERROR_BERR);
        }

        tmp = (pI2C_Handle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
        if(tmp) {
            // Arbitration lost error
            // Clear flag and notify application
            pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);
            I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERROR_ARLO);
        }

        tmp = (pI2C_Handle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
        if(tmp) {
            // ACK failure error
            // Clear flag and notify application
            pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);
            I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERROR_AF);
        }

        tmp = (pI2C_Handle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
        if(tmp) {
            // Overrun/underrun
            // Clear flag and notify application
            pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);
            I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERROR_OVR);
        }

        tmp = (pI2C_Handle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
        if(tmp) {
            // Time out error
            // Clear flag and notify application
            pI2C_Handle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
            I2C_ApplicationEventCallback(pI2C_Handle, I2C_ERROR_TIMEOUT);
        }
    }
}

__attribute__((weak)) void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t event) {

}
