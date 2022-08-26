#include "../inc/stm32f411x.h"

void SPI_Init(SPI_Handle_t *pSPI_Handle) {
    SPI_PCLK_Control(pSPI_Handle->pSPIx, ENABLE);

    uint32_t tmp = 0;

    // configure device mode: master or slave
    tmp |= pSPI_Handle->SPIConfig.DeviceMode << SPI_CR1_MSTR;

    // configure the bus: full-duplex, half-duplex, simplex rx only
    if (pSPI_Handle->SPIConfig.BusConfig == SPI_BUS_CFG_FD) {
        tmp &= ~(1 << SPI_CR1_BIDI_MODE);
    } else if (pSPI_Handle->SPIConfig.BusConfig == SPI_BUS_CFG_HD) {
        tmp |= (1 << SPI_CR1_BIDI_MODE);
    } else if (pSPI_Handle->SPIConfig.BusConfig == SPI_BUS_CFG_SIMPLEX_RXONLY) {
        tmp &= ~(1 << SPI_CR1_BIDI_MODE);
        tmp |= (1 << SPI_CR1_RX_ONLY);
    }

    // configure SCLK
    tmp |= pSPI_Handle->SPIConfig.SclkSpeed << SPI_CR1_BR;

    // configure SSM
    tmp |= pSPI_Handle->SPIConfig.SSM << SPI_CR1_SSM;

    // configure CPOL
    tmp |= pSPI_Handle->SPIConfig.CPOL << SPI_CR1_CPOL;

    // configure CPHA
    tmp |= pSPI_Handle->SPIConfig.CPHA << SPI_CR1_CPHA;

    // configure data frame format
    tmp |= pSPI_Handle->SPIConfig.DFF << SPI_CR1_DFF;

    pSPI_Handle->pSPIx->CR1 = tmp;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
    if (pSPIx == SPI1) {
        SPI1_REG_RESET();
    } else if (pSPIx == SPI2) {
        SPI2_REG_RESET();
    } else if (pSPIx == SPI3) {
        SPI3_REG_RESET();
    } else {
        SPI4_REG_RESET();
    }
}

void SPI_PCLK_Control(SPI_RegDef_t *pSPIx, uint8_t en) {
    if (en == ENABLE) {
        if (pSPIx == SPI1) {
            SPI1_PCLK_EN();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_EN();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_EN();
        } else {
            SPI4_PCLK_EN();
        }
    } else {
        if (pSPIx == SPI1) {
            SPI1_PCLK_DI();
        } else if (pSPIx == SPI2) {
            SPI2_PCLK_DI();
        } else if (pSPIx == SPI3) {
            SPI3_PCLK_DI();
        } else {
            SPI4_PCLK_DI();
        }
    }
}

void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t en) {
    if (en == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
    }
}

void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t en) {
    // setting SSI bit is needed to prevent Master mode fault (MODF)
    // while NSS is managed by software
    // SSI = NSS
    if (en == ENABLE) {
        pSPIx->CR1 |= (1 << SPI_CR1_SSI);
    } else {
        pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
    }
}

void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t en) {
    if (en == ENABLE) {
        pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
    } else {
        pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
    }
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName) {
    uint8_t flagStatus = pSPIx->SR & flagName;
    if (flagStatus) {
        return SPI_FLAG_SET;
    }
    return SPI_FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {
    while (len) {
        while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == SPI_FLAG_RESET); // wait until tx buffer is empty
        uint8_t dataSize = (pSPIx->CR1 >> SPI_CR1_DFF) & 1;
        if (dataSize == SPI_DFF_8BITS) {
            pSPIx->DR = *pTxBuffer;
            pTxBuffer++;
            len--;
        }
        if (dataSize == SPI_DFF_16BITS) {
            pSPIx->DR = *((uint16_t *) pTxBuffer);
            pTxBuffer += 2;
            len -= 2;
        }
    }
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {
    while (len) {
        while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == SPI_FLAG_RESET); // wait until rx buffer is non-empty
        uint8_t dataSize = (pSPIx->CR1 >> SPI_CR1_DFF) & 1;
        if (dataSize == SPI_DFF_8BITS) {
            *pRxBuffer = pSPIx->DR;
            pRxBuffer++;
            len--;
        }
        if (dataSize == SPI_DFF_16BITS) {
            *((uint16_t *) pRxBuffer) = pSPIx->DR;
            pRxBuffer += 2;
            len -= 2;
        }
    }
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t len) {
    uint8_t state = pSPI_Handle->TxState;
    if (state != SPI_BUSY_IN_TX) {
        pSPI_Handle->pTxBuffer = pTxBuffer;
        pSPI_Handle->TxLen = len;

        pSPI_Handle->TxState = SPI_BUSY_IN_TX;

        pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
    }
    return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t len) {
    uint8_t state = pSPI_Handle->RxState;
    if (state != SPI_BUSY_IN_RX) {
        pSPI_Handle->pRxBuffer = pRxBuffer;
        pSPI_Handle->RxLen = len;

        pSPI_Handle->RxState = SPI_BUSY_IN_RX;

        pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
    }
    return state;
}

void SPI_IRQ_InterruptConfig(uint8_t IRQ_number, uint8_t en) {
    if (en == ENABLE) {
        *NVIC_ISER |= (1 << IRQ_number);
    } else {
        *NVIC_ICER &= ~(1 << IRQ_number);
    }
}

void SPI_IRQ_PriorityConfig(uint8_t IRQ_number, uint32_t priority) {
    uint8_t iprx = IRQ_number / 4;
    uint8_t prix = IRQ_number % 4;

    uint8_t shift_amount = (8 * prix) + (8 - NO_BITS_IMPLEMENTED);
    *(NVIC_PR_BASEADDR + 4 * iprx) |= (priority << shift_amount);
}

void SPI_IRQ_Handling(SPI_Handle_t *pSPI_Handle) {
    uint8_t tmp1, tmp2;

    tmp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_TXE);
    tmp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
    if (tmp1 && tmp2) {
        spi_txe_interrupt_handle(pSPI_Handle);
    }

    tmp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_RXNE);
    tmp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
    if (tmp1 && tmp2) {
        spi_rxne_interrupt_handle(pSPI_Handle);
    }

    tmp1 = pSPI_Handle->pSPIx->SR & (1 << SPI_SR_OVR);
    tmp2 = pSPI_Handle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
    if (tmp1 && tmp2) {
        spi_ovr_err_interrupt_handle(pSPI_Handle);
    }
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPI_Handle) {
    uint8_t dataSize = (pSPI_Handle->pSPIx->CR1 >> SPI_CR1_DFF) & 1;
    if (dataSize == SPI_DFF_8BITS) {
        pSPI_Handle->pSPIx->DR = *pSPI_Handle->pTxBuffer;
        pSPI_Handle->pTxBuffer++;
        pSPI_Handle->TxLen--;
    }
    if (dataSize == SPI_DFF_16BITS) {
        pSPI_Handle->pSPIx->DR = *((uint16_t *) pSPI_Handle->pTxBuffer);
        pSPI_Handle->pTxBuffer += 2;
        pSPI_Handle->TxLen -= 2;
    }
    if (pSPI_Handle->TxLen == 0) {
        SPI_CloseTransmission(pSPI_Handle);
        SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_TX_CMPLT);
    }
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPI_Handle) {
    uint8_t dataSize = (pSPI_Handle->pSPIx->CR1 >> SPI_CR1_DFF) & 1;
    if (dataSize == SPI_DFF_8BITS) {
        *pSPI_Handle->pRxBuffer = pSPI_Handle->pSPIx->DR;
        pSPI_Handle->pRxBuffer++;
        pSPI_Handle->RxLen--;
    }
    if (dataSize == SPI_DFF_16BITS) {
        *((uint16_t *) pSPI_Handle->pRxBuffer) = pSPI_Handle->pSPIx->DR;
        pSPI_Handle->pRxBuffer += 2;
        pSPI_Handle->RxLen -= 2;
    }
    if (pSPI_Handle->RxLen == 0) {
        SPI_CloseReception(pSPI_Handle);
        SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_RX_CMPLT);
    }
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPI_Handle) {
    uint8_t tmp;
    if (pSPI_Handle->TxState != SPI_BUSY_IN_TX) {
        // procedure to clear ovr flag
        tmp = pSPI_Handle->pSPIx->DR;
        tmp = pSPI_Handle->pSPIx->SR;
        (void) tmp; // get rid of unused variable warning
    }
    // inform the application
    SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_OVR_ERR);
}

void SPI_ClearOVRFlag(SPI_Handle_t *pSPI_Handle) {
    uint8_t tmp;
    // procedure to clear ovr flag
    tmp = pSPI_Handle->pSPIx->DR;
    tmp = pSPI_Handle->pSPIx->SR;
    (void) tmp; // get rid of unused variable warning
}

void SPI_CloseTransmission(SPI_Handle_t *pSPI_Handle) {
    pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
    pSPI_Handle->pTxBuffer = NULL;
    pSPI_Handle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPI_Handle) {
    pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
    pSPI_Handle->pRxBuffer = NULL;
    pSPI_Handle->RxState = SPI_READY;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t event) {

}
