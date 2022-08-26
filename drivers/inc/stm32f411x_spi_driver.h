#ifndef DRIVER_DEVELOPMENT_STM32F4_SPI_DRIVER_H
#define DRIVER_DEVELOPMENT_STM32F4_SPI_DRIVER_H

#include <stdint.h>
#include "stm32f411x.h"

// SPI CR1 values

#define SPI_DEVICE_MODE_SLAVE       0
#define SPI_DEVICE_MODE_MASTER      1

#define SPI_BUS_CFG_FD                  1
#define SPI_BUS_CFG_HD                  2
// #define SPI_BUS_CFG_SIMPLEX_TXONLY      3    ---     this is just full duplex without MISO
#define SPI_BUS_CFG_SIMPLEX_RXONLY      3

#define SPI_SCLK_SPEED_DIV2             0b000
#define SPI_SCLK_SPEED_DIV4             0b001
#define SPI_SCLK_SPEED_DIV8             0b010
#define SPI_SCLK_SPEED_DIV16            0b011
#define SPI_SCLK_SPEED_DIV32            0b100
#define SPI_SCLK_SPEED_DIV64            0b101
#define SPI_SCLK_SPEED_DIV128           0b110
#define SPI_SCLK_SPEED_DIV256           0b111

#define SPI_DFF_8BITS                   0
#define SPI_DFF_16BITS                  1

#define SPI_CPOL_LOW                    0
#define SPI_CPOL_HIGH                   1

#define SPI_CPHA_LOW                    0
#define SPI_CPHA_HIGH                   1

#define SPI_SSM_EN                      1
#define SPI_SSM_DI                      0

// SPIx SR bit positions

#define SPI_RXNE_FLAG                    (1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG                     (1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG                  (1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG                     (1 << SPI_SR_UDR)
#define SPI_CRC_ERR_FLAG                 (1 << SPI_SR_CRC_ERR)
#define SPI_MODF_FLAG                    (1 << SPI_SR_MODF)
#define SPI_OVR_FLAG                     (1 << SPI_SR_OVR)
#define SPI_BSY_FLAG                     (1 << SPI_SR_BSY)
#define SPI_FRE_FLAG                     (1 << SPI_SR_FRE)
#define SPI_FRLVL_FLAG                   (1 << SPI_SR_FRLVL)
#define SPI_FTLVL_FLAG                   (1 << SPI_SR_FTLVL)

// SPI states
#define SPI_READY           0
#define SPI_BUSY_IN_TX      1
#define SPI_BUSY_IN_RX      2

// Events

#define SPI_EVENT_TX_CMPLT          1
#define SPI_EVENT_RX_CMPLT          2
#define SPI_EVENT_OVR_ERR           3
#define SPI_EVENT_CRC_ERR           3

typedef struct {
    uint8_t DeviceMode;
    uint8_t BusConfig;
    uint8_t SclkSpeed;
    uint8_t DFF;
    uint8_t CPOL;
    uint8_t CPHA;
    uint8_t SSM;
} SPI_Config_t;

typedef struct {
    SPI_RegDef_t    *pSPIx;
    SPI_Config_t    SPIConfig;
    uint8_t         *pTxBuffer;
    uint8_t         *pRxBuffer;
    uint32_t        TxLen;
    uint32_t        RxLen;
    uint8_t         TxState;
    uint8_t         RxState;
} SPI_Handle_t;

void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_PCLK_Control(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_Peripheral_Control(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SSI_Config(SPI_RegDef_t *pSPIx, uint8_t en);
void SPI_SSOE_Config(SPI_RegDef_t *pSPIx, uint8_t en);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t len);

void SPI_IRQ_InterruptConfig(uint8_t IRQ_number, uint8_t en);
void SPI_IRQ_PriorityConfig(uint8_t IRQ_number, uint32_t priority);
void SPI_IRQ_Handling(SPI_Handle_t *pSPI_Handle);
void SPI_ClearOVRFlag(SPI_Handle_t *pSPI_Handle);
void SPI_CloseTransmission(SPI_Handle_t *pSPI_Handle);
void SPI_CloseReception(SPI_Handle_t *pSPI_Handle);

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPI_Handle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPI_Handle, uint8_t event);

#endif //DRIVER_DEVELOPMENT_STM32F4_SPI_DRIVER_H
