#ifndef DRIVER_DEVELOPMENT_STM32F4_I2C_DRIVER_H
#define DRIVER_DEVELOPMENT_STM32F4_I2C_DRIVER_H

#include "stm32f411x.h"

#define I2C_SCL_SPEED_SM_100kHz 	100000
#define I2C_SCL_SPEED_FM_200kHz     200000
#define I2C_SCL_SPEED_FM_400kHz 	400000

#define I2C_ACK_DISABLE                 0
#define I2C_ACK_ENABLE                  1

#define I2C_FM_DUTY_2                   0
#define I2C_FM_DUTY_16_9                1

#define I2C_WRITE_OPERATION             0
#define I2C_READ_OPERATION              1

// I2C states
#define I2C_READY                       0
#define I2C_BUSY_IN_TX                  1
#define I2C_BUSY_IN_RX                  2

// I2C Flags in SR1 register

#define I2C_FLAG_SB                     (1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR                   (1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF                    (1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10                  (1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF                  (1 << I2C_SR1_STOPF)
#define I2C_FLAG_RXNE                   (1 << I2C_SR1_RXNE)
#define I2C_FLAG_TXE                    (1 << I2C_SR1_TXE)
#define I2C_FLAG_BERR                   (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO                   (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF                     (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR                    (1 << I2C_SR1_OVR)
#define I2C_FLAG_PEC                    (1 << I2C_SR1_PEC_ERR)
#define I2C_FLAG_TIMEOUT                (1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMB                    (1 << I2C_SR1_SMB_ALERT)

// Possible events and errors

#define I2C_EVENT_TX_CMPLT  	 	0
#define I2C_EVENT_RX_CMPLT  	 	1
#define I2C_EVENT_STOP       		2
#define I2C_ERROR_BERR 	 		    3
#define I2C_ERROR_ARLO  		    4
#define I2C_ERROR_AF    		    5
#define I2C_ERROR_OVR   		    6
#define I2C_ERROR_TIMEOUT 		    7
#define I2C_EVENT_DATA_REQ          8
#define I2C_EVENT_DATA_RCV          9

// Address phase actions

#define I2C_ADDRESS_PHASE_WRITE     0
#define I2C_ADDRESS_PHASE_READ      1

// Repeated start macros

#define I2C_DISABLE_SR  	        RESET
#define I2C_ENABLE_SR   	        SET

typedef struct {
    uint32_t      I2C_SCLSpeed;
    uint8_t       I2C_DeviceAddress;
    uint8_t       I2C_ACKControl;
    uint16_t      I2C_FMDutyCycle;
} I2C_Config_t;

typedef struct {
    I2C_RegDef_t  *pI2Cx;
    I2C_Config_t  I2C_Config;
    uint8_t       *pTxBuffer;
    uint8_t       *pRxBuffer;
    uint32_t      TxLen;
    uint32_t      RxLen;
    uint8_t       TxRxState;
    uint8_t       DevAddr;
    uint32_t      RxSize;
    uint8_t       Sr;
} I2C_Handle_t;

void I2C_Init(I2C_Handle_t *pI2C_Handle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t en);
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t rw_bit);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2C_Handle);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t en);

void I2C_PCLK_Control(I2C_RegDef_t *pI2Cx, uint8_t en);
void I2C_Peripheral_Control(I2C_RegDef_t *pI2Cx, uint8_t en);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t flagName);

void I2C_IRQ_InterruptConfig(uint8_t IRQ_number, uint8_t en);
void I2C_IRQ_PriorityConfig(uint8_t IRQ_number, uint32_t priority);

void I2C_IRQ_EventHandling(I2C_Handle_t *pI2C_Handle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
static void I2C_IRQ_Handle_SB(I2C_Handle_t *pI2C_Handle);
static void I2C_IRQ_Handle_BTF(I2C_Handle_t *pI2C_Handle);
static void I2C_IRQ_Handle_STOPF(I2C_Handle_t *pI2C_Handle);
static void I2C_IRQ_Handle_TXE(I2C_Handle_t *pI2C_Handle);
static void I2C_IRQ_Handle_RXNE(I2C_Handle_t *pI2C_Handle);

void I2C_IRQ_ErrorHandling(I2C_Handle_t *pI2C_Handle);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t event);

#endif //DRIVER_DEVELOPMENT_STM32F4_I2C_DRIVER_H
