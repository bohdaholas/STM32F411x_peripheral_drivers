#ifndef DRIVER_DEVELOPMENT_STM32F4_USART_DRIVER_H
#define DRIVER_DEVELOPMENT_STM32F4_USART_DRIVER_H

#include "stm32f411x.h"

typedef struct {
    uint8_t          USART_Mode;
    uint32_t         USART_BaudRate;
    uint8_t          USART_NoOfStopBits;
    uint8_t          USART_WordLength;
    uint8_t          USART_ParityControl;
    uint8_t          USART_HWFlowControl;
} USART_Config_t;

typedef struct {
    USART_RegDef_t   *pUSARTx;
    USART_Config_t   USART_Config;
    uint8_t          *pTxBuffer;
    uint8_t          *pRxBuffer;
    uint32_t         TxLen;
    uint32_t         RxLen;
    uint8_t          TxState;
    uint8_t          RxState;
}USART_Handle_t;

#define USART_MODE_ONLY_TX 	        0
#define USART_MODE_ONLY_RX 	        1
#define USART_MODE_TXRX  	        2

#define USART_BAUD_RATE_1200        1200
#define USART_BAUD_RATE_2400        400
#define USART_BAUD_RATE_9600        9600
#define USART_BAUD_RATE_19200       19200
#define USART_BAUD_RATE_38400       38400
#define USART_BAUD_RATE_57600		57600
#define USART_BAUD_RATE_115200 		115200
#define USART_BAUD_RATE_230400 		230400
#define USART_BAUD_RATE_460800 		460800
#define USART_BAUD_RATE_921600 		921600
#define USART_BAUD_RATE_2M 			2000000
#define USART_BAUD_RATE_3M 			3000000

#define USART_PARITY_DISABLE        0
#define USART_PARITY_EN_EVEN        1
#define USART_PARITY_EN_ODD         2

#define USART_WORDLEN_8BITS         0
#define USART_WORDLEN_9BITS         1

#define USART_STOPBITS_0_5          1
#define USART_STOPBITS_1            0
#define USART_STOPBITS_1_5          3
#define USART_STOPBITS_2            2

#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

#define USART_FLAG_TXE 			    (1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		    (1 << USART_SR_RXNE)
#define USART_FLAG_TC 			    (1 << USART_SR_TC)

#define USART_READY                 0
#define USART_BUSY_IN_RX            1
#define USART_BUSY_IN_TX            2

#define USART_EVENT_TX_CMPLT        0
#define USART_EVENT_RX_CMPLT        1
#define USART_EVENT_IDLE            2
#define USART_EVENT_CTS             3
#define USART_EVENT_PE              4
#define USART_ERR_FE     	        5
#define USART_ERR_NE    	        6
#define USART_ERR_ORE    	        7

void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t baud_rate);
void USART_Init(USART_Handle_t *pUSART_Handle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

void USART_SendData(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint32_t len);
void USART_ReceiveData(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint32_t len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSART_Handle, uint8_t *pTxBuffer, uint32_t len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSART_Handle, uint8_t *pRxBuffer, uint32_t len);

void USART_PCLK_Control(USART_RegDef_t *pUSARTx, uint8_t en);
void USART_Peripheral_Control(USART_RegDef_t *pUSARTx, uint8_t en);
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t flagName);

void USART_IRQ_InterruptConfig(uint8_t IRQ_number, uint8_t en);
void USART_IRQ_PriorityConfig(uint8_t IRQ_number, uint32_t priority);

void USART_IRQHandling(USART_Handle_t *pUSART_Handle);
static void USART_IRQ_Handle_TC(USART_Handle_t *pUSART_Handle);
static void USART_IRQ_Handle_TXE(USART_Handle_t *pUSART_Handle);
static void USART_IRQ_Handle_RXNE(USART_Handle_t *pUSART_Handle);
static void USART_IRQ_Handle_CTS(USART_Handle_t *pUSART_Handle);
static void USART_IRQ_Handle_IDLE(USART_Handle_t *pUSART_Handle);
static void USART_IRQ_Handle_ORE(USART_Handle_t *pUSART_Handle);
static void USART_IRQ_Handle_FE(USART_Handle_t *pUSART_Handle);
static void USART_IRQ_Handle_NE(USART_Handle_t *pUSART_Handle);

__attribute__((weak)) void USART_ApplicationEventCallback(USART_Handle_t *pUSART_Handle, uint8_t event);

#endif //DRIVER_DEVELOPMENT_STM32F4_USART_DRIVER_H
