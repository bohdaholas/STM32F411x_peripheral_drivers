#include <string.h>
#include "../../drivers/inc/stm32f411x.h"

/*
 * PB6 -> USART1 TX
 * PB7 -> USART1 RX
 * AF7
 */

#define BTN_PRESSED     1
#define BTN_RELEASED    0

USART_Handle_t USART_Handle;

void delay() {
    for (int i = 0; i < 1000000; ++i);
}

void USART1_GPIO_Inits() {
    GPIO_Handle_t USART_Pin;

    USART_Pin.pGPIOx = GPIOB;
    USART_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
    USART_Pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    USART_Pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    USART_Pin.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
    USART_Pin.GPIO_PinConfig.GPIO_PinAltFunMode = AF7;

    // TX
    USART_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&USART_Pin);

    // RX
    USART_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&USART_Pin);
}

void USART_Inits() {
    USART_Handle.pUSARTx = USART1;

    USART_Handle.USART_Config.USART_BaudRate = USART_BAUD_RATE_115200;
    USART_Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    USART_Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    USART_Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    USART_Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    USART_Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(&USART_Handle);
}

void UserButtonInit() {
    GPIO_Handle_t PA0_UserButton;

    PA0_UserButton.pGPIOx = GPIOA;
    PA0_UserButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    PA0_UserButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    PA0_UserButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    PA0_UserButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
    PA0_UserButton.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;

    GPIO_Init(&PA0_UserButton);
}

int main(void) {
    USART1_GPIO_Inits();
    USART_Inits();
    USART_Peripheral_Control(USART_Handle.pUSARTx, ENABLE);
    UserButtonInit();
    USART_IRQ_InterruptConfig(IRQ_NO_USART1, ENABLE);

    const char *data[] = {
            "Hi there!\n",
            "Hello!\n",
            "How are you?\n",
    };
    const size_t n = 3;
    const uint32_t lens[] = {strlen(data[0]), strlen(data[1]), strlen(data[2])};
    size_t i = 0;

    char buff[255];

    for (;;) {
        while (GPIO_ReadPin(GPIOA, GPIO_PIN_NO_0) == BTN_RELEASED);
        delay();

        while (USART_SendDataIT(&USART_Handle, (uint8_t *) data[i], lens[i]) != USART_READY);

        while (USART_ReceiveDataIT(&USART_Handle, (uint8_t *) buff, lens[i]) != USART_READY);

        i = (i + 1) % n;
    }
}

void USART1_IRQHandler() {
    USART_IRQHandling(&USART_Handle);
}