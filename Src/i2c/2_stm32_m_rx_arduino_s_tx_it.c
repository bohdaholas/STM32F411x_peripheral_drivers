#include <string.h>
#include "../../drivers/inc/stm32f411x.h"

/*
 * PB6 -> I2C1 SCL
 * PB9 -> I2C1 SDA
 * AF4
 */

uint8_t rxComplete = RESET;
char buffer[255];

#define BTN_PRESSED     1
#define BTN_RELEASED    0

I2C_Handle_t I2C_Handle;

void delay() {
    for (int i = 0; i < 1000000; ++i);
}

void I2C1_GPIO_Inits() {
    GPIO_Handle_t I2C_Pin;

    I2C_Pin.pGPIOx = GPIOB;
    I2C_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
    I2C_Pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    I2C_Pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    I2C_Pin.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
    I2C_Pin.GPIO_PinConfig.GPIO_PinAltFunMode = AF4;

    // SCL
    I2C_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2C_Pin);

    // SDA
    I2C_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    GPIO_Init(&I2C_Pin);
}

void I2C_Inits() {
    I2C_Handle.pI2Cx = I2C1;

    I2C_Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    I2C_Handle.I2C_Config.I2C_DeviceAddress = 0x61;
    I2C_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM_100kHz;

    I2C_Init(&I2C_Handle);
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
    I2C1_GPIO_Inits();
    I2C_Inits();

    I2C_IRQ_InterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQ_InterruptConfig(IRQ_NO_I2C1_ERR, ENABLE);

    I2C_Peripheral_Control(I2C_Handle.pI2Cx, ENABLE);

    UserButtonInit();

    uint8_t read_len_command = 0x51;
    uint8_t read_data_command = 0x52;

    uint8_t len;

    for (;;) {
        while (GPIO_ReadPin(GPIOA, GPIO_PIN_NO_0) == BTN_RELEASED);
        delay();

        // TODO
        while (I2C_MasterSendDataIT(&I2C_Handle, &read_len_command, 1, 0x68, I2C_ENABLE_SR) != I2C_READY);
        while (I2C_MasterReceiveDataIT(&I2C_Handle, &len, 1, 0x68, I2C_ENABLE_SR) != I2C_READY);

        while (I2C_MasterSendDataIT(&I2C_Handle, &read_data_command, 1, 0x68, I2C_ENABLE_SR) != I2C_READY);
        while (I2C_MasterReceiveDataIT(&I2C_Handle, (uint8_t *) buffer, len, 0x68, I2C_DISABLE_SR) != I2C_READY);

        rxComplete = RESET;

        while (rxComplete != SET);
        int a = 10;
        (void) a;
    }
}

void I2C1_EV_IRQHandler() {
    I2C_IRQ_EventHandling(&I2C_Handle);
}

void I2C1_ER_IRQHandler() {
    I2C_IRQ_ErrorHandling(&I2C_Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t event) {
    // TODO
    if (event == I2C_EVENT_TX_CMPLT) {

    } else if (event == I2C_EVENT_RX_CMPLT) {
        rxComplete = SET;
    } else if (event == I2C_ERROR_AF) {
        I2C_CloseSendData(pI2C_Handle);
        I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
    }
}
