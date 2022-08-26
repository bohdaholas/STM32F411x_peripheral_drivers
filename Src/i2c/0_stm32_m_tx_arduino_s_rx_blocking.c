#include <string.h>
#include "../../drivers/inc/stm32f411x.h"

/*
 * PB6 -> I2C1 SCL
 * PB9 -> I2C1 SDA
 * AF4
 */

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

int main(void)
{
    I2C1_GPIO_Inits();
    I2C_Inits();
    I2C_Peripheral_Control(I2C_Handle.pI2Cx, ENABLE);
    UserButtonInit();

    const char data[] = "Hello Arduino!";
    const uint32_t len = strlen(data);

    for(;;) {
        while (GPIO_ReadPin(GPIOA, GPIO_PIN_NO_0) == BTN_RELEASED);
        delay();

        I2C_MasterSendData(&I2C_Handle, (uint8_t *) data, len, 0x68, I2C_DISABLE_SR);
    }
}
