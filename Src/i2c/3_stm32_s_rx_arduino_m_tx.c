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
    I2C_Handle.I2C_Config.I2C_DeviceAddress = 0x68;
    I2C_Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM_100kHz;

    I2C_Init(&I2C_Handle);
}

int main(void) {
    I2C1_GPIO_Inits();
    I2C_Inits();

    I2C_IRQ_InterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQ_InterruptConfig(IRQ_NO_I2C1_ERR, ENABLE);

    I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

    I2C_Peripheral_Control(I2C_Handle.pI2Cx, ENABLE);

    I2C_ManageAcking(I2C_Handle.pI2Cx, ENABLE);

    for (;;) {
    }
}

void I2C1_EV_IRQHandler() {
    I2C_IRQ_EventHandling(&I2C_Handle);
}

void I2C1_ER_IRQHandler() {
    I2C_IRQ_ErrorHandling(&I2C_Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle, uint8_t event) {
    static const char data[] = "Hello Arduino!";
    static uint8_t i = 0;
    static uint8_t command_code;

    if (event == I2C_EVENT_DATA_RCV) {
        command_code = I2C_SlaveReceiveData(pI2C_Handle->pI2Cx);
    } else if (event == I2C_EVENT_DATA_REQ) {
        if (command_code == 0x51) {
            I2C_SlaveSendData(pI2C_Handle->pI2Cx, strlen(data));
        } else if (command_code == 0x52) {
            I2C_SlaveSendData(pI2C_Handle->pI2Cx, data[i++]);
        }
    } else if (event == I2C_ERROR_AF) {
        // TODO: ack failure not getting called
        command_code = 0xFF;
        i = 0;
    } else if (event == I2C_EVENT_STOP) {

    }
}
