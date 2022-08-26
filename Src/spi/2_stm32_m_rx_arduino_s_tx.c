#include <string.h>
#include "../../drivers/inc/stm32f411x.h"

/*
 * PB12 -> NSS
 * PB13 -> SCLK
 * PB14 -> MISO
 * PB15 -> MOSI
 * AF0
 */

#define BTN_PRESSED     1
#define BTN_RELEASED    0

//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

#define LED_PIN         9

uint8_t SPI_VerifyResponse(uint8_t ack_byte) {
    if (ack_byte == 0xF5)  {
        return 1;
    }
    return 0;
}

void delay() {
    for (int i = 0; i < 1000000; ++i);
}

void SPI2_GPIO_Inits() {
    GPIO_Handle_t SPI_Pin;

    SPI_Pin.pGPIOx = GPIOB;
    SPI_Pin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT;
    SPI_Pin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    SPI_Pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPI_Pin.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
    SPI_Pin.GPIO_PinConfig.GPIO_PinAltFunMode = AF5;

    // SCLK
    SPI_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPI_Pin);

    // MOSI
    SPI_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPI_Pin);

//    // MISO
    SPI_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPI_Pin);

    // NSS
    SPI_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPI_Pin);

}

void SPI2_Inits() {
    SPI_Handle_t SPI2_Handle;

    SPI2_Handle.pSPIx = SPI2;
    SPI2_Handle.SPIConfig.BusConfig = SPI_BUS_CFG_FD;
    SPI2_Handle.SPIConfig.DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2_Handle.SPIConfig.SclkSpeed = SPI_SCLK_SPEED_DIV8;
    SPI2_Handle.SPIConfig.DFF = SPI_DFF_8BITS;
    SPI2_Handle.SPIConfig.CPOL = SPI_CPOL_LOW;
    SPI2_Handle.SPIConfig.CPHA = SPI_CPHA_LOW;
    SPI2_Handle.SPIConfig.SSM = SPI_SSM_DI;

    SPI_Init(&SPI2_Handle);
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

void UserLedInit() {
    GPIO_Handle_t PD13_LED;

    PD13_LED.pGPIOx = GPIOD;
    PD13_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    PD13_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    PD13_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    PD13_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    PD13_LED.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;

    GPIO_Init(&PD13_LED);
}

int main(void)
{
    SPI2_GPIO_Inits();
    SPI2_Inits();
    SPI_SSOE_Config(SPI2, ENABLE);
    UserButtonInit();
    UserLedInit();

    uint8_t dummy_write = 0xFF;
    uint8_t dummy_read;
    for(;;) {
        while (GPIO_ReadPin(GPIOA, GPIO_PIN_NO_0) == BTN_RELEASED);

        GPIO_TogglePin(GPIOD, GPIO_PIN_NO_13);
        delay();
        GPIO_TogglePin(GPIOD, GPIO_PIN_NO_13);

        SPI_Peripheral_Control(SPI2, ENABLE);

        uint8_t command_code = COMMAND_LED_CTRL;
        SPI_SendData(SPI2, &command_code, 1);
        SPI_ReceiveData(SPI2, &dummy_read, 1);

        // sending dummy byte in order to fetch ACK/NACK response on MISO line
        SPI_SendData(SPI2, &dummy_write, 1);
        uint8_t ack_byte;
        SPI_ReceiveData(SPI2, &ack_byte, 1);

        uint8_t args[2];
        if (SPI_VerifyResponse(ack_byte)) {
            args[0] = LED_PIN;
            args[1] = LED_ON;
            SPI_SendData(SPI2, args, 2);
        }

        while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) == SPI_FLAG_SET);
        SPI_Peripheral_Control(SPI2, DISABLE);
    }
}
