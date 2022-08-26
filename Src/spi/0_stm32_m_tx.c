#include <stdint.h>
#include "../../drivers/inc/stm32f411x.h"

/*
 * PB12 -> NSS
 * PB13 -> SCK
 * PB14 -> MISO
 * PB15 -> MOSI
 * AF5
 */

void delay() {
    for (int i = 0; i < 1000; ++i);
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
//    SPI_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//    GPIO_Init(&SPI_Pin);
//
//    // NSS
//    SPI_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
//    GPIO_Init(&SPI_Pin);
}

void SPI2_Inits() {
    SPI_Handle_t SPI2_Handle;

    SPI2_Handle.pSPIx = SPI2;
    SPI2_Handle.SPIConfig.BusConfig = SPI_BUS_CFG_FD;
    SPI2_Handle.SPIConfig.DeviceMode = SPI_DEVICE_MODE_MASTER;
    SPI2_Handle.SPIConfig.SclkSpeed = SPI_SCLK_SPEED_DIV256;
    SPI2_Handle.SPIConfig.DFF = SPI_DFF_8BITS;
    SPI2_Handle.SPIConfig.CPOL = SPI_CPOL_LOW;
    SPI2_Handle.SPIConfig.CPHA = SPI_CPHA_LOW;
    SPI2_Handle.SPIConfig.SSM = SPI_SSM_EN;

    SPI_Init(&SPI2_Handle);
}

int main(void)
{
    SPI2_GPIO_Inits();
    SPI2_Inits();
//    SPI_SSI_Config(SPI2, ENABLE);
    SPI_SSOE_Config(SPI2, ENABLE);

    SPI_Peripheral_Control(SPI2, ENABLE);

//    delay();

    uint8_t data[] = "Hello world!";
    SPI_SendData(SPI2, data, sizeof(data));

//    while (SPI_GetFlagStatus(SPI2))
//
    SPI_Peripheral_Control(SPI2, DISABLE);

    for(;;) {
    }
}
