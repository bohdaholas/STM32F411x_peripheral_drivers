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
    SPI_Pin.GPIO_PinConfig.GPIO_PinAltFunMode = AF0;

    // SCLK
    SPI_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPI_Pin);

    // MOSI
    SPI_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPI_Pin);

//    // MISO
//    SPI_Pin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//    GPIO_Init(&SPI_Pin);

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

int main(void)
{
    SPI2_GPIO_Inits();
    SPI2_Inits();
    SPI_SSOE_Config(SPI2, ENABLE);
    UserButtonInit();
//    GPIO_IRQ_InterruptConfig(IRQ_NO_EXTI0_1, ENABLE);

    for(;;) {
        while (GPIO_ReadPin(GPIOA, GPIO_PIN_NO_0) == BTN_RELEASED);

        delay();

        const char data[] = "Hello Arduino!";
        const uint8_t len = strlen(data);

        SPI_Peripheral_Control(SPI2, ENABLE);
        SPI_SendData(SPI2, (uint8_t *) &len, 1);
        SPI_SendData(SPI2, (uint8_t *) data, len);
        while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) == SPI_FLAG_SET);
        SPI_Peripheral_Control(SPI2, DISABLE);
    }
}

//void EXTI0_1_IRQHandler() {
//    GPIO_IRQ_Handling(IRQ_NO_EXTI0_1);
//
//    SPI_Peripheral_Control(SPI2, ENABLE);
//
//    SPI_SendData(SPI2, (uint8_t *) &len, 1);
//    SPI_SendData(SPI2, (uint8_t *) data, sizeof(data));
//
//    while (SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));
//    SPI_Peripheral_Control(SPI2, DISABLE);
//}


