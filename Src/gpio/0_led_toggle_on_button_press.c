#include <string.h>
#include "../../drivers/inc/stm32f411x.h"

#define FCLK            4000000

void delay(int delay_ms) {
    for (int i = 0; i < (FCLK * delay_ms) / 1000; ++i);
}

int main(void) {
    // configure led

    GPIO_Handle_t PD13_ORANGE_LED;
    memset(&PD13_ORANGE_LED, 0, sizeof(PD13_ORANGE_LED));

    PD13_ORANGE_LED.pGPIOx = GPIOD;
    PD13_ORANGE_LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    PD13_ORANGE_LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    PD13_ORANGE_LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    PD13_ORANGE_LED.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
    PD13_ORANGE_LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_Init(&PD13_ORANGE_LED);

    GPIO_WritePin(GPIOD, GPIO_PIN_NO_13, GPIO_PIN_SET);

    // configure button

    GPIO_Handle_t PA0_USER_BUTTON;
    memset(&PA0_USER_BUTTON, 0, sizeof(PA0_USER_BUTTON));

    PA0_USER_BUTTON.pGPIOx = GPIOA;
    PA0_USER_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    PA0_USER_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
    PA0_USER_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    PA0_USER_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;

    GPIO_IRQ_InterruptConfig(IRQ_NO_EXTI0, ENABLE);
    GPIO_Init(&PA0_USER_BUTTON);

    for(;;) {
    }
}

void EXTI0_IRQHandler() {
    delay(100);
    GPIO_IRQ_Handling(GPIO_PIN_NO_0);
    GPIO_TogglePin(GPIOD, GPIO_PIN_NO_13);
}














