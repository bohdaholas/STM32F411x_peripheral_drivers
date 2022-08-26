#include <string.h>
#include "../../drivers/inc/stm32f411x.h"

#define BTN_PRESSED     1
#define FCLK            4000000

int nxt = 1;
int LEDS[] = {GPIO_PIN_NO_12, GPIO_PIN_NO_13, GPIO_PIN_NO_14, GPIO_PIN_NO_15};

void turn_off_all();
void delay(int delay_ms);

int main(void) {
    // configure leds

    GPIO_Handle_t LED;
    memset(&LED, 0, sizeof(LED));

    LED.pGPIOx = GPIOD;
    LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    LED.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
    LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&LED);

    LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&LED);

    LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&LED);

    LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&LED);

    // configure button

    GPIO_Handle_t PA0_USER_BUTTON;
    memset(&PA0_USER_BUTTON, 0, sizeof(PA0_USER_BUTTON));

    PA0_USER_BUTTON.pGPIOx = GPIOA;
    PA0_USER_BUTTON.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    PA0_USER_BUTTON.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    PA0_USER_BUTTON.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    PA0_USER_BUTTON.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;

    GPIO_Init(&PA0_USER_BUTTON);

    size_t i = 0;

    for(;;) {
        if (GPIO_ReadPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED) {
            if (nxt == 1) {
                nxt = -1;
            } else {
                nxt = 1;
            }
        } else {
            turn_off_all();
            i += nxt;
            GPIO_WritePin(GPIOD, LEDS[i % 4], GPIO_PIN_SET);
        }
        delay(50);
    }
}

void turn_off_all() {
    for (int i = 0; i < 4; ++i) {
        GPIO_WritePin(GPIOD, LEDS[i], GPIO_PIN_RESET);
    }
}

void delay(int delay_ms) {
    for (int i = 0; i < (FCLK * delay_ms) / 1000; ++i);
}
