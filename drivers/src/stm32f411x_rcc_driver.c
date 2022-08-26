#include "../inc/stm32f411x.h"

uint32_t RCC_GetSYSCLKValue() {
    uint8_t clk_src = (RCC->CFGR >> 2) & 0x3;
    if (clk_src == 0) {
        // HSI
        return 16000000;
    } else if(clk_src == 1) {
        // HSE
        return 8000000;
    } else {
        // PLL
        return RCC_GetPLLOutputClock();
    }
}

uint32_t RCC_GetAHBPrescaler() {
    uint8_t tmp;
    tmp = (RCC->CFGR >> 4 ) & 0xF;
    uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
    if(tmp < 8) {
        return 1;
    } else {
        return AHB_PreScaler[tmp - 8];
    }
}

uint32_t RCC_GetAPBxPrescaler(uint8_t PREx) {
    uint8_t tmp;
    tmp = (RCC->CFGR >> PREx) & 0x7;
    uint8_t APBx_PreScaler[] = {2, 4, 8, 16};
    if (tmp < 4) {
        return 1;
    } else {
        return APBx_PreScaler[tmp - 4];
    }
}

uint32_t RCC_GetPCLKxValue(uint8_t PREx) {
    // Get frequency of peripheral clock (APB1, APB2)
    uint32_t pclkx, system_clk;
    uint8_t ahb_prescaler, apbx_prescaler;

    system_clk = RCC_GetSYSCLKValue();
    ahb_prescaler = RCC_GetAHBPrescaler();
    apbx_prescaler = RCC_GetAPBxPrescaler(PREx);

    pclkx = (system_clk / ahb_prescaler) / apbx_prescaler;

    return pclkx;
}

uint32_t RCC_GetPLLOutputClock() {
    // not implemented
    return 0;
}
