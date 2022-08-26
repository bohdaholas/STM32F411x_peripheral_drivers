#ifndef DRIVER_DEVELOPMENT_STM32F4_H
#define DRIVER_DEVELOPMENT_STM32F4_H

#include <stdint.h>
#include <stddef.h>

/* ARM Cortex M4 specific info */

#define NVIC_ISER              ((volatile uint32_t *) 0xE000E100)
#define NVIC_ICER              ((volatile uint32_t *) 0xE000E180)
#define NVIC_PR_BASEADDR       ((volatile uint32_t *) 0xE000E400)

/* Board specific info */

#define NO_BITS_IMPLEMENTED    4

/* Memory base addresses */

#define FLASH_BASEADDR         0x08000000U
#define SRAM1_BASEADDR         0x20000000
#define SRAM                   SRAM1_BASE_ADDR
#define ROM                    0x1FFF0000

/* Busses base addresses */

#define PERIPH_BASEADDR        0x40000000
#define APB1_BASEADDR          PERIPH_BASEADDR
#define APB2_BASEADDR          0x40010000
#define AHB1_BASEADDR          0x40020000
#define AHB2_BASEADDR          0x50000000

/* Peripherals base addresses */

#define RCC_BASEADDR           (AHB1_BASEADDR + 0x3800)

#define GPIOA_BASEADDR         (AHB1_BASEADDR + 0x0000)
#define GPIOB_BASEADDR         (AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR         (AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR         (AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR         (AHB1_BASEADDR + 0x1000)
#define GPIOH_BASEADDR         (AHB1_BASEADDR + 0x1C00)

#define I2C1_BASEADDR          (APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR          (APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR          (APB1_BASEADDR + 0x5C00)

#define SPI1_BASEADDR          (APB2_BASEADDR + 0x3000)
#define SPI2_BASEADDR          (APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR          (APB1_BASEADDR + 0x3C00)
#define SPI4_BASEADDR          (APB2_BASEADDR + 0x3400)

#define USART1_BASEADDR        (APB2_BASEADDR + 0x1000)
#define USART2_BASEADDR        (APB1_BASEADDR + 0x4400)
#define USART6_BASEADDR        (APB2_BASEADDR + 0x1400)

#define EXTI_BASEADDR          (APB2_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR        (APB2_BASEADDR + 0x3800)

/* Registers structures for peripherals */

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFR[2];
} GPIO_RegDef_t;

typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t RESERVERD1[2];
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVERD2[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t RESERVERD3[2];
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
    volatile uint32_t RESERVERD4[2];
    volatile uint32_t AHB1LPENR;
    volatile uint32_t AHB2LPENR;
    volatile uint32_t RESERVERD5[2];
    volatile uint32_t APB1LPENR;
    volatile uint32_t APB2LPENR;
    volatile uint32_t RESERVERD6[2];
    volatile uint32_t BDCR;
    volatile uint32_t CSR;
    volatile uint32_t RESERVERD7[2];
    volatile uint32_t SSCGR;
    volatile uint32_t PLLI2SCFGR;
    volatile uint32_t DCKCFGR;
} RCC_RegDef_t;

typedef struct {
    volatile uint32_t IMR;
    volatile uint32_t EMR;
    volatile uint32_t RTSR;
    volatile uint32_t FTSR;
    volatile uint32_t SWIER;
    volatile uint32_t PR;
} EXTI_RegDef_t;

typedef struct {
    volatile uint32_t MEMRMP;
    volatile uint32_t PMC;
    volatile uint32_t EXTICR[4];
    volatile uint32_t CMPCR;
} SYSCFG_RegDef_t;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t CRCPR;
    volatile uint32_t RXCRCR;
    volatile uint32_t TXCRCR;
    volatile uint32_t I2SCFGR;
    volatile uint32_t I2SPR;
} SPI_RegDef_t;

typedef struct {
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t OAR1;
    volatile uint32_t OAR2;
    volatile uint32_t DR;
    volatile uint32_t SR1;
    volatile uint32_t SR2;
    volatile uint32_t CCR;
    volatile uint32_t TRISE;
    volatile uint32_t FLTR;
} I2C_RegDef_t;

typedef struct {
    volatile uint32_t SR;
    volatile uint32_t DR;
    volatile uint32_t BRR;
    volatile uint32_t CR1;
    volatile uint32_t CR2;
    volatile uint32_t CR3;
    volatile uint32_t GTPR;
} USART_RegDef_t;

/* Casted pointers of peripherals */

#define RCC                 ((RCC_RegDef_t *) RCC_BASEADDR)
#define EXTI                ((EXTI_RegDef_t *) EXTI_BASEADDR)
#define SYSCFG              ((SYSCFG_RegDef_t *) SYSCFG_BASEADDR)

#define GPIOA               ((GPIO_RegDef_t *) GPIOA_BASEADDR)
#define GPIOB               ((GPIO_RegDef_t *) GPIOB_BASEADDR)
#define GPIOC               ((GPIO_RegDef_t *) GPIOC_BASEADDR)
#define GPIOD               ((GPIO_RegDef_t *) GPIOD_BASEADDR)
#define GPIOE               ((GPIO_RegDef_t *) GPIOE_BASEADDR)
#define GPIOH               ((GPIO_RegDef_t *) GPIOH_BASEADDR)

#define SPI1                ((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2                ((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3                ((SPI_RegDef_t *) SPI3_BASEADDR)
#define SPI4                ((SPI_RegDef_t *) SPI4_BASEADDR)

#define I2C1                ((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2                ((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3                ((I2C_RegDef_t *) I2C3_BASEADDR)

#define USART1              ((USART_RegDef_t *) USART1_BASEADDR)
#define USART2              ((USART_RegDef_t *) USART2_BASEADDR)
#define USART6              ((USART_RegDef_t *) USART6_BASEADDR)

/* Enabling peripheral clocks macros */

#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= (1 << 7))

#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()          (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()          (RCC->APB1ENR |= (1 << 23))

#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()          (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()          (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()          (RCC->APB2ENR |= (1 << 13))

#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()        (RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()        (RCC->APB2ENR |= (1 << 5))

#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1 << 14))

/* Disabling clocks macros */

#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 7))

#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 23))

#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 13))

#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()        (RCC->APB1ENR &= ~(1 << 17))
#define USART6_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 5))

#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 14))

/* Macros to reset peripherals */

#define GPIOA_REG_RESET()         do { (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()         do { (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()         do { (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()         do { (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()         do { (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOH_REG_RESET()         do { (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)

#define SPI1_REG_RESET()          do { (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()          do { (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()          do { (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); } while(0)
#define SPI4_REG_RESET()          do { (RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13)); } while(0)

#define I2C1_REG_RESET()          do { (RCC->APB1RSTR |= (1 << 21)); (RCC->APB2RSTR &= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET()          do { (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); } while(0)
#define I2C3_REG_RESET()          do { (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); } while(0)

#define USART1_REG_RESET()        do { (RCC->APB1RSTR |= (1 << 4));  (RCC->APB2RSTR &= ~(1 << 4)); }  while(0)
#define USART2_REG_RESET()        do { (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); } while(0)
#define USART6_REG_RESET()        do { (RCC->APB1RSTR |= (1 << 5));  (RCC->APB1RSTR &= ~(1 << 5)); }  while(0)

/* Interrupts */

#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40
#define IRQ_NO_EXTI16           1
#define IRQ_NO_EXTI17           41
#define IRQ_NO_EXTI18           42
#define IRQ_NO_EXTI21           2
#define IRQ_NO_EXTI22           3

#define IRQ_NO_SPI1             35
#define IRQ_NO_SPI2             36
#define IRQ_NO_SPI3             51
#define IRQ_NO_SPI4             84

#define IRQ_NO_I2C1_EV          31
#define IRQ_NO_I2C1_ERR         32
#define IRQ_NO_I2C2_EV          33
#define IRQ_NO_I2C2_ERR         34
#define IRQ_NO_I2C3_EV          72
#define IRQ_NO_I2C3_ERR         73

#define IRQ_NO_USART1           37
#define IRQ_NO_USART2           38
#define IRQ_NO_USART6           71

/* Common macros */

#define ENABLE               1
#define DISABLE              0
#define SET                  ENABLE
#define RESET                DISABLE
#define GPIO_PIN_SET         SET
#define GPIO_PIN_RESET       RESET
#define SPI_FLAG_SET         SET
#define SPI_FLAG_RESET       RESET
#define I2C_FLAG_SET         SET
#define I2C_FLAG_RESET       RESET
#define USART_FLAG_SET        SET
#define USART_FLAG_RESET      RESET

/* Bit position definitions of the peripherals */

// SPI

#define SPI_CR1_CPHA                0
#define SPI_CR1_CPOL                1
#define SPI_CR1_MSTR                2
#define SPI_CR1_BR                  3
#define SPI_CR1_SPE                 6
#define SPI_CR1_LSB_FIRST           7
#define SPI_CR1_SSI                 8
#define SPI_CR1_SSM                 9
#define SPI_CR1_RX_ONLY             10
#define SPI_CR1_DFF                 11
#define SPI_CR1_CRC_NEXT            12
#define SPI_CR1_CRC_EN              13
#define SPI_CR1_BIDI_OE             14
#define SPI_CR1_BIDI_MODE           15

#define SPI_CR2_RXDMAEN             0
#define SPI_CR2_TXDMAEN             1
#define SPI_CR2_SSOE                2
#define SPI_CR2_FRF                 4
#define SPI_CR2_ERRIE               5
#define SPI_CR2_RXNEIE              6
#define SPI_CR2_TXEIE               7

#define SPI_SR_RXNE                 0
#define SPI_SR_TXE                  1
#define SPI_SR_CHSIDE               2
#define SPI_SR_UDR                  3
#define SPI_SR_CRC_ERR              4
#define SPI_SR_MODF                 5
#define SPI_SR_OVR                  6
#define SPI_SR_BSY                  7
#define SPI_SR_FRE                  8

// I2C

#define I2C_CR1_PE                          0
#define I2C_CR1_SMBUS                       1
#define I2C_CR1_SMB_TYPE                    3
#define I2C_CR1_ENARP                       4
#define I2C_CR1_ENPEC                       5
#define I2C_CR1_ENGC                        6
#define I2C_CR1_NO_STRETCH                  7
#define I2C_CR1_START                       8
#define I2C_CR1_STOP                        9
#define I2C_CR1_ACK                         10
#define I2C_CR1_POS                         11
#define I2C_CR1_PEC                         12
#define I2C_CR1_ALERT                       13
#define I2C_CR1_SWRST                       15

#define I2C_CR2_FREQ                        0
#define I2C_CR2_ITERREN                     8
#define I2C_CR2_ITEVTEN                     9
#define I2C_CR2_ITBUFEN                     10
#define I2C_CR2_DMAEN                       11
#define I2C_CR2_LAST                        12

#define I2C_SR1_SB                          0
#define I2C_SR1_ADDR                        1
#define I2C_SR1_BTF                         2
#define I2C_SR1_ADD10                       3
#define I2C_SR1_STOPF                       4
#define I2C_SR1_RXNE                        6
#define I2C_SR1_TXE                         7
#define I2C_SR1_BERR                        8
#define I2C_SR1_ARLO                        9
#define I2C_SR1_AF                          10
#define I2C_SR1_OVR                         11
#define I2C_SR1_PEC_ERR                     12
#define I2C_SR1_TIMEOUT                     14
#define I2C_SR1_SMB_ALERT                   15

#define I2C_SR2_MSL                         0
#define I2C_SR2_BUSY                        1
#define I2C_SR2_TRA                         2
#define I2C_SR2_GEN_CALL                    4
#define I2C_SR2_SMBDE_FAULT                 5
#define I2C_SR2_SMB_HOST                    6
#define I2C_SR2_DUALF                       7
#define I2C_SR2_PEC                         8

#define I2C_CCR_CCR                         0
#define I2C_CCR_DUTY                        14
#define I2C_CCR_FS                          15

// USART

#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15

#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14

#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

// RCC

#define RCC_CFGR_PRE1        			10  // AB1 prescaler
#define RCC_CFGR_PRE2        			13  // AB2 prescaler

#include "stm32f411x_gpio_driver.h"
#include "stm32f411x_spi_driver.h"
#include "stm32f411x_i2c_driver.h"
#include "stm32f411x_usart_driver.h"
#include "stm32f411x_rcc_driver.h"

#endif //DRIVER_DEVELOPMENT_STM32F4_H
