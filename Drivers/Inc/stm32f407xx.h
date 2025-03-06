/*
 * stm32f407xx.h
 *
 *  Created on: Sep 4, 2024
 *      Author: sanic
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_


#include "stdint.h"

/********************************************************************************************
 * Generic Macro Definitions
********************************************************************************************/
/*
 * Volatile definition macrro
 */
#define vo volatile





/********************************************************************************************
 * ARM Cortex M4 Processor NVIC registers address definitions
********************************************************************************************/
/*
 * ARM Cortex M4 Processor NVIC ISERx register addresses
 */
#define NVIC_ISER0								( (vo uint32_t*)0xE000E100 )
#define NVIC_ISER1								( (vo uint32_t*)0xE000E104 )
#define NVIC_ISER2								( (vo uint32_t*)0xE000E108 )
#define NVIC_ISER3								( (vo uint32_t*)0xE000E10C )

/*
 * ARM Cortex M4 Processor NVIC ICERx register addresses
 */
#define NVIC_ICER0								( (vo uint32_t*)0XE000E180 )
#define NVIC_ICER1								( (vo uint32_t*)0XE000E184 )
#define NVIC_ICER2								( (vo uint32_t*)0XE000E188 )
#define NVIC_ICER3								( (vo uint32_t*)0XE000E18C )

/*
 * ARM Cortex M4 Processor NVIC IPRx register addresses
 */
#define NVIC_PR_BASE_ADDR						( (vo uint32_t*)0xE000E400 )

/*
 * ARM Cortex M4 Processor number of priority bits implemented in Priority register
 */
#define NO_PR_BITS_IMPLEMENTED					4





/**************************************************************************************************************************************************************
 * Base address of these peripheral
 * -FLASH, SRAM1(SRAM), SRAM2, ROM
 * -APB1, APB2, AHB1, AHB2
 * -GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI
 * -I2C1, I2C2, I2C3, SPI2, SPI3, USART2, USART3, UART4, UART5,
 * -SPI1, USART1, USART6, EXTI, SYSCFG, RCC
 *************************************************************************************************************************************************************/


/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR								0x08000000U							//Base address of FLASH memory
#define SRAM1_BASEADDR								0x20000000U							//Base address of SRAM 1 memory (112 kB)
#define SRAM2_BASEADDR 								0x2001C000U							//Base address of SRAM 2 memory	(16 kB)
#define ROM_BASEADDR 								0x1FFF0000U							//Base address of ROM memory (System memory)
#define SRAM_BASEADDR								SRAM1_BASEADDR						//Base address of SRAM memory




/*
 * AHBx and APBx Peripheral buss addresses
 */

#define PERIPH_BASEADDR								0x40000000U							//Base address of peripheral bus
#define APB1PERIPH_BASEADDR							PERIPH_BASEADDR						//Base address of APB1 peripheral bus
#define APB2PERIPH_BASEADDR							0x40010000U							//Base address of APB2 peripheral bus
#define AHB1PERIPH_BASEADDR							0x40020000U							//Base address of AHB1 peripheral bus
#define AHB2PERIPH_BASEADDR							0x50000000U							//Base address of AHB2 peripheral bus





/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR								(AHB1PERIPH_BASEADDR + 0x0000)		//Base address of GPIOA peripheral offset: 0x0000
#define GPIOB_BASEADDR								(AHB1PERIPH_BASEADDR + 0x0400)		//Base address of GPIOB peripheral offset: 0x0400
#define GPIOC_BASEADDR								(AHB1PERIPH_BASEADDR + 0x0800)		//Base address of GPIOC peripheral offset: 0x0800
#define GPIOD_BASEADDR								(AHB1PERIPH_BASEADDR + 0x0C00)		//Base address of GPIOD peripheral offset: 0x0C00
#define GPIOE_BASEADDR								(AHB1PERIPH_BASEADDR + 0x1000)		//Base address of GPIOE peripheral offset: 0x1000
#define GPIOF_BASEADDR								(AHB1PERIPH_BASEADDR + 0x1400)		//Base address of GPOIF peripheral offset: 0x1400
#define GPIOG_BASEADDR								(AHB1PERIPH_BASEADDR + 0x1800)		//Base address of GPIOG peripheral offset: 0x1800
#define GPIOH_BASEADDR								(AHB1PERIPH_BASEADDR + 0x1C00)		//Base address of GPIOH peripheral offset: 0x1C00
#define GPIOI_BASEADDR								(AHB1PERIPH_BASEADDR + 0x2000)		//Base address of GPIOI peripheral offset: 0x2000
//#define GPIOJ_BASEADDR								(AHB1PERIPH_BASEADDR + 0x2400)		//Base address of GPIOJ peripheral offset: 0x2400
//#define GPIOK_BASEADDR								(AHB1PERIPH_BASEADDR + 0x2800)		//Base address of GPIOJ peripheral offset: 0x2800

/*
 * Base addresses of RCC which is hanging on AHB1 bus
 */

#define RCC_BASEADDR								(AHB1PERIPH_BASEADDR + 0x3800)		//Base address of RCC offset: 0x3800





/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR								(APB1PERIPH_BASEADDR + 0x5400)		//Base address of I2C1 peripheral offset: 0x5400
#define I2C2_BASEADDR								(APB1PERIPH_BASEADDR + 0x5800)		//Base address of I2C2 peripheral offset: 0x5800
#define I2C3_BASEADDR								(APB1PERIPH_BASEADDR + 0x5C00)		//Base address of I2C3 peripheral offset: 0x5C00


#define SPI2_BASEADDR								(APB1PERIPH_BASEADDR + 0x3800)		//Base address of SPI2 peripheral offset: 0x3800
#define SPI3_BASEADDR								(APB1PERIPH_BASEADDR + 0x3C00)		//Base address of SPI3 peripheral offset: 0x3C00


#define USART2_BASEADDR								(APB1PERIPH_BASEADDR + 0x4400)		//Base address of USART2 peripheral offset: 0x4400
#define USART3_BASEADDR								(APB1PERIPH_BASEADDR + 0x4800)		//Base address of USART3 peripheral offset: 0x4800


#define UART4_BASEADDR								(APB1PERIPH_BASEADDR + 0x4C00)		//Base address of UART4 peripheral offset: 0x4C00
#define UART5_BASEADDR								(APB1PERIPH_BASEADDR + 0x5000)		//Base address of UART5 peripheral offset:0x5000



/*
 * Base addresses of peripheral which are hanging on APB2 bus
 */
#define SPI1_BASEADDR								(APB2PERIPH_BASEADDR + 0x3000)		//Base address of SPI1 peripheral offset 0x3000
#define SPI4_BASEADDR								(APB2PERIPH_BASEADDR + 0x3400)		//Base address of SPI4 peripheral offset 0x3400


#define USART1_BASEADDR								(APB2PERIPH_BASEADDR + 0x1000)		//Base address of USART1 peripheral offset: 0x1000
#define USART6_BASEADDR								(APB2PERIPH_BASEADDR + 0x1400)		//Base address of USART6 peripheral offset: 0x1400


#define EXTI_BASEADDR								(APB2PERIPH_BASEADDR + 0x3C00)		//Base address of EXTI peripheral offset: 0x3C00


#define SYSCFG_BASEADDR								(APB2PERIPH_BASEADDR + 0x3800)		//Base address of SYSCFG peripheral offset: 0x3800









/********************************************************************************************
 * Peripheral Base Address Macros
********************************************************************************************/


/*
 *-GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI definition macro ( Peripheral base address typecasted to GPIO_RegDef_t)
 */

#define GPIOA										((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB										((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC										((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD										((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE										((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF										((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG										((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH										((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI										((GPIO_RegDef_t*)GPIOI_BASEADDR)

/*
 * RCC definition macro (RCC base address typecasted to RCC_RegDef_t)
 */

#define RCC											((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * EXTI definition macro (EXTI base address typecasted to EXTI_RegDef_t)
 */

#define EXTI 										((EXTI_RegDef_t*)EXTI_BASEADDR)

/*
 * SYSCFG definition macro (SYSCFG base address typecasted to SYSCFG_RegDef_t)
 */


#define SYSCFG										((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 * SPI definition macro (SPI base address typecasted to SPI_RegDef_t)
 */

#define SPI1 										((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2										((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3										((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4										((SPI_RegDef_t*)SPI4_BASEADDR)


/*
 * I2C definition macro (SPI base address typecasted to I2C_RegDef_t)
 */

#define I2C1										((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2										((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3										((I2C_RegDef_t*)I2C3_BASEADDR)


/**************************************************************************************************************************************************************
 *Register Definition Structures for peripheral
 **************************************************************************************************************************************************************/


/*
 * Peripheral register definition structure for GPIO
 */

typedef struct
{

	vo uint32_t MODER;				//GPIO port mode register					address offset: 0x00
	vo uint32_t OTYPER;				//GPIO port output type register			address offset: 0x04
	vo uint32_t OSPEEDR;			//GPIO port output speed register			address offset: 0x08
	vo uint32_t PUPDR;				//GPIO port pull-up//pull-down register		address offset: 0x0C
	vo uint32_t IDR;				//GPIO port input data register				address offset: 0x10
	vo uint32_t ODR;				//GPIO port output data register			address offset: 0x14
	vo uint32_t BSRR;				//GPIO port bit set/reset register			address offset: 0x18
	vo uint32_t LCKR;				//GPIO port configuration lock register		address offset: 0x1C
	vo uint32_t AFR[2];				//GPIO alternate function low-high register	address offset: 0x20


}GPIO_RegDef_t;


/*
 * Peripheral register definition structre for RCC
 */

typedef struct
{

	vo uint32_t CR;					//RCC clock control register				address offset: 0x00
	vo uint32_t PLLCFGR;			//RCC PLL configuration register			address offset: 0x04
	vo uint32_t CFGR;				//RCC clock configuration register			address offset: 0x08
	vo uint32_t CIR;				//RCC clock interrupt register				address offset: 0x0C
	vo uint32_t AHB1RSTR;			//RCC AHB1 peripheral reset register		address offset: 0x10
	vo uint32_t AHB2RSTR;			//RCC AHB2 peripheral reset register		address offset: 0x14
	vo uint32_t AHB3RSTR;			//RCC AHB3 peripheral reset register		address offset: 0x18
	   uint32_t RESERVED0;			//reserved
	vo uint32_t APB1RSTR;			//RCC APB1 peripheral reset register 		address offset: 0x20
	vo uint32_t APB2RSTR;			//RCC APB2 peripheral reset register		address offset: 0x24
	   uint32_t RESERVED1[2];       //reserved
	vo uint32_t AHB1ENR;			//RCC AHB1 peripheral clock enable register	address offset: 0x30
	vo uint32_t AHB2ENR;			//RCC AHB2 peripheral clock enable register	address offset: 0x34
	vo uint32_t AHB3ENR;			//RCC AHB3 peripheral clock enable register	address offset: 0x38
	   uint32_t RESERVED2;			//reserved
	vo uint32_t APB1ENR;			//RCC APB1 peripheral clock enable register	address offset: 0x40
	vo uint32_t APB2ENR;			//RCC APB2 peripheral clock enable register address offset: 0x44
	   uint32_t RESERVED3[2];		//reserved
	vo uint32_t AHB1LPENR;			//RCC AHB1 peripheral clock enable in low power mode register address offset: 0x50
	vo uint32_t AHB2LPENR;			//RCC AHB2 peripheral clock enable in low power mode register address offset: 0x54
	vo uint32_t AHB3PLENR;			//RCC AHB3 peripheral clock enable in low power mode register address offset: 0x58
	   uint32_t RESERVED4;			//reserved
	vo uint32_t APB1LPENR;			//RCC APB1 peripheral clock enable in low power mode register address offset: 0x60
	vo uint32_t APB2LPENR;			//RCC APB2 peripheral clock enable in low power mode register address offset: 0x64
	   uint32_t RESERVED5[2];		//reserved
	vo uint32_t BDCR;				//RCC Backup domain control register 						address offset: 0x70
	vo uint32_t CSR;				//RCC clock control & status register						address offset: 0x74
	   uint32_t RESERVED6[2];		//reserved
	vo uint32_t SSCGR;				//RCC spread spectrum clock generation register 			address offset: 0x80
	vo uint32_t PLLI2SCFGR;			//RCC PLLI2S configuration register							address offset: 0x84

}RCC_RegDef_t;

/*
 * Peripheral register definition structure for EXTI
 */

typedef struct
{

	vo uint32_t IMR;				//EXTI interrupt mask register								address offset: 0x00
	vo uint32_t EMR;				//EXTI event mask register									address offset: 0x04
	vo uint32_t RTSR;				//EXTI rising trigger selection register					address offset: 0x08
	vo uint32_t FTSR;				//EXTI falling trigger selection register 					address offset: 0x0C
	vo uint32_t SWIER;				//EXTI software interrupt event register 					address offset: 0x10
	vo uint32_t PR;					//EXTI pending register										address offset: 0x14

}EXTI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */

typedef struct
{
	vo uint32_t MEMRMP;				//SYSCFG memory map register								address offset: 0x00
	vo uint32_t PMC;				//SYSCFG peripheral mode configuration register				address offset: 0x04
	vo uint32_t EXTICR[4];			//SYSCFG external interrupt 1-4 control register			address offset: 0x08
	   uint32_t RESERVED0[2];		//reserved
	vo uint32_t CMPCR;				//SYSCFG compensation cell control register					address offset: 0x20

}SYSCFG_RegDef_t;


/*
 * Peripheral register definition structure for SPI
 */

typedef struct
{

	vo uint32_t CR1;				//SPI control register 1									address offset: 0x00
	vo uint32_t CR2;				//SPI control register 2									address offset: 0x04
	vo uint32_t SR;					//SPI status register										address offset: 0x08
	vo uint32_t DR;					//SPI data register											address offset: 0x0C
	vo uint32_t CRCPR;				//CRC polynomial register									address offset: 0x10
	vo uint32_t RXCRCR;				//RX CRC register			 								address offset: 0x14
	vo uint32_t TXCRCR;				//TX CRC register											address offset: 0x18
	vo uint32_t I2SCFGR;			//SPI I2S configuration register							address offset: 0x1C
	vo uint32_t I2SPR;				//SPI I2S prescaler register								address offset: 0x20
}SPI_RegDef_t;


/*
 * Peripheral register definition structure for I2C
 */

typedef struct
{
	vo uint32_t CR1;				//I2C control register 1									address offset: 0x00
	vo uint32_t CR2;				//I2C control register 2									address offset: 0x04
	vo uint32_t OAR1;				//I2C Own address register 1								address offset: 0x08
	vo uint32_t OAR2;				//I2C Own address register 2								address offset: 0x0C
	vo uint32_t DR;					//I2C Data register											address offset: 0x10
	vo uint32_t SR1;				//I2C Status register 1			 							address offset: 0x14
	vo uint32_t SR2;				//I2C Status register 2										address offset: 0x18
	vo uint32_t CCR;				//I2C Clock control register								address offset: 0x1C
	vo uint32_t TRISE;				//I2C TRISE register										address offset: 0x20
	vo uint32_t FLTR;				//I2C FLTR register											address offset: 0x24
}I2C_RegDef_t;

/*
 * Periprheral register definition structure for USART
 */

typedef struct
{
	vo uint32_t SR;					//USART status register										address offset: 0x00
	vo uint32_t DR;					//USART data register										address offset: 0x04
	vo uint32_t BRR;				//USART baud rate register									address offset: 0x08
	vo uint32_t CR1;				//USART control register 1									address offset: 0x0C
	vo uint32_t CR2;				//USART control register 2									address offset: 0x10
	vo uint32_t CR3;				//USART control register 3									address offset: 0x14
	vo uint32_t GTPR;				//USART  guard time and prescaler register					address offset: 0x18
}USART_RefDef_t;



/**************************************************************************************************************************************************************
 * Clock Enable/Disable Macros for Peripherals
 **************************************************************************************************************************************************************/

/*
 * Clock enable/disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN() 					( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN() 					( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN() 					( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN() 					( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN() 					( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN() 					( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN() 					( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN() 					( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN() 					( RCC->AHB1ENR |= (1 << 8) )


#define GPIOA_PCLK_DI() 					( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI() 					( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI() 					( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI() 					( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI() 					( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI() 					( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI() 					( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI() 					( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI() 					( RCC->AHB1ENR &= ~(1 << 8) )


/*
 * Clock enable/disable macros for I2C1-2-3 peripherals
 */

#define I2C1_PCLK_EN()						( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()						( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()						( RCC->APB1ENR |= (1 << 23) )


#define I2C1_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 23) )

/*
 * Clock enable/disable macros for SPI2-3 peripherals
 */
#define SPI2_PCLK_EN()						( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()						( RCC->APB1ENR |= (1 << 15) )


#define SPI2_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 15) )


/*
 * Clock enable/disable macros for USART2-3 peripherals
 */
#define USART2_PLCK_EN()					( RCC->APB1ENR |= (1 << 17) )
#define USART3_PCLK_EN() 					( RCC->APB1ENR |= (1 << 18) )


#define USART2_PLCK_DI()					( RCC->APB1ENR &= ~(1 << 17) )
#define USART3_PCLK_DI() 					( RCC->APB1ENR &= ~(1 << 18) )


/*
 * Clock enable/disable macros for UART4-5 peripherals
 */
#define UART4_PCLK_EN()						( RCC->APB1ENR |= (1 << 19) )
#define UART5_PCLK_EN()						( RCC->APB1ENR |= (1 << 20) )

#define UART4_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 19) )
#define UART5_PCLK_DI()						( RCC->APB1ENR &= ~(1 << 20) )


/*
 *Clock enable/disable macros for SPI1 peripherals
 */
#define SPI1_PCLK_EN()						( RCC->APB2ENR |= (1 << 12) )
#define SPI4_PLCK_EN()						( RCC->APB2ENR |= (1 << 13) )


#define SPI1_PCLK_DI()						( RCC->APB2ENR &= ~(1 << 12) )
#define SPI4_PLCK_DI()						( RCC->APB2ENR &= ~(1 << 13) )


/*
 * Clock enable/disable macros for USART1-6 peripherals
 */
#define USART1_PCLK_EN()					( RCC->APB2ENR |= (1 << 4) )
#define USART6_PCLK_EN()					( RCC->APB2ENR |= (1 << 5) )


#define USART1_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 4) )
#define USART6_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 5) )

/*
 * Clock enable disable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()					( RCC->APB2ENR |= (1 << 14) )
#define SYSCFG_PCLK_DI()					( RCC->APB2ENR &= ~(1 << 14) )


/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()					do{ ( RCC->AHB1RSTR |= (1 << 0) ); ( RCC->AHB1RSTR &= ~(1 << 0) ); }while(0)
#define GPIOB_REG_RESET()					do{ ( RCC->AHB1RSTR |= (1 << 1) ); ( RCC->AHB1RSTR &= ~(1 << 1) ); }while(0)
#define GPIOC_REG_RESET()					do{ ( RCC->AHB1RSTR |= (1 << 2) ); ( RCC->AHB1RSTR &= ~(1 << 2) ); }while(0)
#define GPIOD_REG_RESET()					do{ ( RCC->AHB1RSTR |= (1 << 3) ); ( RCC->AHB1RSTR &= ~(1 << 3) ); }while(0)
#define GPIOE_REG_RESET()					do{ ( RCC->AHB1RSTR |= (1 << 4) ); ( RCC->AHB1RSTR &= ~(1 << 4) ); }while(0)
#define GPIOF_REG_RESET()					do{ ( RCC->AHB1RSTR |= (1 << 5) ); ( RCC->AHB1RSTR &= ~(1 << 5) ); }while(0)
#define GPIOG_REG_RESET()					do{ ( RCC->AHB1RSTR |= (1 << 6) ); ( RCC->AHB1RSTR &= ~(1 << 6) ); }while(0)
#define GPIOH_REG_RESET()					do{ ( RCC->AHB1RSTR |= (1 << 7) ); ( RCC->AHB1RSTR &= ~(1 << 7) ); }while(0)
#define GPIOI_REG_RESET()					do{ ( RCC->AHB1RSTR |= (1 << 8) ); ( RCC->AHB1RSTR &= ~(1 << 8) ); }while(0)


/*
 * Macros to reset I2Cx peripherals
 */
#define I2C1_REG_RESET()					do{ ( RCC->APB1RSTR |= (1 << 21) ); ( RCC->APB1RSTR &= ~(1 << 21) ); }while(0)
#define I2C2_REG_RESET()					do{ ( RCC->APB1RSTR |= (1 << 22) ); ( RCC->APB1RSTR &= ~(1 << 22) ); }while(0)
#define I2C3_REG_RESET()					do{ ( RCC->APB1RSTR |= (1 << 23) ); ( RCC->APB1RSTR &= ~(1 << 23) ); }while(0)



/**************************************************************************************************************************************************************
 * Generic Macros
 **************************************************************************************************************************************************************/

#define ENABLE 								1
#define DISABLE 							0
#define SET									ENABLE
#define RESET								DISABLE
#define GPIO_PIN_SET						SET
#define GPIO_PIN_RESET						RESET
#define FLAG_RESET							RESET
#define FLAG_SET							SET


#define GPIO_BASEADDR_TO_CODE(x)		  ( (x == GPIOA) ? 0 :\
											(x == GPIOB) ? 1 :\
											(x == GPIOC) ? 2 :\
											(x == GPIOD) ? 3 :\
											(x == GPIOE) ? 4 :\
											(x == GPIOF) ? 5 :\
											(x == GPIOG) ? 6 :\
											(x == GPIOH) ? 7 :\
											(x == GPIOI) ? 8 : 0 )

/*
 * Interrupt Request Number macros
 */

#define IRQ_NO_EXTI0 						6
#define IRQ_NO_EXTI1						7
#define IRQ_NO_EXTI2						8
#define IRQ_NO_EXTI3						9
#define IRQ_NO_EXTI4						10
#define IRQ_NO_EXTI9_5						23
#define IRQ_NO_EXTI15_10					40

#define IRQ_NO_SPI_1						35
#define IRQ_NO_SPI_2						36
#define IRQ_NO_SPI_3						51


#define IRQ_NO_I2C1_EV						31
#define IRQ_NO_I2C1_ER						32
#define IRQ_NO_I2C2_EV						33
#define IRQ_NO_I2C2_ER						34
#define IRQ_NO_I2C3_EV						72
#define IRQ_NO_I2C3_ER						73


/*
 *NVIC priority levels macros
 */
#define NVIC_IRQ_PRI0						0
#define NVIC_IRQ_PRI1						1
#define NVIC_IRQ_PRI2						2
#define NVIC_IRQ_PRI3						3
#define NVIC_IRQ_PRI4						4
#define NVIC_IRQ_PRI5						5
#define NVIC_IRQ_PRI6						6
#define NVIC_IRQ_PRI7						7
#define NVIC_IRQ_PRI8						8
#define NVIC_IRQ_PRI9						9
#define NVIC_IRQ_PRI10						10
#define NVIC_IRQ_PRI11						11
#define NVIC_IRQ_PRI12						12
#define NVIC_IRQ_PRI13						13
#define NVIC_IRQ_PRI14						14
#define NVIC_IRQ_PRI15						15



/**************************************************************************************************************************************************************
 * Bit position definitions of SPI peripheral
 **************************************************************************************************************************************************************/

/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA						0
#define SPI_CR1_CPOL						1
#define SPI_CR1_MSTR						2
#define SPI_CR1_BR							3
#define SPI_CR1_SPE							6
#define SPI_CR1_LSBFIRST					7
#define SPI_CR1_SSI 						8
#define SPI_CR1_SSM							9
#define SPI_CR1_RXONLY						10
#define SPI_CR1_DFF							11
#define SPI_CR1_CRCNEXT						12
#define SPI_CR1_CRCEN						13
#define SPI_CR1_BIDIOE						14
#define SPI_CR1_BIDIMODE					15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN						0
#define SPI_CR2_TXDMAEN						1
#define SPI_CR2_SSOE						2
#define SPI_CR2_FRF							4
#define SPI_CR2_ERRIE						5
#define SPI_CR2_RXNEIE						6
#define SPI_CR2_TXEIE						7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE							0
#define SPI_SR_TXE							1
#define SPI_SR_CHSIDE						2
#define SPI_SR_UDR							3
#define SPI_SR_CRCERR						4
#define SPI_SR_MODF							5
#define SPI_SR_OVR							6
#define SPI_SR_BSY							7
#define SPI_SR_FRE							8


/**************************************************************************************************************************************************************
 * Bit position definitions of SPI peripheral
 **************************************************************************************************************************************************************/
/*
 * Bit position definition I2C Control Register 1
 */
#define I2C_CR1_PE							0
#define I2C_CR1_SMBUS						1
//#define I2C_CR1_RESERVED					2
#define I2C_CR1_SMBTYPE						3
#define I2C_CR1_ENARP						4
#define I2C_CR1_ENPEC						5
#define I2C_CR1_ENGC						6
#define I2C_CR1_NOSTRECH					7
#define I2C_CR1_START						8
#define I2C_CR1_STOP						9
#define I2C_CR1_ACK							10
#define I2C_CR1_POS							11
#define I2C_CR1_PEC							12
#define I2C_CR1_ALERT						13
//#define I2C_CR1_RESERVED					14
#define I2C_CR1_SWRST						15


/*
 * Bit position definition I2C Control Register 2
 */
#define I2C_CR2_FREQ_0_5					0
//#define I2C_CR2_RESERVED					6
//#define I2C_CR2_RESERVED					7
#define I2C_CR2_ITERREN						8
#define I2C_CR2_ITEVTEN						9
#define I2C_CR2_ITBUFEN						10
#define I2C_CR2_DMAEN						11
#define I2C_CR2_LAST						12
//#define I2C_CR2_RESERVED					13
//#define I2C_CR2_RESERVED					14
//#define I2C_CR2_RESERVED					15

/*
 * Bit position definition I2C Status Register 1
 */
#define I2C_SR1_SB							0
#define I2C_SR1_ADDR						1
#define I2C_SR1_BTF							2
#define I2C_SR1_ADD10						3
#define I2C_SR1_STOPF						4
//#define I2C_SR1_RESERVED					5
#define I2C_SR1_RXNE						6
#define I2C_SR1_TXE							7
#define I2C_SR1_BERR						8
#define I2C_SR1_ARLO						9
#define I2C_SR1_AF							10
#define I2C_SR1_OVR							11
#define I2C_SR1_PECERR						12
//#define I2C_SR1_RESERVED					13
#define I2C_SR1_TIMEOUT						14
#define I2C_SR1_SMBALERT					15


/*
 * Bit position definition I2C Status Register 2
 */
#define I2C_SR2_MSL							0
#define I2C_SR2_BUSY						1
#define I2C_SR2_TRA							2
//#define I2C_SR2_RESERVED					3
#define I2C_SR2_GENCALL						4
#define I2C_SR2_SMBDEFAULT					5
#define I2C_SR2_SMBHOST						6
#define I2C_SR2_DUALF						7
#define I2C_SR2_PEC							15


/*
 * Bit position definition I2C Clock Control Register
 */
#define I2C_CCR_CCR							11
//#define I2C_CCR_RESERVED					12
//#define I2C_CCR_RESERVED					13
#define I2C_CCR_DUTY						14
#define I2C_CCR_FS							15













#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"







#endif /* STM32F407XX_H_ */
