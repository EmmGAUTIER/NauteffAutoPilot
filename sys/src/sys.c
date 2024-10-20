/*
 * This file is part of the Nauteff Autopilot project.
 *
 * Copyright (C) 2022 Nauteff https://nauteff.com
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * \author Emmanuel Gautier
 * \brief Initializing functions
 * \version first
 * \date February, 23 2022
 * \param none
 * \return none
 */
#include <stdbool.h>
#include "stm32F446.h"
#include "gpio.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "sys.h"
#include "rcc.h"
#include "gpio.h"
#include "usart.h"
#include "i2c.h"
#include "exti.h"

void main(void);
void xPortPendSVHandler(void) __attribute__ (( naked ));
void xPortSysTickHandler(void);
void vPortSVCHandler(void) __attribute__ (( naked ));

void sys_init() {
	rcc_start_SYSCFG();
	sys_init_clocks();
	sys_init_Data();
	sys_init_FPU();
	sys_init_GPIOs();
	sys_init_EXTI();
	sys_init_I2C();

#if 0
    sys_init_DMA ();
    sys_init_USART1 ();
    sys_init_I2C1 ();
    sys_init_ADC ();
#endif
}

void sys_init_clocks(void) {
	RCC_config_t rcc_config = { 40, AHB_PRESCALER_NO_DEVIDE,
	APBX_PRESCALER_DEVIDED_BY_2,
	APBX_PRESCALER_DEVIDED_BY_4 };

	RCC_init_for_nauteff(&rcc_config);

}

void sys_init_Data(void) {
	unsigned *src;
	unsigned *dst;

	/* copy the data segment from FLASH to RAM */
	/* Copy is made with 32 bit words */
	/* data areas (FLASH and RAM) are 4-byte aligned */
	/* &_sidata is the address of initialized data in FLASH */
	/* &_sdata is the first address of initialized data in RAM */
	/* &_edata is the address of the end of initialized data in RAM*/
	/* see linker script (.ld file) */
	src = &_sidata;
	dst = &_sdata;
	while (dst < &_edata)
		*(dst++) = *(src++);

	/* zero the bss segment */
	dst = &_sbss;
	while (dst < &_ebss)
		*(dst++) = 0;
	;
}

/*
 * \brief Enable FPU
 * \see PM0214 Programming manual 4.6 floating point unit p. 253
 */
void sys_init_FPU(void) {

	CPACR |= 0x03 << 20;
}

#if 0
inline void rcc_start_GPIOA(void) {RCC_AHB1ENR |= (0x1 << 0);}
inline void rcc_start_GPIOB(void) {RCC_AHB1ENR |= (0x1 << 1);}
inline void rcc_start_GPIOC(void) {RCC_AHB1ENR |= (0x1 << 2);}
inline void rcc_start_GPIOD(void) {RCC_AHB1ENR |= (0x1 << 3);}
#endif

void sys_init_GPIOs(void) {
	rcc_start_GPIOA();
	rcc_start_GPIOB();
	rcc_start_GPIOC();
	rcc_start_GPIOD();

	/*@formatter:off  */
    GPIO_init_pin_unused (GPIOA, 0);
    GPIO_init_pin_unused (GPIOA, 1);
    GPIO_init_pin_unused (GPIOA, 2);
    GPIO_init_pin_unused (GPIOA, 3);
    GPIO_init_pin_unused (GPIOA, 4);
    /* GPIOA, pin 5 : Output, slow, psush/pull : clutch */
    GPIO_init_pin_output (GPIOA, 5, GPIO_OTYPER_PUSHPULL, GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE);
    GPIO_init_pin_unused (GPIOA, 6);
    GPIO_init_pin_unused (GPIOA, 7);
    GPIO_init_pin_unused (GPIOA, 8);
    GPIO_init_pin_unused (GPIOA, 9);
    GPIO_init_pin_unused (GPIOA, 10);
    GPIO_init_pin_unused (GPIOA, 11);
    GPIO_init_pin_unused (GPIOA, 12);

    GPIO_init_pin_unused (GPIOB, 0);
    GPIO_init_pin_unused (GPIOB, 1);
    GPIO_init_pin_unused (GPIOB, 2);
    /* PB3 : SYS_JTDO_SWO*/
    GPIO_init_pin_unused (GPIOB, 4);
    GPIO_init_pin_unused (GPIOB, 5);
    GPIO_init_pin_unused (GPIOB, 6);

    /* *I2C1 SDA */
    GPIO_init_pin_unused (GPIOB, 7);
    GPIO_init_pin_altfct(GPIOB, 7, GPIO_OTYPER_OPENDRAIN,
                         GPIO_OSPEEDR_MEDIUMSPEED, GPIO_PUPDR_PULLUP, 4);

    /* I2C1 SCL */
    GPIO_init_pin_unused (GPIOB, 8);
    GPIO_init_pin_altfct(GPIOB, 8, GPIO_OTYPER_OPENDRAIN,
                         GPIO_OSPEEDR_MEDIUMSPEED, GPIO_PUPDR_PULLUP, 4);

    GPIO_init_pin_unused (GPIOB, 9);
    GPIO_init_pin_unused (GPIOB, 10);
    GPIO_init_pin_unused (GPIOB, 11);
    GPIO_init_pin_unused (GPIOB, 12);
    /*  PB12 réservé */
    GPIO_init_pin_unused (GPIOB, 14);
    GPIO_init_pin_unused (GPIOB, 15);

    GPIO_init_pin_unused (GPIOC, 0);
    GPIO_init_pin_unused (GPIOC, 1);
    GPIO_init_pin_unused (GPIOC, 2);
    GPIO_init_pin_unused (GPIOC, 3);
    GPIO_init_pin_unused (GPIOC, 4);
    GPIO_init_pin_unused (GPIOC, 5);
    GPIO_init_pin_unused (GPIOC, 6);
    GPIO_init_pin_unused (GPIOC, 7);
    GPIO_init_pin_unused (GPIOC, 8);
    GPIO_init_pin_unused (GPIOC, 9);
    GPIO_init_pin_unused (GPIOC, 10);
    GPIO_init_pin_unused (GPIOC, 11);
    GPIO_init_pin_unused (GPIOC, 12);
    /* PC13 connected to blue button  */
    GPIO_init_pin_input  (GPIOC, 13, GPIO_PUPDR_NONE);
    /* PC14 & 15 : used by cristal */

    GPIO_init_pin_unused (GPIOD, 0);
    GPIO_init_pin_unused (GPIOD, 1);

                    	/*@formatter:on  */

#if 0
	/*----- GPIO A -----*/
	/* GPIOA, pin 0 : Analog input, ADC1 IN0, power voltage */
	GPIO_pin_init(GPIOA, 0, GPIO_MODER_INPUT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE, 0);

	/* GPIOA, pin 1 : Analog input, ADC1 IN0, Motor current */
	GPIO_pin_init(GPIOA, 1, GPIO_MODER_INPUT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE, 0);

	/* GPIOA, pin 2 : Alt. fct. #7 : USART2 TX */
	GPIO_pin_init(GPIOA, 2, GPIO_MODER_ALTFCT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE, 7);

	/* GPIOA, pin 3 : Alt. fct. #7 : USART2 RX */
	GPIO_pin_init(GPIOA, 3, GPIO_MODER_ALTFCT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_PULLDOWN, 7);

	/* GPIOA, pin 4 : Output, slow, psush/pull : motor */
	GPIO_pin_init(GPIOA, 4, GPIO_MODER_OUTPUT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE, 0);

	/* GPIOA, pin 5 : Output, slow, psush/pull : clutch */
	GPIO_pin_init(GPIOA, 5, GPIO_MODER_OUTPUT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE, 0);

	/* GPIOA, pin 6 : Output, slow, psush/pull : INA */
	GPIO_pin_init(GPIOA, 6, GPIO_MODER_OUTPUT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE, 0);

	/* GPIOA, pin 7 : Output, slow, psush/pull : INB */
	GPIO_pin_init(GPIOA, 7, GPIO_MODER_OUTPUT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE, 0);

	/* GPIOA, pin 8 : alt. fct. #4 : I2C3 SCL */
	GPIO_pin_init(GPIOA, 8, GPIO_MODER_ALTFCT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE, 0);

	/* GPIOA, pin 9 : alt. fct. #7 : USART1 TX  */
	GPIO_pin_init(GPIOA, 5, GPIO_MODER_ALTFCT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE, 0);

	/* GPIOA, pin 10 : alt. fct. #7 : USART1 RX  */
	GPIO_pin_init(GPIOA, 5, GPIO_MODER_ALTFCT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_PULLDOWN, 0);

	/* GPIOA, pin 11 : alt. fct. #9 : CAN1 RX  */
	GPIO_pin_init(GPIOA, 5, GPIO_MODER_ALTFCT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_PULLDOWN, 0);

	/* GPIOA, pin 12 : alt. fct. #9 : CAN1 TX  */
	GPIO_pin_init(GPIOA, 5, GPIO_MODER_ALTFCT, GPIO_OTYPER_PUSHPULL,
	GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_PULLDOWN, 0);


	/*----- GPIO A config -----*/
	/* PA13..15 already configured after reset and used for JTAG : */
	/* PA13 -> JTMS-SWDIO, PA14 -> JTCK-SWCLK, PA15 -> JTDI */
	/* first, make sure that PA0..12 = 0 */
	GPIOA->MODER &= (0x3) << (15 * 2) | (0x3) << (14 * 2) | (0x3) << (14 * 2);
	/* then set PA0..12 */
	GPIOA->MODER |= (0x3) << (0 * 2) /* PA0 Analog input ADC1 IN0 pwr voltage */
	| (0x3) << (1 * 2) /* PA1 Analog input ADC1 IN0 */
	| (0x2) << (2 * 2) /* PA2 USART 2 TX, alternate function */
	| (0x2) << (3 * 2) /* PA3 USART 2 RX, alternate function*/
	| (0x1) << (4 * 2) /* Motor */
	| (0x1) << (5 * 2) /* Clutch and LED */
	| (0x1) << (6 * 2) /* INA */
	| (0x1) << (7 * 2) /* INB */
	| (0x2) << (8 * 2) /* I2C3 serial clock, alternate function */
	| (0x2) << (9 * 2) /* USART1 TX, alternate function */
	| (0x2) << (10 * 2) /* USART1 RX, alternate function */
	| (0x2) << (11 * 2) /* CAN1 TX, alternate function */
	| (0x2) << (12 * 2) /* CAN1 RX, alternate function */
	;

	/*----- GPIO B config -----*/
	/* PB3, PB11, PB12 left unchanged */
	GPIOB->MODER &= (0x3) << (3 * 2) | (0x3) << (11 * 2);
	GPIOB->MODER |= (0x2) << (0 * 2) /* PB0 GPIO output Buzzer alarm */
	| (0x0) << (1 * 2) /* PB1 input, Gyro./Acc. data ready */
	| (0x0) << (2 * 2) /* PA2 Input, Mag. data ready       */
	| (0x2) << (4 * 2) /* I2C3 SDA ,alternate function     */
	| (0x2) << (5 * 2) /* CAN2 RX ,alternate function      */
	| (0x2) << (6 * 2) /* CAN2 TX, alternate function      */
	| (0x1) << (7 * 2) /* I2C1 SDA, alternate function     */
	| (0x2) << (8 * 2) /* I2C1 SCL, alternate function     */
	| (0x2) << (10 * 2) /* USART3 TX, alternate function   */
	| (0x2) << (13 * 2) /* SPI2 SCK, alternate function    */
	| (0x2) << (14 * 2) /* SPI2 MISO, alternate function   */
	| (0x2) << (15 * 2) /* SPI2 MOSI, alternate function   */
	;

	/*----- GPIO C config -----*/
	/* PC8, PC14 & PC15 left unchanged */
	GPIOB->MODER &= (0x3) << (14 * 2) | (0x3) << (15 * 2);
	GPIOB->MODER |= (0x2) << (0 * 2) /* Input, Auto btn*/
	| (0x0) << (1 * 2) /* Input, Standby btn*/
	| (0x0) << (2 * 2) /* Input, +1 btn*/
	| (0x0) << (3 * 2) /* Input, -1 btn*/
	| (0x0) << (4 * 2) /* Input, +10 btn*/
	| (0x0) << (5 * 2) /* Input, -10 btn*/
	| (0x0) << (6 * 2) /* Input, Auxiliary btn 1 */
	| (0x0) << (7 * 2) /* Input, Auxiliary btn 2 */
	| (0x0) << (9 * 2) /* MCO output, alternate function*/
	| (0x2) << (10 * 2) /* USART 4 TX, alternate function */
	| (0x2) << (11 * 2) /* USART 4 RX, alternate function */
	| (0x2) << (12 * 2) /* USART 5 Tx, alternate function */
	| (0x0) << (13 * 2) /* Input, blue btn */
#if 0
	| (0x0) << (14 * 2) /* Cristal 32.768 kHz */
	| (0x0) << (15 * 2) /* Cristal 32.768 kHz */
#endif
	;

	/*----- GPIO D config -----*/
	/* Only PD2 used */
    GPIOD->MODER = (0x2) << (2*2);
#endif
}

void sys_init_DMA(void) {
	;
}

void sys_init_USART1(void) {
	;
}

void initADC(void) {
	;
}

void sys_init_EXTI(void) {
#if 0
    rcc_start_SYSCFG();
    /* Le bouton bleu sur PC13 */
    EXTI_Select_Edges (13, true, true);
    EXTI_Select_srce_input (13, 3);
    EXTI_Intr_enable_line (13, true);
    EXTI_Event_enable_line (13, true);
    NVIC_ISER0 = ~0U;
    NVIC_ISER1 = ~0U;
    NVIC_ISER2 = ~0U;
#endif
}

void sys_init_I2C(void) {
	I2C_init();
	I2C_start_periph(&i2c1);
	I2C1_set_std_speed(&i2c1);
	I2C1_enable(&i2c1);

}

__attribute__ ((interrupt("irq")))
void NMI_Handler(void) {
	while (1) {
		;
	}
}

__attribute__ ((interrupt("irq")))
void HardFault_Handler(void) {
	while (1) {
		;
	}
}

__attribute__ ((interrupt("irq")))
void MemManage_Handler(void) {
	while (1) {
		;
	}
}

__attribute__ ((interrupt("irq")))
void BusFault_Handler(void) {
	while (1) {
		;
	}
}

__attribute__ ((interrupt("irq")))
void UsageFault_Handler(void) {
	while (1) {
		;
	}
}

/* vector table */
__attribute__ ((section(".vector_table")))
void (*const vector_table[])(void) =
{
	(void (*)(void)) &_estack,/*0 initial stack pointer */
	main,/*                     1 Reset Handler */
	NMI_Handler,/*              2 NMI Handler */
	HardFault_Handler,/*        3 Hard Fault Handler */
	MemManage_Handler,/*        4 MPU Fault Handler */
	BusFault_Handler,/*         5 Bus Fault Handler */
	UsageFault_Handler,/*       6 Usage Fault Handler */
	(void (*)()) 0,/*           7 Reserved */
	(void (*)()) 0,/*           8 Reserved */
	(void (*)()) 0,/*           9 Reserved */
	(void (*)()) 0,/*          10 Reserved */
	vPortSVCHandler,/*         11 SVCall Handler */
	(void (*)()) 0,/*          12 Debug Monitor Handler */
	(void (*)()) 0,/*          13 Reserved */
	xPortPendSVHandler,/*      14 PendSV Handler */
	xPortSysTickHandler,/*     15 SysTick Handler */
	/* External Interrupts */
	(void (*)()) 0,/*       16 Window Watchdog */
	(void (*)()) 0,/*       17 PVD through EXTI Line detect */
	(void (*)()) 0,/*       18 Tamper and TimeStamp intr through EXTI line */
	(void (*)()) 0,/*       19 RTC_WKUP */
	(void (*)()) 0,/*       20 Flash */
	(void (*)()) 0,/*       21 RCC */
	EXTI0_Handler,/*        22 EXTI Line 0 */
	EXTI1_Handler,/*        23 EXTI Line 1 */
	EXTI2_Handler,/*        24 EXTI Line 2 and touch sensing */
	EXTI3_Handler,/*        25 EXTI Line 3 */
	EXTI4_Handler,/*        26 EXTI Line 4 */
	DMA1_stream0,/*         27 DMA1 stream 0 : UART5 RX */
	DMA1_stream1,/*         28 DMA1 stream 1 : I2C3 RX */
	DMA1_stream2,/*         29 DMA1 stream 2 : I2C3 TX */
	(void (*)()) 0,/*       30 DMA1 stream 3 */
	(void (*)()) 0,/*       31 DMA1 stream 4 */
	DMA1_stream5,/*         32 DMA1 stream 5 : USART2 RX */
	DMA1_stream6,/*         33 DMA1 stream 6 : USART2 TX */
	(void (*)()) 0,/*       34 ADC1 & ADC2 & ADC3 */
	(void (*)()) 0,/*       35 CAN1 TX */
	(void (*)()) 0,/*       36 CAN1 RX0 */
	(void (*)()) 0,/*       37 CAN1 RX1 */
	(void (*)()) 0,/*       38 CAN1 SCE */
	EXTI5_9_Handler,/*      39 EXTI Line 9..5 */
	(void (*)()) 0,/*       40 TIM1 Break and TIM9 */
	(void (*)()) 0,/*       41 TIM1 Update and TIM10 */
	(void (*)()) 0,/*       42 TIM1 Trigger and Commutation and TIM11 */
	(void (*)()) 0,/*       43 TIM1 Capture Compare */
	(void (*)()) 0,/*       44 TIM2 */
	(void (*)()) 0,/*       45 TIM3 */
	(void (*)()) 0,/*       46 TIM4 */
	I2C1_Event,/*           47 I2C1 Event */
	I2C1_Error,/*           48 I2C1 Error */
	I2C2_Event,/*           49 I2C2 Event */
	I2C2_Error,/*           50 I2C2 Error */
	(void (*)()) 0,/*       51 SPI1 */
	(void (*)()) 0,/*       52 SPI2 */
	USART1_Event,/*         53 USART1 */
	(void (*)()) 0,/*       54 USART2 */
	(void (*)()) 0,/*       55 USART3 */
	EXTI10_15_Handler,/*    56 EXTI Line 15..10 */
	(void (*)()) 0,/*       57 RTC Alarms (A and B) through EXTI Line */
	(void (*)()) 0,/*       58 USB On the Go FS Wakeup through EXTI line */
	(void (*)()) 0,/*       59 TIM8 break intr. and TIM12 global intr. */
	(void (*)()) 0,/*       60 TIM8 Update intr. & TIM13 global intr. */
	(void (*)()) 0,/*       61 TIM8 Trig. & Commutation intr. & TIM14 intr. */
	(void (*)()) 0,/*       62 TIM8 capture compare intr. */
	DMA1_stream7,/*         63 DMA1 Stream 7 global intr. : UART5 TX */
	(void (*)()) 0,/*       64 FMC global intr. */
	(void (*)()) 0,/*       65 SDIO global intr. */
	(void (*)()) 0,/*       66 TIM5 global intr. */
	(void (*)()) 0,/*       67 SPI3 global intr. */
	(void (*)()) 0,/*       68 UART 4 intr. */
	(void (*)()) 0,/*       69 UART 5 global intr. */
	(void (*)()) 0,/*       70 TIM6 global intr. and DAC1/2 underrun intr. */
	(void (*)()) 0,/*       71 TIM7 global intr. */
	(void (*)()) 0,/*       72 DMA2 Stream 0 global intr. */
	(void (*)()) 0,/*       73 DMA2 Stream 1 global intr. */
	DMA2_stream2,/*         74 DMA2 Stream 2 global intr. : USART1 RX */
	(void (*)()) 0,/*       75 DMA2 Stream 3 global intr. */
	(void (*)()) 0,/*       76 DMA2 Stream 4 global intr. */
	(void (*)()) 0,/*       77 Reserved */
	(void (*)()) 0,/*       78 Reserved */
	(void (*)()) 0,/*       79 CAN2 TX intr. */
	(void (*)()) 0,/*       80 CAN2 RX0 intr. */
	(void (*)()) 0,/*       81 CAN2 RX1 intr. */
	(void (*)()) 0,/*       82 CAN2 CSE */
	(void (*)()) 0,/*       83 USB On The Go FS global int */
	(void (*)()) 0,/*       84 DMA2 Stream 5 global intr. */
	(void (*)()) 0,/*       85 DMA2 Stream 6 global intr. */
	DMA2_stream7,/*         86 DMA2 Stream 7 global intr. : USART1 TX */
	(void (*)()) 0,/*       87 USART6 global intr. */
	I2C3_Event,/*           88 I2C3 event intr. */
	I2C3_Error,/*           89 I2C3 error intr. */
	(void (*)()) 0,/*       90 USB On The Go HS End Point 1 Out	global intr. */
	(void (*)()) 0,/*       91 USB On The Go HS End Point 1 In global intr. */
	(void (*)()) 0,/*       92 USB On The Go HS Wakeup through EXTI intr. */
	(void (*)()) 0,/*       93 USB On The Go HS global intr. */
	(void (*)()) 0,/*       94 DCMI global interrupt */
	(void (*)()) 0,/*       95 Reserved */
	(void (*)()) 0,/*       96 Reserved */
	(void (*)()) 0,/*       97 FPU global intr. */
	(void (*)()) 0,/*       98 Reserved */
	(void (*)()) 0,/*       99 Reserved */
	(void (*)()) 0,/*      100 SPI4 global intr. */
	(void (*)()) 0,/*      101 Reserved */
	(void (*)()) 0,/*      102 Reserved */
	(void (*)()) 0,/*      103 SAI1 global intr. */
	(void (*)()) 0,/*      104 Reserved */
	(void (*)()) 0,/*      105 Reserved */
	(void (*)()) 0,/*      106 Reserved */
	(void (*)()) 0,/*      107 SAI2 global intr. */
	(void (*)()) 0,/*      108 Quad SPI global intr. */
	(void (*)()) 0,/*      109 HDMI-CEC global intr. */
	(void (*)()) 0,/*      110 SPDIF-Rx global intr. */
	(void (*)()) 0,/*      111 FMPI2C1 event intr. */
	(void (*)()) 0,/*      112 FMPI2C1 error intr. */
	(void (*)()) 0,/*      113 Reserved */
	(void (*)()) 0,/*      114 Reserved */
	(void (*)()) 0,/*      115 Reserved */
	(void (*)()) 0,/*      116 Reserved */
	(void (*)()) 0,/*      117 Reserved */
	(void (*)()) 0,/*      118 Reserved */
	(void (*)()) 0,/*      119 Reserved */
	(void (*)()) 0,/*      120 Reserved */
	(void (*)()) 0,/*      121 Reserved */
	(void (*)()) 0,/*      122 Reserved */
	(void (*)()) 0,/*      123 Reserved */
	(void (*)()) 0,/*      124 Reserved */
	(void (*)()) 0,/*      125 Reserved */
	(void (*)()) 0,/*      126 Reserved */
	(void (*)()) 0,/*      127 Reserved */
};
