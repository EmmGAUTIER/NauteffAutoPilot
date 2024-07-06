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
 * \version  first
 * \date  February, 23 2022
 * \param  none
 * \return  none
 */

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "event_groups.h"
#include "semphr.h"
#include <stdbool.h>
#include "stm32F446.h"
#include "nassert.h"
#include "gpio.h"
#include "sys.h"
#include "nvic.h"
#include "dma.h"
#include "rcc.h"
#include "gpio.h"
#include "usart.h"
#include "i2c.h"
#include "adc.h"
#include "exti.h"
#include "nvic.h"

void main(void);
void xPortPendSVHandler(void) __attribute__ (( naked ));
void xPortSysTickHandler(void);
void vPortSVCHandler(void) __attribute__ (( naked ));

void start(void);

void start(void) {

	unsigned *src;
	unsigned *dst;

	/*----- Data initialization -----*/
	/* Copy initialized data from flash and zero bss */
	/* Values of initialized var are stored in flash and have to be copied in RAM */
	/* bss have to be zeros for compliance with standards : C, MISRA-C */
	/* copy the data segment from FLASH to RAM */
	/* Copy is made with 32 bit words */
	/* data areas (FLASH and RAM) are 4-byte aligned */
	/* &_sidata is the address of initialized data in FLASH */
	/* &_sdata is the first address of initialized data in RAM */
	/* &_edata is the address of the end of initialized data in RAM*/
	/* see linker script (.ld file) */
#if 0
	src = &_sidata;
	dst = &_sdata;
	while (dst < &_edata)
		*dst++ = *src++;
#else
    src = &_la_data;
	dst = &_sdata;
	while (dst < &_edata)
		*dst++ = *src++;
#endif

	/* zero the bss segment */
	dst = &_sbss;
	while (dst < &_ebss)
		*(dst++) = 0;

	/*-----  -----*/
	rcc_start_SYSCFG();

	/*----- Clocks configuration -----*/
	RCC_init_for_nauteff();
#if 0
	RCC_config_t rcc_config = { 1, AHB_PRESCALER_NO_DEVIDE,
	APBX_PRESCALER_DEVIDED_BY_4,
	APBX_PRESCALER_DEVIDED_BY_2 };
	RCC_init_for_nauteff(&rcc_config);
#endif

	/*----- FPU Start -----*/
	FPU_enable();

	/*----- Peripherals initialization -----*/
	//DMA_init_all();
	ADC_init_all();
	USART_init_all();

	/*----- Initialization des GPIO -----*/
	/* See docs for pin use and config */
	/* Start GPIOA..D before */
	rcc_start_GPIOA();
	rcc_start_GPIOB();
	rcc_start_GPIOC();
	rcc_start_GPIOD();

	/* Set pins individually */
	/*@formatter:off  */
    GPIO_init_pin_adc (GPIOA, 0);
    GPIO_init_pin_adc (GPIOA, 1);
	/* USART2 TX : PA2 / D1,  RX : PA3 / D0  AF 7 */
    GPIO_init_pin_altfct (GPIOA, 2, GPIO_OTYPER_PUSHPULL,GPIO_OSPEEDR_MEDIUMSPEED,GPIO_PUPDR_NONE, 7);
    GPIO_init_pin_altfct (GPIOA, 3, GPIO_OTYPER_PUSHPULL,GPIO_OSPEEDR_MEDIUMSPEED,GPIO_PUPDR_NONE, 7);
    GPIO_init_pin_output (GPIOA, 4, GPIO_OTYPER_PUSHPULL, GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE);
    /* GPIOA, pin 5 : Output, slow, push/pull : clutch */
    GPIO_init_pin_output (GPIOA, 5, GPIO_OTYPER_PUSHPULL, GPIO_OSPEEDR_LOWSPEED, GPIO_PUPDR_NONE);
    GPIO_init_pin_unused (GPIOA, 6);
    GPIO_init_pin_unused (GPIOA, 7);
    GPIO_init_pin_unused (GPIOA, 8);

    /* USART 1 TX (PA9) et RX (PA10) */
    GPIO_init_pin_altfct (GPIOA, 9, GPIO_OTYPER_PUSHPULL,GPIO_OSPEEDR_MEDIUMSPEED,GPIO_PUPDR_NONE, 7);
    GPIO_init_pin_altfct (GPIOA, 10, GPIO_OTYPER_PUSHPULL,GPIO_OSPEEDR_MEDIUMSPEED,GPIO_PUPDR_NONE, 7);
    /*GPIO_init_pin_unused (GPIOA, 10);*/
    /*GPIO_init_pin_altfct (GPIOA, 9, GPIO_OTYPER_PUSHPULL,GPIO_OSPEEDR_MEDIUMSPEED,GPIO_PUPDR_NONE, 7);*/
    /*GPIO_init_pin_altfct (GPIOA, 10, GPIO_OTYPER_OPENDRAIN, GPIO_OSPEEDR_MEDIUMSPEED, GPIO_PUPDR_NONE, 7);*/

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
    /* Main clock output 2 : MCO2 */
    GPIO_init_pin_altfct (GPIOC, 9, GPIO_OTYPER_PUSHPULL, GPIO_OSPEEDR_HIGHSPEED, GPIO_OTYPER_PUSHPULL, 0);

    GPIO_init_pin_altfct (GPIOC, 10, GPIO_OTYPER_PUSHPULL,GPIO_OSPEEDR_MEDIUMSPEED,GPIO_PUPDR_NONE, 8);
    GPIO_init_pin_altfct (GPIOC, 11, GPIO_OTYPER_PUSHPULL,GPIO_OSPEEDR_MEDIUMSPEED,GPIO_PUPDR_NONE, 8);
    GPIO_init_pin_altfct (GPIOC, 12, GPIO_OTYPER_PUSHPULL,GPIO_OSPEEDR_MEDIUMSPEED,GPIO_PUPDR_NONE, 8);
    /* PC13 connected to blue button  */
    GPIO_init_pin_input  (GPIOC, 13, GPIO_PUPDR_NONE);
    /* PC14 & 15 : used by cristal */

    GPIO_init_pin_unused (GPIOD, 0);
    GPIO_init_pin_unused (GPIOD, 1);
    GPIO_init_pin_altfct (GPIOD, 2, GPIO_OTYPER_PUSHPULL,GPIO_OSPEEDR_MEDIUMSPEED,GPIO_PUPDR_NONE, 8);

    /*@formatter:on  */

	/*----- EXTI -----*/

	/*----- DMAs -----*/
	rcc_start_DMA1();
	rcc_start_DMA2();
	//DMA_init_all();


//*((int*)((void*)DMA2 + 0x10 + 0x18*4)) = 0x7 << 25;

	/*DMA_init_Stream(dma1, 0);*/
	/*DMA_CHANNEL_enable(dma1_ch0);*/

	/*----- UARTs -----*/
#if SYS_USE_USART1
	rcc_start_USART1();
	USART_init(usart1);
	USART_set_baudrate(usart1, 38400);
	USART_set_parity_none(usart1);
	USART_set_stop_bits(usart1, twoStopBits);
	USART_enable(usart1);
	NVIC_enable_IRQ(NVIC_NUM_IRQ_USART1);
#endif

#if SYS_USE_USART2
	rcc_start_USART2();
	USART_init(usart2);
	USART_set_baudrate(usart2, 38400);
	USART_set_parity_none(usart2);
	USART_set_stop_bits(usart2, twoStopBits);
	USART_set_buffer_mode(usart2, lineBuffered);
	USART_enable(usart2);
	NVIC_enable_IRQ(NVIC_NUM_IRQ_USART2);
#endif

#if SYS_USE_USART3
	rcc_start_USART3();
	USART_init(usart3);
	USART_set_baudrate(usart3, 4800);
	USART_set_parity_none(usart3);
	USART_set_stop_bits(usart3, oneStopBit);
	USART_enable(usart3);
	NVIC_enable_IRQ(NVIC_NUM_IRQ_USART3);
#endif

#if SYS_USE_USART4
	rcc_start_USART4();
	USART_init(usart4, USART4);
	USART_set_baudrate(usart4, 4800);
	USART_set_parity_none(usart4);
	USART_set_stop_bits(usart4, oneStopBit);
	USART_enable(usart4);
	NVIC_enable_IRQ(NVIC_NUM_IRQ_UART4);
#endif

#if SYS_USE_USART5
	rcc_start_USART5();
	USART_init(usart5);
	USART_set_baudrate(usart5, 115200);
	USART_set_parity_none(usart5);
	USART_set_stop_bits(usart5, oneStopBit);
	USART_enable(usart5);	
	USART_set_buffer_mode(usart5, blockBuffered);
	NVIC_enable_IRQ(NVIC_NUM_IRQ_UART5);
#endif

#if SYS_USE_USART6
	rcc_start_USART6();
	USART_init(usart6, USART6);
	USART_set_baudrate(usart6, 4800);
	USART_set_parity_none(usart6);
	USART_set_stop_bits(usart6, oneStopBit);
	USART_enable(usart6);
	NVIC_enable_IRQ(NVIC_NUM_IRQ_USART6);
#endif

	/*----- ADC1 et ADC2 et ADC3 -----*/
#if SYS_USE_ADC1
	rcc_start_ADC1();
	ADC_set_on(adc1);
	ADC_select_channel(adc1, 1U);
#endif

#if SYS_USE_ADC2
	rcc_start_ADC2();
	ADC_set_on(adc2);
	ADC_select_channel(adc2, 1U);
#endif

#if SYS_USE_ADC3
	rcc_start_ADC3();
	ADC_set_on(adc3);
	ADC_select_channel(adc3, 1U);
#endif

	/*----- I2C -----*/
	rcc_start_I2C1();
	I2C_init(i2c1, I2C1);
	I2C_set_std_speed(i2c1);

	I2C_init(i2c3, I2C3);
	rcc_start_I2C3();
	I2C_set_std_speed(i2c3);

	/*----- So, now let's work -----*/
	main();
}

/*
 * \brief Enable FPU
 * \see PM0214 Programming manual 4.6 floating point unit p. 253
 */
void FPU_enable(void) {

	CPACR |= 0x03 << 20;
}

void sys_init_DMA(void) {
	;
}

void sys_init_USART1(void) {
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
	start,/*                    1 Reset Handler */
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
	(void (*)()) 0,/*       17 PVD through EXTI Line detectcube */
	(void (*)()) 0,/*       18 Tamper and TimeStamp intr through EXTI line */
	(void (*)()) 0,/*       19 RTC_WKUP */
	(void (*)()) 0,/*       20 Flash */
	(void (*)()) 0,/*       21 RCC */
	EXTI0_Handler,/*        22 EXTI Line 0 */
	EXTI1_Handler,/*        23 EXTI Line 1 */
	EXTI2_Handler,/*        24 EXTI Line 2 and touch sensing */
	EXTI3_Handler,/*        25 EXTI Line 3 */
	EXTI4_Handler,/*        26 EXTI Line 4 */
	DMA1_Ch0_Handler,/*     27 DMA1 stream 0 : UART5 RX */
	DMA1_Ch1_Handler,/*     28 DMA1 stream 1 : I2C3 RX */
	DMA1_Ch2_Handler,/*     29 DMA1 stream 2 : I2C3 TX */
	DMA1_Ch3_Handler,/*     30 DMA1 stream 3 */
	DMA1_Ch4_Handler,/*     31 DMA1 stream 4 */
	DMA1_Ch5_Handler,/*     32 DMA1 stream 5 : USART2 RX */
	DMA1_Ch2_Handler,/*     33 DMA1 stream 6 : USART2 TX */
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
	USART2_Event,/*         54 USART2 */
	USART3_Event,/*         55 USART3 */
	EXTI10_15_Handler,/*    56 EXTI Line 15..10 */
	(void (*)()) 0,/*       57 RTC Alarms (A and B) through EXTI Line */
	(void (*)()) 0,/*       58 USB On the Go FS Wakeup through EXTI line */
	(void (*)()) 0,/*       59 TIM8 break intr. and TIM12 global intr. */
	(void (*)()) 0,/*       60 TIM8 Update intr. & TIM13 global intr. */
	(void (*)()) 0,/*       61 TIM8 Trig. & Commutation intr. & TIM14 intr. */
	(void (*)()) 0,/*       62 TIM8 capture compare intr. */
	DMA1_Ch2_Handler,/*     63 DMA1 Stream 7 global intr. : UART5 TX */
	(void (*)()) 0,/*       64 FMC global intr. */
	(void (*)()) 0,/*       65 SDIO global intr. */
	(void (*)()) 0,/*       66 TIM5 global intr. */
	(void (*)()) 0,/*       67 SPI3 global intr. */
	USART4_Event,/*         68 UART 4 intr. */
	USART5_Event,/*         69 UART 5 global intr. */
	(void (*)()) 0,/*       70 TIM6 global intr. and DAC1/2 underrun intr. */
	(void (*)()) 0,/*       71 TIM7 global intr. */
	DMA1_Ch0_Handler,/*     72 DMA2 Stream 0 global intr. */
	DMA1_Ch1_Handler,/*     73 DMA2 Stream 1 global intr. */
	DMA2_Ch2_Handler,/*     74 DMA2 Stream 2 global intr. : USART1 RX */
	DMA2_Ch3_Handler,/*     75 DMA2 Stream 3 global intr. */
	DMA2_Ch4_Handler,/*     76 DMA2 Stream 4 global intr. */
	(void (*)()) 0,/*       77 Reserved */
	(void (*)()) 0,/*       78 Reserved */
	(void (*)()) 0,/*       79 CAN2 TX intr. */
	(void (*)()) 0,/*       80 CAN2 RX0 intr. */
	(void (*)()) 0,/*       81 CAN2 RX1 intr. */
	(void (*)()) 0,/*       82 CAN2 CSE */
	(void (*)()) 0,/*       83 USB On The Go FS global int */
	DMA2_Ch4_Handler,/*     84 DMA2 Stream 5 global intr. */
	DMA2_Ch4_Handler,/*     85 DMA2 Stream 6 global intr. */
	DMA2_Ch7_Handler,/*     86 DMA2 Stream 7 global intr. : Uisr_vectorSART1 TX */
	USART6_Event,/*         87 USART6 global intr. */
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
