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

#ifndef NVIC_H
#define NVIC_H

#include "util.h"
#include "stdbool.h"

#define NVIC_BASE    (0xE000E000)
#define NVIC_ISER0   (*(vuint32_t*)(NVIC_BASE + 0x100))
#define NVIC_ISER1   (*(vuint32_t*)(NVIC_BASE + 0x104))
#define NVIC_ISER2   (*(vuint32_t*)(NVIC_BASE + 0x108))
#define NVIC_ISER3   (*(vuint32_t*)(NVIC_BASE + 0x10C))
#define NVIC_ISER4   (*(vuint32_t*)(NVIC_BASE + 0x110))
#define NVIC_ISER5   (*(vuint32_t*)(NVIC_BASE + 0x114))
#define NVIC_ISER6   (*(vuint32_t*)(NVIC_BASE + 0x118))
#define NVIC_ISER7   (*(vuint32_t*)(NVIC_BASE + 0x11C))

#define NVIC_ICER0   (*(vuint32_t)(NVIC_BASE + 0x180))
#define NVIC_ICER1   (*(vuint32_t)(NVIC_BASE + 0x184))
#define NVIC_ICER2   (*(vuint32_t)(NVIC_BASE + 0x188))
#define NVIC_ICER3   (*(vuint32_t)(NVIC_BASE + 0x18C))
#define NVIC_ICER4   (*(vuint32_t)(NVIC_BASE + 0x190))
#define NVIC_ICER5   (*(vuint32_t)(NVIC_BASE + 0x194))
#define NVIC_ICER6   (*(vuint32_t)(NVIC_BASE + 0x198))
#define NVIC_ICER7   (*(vuint32_t)(NVIC_BASE + 0x19C))

#define NVIC_ISPR0   (*(vuint32_t)(NVIC_BASE + 0x200))
#define NVIC_ISPR1   (*(vuint32_t)(NVIC_BASE + 0x204))
#define NVIC_ISPR2   (*(vuint32_t)(NVIC_BASE + 0x208))
#define NVIC_ISPR3   (*(vuint32_t)(NVIC_BASE + 0x20C))
#define NVIC_ISPR4   (*(vuint32_t)(NVIC_BASE + 0x210))
#define NVIC_ISPR5   (*(vuint32_t)(NVIC_BASE + 0x214))
#define NVIC_ISPR6   (*(vuint32_t)(NVIC_BASE + 0x218))
#define NVIC_ISPR7   (*(vuint32_t)(NVIC_BASE + 0x21C))

#define NVIC_ICPR0   (*(vuint32_t)(NVIC_BASE + 0x280))
#define NVIC_ICPR1   (*(vuint32_t)(NVIC_BASE + 0x284))
#define NVIC_ICPR2   (*(vuint32_t)(NVIC_BASE + 0x288))
#define NVIC_ICPR3   (*(vuint32_t)(NVIC_BASE + 0x28C))
#define NVIC_ICPR4   (*(vuint32_t)(NVIC_BASE + 0x290))
#define NVIC_ICPR5   (*(vuint32_t)(NVIC_BASE + 0x294))
#define NVIC_ICPR6   (*(vuint32_t)(NVIC_BASE + 0x298))
#define NVIC_ICPR7   (*(vuint32_t)(NVIC_BASE + 0x29C))

#define NVIC_IABR0   (*(vuint32_t)(NVIC_BASE + 0x300))
#define NVIC_IABR1   (*(vuint32_t)(NVIC_BASE + 0x304))
#define NVIC_IABR2   (*(vuint32_t)(NVIC_BASE + 0x308))
#define NVIC_IABR3   (*(vuint32_t)(NVIC_BASE + 0x30C))
#define NVIC_IABR4   (*(vuint32_t)(NVIC_BASE + 0x310))
#define NVIC_IABR5   (*(vuint32_t)(NVIC_BASE + 0x314))
#define NVIC_IABR6   (*(vuint32_t)(NVIC_BASE + 0x318))
#define NVIC_IABR7   (*(vuint32_t)(NVIC_BASE + 0x31C))

#define NVIC_NUM_IRQ_WWDG                0
#define NVIC_NUM_IRQ_PVD                 1
#define NVIC_NUM_IRQ_TAMP_STAMP          2
#define NVIC_NUM_IRQ_RTC_WKUP            3
#define NVIC_NUM_IRQ_FLASH               4
#define NVIC_NUM_IRQ_RCC                 5
#define NVIC_NUM_IRQ_EXTI0               6
#define NVIC_NUM_IRQ_EXTI1               7
#define NVIC_NUM_IRQ_EXTI2               8
#define NVIC_NUM_IRQ_EXTI3               9
#define NVIC_NUM_IRQ_EXTI4              10
#define NVIC_NUM_IRQ_DMA1_Stream0       11
#define NVIC_NUM_IRQ_DMA1_Stream1       12
#define NVIC_NUM_IRQ_DMA1_Stream2       13
#define NVIC_NUM_IRQ_DMA1_Stream3       14
#define NVIC_NUM_IRQ_DMA1_Stream4       15
#define NVIC_NUM_IRQ_DMA1_Stream5       16
#define NVIC_NUM_IRQ_DMA1_Stream6       17
#define NVIC_NUM_IRQ_ADC                18
#define NVIC_NUM_IRQ_CAN1_TX            19
#define NVIC_NUM_IRQ_CAN1_RX0           20
#define NVIC_NUM_IRQ_CAN1_RX1           21
#define NVIC_NUM_IRQ_CAN1_SCE           22
#define NVIC_NUM_IRQ_EXTI9_5            23
#define NVIC_NUM_IRQ_TIM1_BRK_TIM9      24
#define NVIC_NUM_IRQ_TIM1_UP_TIM10      25
#define NVIC_NUM_IRQ_TIM1_TRG_COM_TIM11 26
#define NVIC_NUM_IRQ_TIM1_CC            27
#define NVIC_NUM_IRQ_TIM2               28
#define NVIC_NUM_IRQ_TIM3               29
#define NVIC_NUM_IRQ_TIM4               30
#define NVIC_NUM_IRQ_I2C1_EV            31
#define NVIC_NUM_IRQ_I2C1_ER            32
#define NVIC_NUM_IRQ_I2C2_EV            33
#define NVIC_NUM_IRQ_I2C2_ER            34
#define NVIC_NUM_IRQ_SPI2               36
#define NVIC_NUM_IRQ_USART1             37
#define NVIC_NUM_IRQ_USART2             38
#define NVIC_NUM_IRQ_USART3             39
#define NVIC_NUM_IRQ_EXTI15_10          40
#define NVIC_NUM_IRQ_RTC_Alarm          41
#define NVIC_NUM_IRQ_OTG_FS_WKUP        42
#define NVIC_NUM_IRQ_TIM8_BRK_TIM12     43
#define NVIC_NUM_IRQ_TIM8_UP_TIM13      44
#define NVIC_NUM_IRQ_TIM8_TRG_COM_TIM14 45
#define NVIC_NUM_IRQ_TIM8_CC            46
#define NVIC_NUM_IRQ_DMA1_Stream7       47
#define NVIC_NUM_IRQ_FMC                48
#define NVIC_NUM_IRQ_SDIO               49
#define NVIC_NUM_IRQ_TIM5               50
#define NVIC_NUM_IRQ_SPI3               51
#define NVIC_NUM_IRQ_UART4              52
#define NVIC_NUM_IRQ_UART5              53
#define NVIC_NUM_IRQ_TIM6_DAC           54
#define NVIC_NUM_IRQ_TIM7               55
#define NVIC_NUM_IRQ_DMA2_Stream0       56
#define NVIC_NUM_IRQ_DMA2_Stream1       57
#define NVIC_NUM_IRQ_DMA2_Stream2       58
#define NVIC_NUM_IRQ_DMA2_Stream3       59
#define NVIC_NUM_IRQ_DMA2_Stream4       60
#define NVIC_NUM_IRQ_CAN2_TX            63
#define NVIC_NUM_IRQ_CAN2_RX0           64
#define NVIC_NUM_IRQ_CAN2_RX1           65
#define NVIC_NUM_IRQ_CAN2_SCE           66
#define NVIC_NUM_IRQ_OTG_FS             67
#define NVIC_NUM_IRQ_DMA2_Stream5       68
#define NVIC_NUM_IRQ_DMA2_Stream6 69
#define NVIC_NUM_IRQ_DMA2_Stream7 70
#define NVIC_NUM_IRQ_USART6 71
#define NVIC_NUM_IRQ_I2C3_EV 72
#define NVIC_NUM_IRQ_I2C3_ER 73
#define NVIC_NUM_IRQ_OTG_HS_EP1_OUT 74
#define NVIC_NUM_IRQ_OTG_HS_EP1_IN 75
#define NVIC_NUM_IRQ_OTG_HS_WKUP 76
#define NVIC_NUM_IRQ_OTG_HS 77
#define NVIC_NUM_IRQ_DCMI 78
#define NVIC_NUM_IRQ_FPU 81
#define NVIC_NUM_IRQ_SAI2 91
#define NVIC_NUM_IRQ_QuadSPI 92
#define NVIC_NUM_IRQ_HDMI_CEC 93
#define NVIC_NUM_IRQ_SPDIF_Rx 94
#define NVIC_NUM_IRQ_FMPI2C1 95
#define NVIC_NUM_IRQ_FMPI2C1_ERR 96

/* Attention : non compris les exceptions (interruptions système)*/
#define NVIC_IPR_BASE ((char*)(volatile unsigned char*) 0xE000E100 + 0x300)

#define NVIC_ISER ((vuint32_t*)(NVIC_BASE + 0x100))
#define NVIC_ICPR ((vuint32_t*)(NVIC_BASE + 0x280))
#define NVIC_IPR ((vuint32_t*)(NVIC_BASE + 0x400))

INLINE void NVIC_enable_IRQ(unsigned irqn) {
	NVIC_ISER[irqn / 32] |= 0x1 << (irqn % 32U);
}

INLINE void NVIC_disable_IRQ(unsigned irqn) {
	NVIC_ISER[irqn / 32] &= ~(0x1 << (irqn % 32U));
}

INLINE void NVIC_clear_pending_IRQ(unsigned irqn) {
	NVIC_ICPR[irqn / 32] |= (0x1 << (irqn % 32U));
}

#if 0
/*Sets the pending status of interrupt or exception to 1. */
INLINE void NVIC_set_pending_IRQ (unsigned IRQn)
{
	;
}

/*  Clears the pending status of interrupt or exception to 0.*/
INLINE void NVIC_clear_pending_IRQ (unsigned IRQn)
{
	;
}

/*
 *  @brief Reads the pending status of interrupt or exception.
 *  @return true if IRQ is pending else false
 */
INLINE bool NVIC_get_pending_IRQ_status (unsigned IRQn)
{
	;
}
#endif
/*
 * Sets the priority of an interrupt or exception
 * with configurable priority level to 1.
 */
INLINE void NVIC_SetPriority(unsigned IRQn, uint32_t priority) {
	NVIC_IPR[IRQn / 4] &= ~(0xF << (IRQn % 4));
	NVIC_IPR[IRQn / 4] |= ~(priority << (IRQn % 4));
}

#if 0
/*
 * Reads the priority of an interrupt or exception
 * with configurable priority level.
 * This function return the current priority level.
 */

uint32_t NVIC_GetPriority(IRQn_Type IRQn)
#endif

#endif
