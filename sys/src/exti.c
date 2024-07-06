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
#include "stm32F446.h"
#include "util.h"
#include "gpio.h"
#include "exti.h"


void EXTI_Select_Edges (unsigned ln, _Bool re, _Bool fe)
{
    if (re)
        {
            EXTI_RTSR |= (0x01U << ln);
        }
    else
        {
            EXTI_RTSR &= ~(0x1 << ln);
        }
    if (fe)
        {
            EXTI_FTSR |= (0x01U << ln);
        }
    else
        {
            EXTI_FTSR &= ~(0x1 << ln);
        }
}

void EXTI_Select_srce_input (unsigned ln, unsigned nb)
{
    /* doc : RM090, chap. SYSCFG */
    vuint32_t *reg;
    unsigned bitshift;

    /*4 registers SYSCFG_EXTICR1..4 containing 4 numbers of 4bits each
     * Register is
     */
    reg = &(SYSCFG_EXTICR) + ln / 4U;
    bitshift = (ln % 4U) * 4U;
    /* effacement des bits */
    *reg &= ~((0xFU) << bitshift);
    *reg = (nb << bitshift);
}

void EXTI_Intr_enable_line (unsigned ln, bool en)
{
    if (en)
        {
            EXTI_IMR |= (0x1U) << ln;
        }
    else
        {
            EXTI_IMR &= ~(0x1U << ln);
        }
}

void EXTI_Event_enable_line (unsigned ln, bool en)
{
    if (en)
        {
            EXTI_EMR |= (0x1U) << ln;
        }
    else
        {
            EXTI_EMR &= ~(0x1U << ln);
        }
}

#if 0
void EXTI_get
#endif

__attribute__ ((interrupt("irq")))
void EXTI0_Handler (void)
{
    EXTI_PR = 0x1;
}

__attribute__ ((interrupt("irq")))
void EXTI1_Handler (void)
{
    while (1)
        {
            ;
        }
}

__attribute__ ((interrupt("irq")))
void EXTI2_Handler (void)
{
    while (1)
        {
            ;
        }
}

__attribute__ ((interrupt("irq")))
void EXTI3_Handler (void)
{
    while (1)
        {
            ;
        }
}

__attribute__ ((interrupt("irq")))
void EXTI4_Handler (void)
{
    while (1)
        {
            ;
        }
}

__attribute__ ((interrupt("irq")))
void EXTI5_9_Handler (void)
{
    EXTI_clr_pending_bit(5U);
    EXTI_clr_pending_bit(6U);
    EXTI_clr_pending_bit(7U);
    EXTI_clr_pending_bit(8U);
    EXTI_clr_pending_bit(9U);
}

__attribute__ ((interrupt("irq")))
void EXTI10_15_Handler (void)
{
    static unsigned nb = 0;

    nb++;
    uint32_t blueButton;

    blueButton = GPIOA->IDR & (0x01U << 13U);
    (void)blueButton;


    EXTI_clr_pending_bit(10U);
    EXTI_clr_pending_bit(11U);
    EXTI_clr_pending_bit(12U);
    EXTI_clr_pending_bit(13U);
    EXTI_clr_pending_bit(14U);
    EXTI_clr_pending_bit(15U);

}
