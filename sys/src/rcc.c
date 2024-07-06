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
#include "flash.h"
#include "rcc.h"

void RCC_init_for_nauteff(void) {

	/* Flash cannot be read when HCLK is over 30Mhz with 3.3V
	 * See sections 3.4 and 3.8.1 Embedded flash memory interface
	 * In FLASH_ACR set :
	 * Latency 2 CPU cycles, bits 3..0, enable Prefetch and instruction cache
	 * 2 CPU cycles latency allows 90 MHz at 3.3V.
	 * PRFTEN  : Prefetch enable bit 8
	 * ICEN    : Instruction cache enable, bit 9
	 * LATENCY : 2 in bits 0..3
	 */

	FLASH_ACR = (0x1 << 8) | (0x1 << 9) | (5U << 0);

	/* Set HSEON High Speed External ON, bit 16 in RCC_CR */
	RCC_CR |= 0x01 << 16 | 0x01 << 18;

	/* HSEBYP High Speed External BYPass on since we use external clock */
	/* bit 18 in RCC_CR */
	RCC_CR |= 0x01 << 18;

	int i = 0;

	/* Wait for HSE ready by testing bit 17, HSERDY, in RCC_CR */
	while ((RCC_CR & (0x01 << 17)) == 0x00) {
		i++;
	}

	/* @formatter:off */
	/*----- RCC_PLL CFGR register -----
	 * Input : HSE (cristal) at 8 Mhz (PLLSRC set to 1 in RCC_PLLCFGR).
	 * Main PLL : Input recomended freq. of VCO is 2MHz (cf RM0390 6.3.2)
	 * so division factor at entry of PLL is 4 (PLLM)
	 * Output frequency of VCO has to be between 100 and 432 MHz
	 * With a multiplication factor of 90 VCO output frequency is 180 MHz
	 * so PLLN[8:0] is 90
	 * PLLP division factor is 2 so PLLCLK is 90MHz
	 *
	 *  PLLR   : 7U   in bits 28..30
	 *  PLLQ   : 15U  in bits 24..27
	 *  PLLSRC : 1    in bit 22
	 *  PLLP   : 0    in bits 16..17 (00b for dividing by 2)
	 *  PLLN   : 90   in bits 6..14
	 *  PLLM   : 4    in bits 0..5
	 */

	RCC_PLLCFGR = (7    << 28) |
			      (15   << 24) |
				  (0x0  << 22) |
				  (0x1  << 16) |
				  (90   <<  6) |
				  (4    <<  0) ;

	/*----- RCC_CFGR -----
	 * MCO2    MCO2 uses PLLCLK (11b) and uses a division factor of 5
	 * AHB not divided : 90MHz(HPRE)
	 * APB1(PPRE1) & APB2(PPRE2) divided by 2 45MHz
	 * System clock switch selects PLLP aka PLLCLK
	 * MCO2PRE 0x3, division by 5
	 * MCO1 and RTC unused and left
	 * MCO2    : 11b   in bits  30..31
	 * MCO2PRE : 111b  in bits  27..29
	 * PPRE2   : 100b  in bits  13..15
	 * PPRE1   : 100b  in bits  10..12
	 * HPRE    : 0000b in bits  4..7
	 * SW      : 10b   in bits  0..1
	 */

	RCC_CFGR = (0x0 << 30) | // SYSCLK selected
               (0x7 << 27) |
               (0x4 << 13) |
               (0x4 << 10) |
               (0x0 <<  4) |
			   (0x2 <<  0) ;

	/* Start main PLL by setting PLLON bit 24 in RCC_CR */
	RCC_CR |= (0x01 << 24);

	/* Wait for main PLL to be ready by testing bit 25 PLLRDY in RCC_CR*/
	int j = 0; // for debugging purpose, to be removed later
	while ((RCC_CR & (0x01 << 25)) == 0x00) {
		j++;
	}

	/* Select PLLCLK aka PLL_P for SYSCLK */
	RCC_CFGR &= ~(0x2 << 0);
	RCC_CFGR |= (0x2 << 0);

	/* @formatter:on */

    return;
}

