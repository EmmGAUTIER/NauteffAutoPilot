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
#include "rcc.h"

/* These variables store some frequencies used by peripherals
 * they are accessed by RCC_get_XXXX_Freq_Hz() functions.
 */
static uint32_t SYSCLK_Freq_Hz = 0;
static uint32_t AHB_Freq_Hz = 0;
static uint32_t APB1_Freq_Hz = 0;
static uint32_t APB2_Freq_Hz = 0;
static uint32_t APB1_Timer_Freq_Hz = 0;
static uint32_t APB2_Timer_Freq_Hz = 0;

int RCC_init_for_nauteff(RCC_config_t *cfg) {

	/*----- RCC_PLLCFGR register -----
	 * Input : HSE (cristal) at 16 Mhz, division factor M=4
	 * so input clock for VCO is 2 Mhz as recommended by doc.
	 * VCO. multiplication factor is the requested freq. in MHz
	 * so its output runs at twice the requested freq. of SYSCLK
	 * it is divided by 2
	 *  PLLR   division factor 2, value 0x2 in bits 28 to 30
	 *  PLLQ   division factor 2, value 0x2 in bits 24 à 27
	 *         these clocks are not used for Nauteff project
	 *         0 and 1 are wrong values according to documentation
	 *  PLLSRC High speed external (HSE, quartz), value 0x1 in bit 22
	 *  PLLP   division factor 2, value  0x2 in bits 16..17
	 *  PLLN   mult. factor of VCO in bits 6..14
	 *  PLLM   division factor 4, value 100b in bits 0..5
	 */

	RCC_PLLCFGR = (0x02 << 28) | (0x02 << 24) | (0x01 << 22) | (0x2 << 16)
			| (cfg->SYSCLK_Freq_Mhz << 6) | (0x4 << 0);

	/*----- RCC_CFGR -----
	 * MCO & RTCPRE configurations if used later APB1,2 and AHB prescalers and
	 * system clock switch.
	 * PPRE2  APB2 prescaler value from macro APBX_PRESCALER_X in bits 13..15
	 * PPRE1  APB1 prescaler value from macro APBX_PRESCALER_X in bits 10..12
	 * HPRE   AHB prescaler value from macro AHB_PRESCALER__X in bits 4..7
	 * SW     system clock switch HSE, value 0x1 in bits 0..1
	 */

	RCC_CFGR = (cfg->APB2_2pwr_Prescaler << 13)
			| (cfg->APB1_2pwr_Prescaler << 10) | (cfg->AHB_2pwr_Prescaler << 4);

	/* HSEBYP High Speed External BYPass on since we use external clock */
	/* bit 18 in RCC_CR */
	RCC_CR |= 0x01 << 18;

	/* then set HSEON High Speed External ON, bit 16 in RCC_CR */
	RCC_CR |= 0x01 << 16;

	/* Wait for HSE ready by testing bit 17, HSERDY, in RCC_CR */
	while ((RCC_CR & (0x01 << 17)) == 0x00) {
		;
	}

	/* Start main PLL by setting PLLON bit 24 in RCC_CR */
	RCC_CR |= (0x01 << 24);

	/* Wait for main PLL to be ready by testing bit 25 PLLRDY in RCC_CR*/
	while ((RCC_CR & (0x01 << 25)) == 0x00) {
		;
	}

	/* Select PLL_P for SYSCLK */
	RCC_CFGR |= (0x2 << 0);

	if (cfg->AHB_2pwr_Prescaler == 0) {
		AHB_Freq_Hz = cfg->SYSCLK_Freq_Mhz * 1000000;
	} else {
		AHB_Freq_Hz = cfg->SYSCLK_Freq_Mhz * 1000000
				/ (2 << (cfg->AHB_2pwr_Prescaler & 0x3));
	}

	if (cfg->APB1_2pwr_Prescaler == 0) {
		APB1_Freq_Hz = AHB_Freq_Hz;
	} else {
		APB1_Freq_Hz = AHB_Freq_Hz / (2 << (cfg->APB1_2pwr_Prescaler & 0x2));
	}

	if (cfg->APB2_2pwr_Prescaler == 0) {
		APB2_Freq_Hz = AHB_Freq_Hz;
	} else {
		APB2_Freq_Hz = AHB_Freq_Hz / (2 << (cfg->APB2_2pwr_Prescaler & 0x2));
	}

	APB1_Timer_Freq_Hz =
			(cfg->APB1_2pwr_Prescaler == 0) ? APB1_Freq_Hz : APB1_Freq_Hz * 2;

	APB2_Timer_Freq_Hz =
			(cfg->APB2_2pwr_Prescaler == 0) ? APB2_Freq_Hz : APB2_Freq_Hz * 2;

	return 0;
}

uint32_t RCC_get_APB1_Freq_Hz() {
	return APB1_Freq_Hz;
}

uint32_t RCC_get_APB2_Freq_Hz() {
	return APB2_Freq_Hz;
}

uint32_t RCC_get_SYSCLK_Freq_Hz() {
	return SYSCLK_Freq_Hz;
}

/**
 * @ brief returns the frequency in Hz of clock for APBI timers
 * @return clock frequency for timers on APB1
 */
uint32_t RCC_get_APB1_Timer_Freq_Hz() {
	return APB1_Timer_Freq_Hz;
}

uint32_t RCC_get_APB2_Timer_Freq_Hz() {
	return APB2_Timer_Freq_Hz;
}

