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
 *
 */

#ifndef STM32F446_H_
#define STM32F446_H_

#include <util.h>
#include "stddef.h"

/* Memory map */

#define FLASH_BASE      0x08000000
#define SRAM_BASE       0x20000000
#define PERIPH_BASE     0x40000000
#define APB2_BASE       (PERIPH_BASE + 0x10000)
#define APB1_BASE       (PERIPH_BASE + 0x00000)
#define AHB1_BASE       (PERIPH_BASE + 0x20000)
#define AHB2_BASE       (PERIPH_BASE + 0x10000000)
#define AHB3_BASE       (0x60000000)
#define _FLASH_ACR      (*(vuint32_t)0x40022000)

#define SYSCFG_BASE (0x40013800)

#define CPACR  (*(vuint32_t*)0xE000ED88) /*Coprocessor access control register (CPACR) on  */
#define FPCCR  (*(vuint32_t*)0xE000EF34) /*Floating-point context control register (FPCCR) */
#define FPCAR  (*(vuint32_t*)0xE000EF38) /*Floating-point context address register (FPCAR) */
#define FPDSCR (*(vuint32_t*)0xE000EF3C) /*Floating-point default status control register  */
/* #define  FPSCR   ?????(not mapped) Floating-point status control register*/

/**
 * GPIO Bases
 */




#endif /* #ifndef _STM32F446RE_*/

