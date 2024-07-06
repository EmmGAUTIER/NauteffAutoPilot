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

#ifndef SYS_H_
#define SYS_H_

#include <sys_config.h>
#include <stm32F446.h>

#define __INTERRUPT__ #pragma GCC interrupt

void sys_init(void);
void sys_init_clocks(void);
void sys_init_Data(void);
void FPU_enable(void);
void sys_init_GPIOs(void);
void sys_init_DMA(void);
void sys_init_USART1(void);
void sys_init_I2C(void);
void sys_init_ADC(void);
void sys_init_EXTI(void);

extern unsigned _sidata;
extern unsigned _edata;
extern unsigned _sbss;
extern unsigned _sdata;
extern unsigned _ebss;
extern unsigned _estack;
extern unsigned _srodata;
extern unsigned _erodata;
extern unsigned _etext;
extern unsigned _la_data;

__attribute__ ((interrupt("irq"))) void NMI_Handler(void);
__attribute__ ((interrupt("irq"))) void HardFault_Handler(void);
__attribute__ ((interrupt("irq"))) void MemManage_Handler(void);
__attribute__ ((interrupt("irq"))) void BusFault_Handler(void);
__attribute__ ((interrupt("irq"))) void UsageFault_Handler(void);

#endif /* SYS_H_ */
