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

#ifndef EXTI_H
#define EXTI_H

#define SYSCFG_MEMRMP       (*(vuint32_t*)(SYSCFG_BASE + 0x00))
#define SYSCFG_PMC          (*(vuint32_t*)(SYSCFG_BASE + 0x04))
#define SYSCFG_EXTICR1      (*(vuint32_t*)(SYSCFG_BASE + 0x08))
#define SYSCFG_EXTICR2      (*(vuint32_t*)(SYSCFG_BASE + 0x0C))
#define SYSCFG_EXTICR3      (*(vuint32_t*)(SYSCFG_BASE + 0x10))
#define SYSCFG_EXTICR4      (*(vuint32_t*)(SYSCFG_BASE + 0x14))
#define SYSCFG_CMPCR        (*(vuint32_t*)(SYSCFG_BASE + 0x20))
#define SYSCFG_CFGR         (*(vuint32_t*)(SYSCFG_BASE + 0x2C))
#define SYSCFG_EXTICR       (*(vuint32_t*)(SYSCFG_BASE + 0x08))

#define EXTI_BASE           (0x40013C00)
#define EXTI_IMR            (*(vuint32_t*)(EXTI_BASE + 0x00))
#define EXTI_EMR            (*(vuint32_t*)(EXTI_BASE + 0x04))
#define EXTI_RTSR           (*(vuint32_t*)(EXTI_BASE + 0x08))
#define EXTI_FTSR           (*(vuint32_t*)(EXTI_BASE + 0x0C))
#define EXTI_SWIER          (*(vuint32_t*)(EXTI_BASE + 0x10))
#define EXTI_PR             (*(vuint32_t*)(EXTI_BASE + 0x14))

void EXTI_Select_Edges (unsigned ln, _Bool re, _Bool fe);
void EXTI_Select_srce_input (unsigned ln, unsigned nb);
void EXTI_Intr_enable_line (unsigned ln, bool en);
void EXTI_Event_enable_line (unsigned ln, bool en);
_inline void EXTI_clr_pending_bit (unsigned bn) {EXTI_PR |= 0x1 << bn;}
_inline bool EXTI_get_pending_bit (unsigned bn) {return EXTI_PR | (0x1 << bn);}

/*
__attribute__ ((interrupt("irq"))) void EXTI0_Handler (void);
__attribute__ ((interrupt("irq"))) void EXTI1_Handler (void);
__attribute__ ((interrupt("irq"))) void EXTI2_Handler (void);
__attribute__ ((interrupt("irq"))) void EXTI3_Handler (void);
__attribute__ ((interrupt("irq"))) void EXTI4_Handler (void);
__attribute__ ((interrupt("irq"))) void EXTI5_9_Handler (void);
__attribute__ ((interrupt("irq"))) void EXTI10_15_Handler (void);
*/

void EXTI0_Handler (void);
void EXTI1_Handler (void);
void EXTI2_Handler (void);
void EXTI3_Handler (void);
void EXTI4_Handler (void);
void EXTI5_9_Handler (void);
void EXTI10_15_Handler (void);




#endif  /* #ifdef EXTI_H */
