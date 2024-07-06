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

#ifndef ADC_H
#define ADC_H

typedef struct {
	vuint32_t SR; /*    Status register */
	vuint32_t CR1; /*   Control Register 1 */
	vuint32_t CR2; /*   Control Register 2 */
	vuint32_t SMPR1; /* Sample time register 1 */
	vuint32_t SMPR2; /* Sample time register 2 */
	vuint32_t JOFR1; /* Injected channel data offset register 1 */
	vuint32_t JOFR2; /*  "          "     "     "       "     2 */
	vuint32_t JOFR3; /*  "          "     "     "       "     3 */
	vuint32_t JOFR4; /*  "          "     "     "       "     4 */
	vuint32_t HTR; /*   Watchdog higher threshold register */
	vuint32_t LTR; /*   Watchdog lower threshold register */
	vuint32_t SQR1; /*  Regular sequence register 1 */
	vuint32_t SQR2; /*     "        "        "    2 */
	vuint32_t SQR3; /*     "        "        "    3 */
	vuint32_t JSQR; /*  Injected sequence register */
	vuint32_t JDR1; /*  Injected data register 1 */
	vuint32_t JDR2; /*  Injected data register 2 */
	vuint32_t JDR3; /*  Injected data register 3 */
	vuint32_t JDR4; /*  Injected data register 4 */
	vuint32_t DR; /*    Regular data register */
} ADC_Regs_t;

typedef struct {
	vuint32_t CSR; /* Common Status Register */
	vuint32_t CCR; /* Common control register */
	vuint32_t CDR; /* common regular data register for dual and triple modes */
} ADC_Common_Regs_Typedef;

typedef struct {
	ADC_Regs_t *registers;
} ADC_Handle;

#define ADC1   ((ADC_Regs_t*)(0x40012000))
#define ADC2   ((ADC_Regs_t*)(0x40012100))
#define ADC3   ((ADC_Regs_t*)(0x40012200))
#define ADC_CR ((ADC_Regs_t*)(0x40012300))

#define adc1 (&adc1_Handle)
#define adc2 (&adc2_Handle)
#define adc3 (&adc3_Handle)

extern ADC_Handle adc1_Handle;
extern ADC_Handle adc2_Handle;
extern ADC_Handle adc3_Handle;

void ADC_init_all(void);

void ADC_init(ADC_Handle *adc);

void ADC_reset(ADC_Handle *adc);

void ADC_select_channel(ADC_Handle*, unsigned n);

INLINE void ADC_set_on(ADC_Handle *adc) {
	adc->registers->CR2 |= 0x1;
}

INLINE void ADC_set_off(ADC_Handle *adc) {
	adc->registers->CR2 &= ~(0x1);
}

INLINE void ADC_start_conversion(ADC_Handle *adc) {
	unsigned n;
	unsigned p;
	adc->registers->CR2 |= 0x1 << 30;
	n = adc->registers->CR2;
	p = n;
	n=p;

}

INLINE unsigned ADC_get_data(ADC_Handle *adc) {
	return adc->registers->DR;
}

#endif /* ADC_H */
