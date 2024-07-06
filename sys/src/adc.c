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

#include "FreeRTOS.h"
#include "semphr.h"
#include "util.h"
#include "sys.h"
#include "nassert.h"
#include "util.h"
#include "rcc.h"
#include "adc.h"


#if SYS_USE_ADC1 | SYS_USE_ADC2 | SYS_USE_ADC3

#endif

#if SYS_USE_ADC1
ADC_Handle adc1_Handle = {
	.registers = ADC1
};
#endif

#if SYS_USE_ADC2
ADC_Handle adc2_Handle = {
	.registers = ADC2
};
#endif

#if SYS_USE_ADC3
ADC_Handle adc3_Handle = {
	.registers = ADC3
};
#endif


#if SYS_USE_ADC1
ADC_Handle adc1_Handle;
#endif

#if SYS_USE_ADC2
ADC_Handle adc2_Handle;
#endif

#if SYS_USE_ADC3
ADC_Handle adc2_Handle;
#endif

void ADC_init_all(void) {

#if SYS_USE_ADC1
	ADC_init(adc1);
#endif

#if SYS_USE_ADC2
	ADC_init(adc2);
#endif

#if SYS_USE_ADC3
	ADC_init(adc3);
#endif

	return;
}

void ADC_init(ADC_Handle *adc) {

	;

	return;
}

void ADC_reset(ADC_Handle *adc) {
	(void) adc;
	return;
}

void ADC_select_channel(ADC_Handle *adc, unsigned n) {

	nassert(n <= 18);
	adc->registers->SQR1 = 0;
	adc->registers->SQR3 = n;

	return;
}
