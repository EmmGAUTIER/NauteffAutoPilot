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

#include "util.h"
#include "sys.h"
#include "rcc.h"
#include "usart.h"


USART_Handle usart_Handle1;
USART_Handle usart_Handle2;
USART_Handle usart_Handle3;
USART_Handle usart_Handle4;
USART_Handle usart_Handle5;
USART_Handle usart_Handle6;

void init_UARTs()
{
	/*
	 void USART
	 void USART_init(&usart_Handle1, USART1);
	 void USART_
	 ;
	 */

	USART_init(&usart_Handle1, USART1);
	USART_set_baudrate(&usart_Handle1, 4800);
	USART_set_parity(&usart_Handle1, noParity);
	USART_set_stop_bits(&usart_Handle1, oneStopBit);
	USART_set_word_length(&usart_Handle1, 8);
}

void USART_init(USART_Handle *uh, USART_Regs_Typedef *regs)
{

	int i;

	uh->baudrate = 0;
	uh->idxStartBuffer = 0;
	uh->idxEndBuffer = 0;
	uh->parity = noParity;
	uh->stopBits = oneStopBit;
	for (i = 0; i < USART_BUFFSIZ; i++)
	{
		uh->buffer[i] = '\0';
	}
	uh->registers = regs;


}

void USART_set_baudrate(USART_Handle *uh, int br)
{
	int clockFreq;
	if ((uh->registers == USART1) || (uh->registers == USART2))
	{
		clockFreq = RCC_get_APB2_Freq_Hz();
	}
	else
	{
		clockFreq = RCC_get_APB2_Freq_Hz();
	}
	uh->registers->BRR = clockFreq / br;
}

void USART_set_stop_bits(USART_Handle *uh, USART_StopBits sb)
{

	uh->registers->CR2 &= ~(0x03 << 12);
	uh->registers->CR2 |= sb << 12;
}

void USART_set_parity(USART_Handle *uh, USART_Parity par)
{
	switch (par)
	{
	case noParity:
		uh->registers->CR1 &= ~(0x01 << 10);
		break;
	case oddParity:
		uh->registers->CR1 |= (0x01 << 10) | (0x01 << 9);
		break;
	case evenParity:
		uh->registers->CR1 |= 0x01 << 10;
		uh->registers->CR1 &= ~(0x01 << 9);
		break;
	default:
		for (;;)
			;
	};
}

void USART_set_word_length(USART_Handle *uh, int wl)
{
	;
}
/*
 * \brief DMA USART RX handle
 */

__attribute__ ((interrupt("irq"))) void DMA1_stream5(void)
{
	return;
}

__attribute__ ((interrupt("irq"))) void DMA1_stream6(void)
{
	return;
}

__attribute__ ((interrupt("irq"))) void DMA2_stream2(void)
{
	return;
}

__attribute__ ((interrupt("irq"))) void DMA2_stream7(void)
{
	return;
}

__attribute__ ((interrupt("irq"))) void DMA1_stream7(void)
{
	return;
}

__attribute__ ((interrupt("irq"))) void DMA1_stream0(void)
{
	return;
}

__attribute__ ((interrupt("irq")))
void USART1_Event(void) {
		;
}


