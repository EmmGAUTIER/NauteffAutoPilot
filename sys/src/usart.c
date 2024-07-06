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
#include "task.h"
#include "stream_buffer.h"
#include "event_groups.h"
#include "semphr.h"
#include "sys_config.h"
#include "util.h"
#include "sys.h"
#include "gpio.h"
#include "nassert.h"
#include "util.h"
#include "rcc.h"
#include "dma.h"
#include "usart.h"
#include "nvic.h"

void USART_Event_Handler(UART_Handle *uh);

/* @formatter:off */
#if SYS_USE_USART1
UART_Handle usart1_Handle = {.baudrate = 4800,
							 .parity = noParity,
							 .stopBits = oneStopBit,
							 .registers = USART1,
							 .transmitting = 0U,
							 .receiving = 0U,
							 .bufferMode = 0U,
							 .streamBufferRx = (StreamBufferHandle_t)0,
							 .streamBufferTx = (StreamBufferHandle_t)0};
#endif /* #if SYS_USE_USART1 */

#if SYS_USE_USART2
UART_Handle usart2_Handle = {.baudrate = 4800,
							 .parity = noParity,
							 .stopBits = oneStopBit,
							 .registers = USART2,
							 .transmitting = 0U,
							 .receiving = 0U,
							 .bufferMode = 0U,
							 .streamBufferRx = (StreamBufferHandle_t)0,
							 .streamBufferTx = (StreamBufferHandle_t)0};
#endif /* #if SYS_USE_USART2 */

#if SYS_USE_USART3
UART_Handle usart3_Handle = {.baudrate = 4800,
							 .parity = noParity,
							 .stopBits = oneStopBit,
							 .registers = USART3,
							 .transmitting = 0U,
							 .receiving = 0U,
							 .bufferMode = 0U,
							 .streamBufferRx = (StreamBufferHandle_t)0,
							 .streamBufferTx = (StreamBufferHandle_t)0};
#endif /* #if SYS_USE_USART5 */

#if SYS_USE_USART4
UART_Handle usart4_Handle = {.baudrate = 4800,
							 .parity = noParity,
							 .stopBits = oneStopBit,
							 .registers = USART4,
							 .transmitting = 0U,
							 .receiving = 0U,
							 .bufferMode = 0U,
							 .streamBufferRx = (StreamBufferHandle_t)0,
							 .streamBufferTx = (StreamBufferHandle_t)0};
#endif /* #if SYS_USE_USART4 */

#if SYS_USE_USART5
UART_Handle usart5_Handle = {
	.baudrate = 4800,
	.parity = noParity,
	.stopBits = oneStopBit,
	.registers = USART5,
	.transmitting = 0U,
	.receiving = 0U,
	.bufferMode = 0U,
	//.streamBufferRx =(StreamBufferHandle_t)0,
	.streamBufferTx = (StreamBufferHandle_t)0,
};

#endif /* #if SYS_USE_USART5 */

#if SYS_USE_USART6
UART_Handle usart6_Handle = {.baudrate = 4800,
							 .cfgparity = noParity,
							 .stopBits = oneStopBit,
							 .registers = USART6,
							 .transmitting = 0U,
							 .receiving = 0U,
							 .bufferMode = 0U,
							 .streamBufferRx = (StreamBufferHandle_t)0,
							 .streamBufferTx = (StreamBufferHandle_t)0

};
#endif /* #if SYS_USE_USART5 */
/* @formatter:on5 */

UART_Handle *usarts[] = {

#if SYS_USE_USART1
	&usart1_Handle,
#endif
#if SYS_USE_USART2
	&usart2_Handle,
#endif
#if SYS_USE_USART3
	&usart3_Handle,
#endif
#if SYS_USE_USART4
	&usart4_Handle,
#endif
#if SYS_USE_USART5
	&usart5_Handle,
#endif
#if SYS_USE_USART6
	&usart6_Handle,
#endif
};

void USART_init_all()
{
	unsigned int i;

	for (i = 0; i < sizeof(usarts) / sizeof(UART_Handle *); i++)
	{
		USART_init(usarts[i]);
	}
	return;
};

void USART_init(UART_Handle *uh)
{

	uh->streamBufferRx = xStreamBufferCreate(USART_BUFFSIZ, 1);
	uh->streamBufferTx = xStreamBufferCreate(USART_BUFFSIZ, 1);

	uh->RxSemlock = xSemaphoreCreateMutex();
	uh->TxSemlock = xSemaphoreCreateMutex();
	uh->RxSemEOL = xSemaphoreCreateMutex();

	uh->registers->SR = 0x0;
	uh->registers->BRR = 0x0;
	uh->registers->CR1 = 0x0;
	uh->registers->CR2 = 0x0;
	uh->registers->CR3 = 0U;
	uh->registers->GTPR = 0x0;

	return;
}

void USART_reset(UART_Handle *uh)
{
	(void)uh;
	return;
}

void USART_set_baudrate(UART_Handle *uh, int br)
{

	int clockFreq;

	uh->baudrate = br;

	/*
	 * Document RM 0390 is quite confusing about value to set in USART_BRR
	 * Here we simply divide the frequency of APB1/2 bus by desired baud rate.
	 */
	if ((uh->registers == USART1) || (uh->registers == USART2))
	{
		clockFreq = RCC_get_APB2_Freq_Hz();
	}
	else
	{
		clockFreq = RCC_get_APB1_Freq_Hz();
	}
	uint16_t valbrr = clockFreq / br;

	uh->registers->BRR = valbrr;

	return;
}

void USART_set_stop_bits(UART_Handle *uh, USART_StopBits sb)
{

	uh->registers->CR2 &= ~(0x03 << 12);
	if (uh->bufferMode == lineBuffered)
		uh->registers->CR2 |= sb << 12;
}

void USART_set_word_length(UART_Handle *uh, int wl)
{
	nassert((wl == 8) || (wl == 9));
	switch (wl)
	{
	case 8:
		clearbit(&uh->registers->CR1, 12);
		break;
	case 9:
		clearbit(&uh->registers->CR1, 12);
		break;
	default:;
	};
}

int USART_write(UART_Handle *uh, const void *buf, size_t count, unsigned delay)
{

	(void)delay;

	size_t sizefree;
	int res = 0;

	vTaskSuspendAll();

	sizefree = xStreamBufferSpacesAvailable(uh->streamBufferTx);
	if ((count <= sizefree) || (count == 0))
	{
		if (uh->transmitting == 0)
		{
			res = xStreamBufferSend(uh->streamBufferTx, buf, count, 0);
			USART_enable_TXE_intr(uh);
			uh->transmitting = 1;
		}
		else
		{
			res = xStreamBufferSend(uh->streamBufferTx, buf, count, 0);
		}

	}
	else
	{
		res = -1;
	}

	xTaskResumeAll();

	return res;
}

int USART_read(UART_Handle *uh, void *buf, const size_t count, unsigned delay)
{

	int res;
	size_t nb;
	size_t nbl;
	// int end;
	char car;

	if (xSemaphoreTake(uh->RxSemlock, 0) == pdTRUE)
	{

		if (uh->bufferMode == lineBuffered)
		{
			nb = 0;
			xSemaphoreTake(uh->RxSemEOL, delay);
			do
			{
				/* TODO : Calculer le temps restant et mettre des delais */
				nbl = xStreamBufferReceive(uh->streamBufferRx, &car, sizeof(car), 0);
				((char *)buf)[nb] = car;
				nb++;
			} while (nbl > 0 && car != '\n');
		}
		else
		{
			nb = xStreamBufferReceive(uh->streamBufferRx, buf, count, delay);
		}
		res = (int)nb;
		xSemaphoreGive(uh->RxSemlock);
	}
	else
	{ /* */
		res = -1;
	}
	return res;
}

int USART_flush_rx_buffer(UART_Handle *uh)
{
	//int ret;
	char data; /* TODO: use long so reads by 8 chunks */
	size_t len;
	/* TODO : check for errors*/

	if (xSemaphoreTake(uh->RxSemlock, 0) == pdTRUE)
	{
		len = xStreamBufferBytesAvailable(uh->streamBufferRx);
		for (int i = 0; i < len; i++)
		{
			xStreamBufferReceive(uh->streamBufferRx,
								 &data,
								 (size_t)1,
								 1U);
		}
		xSemaphoreGive(uh->RxSemlock);
	}
	return 0;
}

void USART_Event_Handler(UART_Handle *uh)
{

	uint32_t status;
	status = uh->registers->SR;
	uh->cnt_intr++; /* for debugging purpose */
	size_t xReceivedBytes;
	BaseType_t xHigherPriorityTaskWoken;
	char car;

	if (status & (0x1 << 7))
	{
		/* TXE (Tx Empty) bit 7 in SR (status register),
		 * if set next char to transmit has to be written in USART_DR */
		xHigherPriorityTaskWoken = pdFALSE;
		xReceivedBytes = xStreamBufferReceiveFromISR(uh->streamBufferTx, &car,
													 1U, &xHigherPriorityTaskWoken);
		if (xReceivedBytes != 0)
		{
			uh->registers->DR = car;
		}
		if (xStreamBufferIsEmpty(uh->streamBufferTx) || (xReceivedBytes == 0))
		{
			USART_disable_TXE_intr(uh);
			uh->transmitting = 0;
		}
	}

	/*----- Réception d'un caractère ----*/
	if (status & (0x1 << 5))
	{ /* RXNE : Receive Not Empty */
		car = uh->registers->DR;
		/* TODO Gérer le cas de dépassement de tampon*/
		xHigherPriorityTaskWoken = pdFALSE;
		(void)xStreamBufferSendFromISR(uh->streamBufferRx, &car, 1U,
									   &xHigherPriorityTaskWoken);
		// TODO taskYIELD_FROM_ISR(xHigherPriorityTaskWoken);
#if 0
		if (uh->bufferMode == lineBuffered && car == '\n') {
			xSemaphoreGiveFromISR(uh->RxSemEOL, &xHigherPriorityTaskWoken);
			// TODO taskYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
#endif
	}
	return;
}

//__attribute__ ((interrupt("irq")))
void __attribute__((interrupt)) USART1_Event(void)
{
#if SYS_USE_USART1
	// xSemaphoreTake(uh->RxSemEOL, 0U);
	// usart1_count++;

	USART_Event_Handler(usart1);
#endif
	return;
}

__attribute__((interrupt("irq"))) void USART2_Event(void)
{
#if SYS_USE_USART2
	// usart2_count++;
	USART_Event_Handler(usart2);
#endif
	return;
}

__attribute__((interrupt)) void USART3_Event(void)
{
#if SYS_USE_USART3
	// usart3_count++;
	USART_Event_Handler(usart3);
#endif
	return;
}

__attribute__((interrupt("irq"))) void USART4_Event(void)
{
#if SYS_USE_USART4
	// usart4_count++;

	USART_Event_Handler(usart4);
#endif
	return;
}

__attribute__((interrupt("irq"))) void USART5_Event(void)
{
#if SYS_USE_USART5
	USART_Event_Handler(usart5);
#endif
	return;
}

__attribute__((interrupt("irq"))) void USART6_Event(void)
{
#if SYS_USE_USART6
	// usart6_count++;

	USART_Event_Handler(usart6);
#endif
	return;
}
