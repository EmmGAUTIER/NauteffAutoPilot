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

#include <stdint.h>
#include <stdbool.h>
#include "util.h"
#include "assert.h"
#include "stm32F446.h"
#include "nassert.h"
#include "sys.h"
#include "dma.h"

#ifdef SYS_USE_DMA1
DMA_Handle dma1_Handle;
DMA_CHANNEL_Handle dma1_channels[8];
#endif

#ifdef SYS_USE_DMA2
DMA_Handle dma2_Handle;
DMA_CHANNEL_Handle dma2_channels[8];
#endif

#if SYS_USE_DMA1
DMA_Handle dma1_Handle;
DMA_CHANNEL_Handle *dma1_ch0 = &dma1_channels[0];
DMA_CHANNEL_Handle *dma1_ch1 = &dma1_channels[1];
DMA_CHANNEL_Handle *dma1_ch2 = &dma1_channels[2];
DMA_CHANNEL_Handle *dma1_ch3 = &dma1_channels[3];
DMA_CHANNEL_Handle *dma1_ch4 = &dma1_channels[4];
DMA_CHANNEL_Handle *dma1_ch5 = &dma1_channels[5];
DMA_CHANNEL_Handle *dma1_ch6 = &dma1_channels[6];
DMA_CHANNEL_Handle *dma1_ch7 = &dma1_channels[7];
#endif

#if SYS_USE_DMA2
DMA_Handle dma2_Handle;
DMA_CHANNEL_Handle *dma2_ch0 = &dma2_channels[0];
DMA_CHANNEL_Handle *dma2_ch1 = &dma2_channels[1];
DMA_CHANNEL_Handle *dma2_ch2 = &dma2_channels[2];
DMA_CHANNEL_Handle *dma2_ch3 = &dma2_channels[3];
DMA_CHANNEL_Handle *dma2_ch4 = &dma2_channels[4];
DMA_CHANNEL_Handle *dma2_ch5 = &dma2_channels[5];
DMA_CHANNEL_Handle *dma2_ch6 = &dma2_channels[6];
DMA_CHANNEL_Handle *dma2_ch7 = &dma2_channels[7];
#endif

void DMA_init_all(void) {
#if 0
#if SYS_USE_DMA1
	DMA_init(dma1, DMA1);
	DMA_CHANNEL_init(dma1_ch0, &dma1_Handle, 0);
	DMA_CHANNEL_init(dma1_ch1, &dma1_Handle, 1);
	DMA_CHANNEL_init(dma1_ch2, &dma1_Handle, 2);
	DMA_CHANNEL_init(dma1_ch3, &dma1_Handle, 3);
	DMA_CHANNEL_init(dma1_ch4, &dma1_Handle, 4);
	DMA_CHANNEL_init(dma1_ch5, &dma1_Handle, 5);
	DMA_CHANNEL_init(dma1_ch6, &dma1_Handle, 6);
	DMA_CHANNEL_init(dma1_ch7, &dma1_Handle, 7);
#endif

#if SYS_USE_DMA2
	DMA_init(dma2, DMA2);
	DMA_CHANNEL_init(dma2_ch0, &dma2_Handle, 0);
	DMA_CHANNEL_init(dma2_ch1, &dma2_Handle, 1);
	DMA_CHANNEL_init(dma2_ch2, &dma2_Handle, 2);
	DMA_CHANNEL_init(dma2_ch3, &dma2_Handle, 3);
	DMA_CHANNEL_init(dma2_ch4, &dma2_Handle, 4);
	DMA_CHANNEL_init(dma2_ch5, &dma2_Handle, 5);
	DMA_CHANNEL_init(dma2_ch6, &dma2_Handle, 6);
	DMA_CHANNEL_init(dma2_ch7, &dma2_Handle, 7);
#endif
#endif

	return;
}

void DMA_init(DMA_Handle *dma, DMA_regs *regs) {

	dma->registers = regs;

	return;
}

void DMA_CHANNEL_init(DMA_CHANNEL_Handle *dma_ch, DMA_Handle *dma, unsigned chn) {

	nassert(chn < 8);

	dma_ch->dma = dma;
	dma_ch->channelNumber = chn;
	dma_ch->registers = &(dma->registers->streamRegs[chn]);
	/* TODO SxFVCR n'a pas la valeur 0x21 attendue, mettons un 0 pour voir */
	dma_ch->registers->DMA_SxFCR=0;

	return;
}

unsigned DMA_CHANNEL_get_flags(DMA_CHANNEL_Handle *dma_ch, unsigned flagsMask) {

	unsigned flags;

	switch (dma_ch->channelNumber) {
	case 0:
		flags = (dma_ch->dma->registers->DMA_LIFCR & ~(0x3D << 0)) >> 0;
		break;
	case 1:
		flags = (dma_ch->dma->registers->DMA_LIFCR & ~(0x3D << 6)) >> 6;
		break;
	case 2:
		flags = (dma_ch->dma->registers->DMA_LIFCR & ~(0x3D << 16)) >> 16;
		break;
	case 3:
		flags = (dma_ch->dma->registers->DMA_LIFCR & ~(0x3D << 22)) >> 22;
		break;
	case 4:
		flags = (dma_ch->dma->registers->DMA_HIFCR & ~(0x3D << 0)) >> 0;
		break;
	case 5:
		flags = (dma_ch->dma->registers->DMA_HIFCR & ~(0x3D << 6)) >> 6;
		break;
	case 6:
		flags = (dma_ch->dma->registers->DMA_HIFCR & ~(0x3D << 16)) >> 16;
		break;
	case 7:
		flags = (dma_ch->dma->registers->DMA_HIFCR & ~(0x3D << 22)) >> 22;
		break;
	default:
		break;
	}
	flags &= flagsMask;

	return flags;
}

void DMA_CHANNEL_clr_flags(DMA_CHANNEL_Handle *dma_ch, unsigned flagsMask) {

	switch (dma_ch->channelNumber) {
	case 0:
		dma_ch->dma->registers->DMA_LIFCR &= ~(flagsMask << 0);
		break;
	case 1:
		dma_ch->dma->registers->DMA_LIFCR &= ~(flagsMask << 6);
		break;
	case 2:
		dma_ch->dma->registers->DMA_LIFCR &= ~(flagsMask << 16);
		break;
	case 3:
		dma_ch->dma->registers->DMA_LIFCR &= ~(flagsMask << 22);
		break;
	case 4:
		dma_ch->dma->registers->DMA_HIFCR &= ~(flagsMask << 0);
		break;
	case 5:
		dma_ch->dma->registers->DMA_HIFCR &= ~(flagsMask << 6);
		break;
	case 6:
		dma_ch->dma->registers->DMA_HIFCR &= ~(flagsMask << 16);
		break;
	case 7:
		dma_ch->dma->registers->DMA_HIFCR &= ~(flagsMask << 16);
		break;
	default:
		break;
	};

	return;
}

#if 0
void DMA_CHANNEL_enable(DMA_CHANNEL_Handle *dma_ch) {

	/* cf. RM0090  section 10.5.5 : "Before setting EN bit to '1'
	 *  to start a new transfer, the event flags
	 *   corresponding to the stream in DMA_LISR or DMA_HISR
	 *    register must be cleared." */
	DMA_CHANNEL_clr_flags(dma_ch, DMA_CHANNEL_STATUS_ALL_FLAGS);

	/* set EN bit in SxCR register */
	dma_ch->registers->DMA_SxCR |= (0x01) << 0;
}

void DMA_CHANNEL_set_mem_to_periph(DMA_CHANNEL_Handle *dma_ch, void *mem,
		void *per) {

	dma_ch->registers->DMA_SxM0AR = mem;
	dma_ch->registers->DMA_SxPAR = per;
	dma_ch->registers->DMA_SxCR &= ~(0x3 << 6);

	return;
}

void DMA_CHANNEL_set_periph_to_mem(DMA_CHANNEL_Handle *dma_ch, void *per,
		void *mem) {

	dma_ch->registers->DMA_SxPAR = per;
	dma_ch->registers->DMA_SxM0AR = mem;
	dma_ch->registers->DMA_SxCR &= ~(0x3 << 6);
	dma_ch->registers->DMA_SxCR |= ~(0x1 << 6);

	return;
}
#endif

/*
 * \brief DMA handlers
 */

__attribute__ ((interrupt("irq"))) void DMA1_Ch0_Handler(void) {

	dma1_ch0->fct(dma1_ch0->peripheral);

	return;
}

__attribute__ ((interrupt("irq"))) void DMA1_Ch1_Handler(void) {

	dma1_ch1->fct(dma1_ch1->peripheral);

	return;
}

__attribute__ ((interrupt("irq"))) void DMA1_Ch2_Handler(void) {

	dma1_ch2->fct(dma1_ch2->peripheral);

	return;
}
__attribute__ ((interrupt("irq"))) void DMA1_Ch3_Handler(void) {

	dma1_ch3->fct(dma1_ch3->peripheral);

	return;
}
__attribute__ ((interrupt("irq"))) void DMA1_Ch4_Handler(void) {

	dma1_ch4->fct(dma1_ch4->peripheral);

	return;
}
__attribute__ ((interrupt("irq"))) void DMA1_Ch5_Handler(void) {

	dma1_ch5->fct(dma1_ch5->peripheral);

	return;
}
__attribute__ ((interrupt("irq"))) void DMA1_Ch6_Handler(void) {

	dma1_ch6->fct(dma1_ch6->peripheral);

	return;
}

__attribute__ ((interrupt("irq"))) void DMA1_Ch7_Handler(void) {

	dma1_ch7->fct(dma1_ch7->peripheral);

	return;
}

__attribute__ ((interrupt("irq"))) void DMA2_Ch0_Handler(void) {

	dma1_ch0->fct(dma2_ch0->peripheral);

	return;
}

__attribute__ ((interrupt("irq"))) void DMA2_Ch1_Handler(void) {

	dma1_ch1->fct(dma2_ch1->peripheral);

	return;
}

__attribute__ ((interrupt("irq"))) void DMA2_Ch2_Handler(void) {

	dma1_ch2->fct(dma2_ch2->peripheral);

	return;
}
__attribute__ ((interrupt("irq"))) void DMA2_Ch3_Handler(void) {

	dma1_ch3->fct(dma2_ch3->peripheral);

	return;
}
__attribute__ ((interrupt("irq"))) void DMA2_Ch4_Handler(void) {

	static int count = 0;

	count++;
	dma1_ch4->fct(dma2_ch4->peripheral);

	return;
}
__attribute__ ((interrupt("irq"))) void DMA2_Ch5_Handler(void) {

	dma1_ch5->fct(dma2_ch5->peripheral);

	return;
}
__attribute__ ((interrupt("irq"))) void DMA2_Ch6_Handler(void) {

	dma1_ch6->fct(dma2_ch6->peripheral);

	return;
}

__attribute__ ((interrupt("irq"))) void DMA2_Ch7_Handler(void) {

	dma1_ch7->fct(dma2_ch7->peripheral);

	return;
}

#if 0

void DMA_stream_init(DMA_CHANNEL_Handle *dmach) {
	int cnt;
	nassert(stream < 8);
	nassert((dma == DMA1) || (dma == DMA2));
	nassert(DMA_stream_is_enabled(dma, stream));

	/* First clear the EN bit in SxCR */
	dma->streamRegs[stream].DMA_SxCR &= ~(0x1 << 0);
	/* Then ensure that the stream is disabled */
	/* TODO : blocking test, correct to limited test*/
	for (cnt = 0; dma->streamRegs[stream].DMA_SxCR &= (0x1 << 0) != 0x0;
			cnt++) {
		;
	}

	/* Set to zero interrupt flags in LISR an HISR */
	/* Status flags of the 8 streams are in Two 32bits registers */
	/* Setting to zero the flags is done by writing 1 to the corresponding */
	/* bit in LIFCR or HIFCR */
	dma->DMA_LIFCR = (0x3D << 0) | (0x3D << 6) | (0x3D << 16) | (0x3D << 22);
	dma->DMA_HIFCR = (0x3D << 0) | (0x3D << 6) | (0x3D << 16) | (0x3D << 22);

	dma->streamRegs[stream].DMA_SxCR = 0x0;
	dma->streamRegs[stream].DMA_SxCR = 0x0;

	return;
}


void DMA_CHANNEL_enable(DMA_CHANNEL_Handle *dma_ch) {

	/* Before setting EN bit to 1 to start a new transfer, */
	/* the event flags in DMA_L/HISR must be cleared       */
	/* they are cleared by writing 1s in DMA_L/HIFCR       */

	switch (dma_ch->channelNumber) {
	case 0:
		dma_ch->dma->registers->DMA_LIFCR &= ~(0x3D << 0);
		break;
	case 1:
		dma_ch->dma->registers->DMA_LIFCR &= ~(0x3D << 6);
		break;
	case 2:
		dma_ch->dma->registers->DMA_LIFCR &= ~(0x3D << 16);
		break;
	case 3:
		dma_ch->dma->registers->DMA_LIFCR &= ~(0x3D << 22);
		break;
	case 4:
		dma_ch->dma->registers->DMA_HIFCR &= ~(0x3D << 0);
		break;
	case 5:
		dma_ch->dma->registers->DMA_HIFCR &= ~(0x3D << 6);
		break;
	case 6:
		dma_ch->dma->registers->DMA_HIFCR &= ~(0x3D << 16);
		break;
	case 7:
		dma_ch->dma->registers->DMA_HIFCR &= ~(0x3D << 22);
		break;
	default:
		break;
	}
	/* Then enable the stream */
	dma_ch->registers->DMA_SxCR |= 0x1 << 8;

	return;
}



void DMA_stream_set_periph_to_mem(DMA_regs *dma, uint32_t stream, void *per,
		void *mem) {
	nassert((dma == DMA1) || (dma == DMA2));
	nassert(stream < 8);
	/* and chek stream is disabled */
	nassert((dma->streamRegs[stream].DMA_SxCR & (0x1 << 0)) == 0x0);

	/* clear PINC : bit 9, peripheral increment mode inc */
	/*       DIR :  bits 6& &7 : data transfer direction */

	dma->streamRegs[stream].DMA_SxCR &= ~((0x1 << 9) | (0x21) << 6);
	/* set MINC   : bit 10 : memory increment               */
	/*     DIR    : bit 6 & 7 : Direction 0x0               */
	/*     PFCTRL : bit  5 : Peripheral is flow controller  */
	dma->streamRegs[stream].DMA_SxCR |= (0x00 << 6 | 0x1 << 5);
	dma->streamRegs[stream].DMA_SxPAR = per;
	dma->streamRegs[stream].DMA_SxM0AR = mem;
}

void DMA_stream_set_mem_to_periph(DMA_regs *dma, uint32_t stream, void *mem,
		void *per) {

	/* clear PINC : bit 9, peripheral increment mode inc */
	/*       DIR :  bits 6& &7 : data transfer direction */
	dma->streamRegs[stream].DMA_SxCR &= ~((0x1 << 9) | (0x21) << 6);
	/* set MINC   : bit 10    : memory increment              */
	/*     PFCTRL : bit  5    : Peripheral is flow controller */
	/*     DIR    : bit 6 & 7 : Direction 0x1                 */
	dma->streamRegs[stream].DMA_SxCR |= (0x010 << 6 | 0x1 << 5);
	dma->streamRegs[stream].DMA_SxM0AR = mem;
	dma->streamRegs[stream].DMA_SxPAR = per;
}

#endif
