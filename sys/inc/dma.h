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

#define DMA1 ((DMA_regs*)(0x40026000))
#define DMA2 ((DMA_regs*)(0x40026400))

/*----- Status flags of DMA channels -----*/
#define DMA_CHANNEL_STATUS_TCIF (0x01 << 5) /*  Transfer Complete Interrupt Flag */
#define DMA_CHANNEL_STATUS_HTIF (0x01 << 4) /*  Half Transfer Interrupt Flag */
#define DMA_CHANNEL_STATUS_TEIF (0x01 << 3) /*  Transfer Error Interrupt Flag */
#define DMA_CHANNEL_STATUS_DMEIF (0x01 << 2) /* Direct Mode Error Interrupt Flag */
#define DMA_CHANNEL_STATUS_FEIF (0x01 << 2) /*  FIFO Error Interrupt Flag */
#define DMA_CHANNEL_STATUS_ALL_FLAGS (DMA_CHANNEL_STATUS_TCIF\
		| DMA_CHANNEL_STATUS_HTIF | DMA_CHANNEL_STATUS_TEIF\
		| DMA_CHANNEL_STATUS_DMEIF|DMA_CHANNEL_STATUS_FEIF)

/*----- Priority of DMA channels -----*/
/* self explanatory names */
#define DMA_PRIORITY_LOW       (0U)
#define DMA_PRIORITY_MEDIUM    (1U)
#define DMA_PRIORITY_HIGH      (2U)
#define DMA_PRIORITY_VERY_HIGH (3U)
#define DMA_PRIORITY_DEFAULT   (DMA_PRIORITY_LOW)

#define DMA_CHANNEL_

typedef struct {
	vuint32_t DMA_SxCR;
	vuint32_t DMA_SxNDTR;
	void *DMA_SxPAR;
	void *DMA_SxM0AR;
	void *DMA_SxM1AR;
	vuint32_t DMA_SxFCR;
} DMA_CHANNEL_Regs;

typedef struct {
	vuint32_t DMA_LISR;
	vuint32_t DMA_HISR;
	vuint32_t DMA_LIFCR;
	vuint32_t DMA_HIFCR;
	DMA_CHANNEL_Regs streamRegs[8];
} DMA_regs;

typedef struct {
	DMA_regs *registers;
} DMA_Handle;

typedef struct {
	DMA_Handle *dma;
	DMA_CHANNEL_Regs *registers;
	unsigned channelNumber;
	void (*fct)(void*);
	void *peripheral;
} DMA_CHANNEL_Handle;

#define dma1 (&dma1_Handle)
#define dma2 (&dma2_Handle)

extern DMA_Handle dma1_Handle;
extern DMA_Handle dma2_Handle;

#if 0
#if SYS_USE_DMA1
extern DMA_Handle* dma1;
extern DMA_CHANNEL_Handle *dma1_ch0;
extern DMA_CHANNEL_Handle *dma1_ch1;
extern DMA_CHANNEL_Handle *dma1_ch2;
extern DMA_CHANNEL_Handle *dma1_ch3;
extern DMA_CHANNEL_Handle *dma1_ch4;
extern DMA_CHANNEL_Handle *dma1_ch5;
extern DMA_CHANNEL_Handle *dma1_ch6;
extern DMA_CHANNEL_Handle *dma1_ch7;
#endif

#if SYS_USE_DMA2
extern DMA_Handle* dma2;
extern DMA_CHANNEL_Handle *dma2_ch0;
extern DMA_CHANNEL_Handle *dma2_ch1;
extern DMA_CHANNEL_Handle *dma2_ch2;
extern DMA_CHANNEL_Handle *dma2_ch3;
extern DMA_CHANNEL_Handle *dma2_ch4;
extern DMA_CHANNEL_Handle *dma2_ch5;
extern DMA_CHANNEL_Handle *dma2_ch6;
extern DMA_CHANNEL_Handle *dma2_ch7;
#endif
#endif
#if 0
typedef enum {
	DMA_priority_low = 0,
	DMA_priority_medium = 1,
	DMA_Priority_high = 2,
	DMA_Priority_very_high = 3
} DMA_priority;

typedef enum {
	DMA_FIFO_error = (0x1 << 0),
	DMA_direct_mode_error = (0x1 << 2),
	DMA_transfer_error = (0x1 << 3),
	DMA_half_transfer = (0x1 << 4),
	DMA_transfer_complete = (0x1 << 5),
	DMA_all_status_flags = (0x1 << 0) | (0x1 << 2)| (0x1 << 3)| (0x1 << 4)| (0x1 << 5)
} DMA_interrupt_flag;
#endif

__attribute__ ((interrupt("irq"))) void DMA1_Ch0_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA1_Ch1_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA1_Ch2_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA1_Ch3_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA1_Ch4_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA1_Ch5_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA1_Ch6_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA1_Ch7_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA2_Ch0_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA2_Ch1_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA2_Ch2_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA2_Ch3_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA2_Ch4_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA2_Ch5_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA2_Ch6_Handler(void);
__attribute__ ((interrupt("irq"))) void DMA2_Ch7_Handler(void);

void DMA_init_all(void);

void DMA_init(DMA_Handle *dma, DMA_regs *regs);

void DMA_CHANNEL_init(DMA_CHANNEL_Handle *dma_ch, DMA_Handle *dma, unsigned chn);

#if 0
void DMA_CHANNEL_enable(DMA_CHANNEL_Handle *dma_ch);
INLINE
void DMA_CHANNEL_disable(DMA_CHANNEL_Handle *dma_ch) {

	dma_ch->registers->DMA_SxCR &= ~(0x01 << 0);

	return;
}

INLINE bool DMA_CHANNEL_is_enabled(DMA_CHANNEL_Handle *dma_ch) {
	bool ret;

	ret = (dma_ch->registers->DMA_SxCR & (0x1 << 0)) == 0x1;

	return ret;
}
#endif

unsigned DMA_CHANNEL_get_flags(DMA_CHANNEL_Handle *dma_ch, unsigned flagsMask);

void DMA_CHANNEL_clr_flags(DMA_CHANNEL_Handle *dma_ch, unsigned flagsMask);

//void DMA_CHANNEL_set_Handler(DMA_CHANNEL_Handle *dma_ch, void (*)(void*));

INLINE void DMA_CHANNEL_set_Handler(DMA_CHANNEL_Handle *dma_ch,
		void (*fct)(void*)) {

	dma_ch->fct = fct;

	return;
}

INLINE void DMA_CHANNEL_set_priority(DMA_CHANNEL_Handle *dma_ch,
		unsigned priority) {

	dma_ch->registers->DMA_SxCR &= ~(0x7 << 16);
	dma_ch->registers->DMA_SxCR |= (priority << 16);

	return;
}

INLINE void DMA_CHANNEL_set_channel(DMA_CHANNEL_Handle *dma_ch, unsigned chn) {

	nassert(chn < 8);
	nassert(
			(dma_ch->dma->registers == DMA1)
					|| (dma_ch->dma->registers == DMA2));

	dma_ch->registers->DMA_SxCR &= ~(0x7 << 25);
	dma_ch->registers->DMA_SxCR |= chn << 25;

	return;
}

INLINE void DMA_CHANNEL_enable_circular(DMA_CHANNEL_Handle *dma_ch) {

	dma_ch->registers->DMA_SxCR |= (0x01 << 8);

	return;
}

INLINE void DMA_CHANNEL_disable_circular(DMA_CHANNEL_Handle *dma_ch) {

	dma_ch->registers->DMA_SxCR &= ~(0x01 << 8);

	return;
}

INLINE void DMA_CHANNEL_set_memory_address(DMA_CHANNEL_Handle *dma_ch,
		void *add) {

	dma_ch->registers->DMA_SxM0AR = add;

	return;
}

INLINE void DMA_CHANNEL_set_peripheral_address(DMA_CHANNEL_Handle *dma_ch,
		void *address) {

	dma_ch->registers->DMA_SxPAR = address;

	return;
}

INLINE void DMA_CHANNEL_set_mem_to_periph(DMA_CHANNEL_Handle *dma_ch) {

	dma_ch->registers->DMA_SxCR &= ~(0x3 << 6);
	dma_ch->registers->DMA_SxCR |= (0x1 << 6) | (0x1 << 10) | (0x1 << 5)
			| (0x1 << 4);

	return;
}

INLINE void DMA_CHANNEL_set_periph_to_mem(DMA_CHANNEL_Handle *dma_ch) {

	dma_ch->registers->DMA_SxCR &= ~(0x3 << 6);
	dma_ch->registers->DMA_SxCR |= (0x1 << 6);

	return;
}

INLINE void DMA_CHANNEL_start_Transfer(DMA_CHANNEL_Handle *dma_ch, void *addmem,
		unsigned ndt) {

	dma_ch->registers->DMA_SxM0AR = addmem;
	dma_ch->registers->DMA_SxNDTR = ndt;
	dma_ch->registers->DMA_SxCR |= (0x01 << 0);

	return;
}

INLINE int DMA_CHANNEL_get_NDT(DMA_CHANNEL_Handle *dma_ch) {
	return dma_ch->registers->DMA_SxNDTR;
}
