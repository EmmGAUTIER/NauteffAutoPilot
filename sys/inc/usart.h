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

#ifndef USART_h
#define USART_h

#define USART_BUFFSIZ (128)

typedef enum
{
	noParity = 0,
	oddParity = 1,
	evenParity = 2
} USART_Parity;

typedef enum
{
	oneStopBit = 0,
	halfStopBit = 1,
	twoStopBits = 2,
	oneAndHalfStopBit = 3
} USART_StopBits;

typedef enum
{
	blockBuffered = 0,
	lineBuffered = 1
} USART_BufferMode;

__attribute__((interrupt("irq"))) void DMA1_stream5(void);
__attribute__((interrupt("irq"))) void DMA1_stream6(void);
__attribute__((interrupt("irq"))) void DMA2_Ch2_Handler(void);
__attribute__((interrupt("irq"))) void DMA2_Ch7_Handler(void);
__attribute__((interrupt("irq"))) void DMA1_stream7(void);
__attribute__((interrupt("irq"))) void DMA1_stream0(void);
__attribute__((interrupt("irq"))) void USART1_Event(void);
__attribute__((interrupt("irq"))) void USART2_Event(void);
__attribute__((interrupt("irq"))) void USART3_Event(void);
__attribute__((interrupt("irq"))) void USART4_Event(void);
__attribute__((interrupt("irq"))) void USART5_Event(void);
__attribute__((interrupt("irq"))) void USART6_Event(void);

typedef struct
{
	vuint32_t SR;	/*   Status register */
	vuint32_t DR;	/*   Data register */
	vuint32_t BRR;	/*  Baud rate register */
	vuint32_t CR1;	/*  Control Register 1 */
	vuint32_t CR2;	/*  Control Register 2 */
	vuint32_t CR3;	/*  Control Register 3 */
	vuint32_t GTPR; /* Guard time and prescaler register */
} USART_Regs_Typedef;

/* SR Registers */
#define USART_SR_CTS (0x1 << 9)	 /* CTS flag*/
#define USART_SR_LBD (0x1 << 8)	 /* LIN break flag */
#define USART_SR_TXE (0x1 << 7)	 /* Transmit data register empty*/
#define USART_SR_TC (0x1 << 6)	 /* Transmission complete */
#define USART_SR_RXNE (0x1 << 5) /* Read data register not empty*/
#define USART_SR_IDLE (0x1 << 4) /* Idle line detected*/
#define USART_SR_ORE (0x1 << 3)	 /* Overun error */
#define USART_SR_NF (0x1 << 2)	 /* Noise detected flag */
#define USART_SR_FE (0x1 << 1)	 /* Framing error */
#define USART_SR_PE (0x1 << 0)	 /* Parity error*/

/* CR1 Registers */

#define USART_CR1_OVER8 (0x1 << 15) /* Oversampling 16 or 8 bits*/
#define USART_CR1_UE (0x1 << 13)	/* USART Enable */
#define USART_CR1_M (0x1 << 12)		/* Word length */
#define USART_CR1_WAKE (0x1 << 11)	/* Wakeup method */
#define USART_CR1_PCE (0x1 << 10)	/* Parity control enable */
#define USART_CR1_PS (0x1 << 9)		/* Parity Selection */
#define USART_CR1_PEIE (0x1 << 8)	/* Parity errror intr. enable */
#define USART_CR1_TXEIE (0x1 << 7)	/* Transmit register empty intr. enable */
#define USART_CR1_TCIE (0x1 << 6)	/* Transmission complete intr. enable */
#define USART_CR1_RXNEIE (0x1 << 5) /* Rx data reg. not empty untr. enable */
#define USART_CR1_IDLEIE (0x1 << 4) /* Idle intr. enable */
#define USART_CR1_TE (0x1 << 3)		/* Transmitter enable */
#define USART_CR1_RE (0x1 << 2)		/* Receiver enable */
#define USART_CR1_RWU (0x1 << 1)	/* Receiver wakeup */
#define USART_CR1_SBK (0x1 << 0)	/* Send Break */

#define USART1 ((USART_Regs_Typedef *)(0x40011000))
#define USART2 ((USART_Regs_Typedef *)(0x40004400))
#define USART3 ((USART_Regs_Typedef *)(0x40004800))
#define USART4 ((USART_Regs_Typedef *)(0x40004C00))
#define USART5 ((USART_Regs_Typedef *)(0x40005000))
#define USART6 ((USART_Regs_Typedef *)(0x40011400))

/* idxStartTx, idxEndTx, idxEndDMA are modified by USART fcts */
/* transmitting is set by USART fcts and reset by DMA_CHANNEL handler */
typedef struct
{
	int baudrate;
	USART_StopBits stopBits;
	USART_Parity parity;
	USART_Regs_Typedef *registers;
	SemaphoreHandle_t TxSemlock;
	SemaphoreHandle_t RxSemlock;
	vuint16_t transmitting;
	vuint16_t receiving;
	StreamBufferHandle_t streamBufferTx;
	StreamBufferHandle_t streamBufferRx;
	SemaphoreHandle_t RxSemEOL;
	vuint8_t bufferMode; /* 0 bloc, other line mode (ie \n */
	/* TOTO : suppress this counter of interrupts : */
	unsigned long cnt_intr;
	unsigned long cnt_rx;
	unsigned long cnt_tx;
	char tampon[100]; // For debugging purpose only, to be removed
} UART_Handle;

#define usart1 (&usart1_Handle)
#define usart2 (&usart2_Handle)
#define usart3 (&usart3_Handle)
#define usart4 (&usart4_Handle)
#define usart5 (&usart5_Handle)
#define usart6 (&usart6_Handle)

extern UART_Handle usart1_Handle;
extern UART_Handle usart2_Handle;
extern UART_Handle usart3_Handle;
extern UART_Handle usart4_Handle;
extern UART_Handle usart5_Handle;
extern UART_Handle usart6_Handle;

#if 0
void USART_initmyTask02_all(void);
#endif

void USART_init_all();

void USART_init(UART_Handle *uh);

void USART_reset(UART_Handle *uh);

INLINE void USART_enable(UART_Handle *uh)
{
	/* Mise à 1 de UE TE et RE, TXEIE et RXNEIE dans CR1myTask02 */
	uh->registers->CR1 |= ((0x1 << 13) | (0x1 << 7) | (0x1 << 5) | (0x1 << 3) | (0x1 << 2));
	/* Mise à 1 de DMAT et DMAR dans CR3 */
	// uh->registers->CR3 |= (0x1 << 7) | (0x1 << 6);
}

INLINE void USART_disable(UART_Handle *uh)
{
	/* Mise à 0 dmyTask02e UE TE et RE, TXEIE et RXNEIE dans CR1 */
	uh->registers->CR1 &= ~((0x1 << 13) | (0x1 << 7) | (0x1 << 5) | (0x1 << 3) | (0x1 << 2));
	/* Mise à 0 de DMAT et DMAR dans CR3 */
	// uh->registers->CR3 |= (0x1 << 7) | (0x1 << 6);
}

INLINE void USART_enable_TXE_intr(UART_Handle *uh)
{
	uh->registers->CR1 |= (0x1 << 7);
}

INLINE void USART_disable_TXE_intr(UART_Handle *uh)
{
	uh->registers->CR1 &= ~(0x1 << 7);
}

/*myTask02
 INLINE void* USARfemtolibc/src/flc.c:189T_get_Tx_reg_add(UART_Handle *uh) {

 return (void*) (&(uh->registers->DR));
 }
 */
__attribute__((always_inline)) inline void USART_raw_put_char(USART_Regs_Typedef *ur, char car)
{
	ur->DR = (uint32_t)car;
}

__attribute__((always_inline)) inline uint32_t USART_raw_get_char(
	USART_Regs_Typedef *ur)
{
	return ur->DR;
}

/*
 * @brief set the baud rate of the UART
 * Usart must be stopped.
 * @param br in bits/s
 * @param uh UART handle
 * @return void
 */
void USART_set_baudrate(UART_Handle *uh, int br);
void USART_set_stop_bits(UART_Handle *uh, USART_StopBits sb);
// void USART_set_parity(UART_Handle *uh, USART_Parity par);

INLINE void USART_set_parity_odd(UART_Handle *uh)
{
	uh->registers->CR1 |= (0x01 << 10) | (0x01 << 9);
}

INLINE void USART_set_parity_even(UART_Handle *uh)
{
	uh->registers->CR1 |= 0x01 << 10;
	uh->registers->CR1 &= ~(0x01 << 9);
}

INLINE void USART_set_parity_none(UART_Handle *uh)
{
	uh->registers->CR1 &= ~(0x01 << 10);
}

void USART_set_word_length(UART_Handle *uh, int wl);

INLINE void USART_set_buffer_mode(UART_Handle *uh, USART_BufferMode bm)
{
	uh->bufferMode = bm;
}

int USART_flush_rx_buffer(UART_Handle *uh);

INLINE int USART_is_RXNE(USART_Regs_Typedef *regs)
{
	return (regs->SR & USART_SR_RXNE) != 0U;
}

INLINE int USART_is_TXE(USART_Regs_Typedef *regs)
{
	return (regs->SR & USART_SR_TXE) != 0U;
}

int USART_write(UART_Handle *uh, const void *buf, size_t count, unsigned delay);
int USART_read(UART_Handle *uh, void *buf, size_t count, unsigned delay);

#endif /* USART_H */
