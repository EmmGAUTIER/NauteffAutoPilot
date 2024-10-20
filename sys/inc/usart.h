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

#define USART_BUFFSIZ	(128)

typedef enum
{
    noParity = 0, oddParity = 1, evenParity = 2
} USART_Parity;

typedef enum
{
    oneStopBit = 0, halfStopBit = 1, twoStopBits = 2, oneAndHalfStopBit = 3
} USART_StopBits;

__attribute__ ((interrupt("irq"))) void DMA1_stream5 (void);
__attribute__ ((interrupt("irq"))) void DMA1_stream6 (void);
__attribute__ ((interrupt("irq"))) void DMA2_stream2 (void);
__attribute__ ((interrupt("irq"))) void DMA2_stream7 (void);
__attribute__ ((interrupt("irq"))) void DMA1_stream7 (void);
__attribute__ ((interrupt("irq"))) void DMA1_stream0 (void);
__attribute__ ((interrupt("irq"))) void USART1_Event (void);

typedef struct
{
    vuint32_t SR;    /* Status register */
    vuint32_t DR;    /* Data register */
    vuint32_t BRR;   /* Baud rate register */
    vuint32_t CR1;   /* Control Register 1 */
    vuint32_t CR2;   /* Control Register 2 */
    vuint32_t CR3;   /* Control Register 3 */
    vuint32_t GTPR;  /* Guard time and prescaler register */
} USART_Regs_Typedef;

#define USART1 ((USART_Regs_Typedef*)(0x40011400))
#define USART2 ((USART_Regs_Typedef*)(0x40004400))
#define USART3 ((USART_Regs_Typedef*)(0x40004800))
#define USART4 ((USART_Regs_Typedef*)(0x40004C00))
#define USART5 ((USART_Regs_Typedef*)(0x40005000))
#define USART6 ((USART_Regs_Typedef*)(0x40011000))

typedef struct
{
    USART_Regs_Typedef *registers;
    int baudrate;
    USART_StopBits stopBits;
    USART_Parity parity;
    char buffer[USART_BUFFSIZ];
    int idxStartBuffer;
    int idxEndBuffer;
} USART_Handle;

extern USART_Handle USART_Handles[6];



void USART_init (USART_Handle *uh, USART_Regs_Typedef *regs);

inline void USART_enable (USART_Handle *uh)
{
    uh->registers->CR1 |= (0x1 << 13);
}
inline void USART_disable (USART_Handle *uh)
{
    uh->registers->CR1 &= ~(0x1 << 0);
}

void USART_set_baudrate (USART_Handle *uh, int br);
void USART_set_stop_bits (USART_Handle *uh, USART_StopBits sb);
void USART_set_parity (USART_Handle *uh, USART_Parity par);
void USART_set_word_length (USART_Handle *uh, int wl);

int USART_write (USART_Handle uh, const void *buf, size_t count);
int USART_read (USART_Handle uh, const void *buf, size_t count);

#endif /* USART_H */
