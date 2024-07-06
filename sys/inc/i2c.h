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

#ifndef I2C_H
#define I2C_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <stm32F446.h>
#include <util.h>
#include "stddef.h"

typedef struct {
	vuint32_t CR1; /*   Control register 1 */
	vuint32_t CR2; /*   Control register 2 */
	vuint32_t OAR1;/*   Own address register 1 */
	vuint32_t OAR2;/*   Own address register 2 */
	vuint32_t DR; /*    Data register */
	vuint32_t SR1; /*   Status register 1*/
	vuint32_t SR2; /*   Status register 2 */
	vuint32_t CCR; /*   Clock control register */
	vuint32_t TRISE; /* TRISE (rise time) register */
	vuint32_t FLTR; /*  FLTR (filter) register */
} I2C_Regs_Typedef;

#define I2C1 ((I2C_Regs_Typedef*)(APB1_BASE + 0x5400))
#define I2C2 ((I2C_Regs_Typedef*)(APB1_BASE + 0x5800))
#define I2C3 ((I2C_Regs_Typedef*)(APB1_BASE + 0x5C00))

typedef enum {
	idle, sending, receiving, error
} I2C_Status;

typedef struct {
	volatile I2C_Status status;
	I2C_Regs_Typedef *i2cRegs; /*    Ptr to peripheral registers */
	vuint32_t addr; /*            Circuit address on I2C bus */
	SemaphoreHandle_t semlock; /* Semaphore verrou */
	const char *txdata; /*        ptr to data to transmit */
	char *rxdata; /*              ptr to received data */
	size_t txnb; /*               Number of char to transmit */
	size_t rxnb; /*               Number of char to receive */
	unsigned irx; /*              Index utilisé par la routine d'interruption */
	unsigned itx; /*               index caractères transmis */
} I2C_Handle;

extern I2C_Handle *i2c1;
extern I2C_Handle *i2c2;
extern I2C_Handle *i2c3;

__attribute__ ((interrupt("irq"))) void I2C1_Event(void);
__attribute__ ((interrupt("irq"))) void I2C1_Error(void);
__attribute__ ((interrupt("irq"))) void I2C2_Event(void);
__attribute__ ((interrupt("irq"))) void I2C2_Error(void);
__attribute__ ((interrupt("irq"))) void I2C3_Event(void);
__attribute__ ((interrupt("irq"))) void I2C3_Error(void);

void I2C_error(I2C_Handle *i2c);
void I2C_event(I2C_Handle *i2c);

void I2C_init_all(void);
void I2C_init(I2C_Handle *id, I2C_Regs_Typedef *i2cbase);

void I2C_enable(I2C_Handle *i2c);
void I2C_disable(I2C_Handle *i2c);

void I2C_set_std_speed(I2C_Handle*);
void I2C_set_fast_speed(I2C_Handle*);

int I2C_transfer(I2C_Handle *i2c, const uint8_t address, const void *wbuf,
		size_t wlen, void *rbuf, size_t rlen);

int I2C_read(I2C_Handle *id, uint8_t addr, char *r, size_t rn);
int I2C_write(I2C_Handle *id, uint8_t addr, const char *w, size_t wn);

#if 0
void I2C_init_buffer (I2C_Handle*, I2C_Regs_Typedef*);
int I2C_transfer (I2C_Handle *i2c, const uint8_t address, const void *wbuf,
                  size_t wlen, void *rbuf, size_t rlen);
int I2C_read (I2C_Handle *id, uint8_t addr, char *r, size_t rn);
int I2C_write (I2C_Handle *id, uint8_t addr, const char *w, size_t wn);
void I2C_start_periph (I2C_Handle *i2c);

void I2C_error(I2C_Handle *i2c);
void I2C_event(I2C_Handle *i2c);
#endif

#endif /* ifndef I2C_H */
