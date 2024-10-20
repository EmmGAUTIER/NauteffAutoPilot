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

#include <stm32F446.h>
/*#include "semphr.h"*/
#include "rcc.h"
#include "nvic.h"
#include "i2c.h"

#include "gpio.h"

I2C_Handle i2c1;
I2C_Handle i2c2;
I2C_Handle i2c3;

__attribute__ ((interrupt("irq")))
void DMA1_stream1(void) {
	for (;;)
		;
}

__attribute__ ((interrupt("irq")))
void DMA1_stream2(void) {
	for (;;)
		;
}

void I2C_init() {
	I2C_init_buffer(&i2c1, I2C1);
	I2C_init_buffer(&i2c2, I2C2);
	I2C_init_buffer(&i2c3, I2C3);
}

/*#define  I2C_BUFF_SIZE (48)*/

void I2C_init_buffer(I2C_Handle *id, I2C_Regs_Typedef *i2cbase) {
	id->status = idle;
	id->i2cRegs = i2cbase;
	id->txnb = id->rxnb = 0;
	id->addr = 0;
	id->addr = 0;
	id->txdata = (char*) 0; /* ptr to data to transmit */
	id->rxdata = (char*) 0; /* ptr to received data */
	id->txnb = 0; /* Number of char to transmit */
	id->rxnb = 0; /* Number of char to receive */
	id->irx = 0; /* Index used by ISR */
	id->semlock = xSemaphoreCreateBinary();
	xSemaphoreGive(id->semlock);
}

/*
 * @brief Starts I2C peripheral
 *
 * STM32 peripherals are not clocked after reset, so they do not sink current.
 * So we have to start them by enabling their clock to use them
 * @param i2c : the pointer to pI2C handle
 * @return void
 */
void I2C_start_periph(I2C_Handle *i2c) {
	if (i2c == &i2c1) {
		rcc_start_I2C1();
	} else {
		if (i2c == &i2c2) {
			rcc_start_I2C2();
		} else {
			if (i2c == &i2c3) {
				rcc_start_I2C3();
			} else {
				for (;;)
					;
			}
		}
	}

}

void I2C1_set_std_speed(I2C_Handle *i2c1) {
	(void) i2c1;
}

void I2C1_enable(I2C_Handle *i2c1) {
	(void) i2c1;
}

void I2C_enable(I2C_Handle *i2c) {
	uint32_t CRR;
	uint32_t APBX_freq_MHZ;

	/* First of all, reset the peripheral */
	i2c->i2cRegs->CR1 |= (1 << 15); /*  Reset the I2C peripheral */
	i2c->i2cRegs->CR1 &= ~(1 << 15); /* Normal operation */

	CRR = ((1.0F + 4.0F) * 0.000001F) / (1.0F / (float) RCC_get_APB1_Freq_Hz());
	APBX_freq_MHZ = (RCC_get_APB1_Freq_Hz() / 1000000);
	/* */
	i2c->i2cRegs->CR2 = APBX_freq_MHZ;
	i2c->i2cRegs->CCR = CRR;
	/* TODO : compute the value for TRISE, now we use 4  */
	i2c->i2cRegs->TRISE = 4;
	i2c->i2cRegs->CR1 |= 0x1;

	/* Enable I2C by setting PE to 1 in CR1 */
	i2c->i2cRegs->CR1 |= 0x1;


	if (i2c == &i2c1) {
		NVIC_enable_IRQ(NVIC_NUM_IRQ_I2C1_EV);
		NVIC_enable_IRQ(NVIC_NUM_IRQ_I2C1_ER);
	}
	if (i2c == &i2c2) {
		NVIC_enable_IRQ(NVIC_NUM_IRQ_I2C2_EV);
		NVIC_enable_IRQ(NVIC_NUM_IRQ_I2C2_ER);
	}
	if (i2c == &i2c3) {
		NVIC_enable_IRQ(NVIC_NUM_IRQ_I2C3_EV);
		NVIC_enable_IRQ(NVIC_NUM_IRQ_I2C3_ER);
	}
}

int I2C_transfer(I2C_Handle *i2c, const uint8_t address, const void *wbuf,
		size_t wlen, void *rbuf, size_t rlen) {
	BaseType_t result;

	if (wlen == 0U && rlen == 0U) {
		return 0;
	}

	result = xSemaphoreTake(i2c->semlock, pdMS_TO_TICKS(1000));
	if (result == pdPASS) {
		return -1;
	}

	i2c->txnb = wlen;
	i2c->txdata = (const char*) wbuf;
	i2c->rxnb = wlen;
	i2c->rxdata = (char*) rbuf;
	i2c->status = sending;
	i2c->irx = 0U;
	i2c->itx = 0U;

	i2c->i2cRegs->CR1 |= (1 << 8); /*  Generate START */

	result = xSemaphoreGive(i2c->semlock);
	if (result == pdPASS) {
		return -1;
	}

	return 1U;
}

#if 0

I2C_write(I2C_Handle *i2c, const uint8_t address, const void *buf, size_t len) {
	/*----- I2C_start() -----*/
	i2c->i2cRegs->CR1 |= (1 << 10); /* Enable the ACK */
	i2c->i2cRegs->CR1 |= (1 << 8); /*  Generate START */
	while (!(i2c->i2cRegs->SR1 & (1 << 0)))
		/* Wait fror SB bit to set */
		;

	/*----- I2C_address() -----*/
	i2c->i2cRegs->DR = address; /* send the address*/
	/* wait for ADDR bit to set */
	while (!(i2c->i2cRegs->SR1 & (1 << 1)))
		;
	/* read SR1 and SR2 to clear the ADDR bit */
	uint8_t temp = i2c->i2cRegs->SR1 | i2c->i2cRegs->SR2;

	/*----- Loop write data -----*/
	/* wait for TXE bit to set */
	while (!(i2c->i2cRegs->SR1 & (1 << 7)))
		;
	for (int count = 0; count < len; count++) {
		/* wait for TXE bit to set */
		while (!(i2c->i2cRegs->SR1 & (1 << 7)))
			;
		i2c->i2cRegs->DR = (uint32_t) buf[count]; /* send data */

		/* wait for BTF to set */
		while (!(i2c->i2cRegs->SR1 & (1 << 2)))
			;
	}
	/*----- Send stop, es genugt -----*/
	i2c->i2cRegs->CR1 |= (0x1U << 9);
}

void MPU_Write(uint8_t Address, uint8_t Reg, uint8_t Data) {
	I2C_Start();
	I2C_Address(Address);
	I2C_Write(Reg);
	I2C_Write(Data);
	I2C_Stop();
}

void MPU_Read(uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size) {
	I2C_Start();
	I2C_Address(Address);
	I2C_Write(Reg);
	I2C_Start(); /* repeated start*/
	I2C_Read(Address + 0x01, buffer, size);
	I2C_Stop();
}
#endif

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax;
float Ay;
float Az;
float Gx;
float Gy;
float Gz;

#if 0

uint8_t check;

void MPU6050_Init(void) {
	uint8_t check;
	uint8_t Data;

	/* check device ID WHO_AM_I */

	MPU_Read(MPU6050_ADDR, WHO_AM_I_REG, &check, 1);

	if (check == 104) /* 0x68 will be returned by the sensor if everything goes well */
	{
		/* power management register 0X6B we should write all 0's to wake the sensor up */
		Data = 0;
		MPU_Write(MPU6050_ADDR, PWR_MGMT_1_REG, Data);

		/* Set DATA RATE of 1KHz by writing SMPLRT_DIV register */
		Data = 0x07;
		MPU_Write(MPU6050_ADDR, SMPLRT_DIV_REG, Data);

		/* Set accelerometer configuration in ACCEL_CONFIG Register */
		/* XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g */
		Data = 0x00;
		MPU_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, Data);

		/* Set Gyroscopic configuration in GYRO_CONFIG Register */
		/* XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 250 ?/s */
		Data = 0x00;
		MPU_Write(MPU6050_ADDR, GYRO_CONFIG_REG, Data);
	}

}

#endif

void I2C_event(I2C_Handle *i2c) {
	(void) i2c;
	for (;;)
		;
}

void I2C_error(I2C_Handle *i2c) {
	(void) i2c;
	for (;;)
		;
}

__attribute__ ((interrupt("irq")))
void I2C1_Event(void) {
	I2C_event(&i2c1);
}

__attribute__ ((interrupt ("IRQ")))
void I2C2_Event(void) {
	I2C_event(&i2c2);
}

__attribute__ ((interrupt ("IRQ")))
void I2C3_Event(void) {
	I2C_event(&i2c3);
}

__attribute__ ((interrupt ("IRQ")))
void I2C1_Error(void) {
	I2C_error(&i2c1);
}

__attribute__ ((interrupt ("IRQ")))
void I2C2_Error(void) {
	I2C_error(&i2c2);
}

__attribute__ ((interrupt ("IRQ")))
void I2C3_Error(void) {
	I2C_error(&i2c3);
}

#if 0


int I2C_write_read (I2C_Handle *i2c, uint8_t addr, const char *w, size_t wn,
                char *r, size_t rn)
{
    int ret = 0;

    if (wn == 0 && rn == 0)
        { /* Rien à transmettre ni à recevoir */
            return 0;
        }

    ret = xSemaphoreTake(i2c->semlock, 100);
    if (ret != pdPASS)
        {
            /*ajouteMessage("transfer_I2C take raté au déb.;");*/
            return -1;
        }
    i2c->txdata = w;
    i2c->rxdata = r;
    i2c->txnb = wn;
    i2c->rxnb = rn;
    i2c->addr = addr;
    i2c->irx = 0;

    I2C1->CR1 |= 0x01; /* Peripheral enable */

    if (w > 0)
        {
            ;
        }

    xSemaphoreGive(i2c->semlock);
}
/*
 * Vieilles fonctions :
 */
void sysinit_I2C1 (void)
{
    I2C_init_buffer (&I2C1_Buffer, I2C1);

    /* Envoyons l'horloge à I2C1 */
    RCC_APB1ENR |= (0x1 << 21);

    /*I2C1 utilise HSI */
    /*RCC_CFGR3 &= ~(0x1 << 4);	// HSI clock (8Mhz) pour cadencer le bus I2C */

    /* Compteurs de temps doc RM0316, p. 849, table 147 */
    /* I2CCLK est HSI à 8Mhz */
    I2C1->TIMINGR = (0x01 << 28) /* PRESC Timing prescaler */
    | (0x13 << 0) /* SCLL SCL low period */
    | (0x0F << 8) /* SCLH SCL high period */
    | (0x02 << 16) /* SDADEL Data hold time */
    | (0x04 << 20); /* SCLDEL Data setup time */
    ;

    //I2C1->CR1 |= 0x01;		// Peripheral enable

    // autorisation interruption
    // 31 : I2C1_Event et 32 : I2C1_Error
    NVIC_ISER0 = (0x1 << 30) | (0x1 << 31);
#if 1
    NVIC_IPR_BASE[31] = 0x60;
    NVIC_IPR_BASE[32] = 0x0;
#endif
}

int I2C_transfer (I2C_Handle *id, uint8_t addr, const char *w, size_t wn,
                  char *r, size_t rn)
{
    int ret;
    int valret = 0;

    if (wn == 0 && rn == 0)
        {	// Rien à transmettre ni à recevoir
            return 0;
        }

    ret = xSemaphoreTake(id->semlock, 100);
    if (ret != pdPASS)
        {
            //ajouteMessage("transfer_I2C take raté au déb.;");
            return -1;
        }

    id->txdata = w;
    id->rxdata = r;
    id->txnb = wn;
    id->rxnb = rn;
    id->addr = addr;
    id->irx = 0;

    I2C1->CR1 |= 0x01;		// Peripheral enable

    id->i2cRegs->CR1 |= (0x1 << 7)   // ERRIE  Error interrupts enable
    | (0x1 << 6)   // TCIE   Transfert complete interrupt enable
            | (0x1 << 5)   // STOPIE Stop detection interrupt enable
            | (0x1 << 4)   // NACKIE Not aknowledge received  interrupt enable
            | (0x1 << 2)   // RXIE   RX Interrupt Enable
            | (0x1 << 1)   // TXIE   TX Interrupt Enable
            ;

    if (wn > 0)
        {
            // NBYTES = wn et SADDR [8:1] = addr
            id->status = sending;
            id->i2cRegs->CR2 = (wn & 0xFF) << 16 | addr << 1;
            id->i2c->CR2 &= ~(0x1 << 10); // RD_WRN (bit 0) <--0 : le maître demande un envoi
            id->i2c->TXDR = w[id->irx++]; // Mettons dans TXDR le premier octet à transmettre
            if (rn == 0)
                {
                    id->i2cRegs->CR2 |= (0x1 << 25);
                }
            else
                {
                    id->i2cRegs->CR2 &= ~(0x1 << 25);
                }
            id->i2c->CR2 |= 0x1 << 13;     // Send Start
            //ajouteMessage("transfer_I2C envoi START;");
            // et c'est parti ...
        }
    else
        {
            // NBYTES = rn et SADDR [8:1] = addr
            id->status = receiving;
            id->i2cRegs->CR2 = (rn & 0xFF) << 16 | addr << 1;
            id->i2c->CR2 |= (0x1 << 10); // RD_WRN (bit 0) <--1 : le maître demande une réception
            id->i2c->CR2 |= (0x1 << 13);  // Send Start
            //ajouteMessage("transfer_I2C envoi START;");
            // et c'est parti ...
        }

    //ajouteMessage("transfer_I2C c'est parti;");

    ret = xSemaphoreTake(id->semlock, 100);

    // interdisons les interruptions
    id->i2c->CR1 &= ~((0x1 << 7)   // ERRIE  Error interrupts enable
    | (0x1 << 6)   // TCIE   Transfert complete interrupt enable
            | (0x1 << 5)   // STOPIE Stop detection interrupt enable
            | (0x1 << 4)   // NACKIE Not aknowledge received  interrupt enable
            | (0x1 << 2)   // RXIE   RX Interrupt Enable
            | (0x1 << 1)   // TXIE   TX Interrupt Enable
    );

    if (ret == pdPASS)
        {
            //ajouteMessage("transfer_I2C take réussi à la fin;");
            valret = (id->status == error) ? -1 : 1;
        }
    else
        {
            //ajouteMessage("transfer_I2C take raté à la fin;");
            valret = -1;
        }
    ret = xSemaphoreGive(id->semlock);

    return valret;
}

//------------------------------------------------------------
//
//  Interruption I2C1
//
//  Attention : routine utilisée par I2C1 et EXTI ligne 13
//------------------------------------------------------------

__attribute__ ((interrupt ("IRQ")))
void I2C_Event (void)
{
    char car;
    static int ret;
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    //static char tampon[20];
    //static int I2C1_Event = 0;

    //int_I2C1_Event++;
    //ajouteMessage("I2C1_Event ");
    //itoa(tampon, 18, int_I2C1_Event);
    //ajouteMessage(tampon);
    //ajouteMessage(" ");

    /*----- Cas TXIS : un caractère de plus à transmettre -----*/
    if (I2C1->ISR & (0x1 << 1))
        {
            //ajouteMessage(" TXIS détecté;");
            // effacement de TXIS en écrivant TXDR
            I2C1->TXDR = I2C1_Buffer.txdata[I2C1_Buffer.irx++];
            return;
        }

    /*----- cas TC transfert complete -----*/
    // en émission uniquement.
    if (I2C1->ISR & (0x1 << 6))
        {
            //ajouteMessage(" TC détecté;");
            // il faut envoyer un START ou un STOP
            if (I2C1_Buffer.rxnb > 0 && I2C1_Buffer.status == sending)
                {
                    I2C1_Buffer.status = receiving;
                    I2C1_Buffer.irx = 0;
                    // Le maître demande une réception
                    I2C1->CR2 = (I2C1_Buffer.rxnb & 0xFF) << 16
                            | I2C1_Buffer.addr << 1;
                    I2C1->CR2 |= (0x1 << 10); // RD_WRN (bit 10) : Master requests read transfer
                    I2C1->CR2 |= (0x1 << 13); // START (bit13) : Restart/start generation
                    // il faut attendre la réception d'un caractère.
                    //ajouteMessage(" envoi START pour reception\n");
                }
            else
                {
                    if (I2C1_Buffer.status == receiving)
                        {
                            I2C1_Buffer.rxdata[I2C1_Buffer.irx++] = I2C1->RXDR;
                        }
                    I2C1->CR2 |= (0x1 << 14);  // STOP (bit14) : STOP generation
                    //ajouteMessage(" envoi STOP;");
                    // Il faut prévenir la fonction de transfer depuis une interruption
                    //ret = xSemaphoreGiveFromISR(I2C1_Buffer.semlock, &xHigherPriorityTaskWoken);
                    //ajouteMessage(" SemGiveFromISR a :");
                    //itoa (tampon, 20, ret);
                    //ajouteMessage(tampon);
                    //ajouteMessage("\n");
                }
            return;
        }
    // Effacement du bit TC dans ISR lorsque START out STOP détecté

    /*----- Stop Detection Flag -----*/
    if (I2C1->ISR & (0x1 << 5))
        {
            //ajouteMessage(" STOP détecté;");
            // STOPF : Stop detection Flag
            I2C1->ICR |= (0x1 << 5);
            // Il faut prévenir la fonction de transfer depuis une interruption
#if 1
            ret = xSemaphoreGiveFromISR(I2C1_Buffer.semlock,
                                        &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            //ajouteMessage(" SemGiveFromISR b : ");
            //inttoa(tampon, 20, ret);
            //ajouteMessage(tampon);
            //ajouteMessage(";");
            I2C1_Buffer.status = idle;
#endif
            return;
        }

    if (I2C1->ISR & (0x1 << 4))
        { // NACKF : Not ACKnowledge received Flag
            ;			  // Il vaudra mieux abandonner ces tranfers ...
            I2C1->ICR |= (0x1 << 4);
            //for (;;) ;
            return;
        }

    /*----- RXNE : Receive data register Not Empty -----*/
    if (I2C1->ISR & (0x1 << 2))
        {
            //ajouteMessage(" RXNE détecté ' ");
            car = I2C1->RXDR;	// effacement de RXNE en lisant RXDR
            /*inttoa(tampon, sizeof(tampon), (int) car);
             ajouteMessage(tampon);
             ajouteMessage("'  reçu;");*/
            I2C1_Buffer.rxdata[I2C1_Buffer.irx++] = car;
            if (I2C1_Buffer.irx >= I2C1_Buffer.rxnb)
                {
                    ret = xSemaphoreGiveFromISR(I2C1_Buffer.semlock,
                                                &xHigherPriorityTaskWoken);
                    //ajouteMessage(" SemGiveFromISR c :");
                    //inttoa(tampon, 20, ret);
                    //ajouteMessage(tampon);
                    //ajouteMessage(";");
                    //ajouteMessage(" et envoi STOP;");
                    I2C1->CR2 |= (0x1 << 14); // Envoyons un STOP
                }
            return;
        }
}

#endif
