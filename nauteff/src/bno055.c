/**
 * \Brief Driver for Bosh IMU BNO055
 * \date July 2024
 * \author Emmanuel Gautier
 *
 * Ces fonction assurent les fonction suivantes:
 *   - Communication avec le BNO055 par ;
 *   - L'étalonnage ;
 *   - La lecture des quaternions et des valeurs ;
 *   - ...
 */

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "message_buffer.h"

#include "util.h"
#include "gpio.h"
#include "usart.h"
#include "nauteff.h"
#include "bno055.h"

#if 0
MessageBufferHandle_t bufferBNO055;
void LetInTheClutch(void);
void letOutTheClutch(void);
void runMotorToPort(void);
void stopMotor(void);
void runMotorToStarboard(void);
#endif

//static void BNO055TimerCallback(TimerHandle_t xTimer);

BNO055_Handle bno055_Handle = {
    .usart = usart5,
    .errno = 0};

int BNO055_init(BNO055_Handle *device, UART_Handle *uh)
{
    //char data;
    //int ret;

    device->usart = uh;
#if 0
    /* Device reset */
    data = 0x20; /* TODO : selectionner le quartz externe */
    ret = BNO055_write(device, 0x3F, &data, 1);
    vTaskDelay(pdMS_TO_TICKS(20)); /* TODO : suprimer ultérieurement */
    if (ret != 1)
    {
        device->errno = 456;
        return -1;
    }
    vTaskDelay(pdMS_TO_TICKS(600));

    /* Config Mode : 0x00 -> OPR_MODE(0x3D) */
    data = 0x00; /* Mode _CONFIG */
    ret = BNO055_write(device, 0x3D, &data, 1);
    if (ret != 1)
    {
        device->errno = 68;
        return -1;
    }

    /* Unit selection */
    data = 0x06; 
    ret = BNO055_write(device, 0x3B, &data, 1);
    if (ret != 1)
    {
        device->errno = 73;
        return -1;
    }

    /* Enable interrupt when ACC_BSX_DRDY */
    data = 0x01; 
    ret = BNO055_write(device, 0x0F, &data, 1);
    if (ret != 1)
    {
        device->errno = 65;
        return -1;
    }

    /* Set NDOF Mode */
    data = 0x0C;
    ret = BNO055_write(device, 0x3D, &data, 1);
    if (ret != 1)
    {
        device->errno = 56;
        return -1;
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(600));

    return 0;
}
/*
 *  @brief Write datat to BNO055
 *  @param device : BNO055 device descriptor address
 *  @param reg    : register address where to write
 *  @param data   : data to be written
 *  @param len    : length od data to write
 *  @return len if success, -1 otherwise.
 */
int BNO055_write(BNO055_Handle *device, unsigned reg, void *data, unsigned len)
{
    int ret;
    int len_rx;
    char commande[4];
    char write_rep[2];
    static char message[128];

    /* write command is 0xAA, 0x00, reg address, len , data bytes */
    commande[0] = 0xAA;
    commande[1] = 0x00;
    commande[2] = (char)reg;
    commande[3] = (char)len;


    //USART_flush_rx_buffer(device->usart);

    ret = USART_write(device->usart, commande, 4, 0U); /* write command sending */
    ret = USART_write(device->usart, data, len, 0U);   /* data sending */

    /* Get response message of BNO055 .. */
    write_rep[0] = 0;
    write_rep[1] = 0;
    len_rx = USART_read(device->usart, write_rep, 2, 600U);
    sprintf (message, "BNO055_write retour commande d'écriture : len = %d, rep = %x %x\n", len_rx, (unsigned)write_rep[0], (unsigned)write_rep[1]);
    USART_write(usart1, message, strlen(message), 0U);

    if (len_rx == 2) /* 2 character response message ? */
    {
        if ((write_rep[0] == 0xEE)) /* status message ? */
        {
            if ((write_rep[1] == 0x01)) /*"Did everything work as expected?"*/
            {
                device->errno = 0;
                ret = len;
            }
            else
            {
                device->errno = write_rep[1];
                ret = -1;
            }
        }
        else
        {
            device->errno = 102;
            ret = -1;
        }
    }
    else
    {
        device->errno = 101;
        ret = -1;
    }

    return ret;
}

int BNO055_read(BNO055_Handle *device, unsigned reg, void *data, unsigned len)
{
    int ret;
    int len_rx;
    char commande[4];
    char read_rep[2];

    /* write command is 0xAA, 0x00, reg address, len , data bytes */
    commande[0] = 0xAA; /* Start o f commande*/
    commande[1] = 0x01; /* Read command */
    commande[2] = (char)reg;
    commande[3] = (char)len;

    /* Assumption is made that sending of 4 bytes command */
    /* is made in about 350us at 115200 bps */
    /* and response 128 bytes max is made in about 12 ms */
    /* and BNO055 responds in les than 4 ms. */
    /* So a max delay of 25 ms is enough (.3+12+4~ 17 < 25 ) */

    /* flush receive buffer */
    USART_flush_rx_buffer(device->usart);

    USART_write(device->usart, commande, 4, pdMS_TO_TICKS(0));

    /* Get response message of BNO055 .. */
    /* First check transfer  with first 2 bytes */
    vTaskDelay(pdMS_TO_TICKS(50));
    len_rx = USART_read(device->usart, read_rep, 2, pdMS_TO_TICKS(0));

    device->errno_EE = 0x00;

    if (len_rx == 2) /* 2 character response message ? */
    {
        device->errno_EE = (read_rep[0] << 8) + read_rep[1];
        if ((read_rep[0] == 0xBB)) /* Is there a response  ? */
        {
            // if (read_rep[1] > 0 );
            /* read_rep[1] contains the length of the response */
            if (((unsigned)read_rep[1]) == len) /* Expected length (len) ? */
            {
                /* So, response should be read by next read */
                len_rx = USART_read(device->usart, data, len, pdMS_TO_TICKS(25));
                if (len_rx >= 0)
                {
                    /* Even if len_rx is less than len we consider ne error */
                    /* as Unices read call would do */
                    device->errno = 0;
                    ret = len_rx;
                }
                else
                {
                    device->errno = 113;
                    ret = -1;
                }
            }
            else
            {
                /* TODO flush rx buffer */
                device->errno = 112;
                ret = -1;
            }
        }
        else
        {
            device->errno = 117;
            ret = -1;
        }
    }
    else
    {
        device->errno = 126;
        ret = -1;
    }
    return ret;
}

#if 0
void BNO055TimerCallback(TimerHandle_t xTimer)
{
    /*

    TimerHandle_t BNO055Timer = xTimerCreate("MotorTimer", pdMS_TO_TICKS(10),
    pdTRUE, 0, BNO055TimerCallback);


    bufferBNO055 = "";
    */
}
#endif