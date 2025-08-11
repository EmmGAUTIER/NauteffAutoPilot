/*
MIT License

Copyright (c) 2025 Emmanuel Gautier / Nauteff

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
 * MEMs driver
 *
 */
#define LSM9DS1_MAG_I2C_ADDR (0x1E) /* Magnetometer device address */
#define LSM9DS1_XLG_I2C_ADDR (0x6B) /* Accelerometer and gyrometer device address */
#define MEMS_PERIOD_MS 200
/*
 * Magnetometer register addresses
 */

/* Offset registers in order to compensate environmental effects */
#define OFFSET_X_REG_L_M 0x05
#define OFFSET_X_REG_H_M 0x06
#define OFFSET_Y_REG_L_M 0x07
#define OFFSET_Y_REG_H_M 0x18
#define OFFSET_Z_REG_L_M 0x09
#define OFFSET_Z_REG_H_M 0x1A

/* Identification register WHO AM I */
#define WHO_AM_I_REG_M 0x0F

/* Magnetic control registers */
#define CTRL_REG1_M 0x20
#define CTRL_REG2_M 0x21
#define CTRL_REG3_M 0x22
#define CTRL_REG4_M 0x23
#define CTRL_REG5_M 0x24

/* Status registers */
#define STATUS_REG_M 0x27

/* Data output registers */
#define OUT_X_L_M 0x28
#define OUT_X_H_M 0x29
#define OUT_Y_L_M 0x2A
#define OUT_Y_H_M 0x2B
#define OUT_Z_L_M 0x2C
#define OUT_Z_H_M 0x2D

// Registres d'interruption
#define INT_CFG_M 0x30
#define INT_SRC_M 0x31
#define INT_THS_L_REG_M 0x32
#define INT_THS_H_REG_M 0x33

/*
 * Register addresses of magnetometer and gyrometer registers
 */

#define ACT_THS 0x04
#define ACT_DUR 0x05
#define INT_GEN_CFG_XL 0x06
#define INT_GEN_THS_X_XL 0x07
#define INT_GEN_THS_Y_XL 0x08
#define INT_GEN_THS_Z_XL 0x09
#define INT_GEN_DUR_XL 0x0A
#define REFERENCE_G 0x0B

#define INT1_CTRL 0x0C
#define INT2_CTRL 0x0D

/* Identification register */
#define WHO_AM_I_REG_XG 0x0F

/* Control registers */
#define CTRL_REG1_G 0x10
#define CTRL_REG2_G 0x11
#define CTRL_REG3_G 0x12
#define ORIENT_CFG_G 0x13
#define INT_GEN_SRC_G 0x14
#define OUT_TEMP_L 0x15
#define OUT_TEMP_H 0x16

#define STATUS_REG_1 0x17

/* Gyrometer output data registers */
#define OUT_X_L_G 0x18
#define OUT_X_H_G 0x19
#define OUT_Y_L_G 0x1A
#define OUT_Y_H_G 0x1B
#define OUT_Z_L_G 0x1C
#define OUT_Z_H_G 0x1D
#define OUT_GYR OUT_X_L_G /* For accessing x,y,z of gyr reading 6 bytes */

#define CTRL_REG4 0x1E
#define CTRL_REG5_XL 0x1F

#define CTRL_REG6_XL 0x20
#define CTRL_REG7_XL 0x21

#define CTRL_REG8 0x22
#define CTRL_REG9 0x23
#define CTRL_REG10 0x24

#define INT_GEN_SRC_XL 0x26

#define STATUS_REG_2 0x27

/* Accelerometer data output registers */
#define OUT_X_L_XL 0x28
#define OUT_X_H_XL 0x29
#define OUT_Y_L_XL 0x2A
#define OUT_Y_H_XL 0x2B
#define OUT_Z_L_XL 0x2C
#define OUT_Z_H_XL 0x2D

#define FIFO_CTRL 0x2E
#define FIFO_SRC 0x2F

#define INT_GEN_CFG_G 0x30
#define INT_GEN_THS_XH_G 0x31
#define INT_GEN_THS_XL_G 0x32
#define INT_GEN_THS_YH_G 0x33
#define INT_GEN_THS_YL_G 0x34
#define INT_GEN_THS_ZH_G 0x35
#define INT_GEN_THS_ZL_G 0x36
#define INT_GEN_DUR_G 0x37

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "printf.h"
#include <stdbool.h>

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>
#include <stm32l4xx_ll_usart.h>

#include "main.h"
#include "stm32l4xx_hal.h"

#include "util.h"
#include "rlib.h"
#include "service.h"

#include "mems.h"

#include "aux_usart.h"
#include "geom.h"
#include "autopilot.h"
// #include "madgwick.h"

/*
 * Offsets and gain values for the device that is connected
 * those values are meant to be obtained from calibration in the future
 * and stored in flash memory.
 */

#define MEMS_ACC_CORR_OFFSET_X ((9.97F - 9.62F) / 2.F)
#define MEMS_ACC_CORR_OFFSET_Y ((9.79F - 9.75F) / 2.F)
#define MEMS_ACC_CORR_OFFSET_Z ((10.15F - 9.68F) / 2.F)

#define MEMS_ACC_CORR_GAIN_X (MEMS_STANDARD_GRAVITY / ((9.97 + 9.62) / 2.F))
#define MEMS_ACC_CORR_GAIN_Y (MEMS_STANDARD_GRAVITY / ((9.79 + 9.75) / 2.F))
#define MEMS_ACC_CORR_GAIN_Z (MEMS_STANDARD_GRAVITY / ((9.68 + 10.15) / 2.F))

#define MEMS_MAG_CORR_OFFSET_X ((2240.F - 3800.F) / 2.F)
#define MEMS_MAG_CORR_OFFSET_Y ((1300.F - 4685.F) / 2.F)
#define MEMS_MAG_CORR_OFFSET_Z ((1450.F - 4100.F) / 2.F)

#define MEMS_MAG_CORR_GAIN_X (MEMS_STANDARD_GRAVITY / ((3800.F + 2240.F) / 2.F))
#define MEMS_MAG_CORR_GAIN_Y (MEMS_STANDARD_GRAVITY / ((1300.F + 4685.F) / 2.F))
#define MEMS_MAG_CORR_GAIN_Z (MEMS_STANDARD_GRAVITY / ((1450.F + 4100.F) / 2.F))

#define MEMS_GYR_CORR_OFFSET_X (+0.0010589F)
#define MEMS_GYR_CORR_OFFSET_Y (-0.0077812F)
#define MEMS_GYR_CORR_OFFSET_Z (-0.0199720F)

void timerMEMsCallback(TimerHandle_t xTimer);

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

/* The callbacks give those semaphores */
/* to signal to I2C_Write/Read the end of transfer*/
SemaphoreHandle_t semi2c1tx = NULL;
SemaphoreHandle_t semi2c1rx = NULL;

/* The queue of messages of the MEMs task */
QueueHandle_t msgQueueMEMs = (QueueHandle_t)0;

/* Timer : sends MSG periodicaly */
static TimerHandle_t timerMEMs = (TimerHandle_t)0;

unsigned cpt1 = 0U; // For debugging
unsigned cpt2 = 0U; // For debugging

/*
 * @brief Read data from a peripheral memory by I2C bus
 *
 * This function reads data over I2C1 bus from a specified memory address of an I2C device.
 * It uses DMA for the transfer and and uses semaphore for synchronisation.
 *
 * @param DevAddress Device address
 * @param MemAddress Memory address to read from
 * @param pData Pointer to the buffer where data will be stored
 * @param Size Number of bytes to read
 * @param delay Maximum time to wait for the semaphore
 * @return Number of bytes read, or -1 on error or timeout

 * The address of the device is restricted to 7 bits.
 * The function shifts the address left by 1 bit.
 * @note This function is not reentrant.
 */

int I2C_Mem_Read(uint16_t DevAddress,
                 uint16_t MemAddress,
                 uint8_t *pData,
                 uint16_t Size,
                 TickType_t delay)
{
    HAL_StatusTypeDef status;
    BaseType_t ret;

    /* No delay, and function is not reentrant, */
    ret = xSemaphoreTake(semi2c1rx, (TickType_t)0);
    if (ret == pdFAIL)
    {
        return -1; /* Not Given */
    }

    status = HAL_I2C_Mem_Read_DMA(&hi2c1,
                                  (DevAddress << 1),
                                  MemAddress,
                                  I2C_MEMADD_SIZE_8BIT,
                                  pData,
                                  Size);

    if (status != HAL_OK)
    {
        /* Error.  Release semaphore, status is ignored */
        /* since give fails only if semaphore was taken by another task */
        xSemaphoreGive(semi2c1rx);
        return -2;
    }

    /* Wait for the semaphore to be given by the ISR */
    ret = xSemaphoreTake(semi2c1rx, delay);
    if (ret == pdFAIL)
    {
        /* Timeout */
        xSemaphoreGive(semi2c1rx); // Release the semaphore
        return -3;                 // Timeout error
    }

    // vTaskDelay(20);
    xSemaphoreGive(semi2c1rx); // Release the semaphore
    return Size;               /* Success */
}

/*
 *
 * This function write data over I2C1 bus to a specified memory address of an I2C device.
 * It uses DMA for the transfer and and uses semaphore for synchronisation.
 *
 * @param DevAddress Device address
 * @param MemAddress Memory address to read from
 * @param pData Pointer to the buffer where data will be stored
 * @param Size Number of bytes to read
 * @param delay Maximum time to wait for the semaphore
 * @return Number of bytes read, or -1 on error or timeout

 * The address of the device is restricted to 7 bits.
 * The function shifts the address left by 1 bit.
 * @note This function is not reentrant.
 */

int I2C_Mem_Write(uint16_t DevAddress,
                  uint16_t MemAddress,
                  uint8_t *pData,
                  uint16_t Size,
                  TickType_t delay)
{
    HAL_StatusTypeDef status;
    BaseType_t ret;

    /* No delay, and function is not reentrant, */
    ret = xSemaphoreTake(semi2c1tx, (TickType_t)0);
    if (ret == pdFAIL)
    {
        return -1; /* Not Given */
    }

    status = HAL_I2C_Mem_Write_DMA(&hi2c1,
                                   (DevAddress << 1),
                                   MemAddress,
                                   I2C_MEMADD_SIZE_8BIT,
                                   pData,
                                   Size);

    if (status != HAL_OK)
    {
        /* Error.  Release semaphore, status is ignored */
        /* since give fails only if semaphore was taken by another task */
        xSemaphoreGive(semi2c1tx);
        return -2;
    }

    /* Wait for the semaphore to be given by the ISR */
    ret = xSemaphoreTake(semi2c1tx, delay);
    if (ret == pdFAIL)
    {
        /* Timeout */
        xSemaphoreGive(semi2c1tx); // Release the semaphore
        return -3;                 // Timeout error
    }

    // vTaskDelay(50);

    // status = HAL_I2C_IsDeviceReady(&hi2c1, DevAddress << 1, 3, 50);

    xSemaphoreGive(semi2c1tx); // Release the semaphore
    return Size;               /* Success */
}

/*
 * @brief Initialize MEMs task
 *
 * Create two binary semaphores used to signal the completion of I2C transfers.
 * Semaphores are given so they can be taken.
 *
 */

int init_taskMEMs()
{
    // aux_USART_Init_all();

    semi2c1tx = xSemaphoreCreateBinary();
    xSemaphoreGive(semi2c1tx);
    semi2c1rx = xSemaphoreCreateBinary();
    xSemaphoreGive(semi2c1rx);

    /* MEMs task queue creation */
    msgQueueMEMs = xQueueCreate(10, sizeof(MEMS_MsgType_t));

    /* create tick timer */
    timerMEMs = xTimerCreate("MEMs",
                             pdMS_TO_TICKS(MEMS_PERIOD_MS),
                             pdTRUE,    /* Auto reload (repeat indefinitely) */
                             (void *)0, /* Timer ID, not used */
                             timerMEMsCallback);
    /* TODO : test creation of objects*/
    return 0;
}

void timerMEMsCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    static MEMS_Msg_t command = {
        .msgType = MEMS_MSG_TICK};

    xQueueSend(msgQueueMEMs, (const void *)&command, (TickType_t)0);

    return;
}

int config_MEMs(void)
{
    int ret_m;
    int ret_xl1;
    int ret_int;

    /* Magnetometer configuration */
    static uint8_t cfg_m_1[] = {0x54, /* X&Y : high perf.,  ODR : 20Hz */
                                0x00,
                                0x00,
                                0x08, /* Z : High perf. */
                                0x00};

    ret_m = I2C_Mem_Write(LSM9DS1_MAG_I2C_ADDR,
                          0x20,
                          cfg_m_1,
                          5,
                          100);

    /* Accelerometer and gyrometer configuration */
    static uint8_t cfg_xl_1[] = {(1) << 5,
                                 0x00,
                                 0x00};
    ret_xl1 = I2C_Mem_Write(LSM9DS1_XLG_I2C_ADDR, 0x10, cfg_xl_1, sizeof(cfg_xl_1), 100);

    /* LSM9DS1 interrupts generation configuration */
    /* Interrupts set INT pins of LSM9DS1 */

    static uint8_t cfg_int[] = {
        0x01, /* INT1 pin : DRDY_XL, data ready accelerometer */
        0x02  /* INT2 pin : DRDY_G , data ready gyrometer */
    };
    ret_int = I2C_Mem_Write(LSM9DS1_XLG_I2C_ADDR, 0x0C, cfg_int, 2, 100);

    if (ret_m < 0 || ret_xl1 < 0 || ret_int < 0)
    {
        return -1;
    }
    else
    {
        return 1;
    }
}

void taskMEMs(void *param)
{
    (void)param;

    BaseType_t ret;
    int res;
    MEMS_Msg_t msg;
    int16_t vec3i16[3]; /* 3 vectors as uint16_t read from MEMs device */
    char message[100];
    size_t len;

    unsigned long compteur = 0UL;

    unsigned accNumber = 0;
    unsigned gyrNumber = 0;
    unsigned magNumber = 0;
    Vector3f accCumul = Vector3f_null;
    Vector3f gyrCumul = Vector3f_null;
    Vector3f magCumul = Vector3f_null;
    Vector3f accMean = Vector3f_null;
    Vector3f gyrMean = Vector3f_null;
    Vector3f magMean = Vector3f_null;

    unsigned accTotal = 0;
    unsigned gyrTotal = 0;
    unsigned magTotal = 0;

    config_MEMs();

    /* Interrupts mustn't be enabled before the message queue is created */
    /* so enable them now */
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    xTimerStart(timerMEMs, (TickType_t)0);

    res = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_XL,
                       (uint8_t *)&vec3i16, 6,
                       pdMS_TO_TICKS(100));
    res = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_G,
                       (uint8_t *)&vec3i16, 6,
                       pdMS_TO_TICKS(100));
    res = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_M,
                       (uint8_t *)&vec3i16, 6,
                       pdMS_TO_TICKS(100));

    for (;;)
    {

        // static char message[100];

        ret = xQueueReceive(msgQueueMEMs, &msg, pdMS_TO_TICKS(500));

        if (ret == pdPASS)
        {
            switch (msg.msgType)
            {
            case MEMS_MSG_ACC_READY: /* Accelerometer data ready */
                res = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_XL,
                                   (uint8_t *)&vec3i16, 6,
                                   pdMS_TO_TICKS(100));
                if (res > 0)
                {
                    accCumul.x += (float)vec3i16[0];
                    accCumul.y += (float)vec3i16[1];
                    accCumul.z += (float)vec3i16[2];
                    accNumber++;
                    accTotal++;
                    len = snprintf(message, sizeof(message),
                                   "ACC %d %d %d\n",
                                   vec3i16[0], vec3i16[1], vec3i16[2]);
                    // svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                }
                break;

            case MEMS_MSG_GYR_READY: /* Gyrometer data ready */
                res = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_G,
                                   (uint8_t *)&vec3i16, 6,
                                   pdMS_TO_TICKS(100));
                if (res > 0)
                {
                    gyrCumul.x += (float)vec3i16[0];
                    gyrCumul.y += (float)vec3i16[1];
                    gyrCumul.z += (float)vec3i16[2];
                    gyrNumber++;
                    gyrTotal++;
                    len = snprintf(message, sizeof(message),
                                   "GYR %d %d %d\n",
                                   vec3i16[0], vec3i16[1], vec3i16[2]);
                    // svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                }
                break;

            case MEMS_MSG_MAG_READY: /* Magnetometer data ready */
                res = I2C_Mem_Read(LSM9DS1_MAG_I2C_ADDR, OUT_X_L_M,
                                   (uint8_t *)&vec3i16, 6,
                                   pdMS_TO_TICKS(100));
                if (res > 0)
                {
                    magCumul.x += (float)vec3i16[0];
                    magCumul.y += (float)vec3i16[1];
                    magCumul.z += (float)vec3i16[2];
                    magNumber++;
                    magTotal++;
                    len = snprintf(message, sizeof(message),
                                   "MAG %d %d %d\n",
                                   vec3i16[0], vec3i16[1], vec3i16[2]);
                    // svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                }
                break;

            case MEMS_MSG_TICK: /* Compute and send orientation */
                if (accNumber > 0)
                {
                    accMean.x = accCumul.x / (float)accNumber;
                    accMean.y = accCumul.y / (float)accNumber;
                    accMean.z = accCumul.z / (float)accNumber;

                    accMean.x += MEMS_ACC_CORR_OFFSET_X;
                    accMean.x *= MEMS_ACC_CORR_GAIN_X;
                    accMean.y += MEMS_ACC_CORR_OFFSET_Y;
                    accMean.y *= MEMS_ACC_CORR_GAIN_Y;
                    accMean.z += MEMS_ACC_CORR_OFFSET_Z;
                    accMean.z *= MEMS_ACC_CORR_GAIN_Z;

                    len = snprintf(message, sizeof(message) - 1,
                                   "ACC  %d %.2f %.2f %.2f    %.2f\n",
                                   accNumber,
                                   accMean.x, accMean.y, accMean.z, vector3f_getNorm(accMean));
                    svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                }

                if (gyrNumber > 0)
                {
                    gyrMean.x = gyrCumul.x / (float)gyrNumber;
                    gyrMean.y = gyrCumul.y / (float)gyrNumber;
                    gyrMean.z = gyrCumul.z / (float)gyrNumber;

                    gyrMean.x += MEMS_GYR_CORR_OFFSET_X;
                    gyrMean.y += MEMS_GYR_CORR_OFFSET_Y;
                    gyrMean.z += MEMS_GYR_CORR_OFFSET_Z;

                    len = snprintf(message, sizeof(message) - 1,
                                   "GYR  %d %.1f %.1f %.1f    %.1f\n",
                                   gyrNumber,
                                   gyrMean.x, gyrMean.y, gyrMean.z, vector3f_getNorm(gyrMean));
                    svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                }
                if (magNumber > 0)
                {
                    magMean.x = magCumul.x / (float)magNumber;
                    magMean.y = magCumul.y / (float)magNumber;
                    magMean.z = magCumul.z / (float)magNumber;

                    magMean.x += MEMS_MAG_CORR_OFFSET_X;
                    magMean.x *= MEMS_MAG_CORR_GAIN_X;
                    magMean.y += MEMS_MAG_CORR_OFFSET_Y;
                    magMean.y *= MEMS_MAG_CORR_GAIN_Y;
                    magMean.z += MEMS_MAG_CORR_OFFSET_Z;
                    magMean.z *= MEMS_MAG_CORR_GAIN_Z;

                    len = snprintf(message, sizeof(message) - 1,
                                   "MAG  %d %.2f %.2f %.2f    %.2f\n",
                                   magNumber,
                                   magMean.x, magMean.y, magMean.z, vector3f_getNorm(magMean));
                    svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                }

                accNumber = 0;
                gyrNumber = 0;
                magNumber = 0;
                accCumul = Vector3f_null;
                gyrCumul = Vector3f_null;
                magCumul = Vector3f_null;

                break;

            case MEMS_MSG_CALIBRATE:
                len = snprintf(message, sizeof(message),
                               "-----> Calibration demandée\n");
                svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));

                /* Allocate memory to store values  */

                break;

            default:
                /* shouldn't hapen */
            }
        }

        compteur++;
    }
}
