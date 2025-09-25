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

#define DBG_PRINT_RAW_VALUES_ACC(X) (X)
#define DBG_PRINT_RAW_VALUES_GYR(X)
#define DBG_PRINT_RAW_VALUES_MAG(X) (X)
#define DBG_PRINT_MEAN_RAW_VALUES(X)  (X)
#define DBG_PRINT_MEAN_CORR_VALUES(X) (X)
#define DBG_PRINT_ATTITUDE(X) (X)

#define LSM9DS1_ODR LSM9DS1_ODR_G_59_5_HZ /* mag and acc ouptut data rate */
#define LSM9DS1_FS_G LSM9DS1_FS_G_500_DPS /* gyro full scale */
#define LSM9DS1_FS_XL LSM9DS1_FS_XL_8G    /* acc full scale */
#define LSM9DS1_DO LSM9DS1_DO_40_HZ       /* Mag output data rate */
#define LSM9DS1_FS_M LSM9DS1_FS_M_8_GAUSS /* Mag full scale */

/*
 * MEMs driver
 *
 */
// #define LSM9DS1_MAG_I2C_ADDR (0x1E) /* Magnetometer device address */
// #define LSM9DS1_XLG_I2C_ADDR (0x6B) /* Accelerometer and gyrometer device address */
#define MEMS_PERIOD_MS 100
#define MEMS_PERIOD_POLL 10
#define MEMS_PERIOD_S ((float)MEMS_PERIOD_MS / 1000.0F)
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

/* ODR in CTRL_REG_1[5..7] */
#define LSM9DS1_ODR_POWER_DOWN (0x0 << 5)
#define LSM9DS1_ODR_G_14_9_HZ (0x1 << 5)
#define LSM9DS1_ODR_G_59_5_HZ (0x2 << 5)
#define LSM9DS1_ODR_G_119_HZ (0x3 << 5)
#define LSM9DS1_ODR_G_238_HZ (0x4 << 5)
#define LSM9DS1_ODR_G_476_HZ (0x5 << 5)
#define LSM9DS1_ODR_G_592_HZ (0x6 << 5)

/* FS_G[0..1] in CTRL_REG1_G[3..4] */
#define LSM9DS1_FS_G_245_DPS (0x0 << 3) /* 245 deg/s */
#define LSM9DS1_FS_G_500_DPS (0x1 << 3) /* 245 deg/s */
/* (0x2 <<3) not available  */
#define LSM9DS1_FS_G_2000_DPS (0x3 << 3) /* 2000 deg/s */

/* FS_XL in CTRL_REG6[3..4] : accelerometer full scale */
#define LSM9DS1_FS_XL_2G (0x0 << 3)
#define LSM9DS1_FS_XL_16G (0x1 << 3)
#define LSM9DS1_FS_XL_4G (0x2 << 3)
#define LSM9DS1_FS_XL_8G (0x3 << 3)

#define LSM9DS1_FS_M_4_GAUSS (0x0 << 5)
#define LSM9DS1_FS_M_8_GAUSS (0x1 << 5)
#define LSM9DS1_FS_M_12_GAUSS (0x2 << 5)
#define LSM9DS1_FS_M_16_GAUSS (0x3 << 5)

/* DO in CTRL_REG1_M[2..4] : data output rate */
#define LSM9DS1_DO_0_625_HZ (0x0 << 2)
#define LSM9DS1_DO_1_25_HZ (0x1 << 2)
#define LSM9DS1_DO_2_5_HZ (0x2 << 2)
#define LSM9DS1_DO_5_HZ (0x3 << 2)
#define LSM9DS1_DO_10_HZ (0x4 << 2)
#define LSM9DS1_DO_20_HZ (0x5 << 2)
#define LSM9DS1_DO_40_HZ (0x6 << 2)
#define LSM9DS1_DO_80_HZ (0x7 << 2)

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "util.h"
#include "printf.h"
#include "rlib.h"
#include <stdbool.h>
#include <math.h>

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>
#include <stm32l4xx_ll_usart.h>

#include "main.h"
#include "stm32l4xx_hal.h"

#include "util.h"
#include "rlib.h"
#include "service.h"

#include "mems.h"

#include "geom.h"
#include "autopilot.h"
#include "imu.h"

#if LSM9DS1_FS_XL == LSM9DS1_FS_XL_2G
#define LSM9DS1_CVT_MKS_FACTOR_ACC (MEMS_STANDARD_GRAVITY * (2.F / 32768.F))
#endif
#if LSM9DS1_FS_XL == LSM9DS1_FS_XL_4G
#define LSM9DS1_CVT_MKS_FACTOR_ACC (MEMS_STANDARD_GRAVITY * (4.F / 32768.F))
#endif
#if LSM9DS1_FS_XL == LSM9DS1_FS_XL_8G
#define LSM9DS1_CVT_MKS_FACTOR_ACC (MEMS_STANDARD_GRAVITY * (8.F / 32768.F))
#endif
#if LSM9DS1_FS_XL == LSM9DS1_FS_XL_16G
/* Bizzarerie du LSM9DS1 avec la pleine échelle à 16 G */
/* Il faut multiplier en plus par 1.5. Il vaut mieux éviter ce calibre */
#warning "Avoid 16G full scale"
#define LSM9DS1_CVT_MKS_FACTOR_ACC (MEMS_STANDARD_GRAVITY * (16.F / 32768.F) * 1.5F)
#endif

#if LSM9DS1_FS_G == LSM9DS1_FS_G_245_DPS
#define LSM9DS1_CVT_MKS_FACTOR_GYR ((M_PI / 180.) * 245.F / 32768.F)
#endif
#if LSM9DS1_FS_G == LSM9DS1_FS_G_500_DPS
#define LSM9DS1_CVT_MKS_FACTOR_GYR ((M_PI / 180.) * 500.F / 32768.F)
#endif
#if LSM9DS1_FS_G == LSM9DS1_FS_G_2000_DPS
#define LSM9DS1_CVT_MKS_FACTOR_GYR ((M_PI / 180.) * 2000.F / 32768.F)
#endif

#if LSM9DS1_FS_M == LSM9DS1_FS_M_4_GAUSS
#define LSM9DS1_CVT_MKS_FACTOR_MAG (MEMS_STANDARD_MAGNETIC_GAUSS * (4.F / 32768.F))
#endif
#if LSM9DS1_FS_M == LSM9DS1_FS_M_8_GAUSS
#define LSM9DS1_CVT_MKS_FACTOR_MAG (MEMS_STANDARD_MAGNETIC_GAUSS * (8.F / 32768.F))
#endif
#if LSM9DS1_FS_M == LSM9DS1_FS_M_12_GAUSS
#define LSM9DS1_CVT_MKS_FACTOR_MAG (MEMS_STANDARD_MAGNETIC_GAUSS * (12.F / 32768.F))
#endif
#if LSM9DS1_FS_M == LSM9DS1_FS_M_16_GAUSS
#define LSM9DS1_CVT_MKS_FACTOR_MAG (MEMS_STANDARD_MAGNETIC_GAUSS * (16.F / 32768.F))
#endif

/*
 * Offsets and gain values for the connected device
 * those values are meant to be obtained from calibration in the future
 * and stored in flash memory.
 */

#define MEMS_ACC_CORR_OFFSET_X (0.298F)
#define MEMS_ACC_CORR_OFFSET_Y (0.083F)
#define MEMS_ACC_CORR_OFFSET_Z (0.050F)

#define MEMS_ACC_CORR_GAIN_X (0.9943F)
#define MEMS_ACC_CORR_GAIN_Y (0.9966F)
#define MEMS_ACC_CORR_GAIN_Z (0.9792F)

#define MEMS_GYR_OFFSET_X (0.008707F)
#define MEMS_GYR_OFFSET_Y (0.013293F)
#define MEMS_GYR_OFFSET_Z (0.038751F)

#define MEMS_GYR_CORR_GAIN_X (1.0F)
#define MEMS_GYR_CORR_GAIN_Y (1.0F)
#define MEMS_GYR_CORR_GAIN_Z (1.1F)

#define MEMS_MAG_OFFSET_X (0.0668F)
#define MEMS_MAG_OFFSET_Y (0.07925F)
#define MEMS_MAG_OFFSET_Z (0.0885F)

#define MEMS_MAG_GAIN_X (.3675F)
#define MEMS_MAG_GAIN_Y (.3585F)
#define MEMS_MAG_GAIN_Z (.3571F)
//#define MEMS_MAG_GAIN_X (1.F)
//#define MEMS_MAG_GAIN_Y (1.F)
//#define MEMS_MAG_GAIN_Z (1.F)

void timerMEMsCallback(TimerHandle_t xTimer);
void timerPOLLCallback(TimerHandle_t xTimer);

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;

/* The queue of messages of the MEMs task */
QueueHandle_t msgQueueMEMs = (QueueHandle_t)0;

/* Timer : sends MSG periodicaly to initiate computation and send values */
static TimerHandle_t timerMEMs = (TimerHandle_t)0;

/* Timer : sends MSG periodicaly to initiate polling MEMs for new data*/
static TimerHandle_t timerPoll = (TimerHandle_t)0;

/* Semaphore pour SPI lock */
SemaphoreHandle_t semspi2 = (SemaphoreHandle_t)0;

//void MEMS_Correction(Vector3f *acc, Vector3f *gyr, Vector3f *mag)
//{
    //;
//}

/*
 * @brief Correct the accelerometer, gyrometer and magnetometer values
 */
void imu_correct(Vector3f *acc, Vector3f *gyr, Vector3f *mag)
{
    float val;

    /* Correct the accelerometer values */
    acc->x = (acc->x - MEMS_ACC_CORR_OFFSET_X) * MEMS_ACC_CORR_GAIN_X;
    acc->y = (acc->y - MEMS_ACC_CORR_OFFSET_Y) * MEMS_ACC_CORR_GAIN_Y;
    acc->z = (acc->z - MEMS_ACC_CORR_OFFSET_Z) * MEMS_ACC_CORR_GAIN_Z;

    /* Correct the gyrometer values */
    gyr->x -= MEMS_GYR_OFFSET_X;
    gyr->y -= MEMS_GYR_OFFSET_Y;
    gyr->z -= MEMS_GYR_OFFSET_Z;

    /* Correct the magnetometer values */
    mag->x = (mag->x - MEMS_MAG_OFFSET_X) * (1./MEMS_MAG_GAIN_X);
    mag->y = (mag->y - MEMS_MAG_OFFSET_Y) * (1./MEMS_MAG_GAIN_Y);
    mag->z = (mag->z - MEMS_MAG_OFFSET_Z) * (1./MEMS_MAG_GAIN_Z);

    /* Mag has different coordinates than mag and gyro */
    /* invert x and y and change sign of y */
    /* magz unchanged */
    //val = mag->x;
    //mag->x  = mag->y;
    //mag->y = -val;
    mag->x *= -1.F;
}

/**
 * @brief  Fonction d’échange SPI en DMA (full-duplex).
 * @param  txBuffer : pointeur vers données à envoyer
 * @param  rxBuffer : pointeur vers buffer de réception
 * @param  size     : taille en octets
 * @param  timeout  : délai max en ticks FreeRTOS
 * @retval HAL_StatusTypeDef : HAL_OK si succès, sinon code d’erreur HAL
 */
HAL_StatusTypeDef SPI_DMA_Transfer(SPI_HandleTypeDef *hspi, uint8_t *txBuffer, uint8_t *rxBuffer, uint16_t size, TickType_t timeout)
{
    //BaseType_t ret;
    HAL_StatusTypeDef status;

    // ret = xSemaphoreTake(semspi2, timeout);
    xSemaphoreTake(semspi2, (TickType_t)0);

    // Lancer l’échange DMA (non bloquant)
    status = HAL_SPI_TransmitReceive_DMA(hspi, txBuffer, rxBuffer, size);
    if (status != HAL_OK)
    {
        return status;
    }

    // Attente de fin via callback et sémaphore
    if (xSemaphoreTake(semspi2, timeout) != pdTRUE)
    {
        return HAL_TIMEOUT;
    }

    return HAL_OK;
}

/**
 * @brief Callback appelé par la HAL quand DMA terminé
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    //BaseType_t ret;
    //unsigned errCode = HAL_SPI_ERROR_NONE;

    //if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    //{
        //errCode = hspi->ErrorCode;
    //}
    if (hspi->Instance == SPI2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(semspi2, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Callback appelé si erreur SPI
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(semspi2, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief  Écrit une valeur dans un registre du LSM9DS1 (SPI).
 * @param  agmag    : 1 : Acc& gyr, 0 : mag
 * @param  reg      : register address (0x00-0x7F)
 * @param  value    : valeur à écrire
 * @retval HAL_StatusTypeDef
 */
HAL_StatusTypeDef LSM9DS1_WriteRegister(int agmag, uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    uint8_t rx[2]; /* dummy */

    /* bit7=0 : écriture */
    tx[0] = reg & 0x7F;
    tx[1] = value;

    /* CS_A/G low : select device */
    LL_GPIO_ResetOutputPin(GPIOB, (agmag == 1) ? LL_GPIO_PIN_11 : LL_GPIO_PIN_12);

    HAL_StatusTypeDef status = SPI_DMA_Transfer(&hspi2, tx, rx, 2, pdMS_TO_TICKS(10));

    /* CS_A/G high : deselect device */
    LL_GPIO_SetOutputPin(GPIOB, (agmag == 1) ? LL_GPIO_PIN_11 : LL_GPIO_PIN_12);

    return status;
}

int LSM9DS1_ReadRegister(int agmag, uint8_t reg, uint8_t *value)
{

    HAL_StatusTypeDef res;

    uint8_t bufferTx[7] = {reg | 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    uint8_t bufferRx[7] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    /* CS_A/G low : select device */
    LL_GPIO_ResetOutputPin(GPIOB, (agmag == 1) ? LL_GPIO_PIN_11 : LL_GPIO_PIN_12);
    res = SPI_DMA_Transfer(&hspi2, bufferTx, bufferRx, 2, pdMS_TO_TICKS(10));
    if (res == HAL_OK)
    {
        *value = bufferRx[1];
    }
    /* CS_A/G high : deselect device */
    LL_GPIO_SetOutputPin(GPIOB, (agmag == 1) ? LL_GPIO_PIN_11 : LL_GPIO_PIN_12);

    return (int)res;
}

int LSM9DS1_ReadVec(int agmag, Vector3f *vec)
{
    HAL_StatusTypeDef res;
    uint8_t regaddr;
    uint32_t pin;

    switch (agmag)
    {
    case 1: // Acc
        regaddr = 0x28 | 0x80;
        pin = LL_GPIO_PIN_11;
        break;

    case 2: // Gyr
        regaddr = 0x18 | 0x80;
        pin = LL_GPIO_PIN_11;
        break;

    case 3: // Mag
        regaddr = 0x28 | 0x80 | 0x40;
        pin = LL_GPIO_PIN_12;
        break;

    default:
        return -1;
    }

    uint8_t bufferTx[7] = {regaddr, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    uint8_t bufferRx[7] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    LL_GPIO_ResetOutputPin(GPIOB, pin); // CS_A/G or CS_M low : select device
    res = SPI_DMA_Transfer(&hspi2, bufferTx, bufferRx, 7, pdMS_TO_TICKS(100));
    if (res == HAL_OK)
    {
        vec->x = (int16_t)(bufferRx[1] | (bufferRx[2] << 8));
        vec->y = (int16_t)(bufferRx[3] | (bufferRx[4] << 8));
        vec->z = (int16_t)(bufferRx[5] | (bufferRx[6] << 8));
    }
    LL_GPIO_SetOutputPin(GPIOB, pin); // CS_A/G high : deselect device

    return (int)res;
}

int LSM9DS1_ReadAcc(Vector3f *acc)
{
    return LSM9DS1_ReadVec(1, acc);
}

int LSM9DS1_ReadGyr(Vector3f *gyr)
{
    return LSM9DS1_ReadVec(2, gyr);
}

int LSM9DS1_ReadMag(Vector3f *mag)
{
    return LSM9DS1_ReadVec(3, mag);
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
    semspi2 = xSemaphoreCreateBinary();
    xSemaphoreGive(semspi2);

    /* MEMs task queue creation */
    msgQueueMEMs = xQueueCreate(100, sizeof(MEMS_MsgType_t));

    /* create tick timer  (without starting it) */
    timerMEMs = xTimerCreate("MEMs",
                             pdMS_TO_TICKS(MEMS_PERIOD_MS),
                             pdTRUE,    /* Auto reload (repeat indefinitely) */
                             (void *)0, /* Timer ID, not used */
                             timerMEMsCallback);

    /* create tick timer  (without starting it) */
    timerPoll = xTimerCreate("MEMs",
                             pdMS_TO_TICKS(MEMS_PERIOD_POLL),
                             pdTRUE,    /* Auto reload (repeat indefinitely) */
                             (void *)0, /* Timer ID, not used */
                             timerPOLLCallback);

    return 0;
}

/*
 * @brief Sends a tick message to the MEMs task
 * It is called every MEMS_PERIOD_MS milliseconds by the timer timerMEMs.
 * Upon reception of this message the task computes mean values of the sensors,
 * compute the attitude and sends it to the autopilot task.
 * @ param xTimer The timer handle (not used)
 * @ return None
 */
void timerMEMsCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    static MEMS_Msg_t command = {
        .msgType = MEMS_MSG_TICK};

    xQueueSend(msgQueueMEMs, (const void *)&command, (TickType_t)0);

    return;
}

/*
 * @brief Sends a poll message to the MEMs task
 * It is called every MEMS_PERIOD_POLL milliseconds by the timer timerPOLLs.
 * Upon reception of this message the task reads the status registers of the sensors,
 * and reads data if they are available
 * @ param xTimer The timer handle (not used)
 * @ return None
 */
void timerPOLLCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    static MEMS_Msg_t command = {
        .msgType = MEMS_MSG_POLL};

    xQueueSend(msgQueueMEMs, (const void *)&command, (TickType_t)0);

    return;
}

int LSM9DS1_init(void)
{
    int res;
    uint8_t whoami = 0;
    // uint8_t value = 0;

    /* Set CS_A/G and CS_M */
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // CS high
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // CS high

    /* check connection of Devices by reading WHOAMI register */
    whoami = 0x0;
    res = LSM9DS1_ReadRegister(1, 0x0F, &whoami); /* Acc&gyr */
    if (res != HAL_OK || whoami != 0x68)
    {
        return -1;
    }

    whoami = 0x0;
    res = LSM9DS1_ReadRegister(2, 0x0F, &whoami); /* mag */
    if (res != HAL_OK || whoami != 0x3D)
    {
        return -1;
    }

    /* Reset devices */
    /* Sofware reset of acc/gyr meter : set SW_RESET an IF_ADD_INC in CTRL_REG8*/
    /* the IF_ADD_INC needs to be preserved, it increments addresses during reads */
    LSM9DS1_WriteRegister(1, CTRL_REG8, 0x05); /* Reboot memory content */

    /* set SOFT_RESET in CTRL_REG2_M */
    LSM9DS1_WriteRegister(2, CTRL_REG2_M, 0x04); /* Software reset */

    /* wait */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* CTRL_REG1_G  Control register gyrometer */
    /* Set ODR_G output data rate and FS gyrometer full scale */
    res = LSM9DS1_WriteRegister(1, CTRL_REG1_G, LSM9DS1_ODR | LSM9DS1_FS_G);
    if (res != HAL_OK)
    {
        return -1;
    }

    LSM9DS1_WriteRegister(1, CTRL_REG6_XL, (0x2 << 5) | LSM9DS1_FS_XL); /* Accelerometer full scale */
    // LSM9DS1_WriteRegister(1, CTRL_REG6_XL, 0x40);
    LSM9DS1_WriteRegister(1, INT1_CTRL, 0x1); /* Interrupt on INT1 : data ready accelerometer */
    LSM9DS1_WriteRegister(1, INT2_CTRL, 0x2); /* Interrupt on INT2 : data ready gyrometer */

    /* Output data rate, x&y ultra high performance mode, Temperature compensation */
    LSM9DS1_WriteRegister(2, CTRL_REG1_M, (0x1 << 7) | (0x3 << 5) | (0x01 << 1) | LSM9DS1_DO);
    LSM9DS1_WriteRegister(2, CTRL_REG2_M, LSM9DS1_FS_M);          /* Full scale  */
    LSM9DS1_WriteRegister(2, CTRL_REG3_M, (0x1 << 7) | 0x00);     /* I2C disable, Continuous conversion mode */
    LSM9DS1_WriteRegister(2, CTRL_REG4_M, 0x03 << 2);             /* Z axis ultra high performance */
    LSM9DS1_WriteRegister(2, CTRL_REG5_M, 0x1 << 6);              /* Block data update */
    LSM9DS1_WriteRegister(2, INT_CFG_M, (0x1 << 2) | (0x1 << 0)); /* TODO configure to use INT_M */

    return 1;
}

void taskMEMs(void *param)
{
    (void)param;
    MEMS_Msg_t MEMS_Msg;
    BaseType_t ret;
    char message[200];
    Vector3f acc, gyr, mag;
    Vector3f magPrevious;
    float magPreviousNorm = 0.F;
    float magMaxDelta = 0.F;
    bool magok;
    uint8_t value;
    TickType_t tickPast = 0U;
    TickType_t tickCurrent = 0U;

    /* Numbers of samples since start of task, for debugging purposes */
    unsigned accNbTot = 0U;
    unsigned gyrNbTot = 0U;
    unsigned magNbTot = 0U;

    /* Numbers of samples since last imu estimate*/
    unsigned accNb = 0U;
    unsigned gyrNb = 0U;
    unsigned magNb = 0U;

    Vector3f accComp = Vector3f_null;
    Vector3f gyrComp = Vector3f_null;
    Vector3f magComp = Vector3f_null;
    Vector3f accCumul = Vector3f_null;
    Vector3f gyrCumul = Vector3f_null;
    Vector3f magCumul = Vector3f_null;

    IMU_Status_t imu;
    MsgAutoPilot_t msgAutopilot;

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // CS_A/G high
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // CS_M high

    LSM9DS1_init();
    IMU_init_status(&imu);

    xTimerStart(timerMEMs, (TickType_t)0);
    xTimerStart(timerPoll, (TickType_t)0);

    xSemaphoreGive(semspi2);

    tickPast = xTaskGetTickCount();
    LSM9DS1_ReadAcc(&mag);
    magPrevious = mag;
    magPreviousNorm = vector3f_getNorm(mag);

    for (;;)
    {
        ret = xQueueReceive(msgQueueMEMs, &MEMS_Msg, pdMS_TO_TICKS(500));
        if (ret == pdPASS)
        {

            switch (MEMS_Msg.msgType)
            {
            case MEMS_MSG_POLL:

                LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9); /* C9 signal synchro */

                LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11); // CS_A/G low
                LSM9DS1_ReadRegister(1, 0x17, &value);
                LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // CS_A/G high
                int gyrReady = (value & 0x2);                /* Gyroscope new data available */
                int accReady = (value & 0x1);                /* accelerometer new data available */

                LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12); // CS_M low
                LSM9DS1_ReadRegister(2, 0x27, &value);
                LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // CS_M high
                // int magReady = (value & 0x8);                /* accelerometer new data available */
                int magReady = (value & 0xF); /* accelerometer new data available */

                // snprintf(message, sizeof(message), "MEMS tick test %u   %d %d %d\n", xTaskGetTickCount(), accReady, gyrReady, magReady);
                // svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1));

                // value = LL_GPIO_ReadInputPort(GPIOB) & 0x7;
                // snprintf(message, sizeof(message), "MEMS tick ports %x\n", value);
                // svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1));

                if (accReady)
                {
                    LSM9DS1_ReadAcc(&acc);
                    DBG_PRINT_RAW_VALUES_ACC((
                        snprintf(message, sizeof(message), "ACC :  %f %f %f\n", acc.x, acc.y, acc.z),
                        svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));
                    accCumul = vector3f_add(accCumul, acc);
                    accNb++;
                    accNbTot++;
                }

                if (gyrReady)
                {
                    LSM9DS1_ReadGyr(&gyr);
                    DBG_PRINT_RAW_VALUES_GYR((
                        snprintf(message, sizeof(message), "GYR :  %f %f %f\n", gyr.x, gyr.y, gyr.z),
                        svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));
                    gyrCumul = vector3f_add(gyrCumul, gyr);
                    gyrNb++;
                    gyrNbTot++;
                }

                if (magReady)
                {
                    LSM9DS1_ReadMag(&mag);
                    magPreviousNorm = vector3f_getNorm(magPrevious);
                    magMaxDelta = magPreviousNorm * 1.5F;
                    if (((fabs(mag.x - magPrevious.x) < magMaxDelta) && (fabs(mag.y - magPrevious.y) < magMaxDelta) && (fabs(mag.z - magPrevious.z) < magMaxDelta)) || !magok)
                    {
                        magCumul = vector3f_add(magCumul, mag);
                        magNb++;
                        magNbTot++;
                        magok = true;
                        magPrevious = mag;
                    }
                    else
                    {
                        magok = false;
                    }

                    DBG_PRINT_RAW_VALUES_MAG((
                        snprintf(message, sizeof(message), "MAG %c  %f %f %f\n",
                                 magok ? ':' : '!',
                                 mag.x, mag.y, mag.z),
                        svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));
                }

                LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9); // C9 signal synchro

                break;

            case MEMS_MSG_TICK:
                // snprintf(message, sizeof(message), "MEMS tick calcul %u\n", xTaskGetTickCount());
                // svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1));

                if (accNb > 0)
                {
                    accComp = vector3f_getScaled(accCumul, LSM9DS1_CVT_MKS_FACTOR_ACC / ((float)accNb));
                    accCumul = Vector3f_null;
                }
                if (gyrNb > 0)
                {
                    gyrComp = vector3f_getScaled(gyrCumul, LSM9DS1_CVT_MKS_FACTOR_GYR / ((float)gyrNb));
                    gyrCumul = Vector3f_null;
                }
                if (magNb > 0)
                {
                    magComp = vector3f_getScaled(magCumul, LSM9DS1_CVT_MKS_FACTOR_MAG / ((float)magNb));
                    magCumul = Vector3f_null;
                }

                tickCurrent = xTaskGetTickCount();
                DBG_PRINT_MEAN_RAW_VALUES((
                    snprintf(message, sizeof(message),
                             "MEMS raw mean %u %d %d %d %d %+7.3f %+7.3f %+7.3f %+7f %+7f %+7f %+7.4f %+7.4f %+7.4f\n",
                             xTaskGetTickCount(),
                             tickCurrent - tickPast,
                             accNb, gyrNb, magNb,
                             accComp.x, accComp.y, accComp.z,
                             gyrComp.x, gyrComp.y, gyrComp.z,
                             magComp.x, magComp.y, magComp.z),
                    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));

                imu_correct(&accComp, &gyrComp, &magComp);

                DBG_PRINT_MEAN_CORR_VALUES((
                    snprintf(message, sizeof(message),
                             "MEMS corrected mean %u %d %d %d %d %+7.3f %+7.3f %+7.3f %+7f %+7f %+7f %+7.4f %+7.4f %+7.4f\n",
                             tickCurrent,
                             tickCurrent - tickPast,
                             accNb, gyrNb, magNb,
                             accComp.x, accComp.y, accComp.z,
                             gyrComp.x, gyrComp.y, gyrComp.z,
                             magComp.x, magComp.y, magComp.z),
                    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));

                IMU_new_values(&imu, &accComp, &gyrComp, &magComp, ((float)(tickCurrent - tickPast)) / 1000.);
                DBG_PRINT_ATTITUDE((
                    snprintf(message, sizeof(message),
                             "ATTITUDE %+f %+f %+f\n",
                             IMU_get_heading(&imu),
                             IMU_get_roll(&imu),
                             IMU_get_pitch(&imu)),
                    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));

                tickPast = tickCurrent;
                accNb = 0U;
                gyrNb = 0U;
                magNb = 0U;

                msgAutopilot.msgType = AP_MSG_AHRS;
                msgAutopilot.data.IMUData.heading = IMU_get_heading(&imu);
                msgAutopilot.data.IMUData.roll  = IMU_get_roll(&imu);
                msgAutopilot.data.IMUData.pitch = IMU_get_pitch(&imu);
                msgAutopilot.data.IMUData.yawRate = IMU_get_yawRate(&imu);
                xQueueSend(msgQueueAutoPilot, &msgAutopilot, pdMS_TO_TICKS(10));

                break;

            case MEMS_MSG_CALIBRATE: /* Calibrate the sensors */

                /* TODO Calibration */

                break;

            case MEMS_MSG_DISPLAY_CONFIG: /* Display the configuration */

                snprintf(message, sizeof(message),
                         "MEMS : %d  %d\n",
                         MEMS_PERIOD_POLL,
                         MEMS_PERIOD_MS);
                svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1));

                break;

            default:

                break;
            }
        }
        else
        {
            strcpy(message, "MEMS Error receive from queue\n");
            svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(100));
        }
    }
}
