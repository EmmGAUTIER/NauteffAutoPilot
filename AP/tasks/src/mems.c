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

#define DBG_PRINT_RAW_VALUES 0
#define DB_PRINT_MEAN_RAW_VALUES 1
#define DB_PRINT_MEAN_COR_VALUES 0

#define LSM9DS1_ODR LSM9DS1_ODR_G_59_5_HZ
#define LSM9DS1_FS_G LSM9DS1_FS_G_245_DPS
#define LSM9DS1_FS_XL LSM9DS1_FS_XL_4G
#define LSM9DS1_DO LSM9DS1_DO_40_HZ

/*
 * MEMs driver
 *
 */
#define LSM9DS1_MAG_I2C_ADDR (0x1E) /* Magnetometer device address */
#define LSM9DS1_XLG_I2C_ADDR (0x6B) /* Accelerometer and gyrometer device address */
#define MEMS_PERIOD_MS 50
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

#include "geom.h"
#include "autopilot.h"
#include "imu.h"

/*
 * Offsets and gain values for the device that is connected
 * those values are meant to be obtained from calibration in the future
 * and stored in flash memory.
 */

#define MEMS_ACC_CORR_OFFSET_X (+150.0F)
#define MEMS_ACC_CORR_OFFSET_Y (-47.0F)
#define MEMS_ACC_CORR_OFFSET_Z (-105.0F)

#define MEMS_ACC_CORR_GAIN_X (0.001200F)
#define MEMS_ACC_CORR_GAIN_Y (0.001204F)
#define MEMS_ACC_CORR_GAIN_Z (0.001174F)

#define MEMS_GYR_CORR_OFFSET_X (-4.F)
#define MEMS_GYR_CORR_OFFSET_Y (-4.F)
#define MEMS_GYR_CORR_OFFSET_Z (-4.F)

#define MEMS_GYR_CORR_GAIN_X (0.0001305F)
#define MEMS_GYR_CORR_GAIN_Y (0.0001305F)
#define MEMS_GYR_CORR_GAIN_Z (0.0001305F)

#define MEMS_MAG_CORR_OFFSET_X (-910.F)
#define MEMS_MAG_CORR_OFFSET_Y (-1615.F)
#define MEMS_MAG_CORR_OFFSET_Z (-1327.F)

#define MEMS_MAG_CORR_GAIN_X (0.34928F)
#define MEMS_MAG_CORR_GAIN_Y (0.36193F)
#define MEMS_MAG_CORR_GAIN_Z (0.36792F)

void timerMEMsCallback(TimerHandle_t xTimer);

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;

/* The queue of messages of the MEMs task */
QueueHandle_t msgQueueMEMs = (QueueHandle_t)0;

/* Timer : sends MSG periodicaly */
static TimerHandle_t timerMEMs = (TimerHandle_t)0;

/* Semaphoe pour SPI lock */
SemaphoreHandle_t semspi2 = (SemaphoreHandle_t)0;

unsigned cpt1 = 0U; // For debugging only
unsigned cpt2 = 0U; // For debugging only

/*
 * @brief Correct the accelerometer, gyrometer and magnetometer values
 */
void imu_correct(Vector3f *acc, Vector3f *gyr, Vector3f *mag)
{
    /* Correct the accelerometer values */
    acc->x = (acc->x - MEMS_ACC_CORR_OFFSET_X) * MEMS_ACC_CORR_GAIN_X;
    acc->y = (acc->y - MEMS_ACC_CORR_OFFSET_Y) * MEMS_ACC_CORR_GAIN_Y;
    acc->z = (acc->z - MEMS_ACC_CORR_OFFSET_Z) * MEMS_ACC_CORR_GAIN_Z;

    /* Correct the gyrometer values */
    gyr->x -= MEMS_GYR_CORR_OFFSET_X;
    gyr->y -= MEMS_GYR_CORR_OFFSET_Y;
    gyr->z -= MEMS_GYR_CORR_OFFSET_Z;

    /* Correct the magnetometer values */
    mag->x = (mag->x - MEMS_MAG_CORR_OFFSET_X) * MEMS_MAG_CORR_GAIN_X;
    mag->y = (mag->y - MEMS_MAG_CORR_OFFSET_Y) * MEMS_MAG_CORR_GAIN_Y;
    mag->z = (mag->z - MEMS_MAG_CORR_OFFSET_Z) * MEMS_MAG_CORR_GAIN_Z;
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
    HAL_StatusTypeDef status;

    xSemaphoreTake(semspi2, timeout);

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

LSM9DS1_ReadRegister(int agmag, uint8_t reg, uint8_t *value)
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
    res = SPI_DMA_Transfer(&hspi2, bufferTx, bufferRx, 7, pdMS_TO_TICKS(10));
    if (res == HAL_OK)
    {
        vec->x = (int16_t)(bufferRx[1] | (bufferRx[2] << 8));
        vec->y = (int16_t)(bufferRx[3] | (bufferRx[4] << 8));
        vec->z = (int16_t)(bufferRx[5] | (bufferRx[6] << 8));
    }
    LL_GPIO_SetOutputPin(GPIOB, pin); // CS_A/G high : deselect device

    return (int)res;
}

#if 0
int LSM9DS1_ReadVec(Vector3f *acc)
{
    HAL_StatusTypeDef res;

    uint8_t bufferTx[7] = {0x28 | 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
    uint8_t bufferRx[7] = {0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};

    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_11); // CS_A/G low : select accelerometer and gyrometer
    res = SPI_DMA_Transfer(&hspi2, bufferTx, bufferRx, 7, pdMS_TO_TICKS(10));
    if (res == HAL_OK)
    {
        acc->x = (int16_t)(bufferRx[1] | (bufferRx[2] << 8));
        acc->y = (int16_t)(bufferRx[3] | (bufferRx[4] << 8));
        acc->z = (int16_t)(bufferRx[5] | (bufferRx[6] << 8));
    }
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // CS_A/G high : deselect device

    return (int)res;
}
#endif

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
    LSM9DS1_WriteRegister(2, CTRL_REG1_M, (0x1 << 7) | (0x3 << 5) | LSM9DS1_DO);
    LSM9DS1_WriteRegister(2, CTRL_REG2_M, 0x00);       /* Full scale +/- 4 gauss */
    LSM9DS1_WriteRegister(2, CTRL_REG3_M, 0x00);       /* Continuous conversion mode */
    LSM9DS1_WriteRegister(2, CTRL_REG4_M, 0x0C);       /* Z axis ultra high performance */
    LSM9DS1_WriteRegister(2, CTRL_REG5_M, (0x1 << 6)); /* Block data update */
    LSM9DS1_WriteRegister(2, INT_CFG_M, 0x05);         /* DRDY_M set when data available so Interrupt configuration useless */

    return 1;
}

void taskMEMs(void *param)
{
    (void)param;
    MEMS_Msg_t MEMS_Msg;
    BaseType_t ret;
    char message[200];
    Vector3f acc, gyr, mag;
    uint8_t value;
    unsigned nbTotAcc = 0U;
    unsigned nbTotGyr = 0U;
    unsigned nbTotMag = 0U;

    int16_t acctx, accty, acctz;
    int16_t gyrtx, gyrty, gyrtz;
    int16_t magtx, magty, magtz;
    int32_t accCumulx, accCumuly, accCumulz;
    int32_t gyrCumulx, gyrCumuly, gyrCumulz;
    int32_t magCumulx, magCumuly, magCumulz;

    unsigned accNb = 0U;
    unsigned gyrNb = 0U;
    unsigned magNb = 0U;

    unsigned accNbId = 0U;
    unsigned gyrNbId = 0U;
    unsigned magNbId = 0U;

    unsigned tickAcc = 0U;
    unsigned tickGyr = 0U;
    unsigned tickMag = 0U;
    unsigned tickNew = 0U;

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // CS_A/G high
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // CS_M high

    LSM9DS1_init();
    /* Interrupts mustn't be enabled before the message queue is created */
    /* so enable them now */
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    xTimerStart(timerMEMs, (TickType_t)0);

    tickAcc = tickGyr = tickMag = xTaskGetTickCount();

    for (;;)
    {
        ret = xQueueReceive(msgQueueMEMs, &MEMS_Msg, pdMS_TO_TICKS(500));
        if (ret == pdPASS)
        {

            switch (MEMS_Msg.msgType)
            {
            case MEMS_MSG_TICK:

                break;

            case MEMS_MSG_ACC_READY: /* Accelerometer data ready */
            case MEMS_MSG_MAG_READY: /* Magnetometer data ready */
                /* set and reset C9 pin for synchronisation */
                LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_9);   // C9 signal synchro
                LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_9); // C9 signal synchro

                tickNew = xTaskGetTickCount();
                LSM9DS1_ReadAcc(&acc);
                nbTotAcc++;
                snprintf(message, sizeof(message),
                         "ACC : %u %u   %f %f %f\n",
                         tickNew - tickAcc, nbTotAcc,
                         acc.x, acc.y, acc.z);
                svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1));
                tickAcc = tickNew;

                LSM9DS1_ReadRegister(2, 0x27, &value);
                if (value & 0x08)
                {
                    tickNew = xTaskGetTickCount();
                    LSM9DS1_ReadMag(&mag);
                    nbTotMag++;
                    snprintf(message, sizeof(message),
                             "MAG : %u %d    %f %f %f\n",
                             tickNew - tickMag, nbTotMag,
                             mag.x, mag.y, mag.z);
                    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1));
                    tickMag = tickNew;
                }

                break;
            case MEMS_MSG_GYR_READY: /* Gyrometer data ready */

                tickNew = xTaskGetTickCount();
                LSM9DS1_ReadGyr(&gyr);
                nbTotGyr++;
                snprintf(message, sizeof(message),
                         "GYR : %u %d    %f %f %f\n",
                         tickNew - tickGyr, nbTotGyr,
                         gyr.x, gyr.y, gyr.z);
                svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1));
                tickGyr = tickNew;

                break;

            case MEMS_MSG_CALIBRATE: /* Calibrate the sensors */
                break;

            case MEMS_MSG_DISPLAY_CONFIG: /* Display the configuration */
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
        // vTaskDelay(pdMS_TO_TICKS(20));
    }
}

#if 0
void taskMEMs(void *param)
{
    (void)param;

    BaseType_t ret;
    int res;
    MEMS_Msg_t msg;
    int16_t vec3i16[3]; /* 3 vectors as uint16_t read from MEMs device */
    static char message[200];
    size_t len;
    unsigned calibration = 0;
    Calibreur_t *calibreur;

    unsigned long compteur = 0UL;

    unsigned accNumber = 0;
    unsigned gyrNumber = 0;
    unsigned magNumber = 0;
    int32_t accCumulx = 0;
    int32_t accCumuly = 0;
    int32_t accCumulz = 0;
    int32_t gyrCumulx = 0;
    int32_t gyrCumuly = 0;
    int32_t gyrCumulz = 0;
    int32_t magCumulx = 0;
    int32_t magCumuly = 0;
    int32_t magCumulz = 0;
    Vector3f accMean = Vector3f_null;
    Vector3f gyrMean = Vector3f_null;
    Vector3f magMean = Vector3f_null;
    Vector3f accCorr = Vector3f_null;
    Vector3f gyrCorr = Vector3f_null;
    Vector3f magCorr = Vector3f_null;

    /* For debugging purpose only, to remove later */
    unsigned accTotal = 0;
    unsigned gyrTotal = 0;
    unsigned magTotal = 0;

    IMU_Status_t memsStatus;

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

    /* Initialize the MEMS status */

    for (;;)
    {

        ret = xQueueReceive(msgQueueMEMs, &msg, pdMS_TO_TICKS(500));

        if (ret == pdPASS)
        {
            switch (msg.msgType)
            {
            case MEMS_MSG_ACC_READY: /* Accelerometer data ready */
                res = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_XL,
                                   (uint8_t *)&vec3i16, 6,
                                   pdMS_TO_TICKS(10));
                if (res > 0)
                {
                    accCumulx += vec3i16[0];
                    accCumuly += vec3i16[1];
                    accCumulz += vec3i16[2];
                    accNumber++;
                    accTotal++;
#if DBG_PRINT_RAW_VALUES == 1
                    len = snprintf(message, sizeof(message),
                                   "ACC %d %d %d\n",
                                   vec3i16[0], vec3i16[1], vec3i16[2]);
                    svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
#endif
                }
                break;

            case MEMS_MSG_GYR_READY: /* Gyrometer data ready */
                res = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_G,
                                   (uint8_t *)&vec3i16, 6,
                                   pdMS_TO_TICKS(10));
                if (res > 0)
                {
                    gyrCumulx += vec3i16[0];
                    gyrCumuly += vec3i16[1];
                    gyrCumulz += vec3i16[2];
                    gyrNumber++;
                    gyrTotal++;
#if DBG_PRINT_RAW_VALUES == 1
                    len = snprintf(message, sizeof(message),
                                   "GYR %d %d %d\n",
                                   vec3i16[0], vec3i16[1], vec3i16[2]);
                    svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
#endif
                }
                break;

            case MEMS_MSG_MAG_READY: /* Magnetometer data ready */
                res = I2C_Mem_Read(LSM9DS1_MAG_I2C_ADDR, OUT_X_L_M,
                                   (uint8_t *)&vec3i16, 6,
                                   pdMS_TO_TICKS(10));
                if (res > 0)
                {
                    magCumulx += vec3i16[0];
                    magCumuly += vec3i16[1];
                    magCumulz += vec3i16[2];
                    magNumber++;
                    magTotal++;
#if DBG_PRINT_RAW_VALUES == 1
                    len = snprintf(message, sizeof(message),
                                   "MAG %d %d %d\n",
                                   vec3i16[0], vec3i16[1], vec3i16[2]);
                    svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
#endif
                }
                break;

            case MEMS_MSG_TICK: /* Compute and send orientation */
                if (accNumber > 0)
                {
                    accMean.x = (float)accCumulx / (float)accNumber;
                    accMean.y = (float)accCumuly / (float)accNumber;
                    accMean.z = (float)accCumulz / (float)accNumber;

                    accCorr.x = (accMean.x + MEMS_ACC_CORR_OFFSET_X) * MEMS_ACC_CORR_GAIN_X;
                    accCorr.y = (accMean.y + MEMS_ACC_CORR_OFFSET_Y) * MEMS_ACC_CORR_GAIN_Y;
                    accCorr.z = (accMean.z + MEMS_ACC_CORR_OFFSET_Z) * MEMS_ACC_CORR_GAIN_Z;

                    // accCorr = vector3f_getScaled(accCorr, MEMS_STANDARD_GRAVITY / 16384.F);
#if 0
                    len = snprintf(message, sizeof(message) - 1,
                                   "ACC  %d %.2f %.2f %.2f    %.2f\n",
                                   accNumber,
                                   accCorr.x, accCorr.y, accCorr.z, vector3f_getNorm(accCorr));
                    // svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
#endif
                }

                if (gyrNumber > 0)
                {
                    gyrMean.x = (float)gyrCumulx / (float)gyrNumber;
                    gyrMean.y = (float)gyrCumuly / (float)gyrNumber;
                    gyrMean.z = (float)gyrCumulz / (float)gyrNumber;

                    gyrCorr.x = (gyrMean.x += MEMS_GYR_CORR_OFFSET_X) * MEMS_GYR_CORR_GAIN_X;
                    gyrCorr.x = (gyrMean.x += MEMS_GYR_CORR_OFFSET_X) * MEMS_GYR_CORR_GAIN_Y;
                    gyrCorr.x = (gyrMean.x += MEMS_GYR_CORR_OFFSET_X) * MEMS_GYR_CORR_GAIN_Z;

                    // gyrCorr = vector3f_getScaled(gyrCorr, (245.F / 16384.F) * (M_PI / 180.F));
#if 0
                    len = snprintf(message, sizeof(message) - 1,
                                   "GYR  %d %.1f %.1f %.1f    %.1f\n",
                                   gyrNumber,
                                   gyrCorr.x, gyrCorr.y, gyrCorr.z, vector3f_getNorm(gyrCorr));
                    // svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
#endif
                }
                if (magNumber > 0)
                {
                    magMean.x = (float)magCumulx / (float)magNumber;
                    magMean.y = (float)magCumuly / (float)magNumber;
                    magMean.z = (float)magCumulz / (float)magNumber;

                    magCorr.x = (magMean.x + MEMS_MAG_CORR_OFFSET_X) * MEMS_MAG_CORR_GAIN_X;
                    magCorr.y = (magMean.y + MEMS_MAG_CORR_OFFSET_Y) * MEMS_MAG_CORR_GAIN_Y;
                    magCorr.z = (magMean.z + MEMS_MAG_CORR_OFFSET_Z) * MEMS_MAG_CORR_GAIN_Z;

#if 0
                    len = snprintf(message, sizeof(message) - 1,
                                   "MAG  %d %.2f %.2f %.2f    %.2f\n",
                                   magNumber,
                                   magCorr.x, magCorr.y, magCorr.z, vector3f_getNorm(magCorr));
                    // svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
#endif
                }
                if ((accNumber > 0) && (gyrNumber > 0) && (magNumber > 0))
                {
                    TickType_t timeStamp = xTaskGetTickCount();
#if DB_PRINT_MEAN_RAW_VALUES == 1
                    snprintf(message, sizeof(message) - 1,
                             "MEMS mr %u %d %6f %6f %6f %d %f %f %f  %d %f %f %f\n",
                             timeStamp,
                             accNumber, accMean.x, accMean.y, accMean.z,
                             gyrNumber, gyrMean.x, gyrMean.y, gyrMean.z,
                             magNumber, magMean.x, magMean.y, magMean.z);
                    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(0));
                    vTaskDelay(pdMS_TO_TICKS(10));
#endif
#if DB_PRINT_MEAN_COR_VALUES == 1
                    len = snprintf(message, sizeof(message) - 1,
                                   "MEMS mean corrected %u   %+8.2f %+8.2f %+8.2f    %+8.2f %+8.2f %+8.2f     %+8.2f %+8.2f %+8.2f\n",
                                   timeStamp,
                                   accCorr.x, accCorr.y, accCorr.z,
                                   gyrCorr.x, gyrCorr.y, gyrCorr.z,
                                   magCorr.x, magCorr.y, magCorr.z);
                    svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
#endif
                    if (!calibration)
                    {
                        /* autopilot_setMEMs(accMean, gyrMean, magMean); */
                        IMU_new_values(&memsStatus, &accCorr, &gyrCorr, &magCorr, MEMS_PERIOD_S);

                        autopilot_sendValues(timeStamp,
                                             IMU_get_heading(&memsStatus),
                                             IMU_get_roll(&memsStatus),
                                             IMU_get_pitch(&memsStatus),
                                             IMU_get_yawRate(&memsStatus));

                        len = snprintf(message, sizeof(message) - 1,
                                       "ATTITUDE  %+.2f %+.2f %+.2f\n",
                                       IMU_get_heading(&memsStatus),
                                       IMU_get_roll(&memsStatus),
                                       IMU_get_pitch(&memsStatus));
                        svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                    }
                    else
                    {
                        /* Calibrating, put sample in calibreur */
                        calibreur_addSample(calibreur, timeStamp, &accMean, &gyrMean, &magMean);
                        if (calibreur_isFull(calibreur))
                        {
#if 0
                            int res = calibreur_calibrate(calibreur, &accCorr, M, &quality);
                            if (res == 0)
                            {
                                len = snprintf(message, sizeof(message) - 1,
                                               "Calibration done : acc %.2f %.2f %.2f\n",
                                               accCorr.x, accCorr.y, accCorr.z);
                                svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                            }
                            else
                            {
                                len = snprintf(message, sizeof(message) - 1,
                                               "Calibration failed\n");
                                svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                            }
#endif
                            calibreur_destroy(calibreur);
                            calibration = 0; // Stop calibration
                        }
                    }
                }

                else
                {
                    HAL_NVIC_DisableIRQ(EXTI0_IRQn);
                    HAL_NVIC_DisableIRQ(EXTI1_IRQn);
                    HAL_NVIC_DisableIRQ(EXTI2_IRQn);
                    config_MEMs();
                    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
                    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
                    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

                    /*
                     * res1 à 3 servent à la mise au point
                     * bidon ne sert à rien pour le calcul.
                     * Il permet uniquement de placer un point d'arrêt avec le debuggueur
                     * pour lire ret1 à 3.
                     * cptReconfig sert à compter les reconfigurations.
                     */
                    static unsigned cptReconfig = 0U;
                    cptReconfig++;

                    int res1 = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_XL,
                                            (uint8_t *)&vec3i16, 6,
                                            pdMS_TO_TICKS(100));
                    int res2 = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_G,
                                            (uint8_t *)&vec3i16, 6,
                                            pdMS_TO_TICKS(100));
                    int res3 = I2C_Mem_Read(LSM9DS1_XLG_I2C_ADDR, OUT_X_L_M,
                                            (uint8_t *)&vec3i16, 6,
                                            pdMS_TO_TICKS(100));
                    int bidon;

                    /* Et bien sûr, une ligne dans le journal pour voir */
                    snprintf(message, sizeof(message), "MEMS Restart %d\n", cptReconfig);
                    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(0));
                }

                accNumber = 0;
                gyrNumber = 0;
                magNumber = 0;
                accCumulx = 0;
                accCumuly = 0;
                accCumulz = 0;
                gyrCumulx = 0;
                gyrCumuly = 0;
                gyrCumulz = 0;
                magCumulx = 0;
                magCumuly = 0;
                magCumulz = 0;

                break;

            case MEMS_MSG_CALIBRATE:
                len = snprintf(message, sizeof(message),
                               "-----> Calibration demandée\n");
                svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));
                calibration = 1; /* Set calibration mode */
                calibreur = calibreur_create(200);

                /* Allocate memory to store values  */

                break;
            case MEMS_MSG_DISPLAY_CONFIG: /* Display config info */
                                          /* three lines */
                len = snprintf(message, sizeof(message),
                               "MEMS config : acc offsets %8f %8f %8f  gains %8f %8f %8f\n",
                               MEMS_ACC_CORR_OFFSET_X,
                               MEMS_ACC_CORR_OFFSET_Y,
                               MEMS_ACC_CORR_OFFSET_Z,
                               MEMS_ACC_CORR_GAIN_X,
                               MEMS_ACC_CORR_GAIN_Y,
                               MEMS_ACC_CORR_GAIN_Z);
                svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));

                len = snprintf(message, sizeof(message),
                               "MEMS config : gyr offsets %8f %8f %8f  gains %8f %8f %8f\n",
                               MEMS_GYR_CORR_OFFSET_X,
                               MEMS_GYR_CORR_OFFSET_Y,
                               MEMS_GYR_CORR_OFFSET_Z,
                               MEMS_GYR_CORR_GAIN_X,
                               MEMS_GYR_CORR_GAIN_Y,
                               MEMS_GYR_CORR_GAIN_Z);
                svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));

                len = snprintf(message, sizeof(message),
                               "MEMS config : mag offsets %8f %8f %8f  gains %8f %8f %8f\n",
                               MEMS_ACC_CORR_OFFSET_X,
                               MEMS_MAG_CORR_OFFSET_X,
                               MEMS_MAG_CORR_OFFSET_Y,
                               MEMS_MAG_CORR_OFFSET_Z,
                               MEMS_MAG_CORR_GAIN_X,
                               MEMS_MAG_CORR_GAIN_Y,
                               MEMS_MAG_CORR_GAIN_Z);
                svc_UART_Write(&svc_uart2, message, len, pdMS_TO_TICKS(0));

            default:
                /* shouldn't hapen */
            }
        }

        compteur++;
    }
}
#endif
