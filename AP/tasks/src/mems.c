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

#define DBG_PRINT_RAW_VALUES_ACC(X)
#define DBG_PRINT_RAW_VALUES_GYR(X)
#define DBG_PRINT_RAW_VALUES_MAG(X)
#define DBG_PRINT_MEAN_RAW_VALUES(X)    (X)
#define DBG_PRINT_CALIB(X) (X)
#define DBG_PRINT_MEAN_CORR_VALUES(X)   (X)
#define DBG_PRINT_ATTITUDE(X)           (X)
#define DBG_PRINT_QUATERNION(X)         (X)
#define DBG_PRINT_MADGWICK(X)

#define LSM9DS1_ODR LSM9DS1_ODR_G_59_5_HZ /* gyr and acc ouptut data rate */
#define LSM9DS1_FS_G LSM9DS1_FS_G_500_DPS /* gyro full scale */
#define LSM9DS1_FS_XL LSM9DS1_FS_XL_8G    /* acc full scale */
#define LSM9DS1_DO LSM9DS1_DO_20_HZ       /* Mag output data rate */
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
#define LSM9DS1_FS_G_500_DPS (0x1 << 3) /* 500 deg/s */
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
#include "quat.h"
#include "madgwickAHRS.h"
#include "kalmanAHRS.h"
#include "calib.h"
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
#define LSM9DS1_CVT_MKS_FACTOR_GYR ((M_PI / 180.) * (500.F / 32768.F) * 1.17F)
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

#if 0
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

//#define MEMS_GYR_OFFSET_X (0.008707F)
//#define MEMS_GYR_OFFSET_Y (0.013293F)
//#define MEMS_GYR_OFFSET_Z (0.051001F)

//#define MEMS_GYR_CORR_GAIN_X (1.0F)
//#define MEMS_GYR_CORR_GAIN_Y (1.0F)
//#define MEMS_GYR_CORR_GAIN_Z (1.1F)

#define MEMS_MAG_OFFSET_X (0.0668F)
#define MEMS_MAG_OFFSET_Y (0.07925F)
#define MEMS_MAG_OFFSET_Z (0.0885F)

#define MEMS_MAG_GAIN_X (.3675F)
#define MEMS_MAG_GAIN_Y (.3585F)
#define MEMS_MAG_GAIN_Z (.3571F)
// #define MEMS_MAG_GAIN_X (1.F)
// #define MEMS_MAG_GAIN_Y (1.F)
// #define MEMS_MAG_GAIN_Z (1.F)
#endif

void timerMEMsCallback(TimerHandle_t xTimer);
void timerPOLLCallback(TimerHandle_t xTimer);

//extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi2;

/* The queue of messages of the MEMs task */
QueueHandle_t msgQueueMEMs = (QueueHandle_t) 0;

/* Timer : sends MSG periodicaly to initiate computation and send values */
static TimerHandle_t timerMEMs = (TimerHandle_t) 0;

/* Timer : sends MSG periodicaly to initiate polling MEMs for new data*/
static TimerHandle_t timerPoll = (TimerHandle_t) 0;

/* Semaphore pour SPI lock */
SemaphoreHandle_t semspi2 = (SemaphoreHandle_t) 0;

static Calib6_t calibreur;

static Corrector_t corrector =
        {
                /* Accelerometer correction */
                50.731567F,
                12.749512F,
                -26.425171F,
                -0.002399F,
                -0.002402F,
                0.002381F,

                /* Gyrometer correction */
                -11.573334F,
                -41.593998F,
                -158.863327F,
                //-0.000312F,
                //-0.000312F,
                //0.000312F,
                +0.000305F,
                -0.000305F,
                -0.000305F,

                /* Magnetometer Corrections */
                -467.892975F,
                -812.839966F,
                -679.364014F,
                -0.366352F,
                0.386369F,
                -0.394172F
        };

INLINE Vector3f Vector3f_fromInts(int i[])
{
    Vector3f v =
            { (float) i[0], (float) i[1], (float) i[2] };
    return v;
}

#if 0
/*
 * @brief Correct the accelerometer, gyrometer and magnetometer values
 */
void imu_correct(Vector3f *acc, Vector3f *gyr, Vector3f *mag)
{
    // float val;

    /* Correct the accelerometer values */
    acc->x = -1.0F * (acc->x - MEMS_ACC_CORR_OFFSET_X) * MEMS_ACC_CORR_GAIN_X;
    acc->y = -1.0F * (acc->y - MEMS_ACC_CORR_OFFSET_Y) * MEMS_ACC_CORR_GAIN_Y;
    acc->z = -1.0F * (acc->z - MEMS_ACC_CORR_OFFSET_Z) * MEMS_ACC_CORR_GAIN_Z;

    /* Correct the gyrometer values */
    gyr->x -= MEMS_GYR_OFFSET_X;
    gyr->y -= MEMS_GYR_OFFSET_Y;
    gyr->z -= MEMS_GYR_OFFSET_Z;

    //gyr->x = 1.02f;
    //gyr->y = 1.02f;
    //gyr->z = 1.02f;

    /* Correct the magnetometer values */
    mag->x = (mag->x - MEMS_MAG_OFFSET_X) * (1. / MEMS_MAG_GAIN_X);
    mag->y = (mag->y - MEMS_MAG_OFFSET_Y) * (1. / MEMS_MAG_GAIN_Y);
    mag->z = (mag->z - MEMS_MAG_OFFSET_Z) * (1. / MEMS_MAG_GAIN_Z);

    /* North East Down (NED) convention */
    /* accelerometer and gyro has same coordinates */
    /* z axis has to be inverted */
    acc->z *= -1.F;
    gyr->z *= -1.F;

    /* Mag has different coordinates than mag and gyro */
    /* invert sign of x and z */
    mag->x *= -1.F;
    mag->z *= -1.F;
}
#endif
/**
 * @brief  Fonction d’échange SPI en DMA (full-duplex).
 * @param  txBuffer : pointeur vers données à envoyer
 * @param  rxBuffer : pointeur vers buffer de réception
 * @param  size     : taille en octets
 * @param  timeout  : délai max en ticks FreeRTOS
 * @retval HAL_StatusTypeDef : HAL_OK si succès, sinon code d’erreur HAL
 */
HAL_StatusTypeDef SPI_DMA_Transfer(SPI_HandleTypeDef *hspi, uint8_t *txBuffer,
        uint8_t *rxBuffer, uint16_t size,
        TickType_t timeout)
{
    // BaseType_t ret;
    HAL_StatusTypeDef status;

    // ret = xSemaphoreTake(semspi2, timeout);
    xSemaphoreTake(semspi2, (TickType_t )0);

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
    // BaseType_t ret;
    // unsigned errCode = HAL_SPI_ERROR_NONE;

    // if (hspi->ErrorCode != HAL_SPI_ERROR_NONE)
    //{
    // errCode = hspi->ErrorCode;
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
    LL_GPIO_ResetOutputPin(GPIOB,
            (agmag == 1) ? LL_GPIO_PIN_11 : LL_GPIO_PIN_12);

    HAL_StatusTypeDef status = SPI_DMA_Transfer(&hspi2, tx, rx, 2,
            pdMS_TO_TICKS(10));

    /* CS_A/G high : deselect device */
    LL_GPIO_SetOutputPin(GPIOB, (agmag == 1) ? LL_GPIO_PIN_11 : LL_GPIO_PIN_12);

    return status;
}

int LSM9DS1_ReadRegister(int agmag, uint8_t reg, uint8_t *value)
{

    HAL_StatusTypeDef res;

    uint8_t bufferTx[7] =
            { reg | 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
    uint8_t bufferRx[7] =
            { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

    /* CS_A/G low : select device */
    LL_GPIO_ResetOutputPin(GPIOB,
            (agmag == 1) ? LL_GPIO_PIN_11 : LL_GPIO_PIN_12);
    res = SPI_DMA_Transfer(&hspi2, bufferTx, bufferRx, 2, pdMS_TO_TICKS(10));

    if (res == HAL_OK)
    {
        *value = bufferRx[1];
    }

    /* CS_A/G high : deselect device */
    LL_GPIO_SetOutputPin(GPIOB, (agmag == 1) ? LL_GPIO_PIN_11 : LL_GPIO_PIN_12);

    return (int) res;
}

int LSM9DS1_ReadVec(int agmag, int *values)
{
    /*
     * MEMs are connected to SPI bus. They are selected by setting their CS pin low.
     * Values are in registers and are 16 bits integers.
     * regaddr is the address of the register and pi is the pin connected to CS.
     */
    HAL_StatusTypeDef res;
    int ret;
    uint8_t regaddr;
    uint8_t maskRDY;
    uint32_t pin;

    switch (agmag)
    {
    case 1: // Acc
        regaddr = 0x27 | 0x80;
        pin = LL_GPIO_PIN_11;
        maskRDY = 0x1 << 0;
        break;

    case 2: // Gyr
        regaddr = 0x17 | 0x80;
        pin = LL_GPIO_PIN_11;
        maskRDY = 0x1 << 1;
        break;

    case 3: // Mag
        regaddr = 0x27 | 0x80 | 0x40; /* bit 7 for reading, bit 6 to inc. addr. */
        pin = LL_GPIO_PIN_12;
        /* Read only XDA and YDA, ZDA and ZYXDA ignored, because they are
         * always 0. X, Y and Z are both available when XDA or YDA are set
         */
        maskRDY = 0x3 << 0;
        break;

    default:
        return -1;
    }

    /* DMA transfer on SPI bus is in duplex mode.
     * STM32 sends first the register number and ignore the received character
     * and receives the 6 bytes containing the x,y & z values as 16 bits integers.
     * TX   | regaddr  |      0   |      0 |     0 |     0 |     0 |     0 |     0 |
     * RX   | ignored  |STATUS_REG|  X MSB | X LSB | Y MSB | Y LSB | Z MSB | Z LSB |
     */
    uint8_t bufferTx[8] =
            { regaddr, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
    uint8_t bufferRx[8] =
            { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };

    LL_GPIO_ResetOutputPin(GPIOB, pin); // CS_A/G or CS_M low : select device
    res = SPI_DMA_Transfer(&hspi2, bufferTx, bufferRx, 8, pdMS_TO_TICKS(100));
    LL_GPIO_SetOutputPin(GPIOB, pin); /* CS_A/G high : deselect device */

    if (agmag == 3)
    {
        static int i = 0;
        i++;
    }

    if ((res == HAL_OK) && ((bufferRx[1] & maskRDY) == maskRDY))
    {
        values[0] = (int16_t) (bufferRx[2] | (bufferRx[3] << 8));
        values[1] = (int16_t) (bufferRx[4] | (bufferRx[5] << 8));
        values[2] = (int16_t) (bufferRx[6] | (bufferRx[7] << 8));
        //ret = 1;
        ret = bufferRx[1];
    }
    else
    {
        values[0] = 0;
        values[1] = 0;
        values[2] = 0;
        ret = 0;
    }

    return ret;
}

int LSM9DS1_ReadAcc(int *acc)
{
    return LSM9DS1_ReadVec(1, acc);
}

int LSM9DS1_ReadGyr(int *gyr)
{
    return LSM9DS1_ReadVec(2, gyr);
}

int LSM9DS1_ReadMag(int *mag)
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
    msgQueueMEMs = xQueueCreate(100, sizeof(MEMS_Msg_t));

    /* create tick timer  (without starting it) */
    timerMEMs = xTimerCreate("MEMs",
            pdMS_TO_TICKS(MEMS_PERIOD_MS),
            pdTRUE, /* Auto reload (repeat indefinitely) */
            (void*) 0, /* Timer ID, not used */
            timerMEMsCallback);

    /* create tick timer  (without starting it) */
    timerPoll = xTimerCreate("MEMs",
            pdMS_TO_TICKS(MEMS_PERIOD_POLL),
            pdTRUE, /* Auto reload (repeat indefinitely) */
            (void*) 0, /* Timer ID, not used */
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
    (void) xTimer;

    static MEMS_Msg_t command =
            {
                    .msgType = MEMS_MSG_TICK
            };

    xQueueSend(msgQueueMEMs, (const void* )&command, (TickType_t )0);

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
    (void) xTimer;

    static MEMS_Msg_t command =
            {
                    .msgType = MEMS_MSG_POLL
            };

    xQueueSend(msgQueueMEMs, (const void* )&command, (TickType_t )0);

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

    LSM9DS1_WriteRegister(1, CTRL_REG8, (0x1 << 6) | (0x1 << 2)); /* BDU and IF_ADD_INC */
    LSM9DS1_WriteRegister(1, CTRL_REG6_XL, (0x2 << 5) | LSM9DS1_FS_XL); /* Accelerometer full scale */
    LSM9DS1_WriteRegister(1, CTRL_REG1_G, (0x2 << 5) | 0x1 << 3);
    LSM9DS1_WriteRegister(1, INT1_CTRL, 0x1); /* Interrupt on INT1 : data ready accelerometer */
    LSM9DS1_WriteRegister(1, INT2_CTRL, 0x2); /* Interrupt on INT2 : data ready gyrometer */

    /* Output data rate, x&y ultra high performance mode, Temperature compensation */
    LSM9DS1_WriteRegister(2, CTRL_REG1_M,
            (0x1 << 7) | (LSM9DS1_DO) | (0x03 << 2));
    LSM9DS1_WriteRegister(2, CTRL_REG2_M, LSM9DS1_FS_M); /* Full scale  */
    LSM9DS1_WriteRegister(2, CTRL_REG3_M, (0x1 << 7) | 0x00); /* I2C disable, Continuous conversion mode */
    LSM9DS1_WriteRegister(2, CTRL_REG4_M, 0x03 << 2); /* Z axis ultra high performance */
    LSM9DS1_WriteRegister(2, CTRL_REG5_M, 0x1 << 6); /* Block data update */
    LSM9DS1_WriteRegister(2, INT_CFG_M, (0x1 << 2) | (0x1 << 0)); /* TODO configure to use INT_M */

    return 1;
}

void taskMEMs(void *param)
{
    (void) param;
    MEMS_Msg_t MEMS_Msg;
    BaseType_t ret;
    int retCalib;
    //int retMEMsRead;
    char message[200];
    int intacc[3];
    int intgyr[3];
    int intmag[3];
    Vector3f acc, gyr, mag;
    MEMS_Status_t status = MEMS_Status_Init;

    //Vector3f accPrevious, gyrPrevious, magPrevious;
    //float magPreviousNorm = 0.F;
    //float magMaxDelta = 0.F;
    //bool magok = false;
    //uint8_t value;
    TickType_t tickPast = 0U;
    TickType_t tickCurrent = 0U;
    // TickType_t tickDelta = 0U;

    float deltat = 0.;

    /* Numbers of samples since start of task, for debugging purposes */
    unsigned accNbTot = 0U;
    unsigned gyrNbTot = 0U;
    unsigned magNbTot = 0U;

    /* Numbers of samples since last imu estimate*/
    unsigned accNb = 0U;
    unsigned gyrNb = 0U;
    unsigned magNb = 0U;

    Vector3f accMeanRaw;
    Vector3f gyrMeanRaw;
    Vector3f magMeanRaw;
    Vector3f accComp = Vector3f_null;
    Vector3f gyrComp = Vector3f_null;
    Vector3f magComp = Vector3f_null;
    Vector3f accCumul = Vector3f_null;
    Vector3f gyrCumul = Vector3f_null;
    Vector3f magCumul = Vector3f_null;

    //float roll, pitch, heading;

    AHRSState_t ahrs;
    IMU_Status_t imu;
    MsgAutoPilot_t msgAutopilot;

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_11); // CS_A/G high
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12); // CS_M high

    LSM9DS1_init();
    IMU_init_status(&imu);
    AHRS_init(&ahrs);

    xTimerStart(timerMEMs, (TickType_t )0);
    xTimerStart(timerPoll, (TickType_t )0);

    xSemaphoreGive(semspi2);

    tickPast = xTaskGetTickCount();
    LSM9DS1_ReadAcc(intacc);
    LSM9DS1_ReadGyr(intgyr);
    LSM9DS1_ReadMag(intmag);
    //accPrevious = Vector3f_fromInts(intacc);
    //gyrPrevious = Vector3f_fromInts(intgyr);
    //magPrevious = Vector3f_fromInts(intmag);
    //magPreviousNorm = vector3f_getNorm(magPrevious);
    status = MEMS_Status_Measure;

    //LSM9DS1_WriteRegister(2, CTRL_REG1_M, (0x1 << 7) | (0x02 << 2)); // | (0x01 << 1) );

    for (;;)
    {
        ret = xQueueReceive(msgQueueMEMs, &MEMS_Msg, pdMS_TO_TICKS(500));

        if (ret == pdPASS)
        {

            switch (MEMS_Msg.msgType)
            {
            case MEMS_MSG_POLL:
                int accrdstat = LSM9DS1_ReadAcc(intacc);
                int gyrrdstat = LSM9DS1_ReadGyr(intgyr);
                int magrdstat = LSM9DS1_ReadMag(intmag);

                if (accrdstat > 0)
                {
                    DBG_PRINT_RAW_VALUES_ACC(
                            (snprintf(message, sizeof(message), "ACC : %u  %+6d %+6d %+6d\n", xTaskGetTickCount(), intacc[0], intacc[1], intacc[2]),
                                    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));
                    if (status == MEMS_Status_Calibrate)
                    {
                        retCalib = Calib6_addSample(&calibreur, 0, intacc);
                    }
                    else
                    {

                        acc = Vector3f_fromInts(intacc);
                        accCumul = vector3f_add(accCumul, acc);
                        accNb++;
                        accNbTot++;
                    }
                }

                if (gyrrdstat > 0)
                {
                    DBG_PRINT_RAW_VALUES_GYR(
                            (snprintf(message, sizeof(message), "GYR : %u  %+6d %+6d %+6d\n", xTaskGetTickCount(), intgyr[0], intgyr[1], intgyr[2]),
                                    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));
                    if (status == MEMS_Status_Calibrate)
                    {
                        retCalib = Calib6_addSample(&calibreur, 1, intgyr);
                    }
                    else
                    {
                        gyr = Vector3f_fromInts(intgyr);
                        gyrCumul = vector3f_add(gyrCumul, gyr);
                        gyrNb++;
                        gyrNbTot++;
                    }
                }

                if (magrdstat > 0)
                {
                    DBG_PRINT_RAW_VALUES_MAG(
                            (snprintf(message, sizeof(message), "MAG : %u  %+6d %+6d %+6d\n", xTaskGetTickCount(), intmag[0], intmag[1], intmag[2]),
                                    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));
                    if (status == MEMS_Status_Calibrate)
                    {
                        retCalib = Calib6_addSample(&calibreur, 2, intmag);
                    }
                    else
                    {
                        mag = Vector3f_fromInts(intmag);
                        magCumul = vector3f_add(magCumul, mag);
                        magNb++;
                        magNbTot++;
                    }
                }

                if (status == MEMS_Status_Calibrate)
                {
                    DBG_PRINT_CALIB(
                            (snprintf(message, sizeof(message), "CALIB : stab. %4x %2d  %2d   nb %5d  %2d %5d\n",
                            calibreur.facesDone,
                            calibreur.stabOnFace,
                            calibreur.faceIdx,
                            calibreur.stabNumber,
                            calibreur.accumulating,
                            calibreur.nbSampleAcc),
                            svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));

                    if (Calib6_hasEnoughSamples(&calibreur))
                    {

                        Calib6_ComputeCorrector(&calibreur, &corrector);

                        xTimerStart(timerMEMs, (TickType_t )0);

                        status = MEMS_Status_Measure;
                        /* Stops sending messages for computing AHRS */

                        DBG_PRINT_CALIB(
                                (snprintf(message, sizeof(message),
                                "CALIB ACC : %8f %8f %8f   %8f %8f %8f\n",
                                corrector.accCorrOffsetx,
                                corrector.accCorrOffsety,
                                corrector.accCorrOffsetz,
                                corrector.accCorrGainx,
                                corrector.accCorrGainy,
                                corrector.accCorrGainz),
                                svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));
                        vTaskDelay(pdMS_TO_TICKS(10));
                        DBG_PRINT_CALIB(
                                (snprintf(message, sizeof(message),
                                "CALIB GYR : %8f %8f %8f   %8f %8f %8f\n",
                                corrector.gyrCorrOffsetx,
                                corrector.gyrCorrOffsety,
                                corrector.gyrCorrOffsetz,
                                corrector.gyrCorrGainx,
                                corrector.gyrCorrGainy,
                                corrector.gyrCorrGainz),
                                svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));
                        for (int i = 0; i < 6; i++)
                        {
                            vTaskDelay(pdMS_TO_TICKS(50));
                            snprintf(message, sizeof(message),
                                    "CALIB MAG %d : %8f %8f %8f\n",
                                    i,
                                    calibreur.mag_x[i],
                                    calibreur.mag_y[i],
                                    calibreur.mag_z[i]),
                                    svc_UART_Write(&svc_uart2, message,
                                            strlen(message), pdMS_TO_TICKS(1));
                        }
                        DBG_PRINT_CALIB(
                                (snprintf(message, sizeof(message),
                                "CALIB MAG : %8f %8f %8f   %8f %8f %8f\n",
                                corrector.magCorrOffsetx,
                                corrector.magCorrOffsety,
                                corrector.magCorrOffsetz,
                                corrector.magCorrGainx,
                                corrector.magCorrGainy,
                                corrector.magCorrGainz),
                                svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));
                    }
                }
                (void) retCalib; /* TODO traiter le résultat de retCalib */
                break;

            case MEMS_MSG_TICK:

                if (accNb > 0)
                {
                    accMeanRaw = vector3f_getScaled(accCumul,
                            1.F / ((float) accNb));
                }

                if (gyrNb > 0)
                {
                    gyrMeanRaw = vector3f_getScaled(gyrCumul,
                            1.F / ((float) gyrNb));
                }

                if (magNb > 0)
                {
                    magMeanRaw = vector3f_getScaled(magCumul,
                            1.F / ((float) magNb));
                }

                tickCurrent = xTaskGetTickCount();
                deltat = ((float) (tickCurrent - tickPast)) / 1000.F;

                DBG_PRINT_MEAN_RAW_VALUES(
                        (snprintf(message, sizeof(message),
                        "MEMS raw mean %u %7.4f %d %d %d %+7.3f %+7.3f %+7.3f %+7f %+7f %+7f %+7.4f %+7.4f %+7.4f\n",
                        tickCurrent,
                        deltat,
                        accNb, gyrNb, magNb,
                        accMeanRaw.x, accMeanRaw.y, accMeanRaw.z,
                        gyrMeanRaw.x, gyrMeanRaw.y, gyrMeanRaw.z,
                        magMeanRaw.x, magMeanRaw.y, magMeanRaw.z),
                        svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));

                /* offset and gain corrections */
                accComp = Corrector_getAccCorrected(&corrector, &accMeanRaw);
                gyrComp = Corrector_getGyrCorrected(&corrector, &gyrMeanRaw);
                magComp = Corrector_getMagCorrected(&corrector, &magMeanRaw);

                DBG_PRINT_MEAN_CORR_VALUES(
                        (snprintf(message, sizeof(message),
                        "MEMS corrected mean %u %7.4f %d %d %d %+7.3f %+7.3f %+7.3f %+7f %+7f %+7f %+7.4f %+7.4f %+7.4f\n",
                        tickCurrent,
                        deltat,
                        accNb, gyrNb, magNb,
                        accComp.x, accComp.y, accComp.z,
                        gyrComp.x, gyrComp.y, gyrComp.z,
                        magComp.x, magComp.y, magComp.z),
                        svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));

                IMU_update_quat(&imu, &accComp, &gyrComp, &magComp, deltat);

                DBG_PRINT_ATTITUDE(
                        (snprintf(message, sizeof(message),
                        "ATTITUDE     %+6f %+6f %+6f   %6f\n",
                        IMU_get_roll(&imu),
                        IMU_get_pitch(&imu),
                        IMU_get_heading(&imu),
                        IMU_get_yawRate((&imu))),
                        svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));

                Quaternionf orient;
                orient = IMU_getQuaternion(&imu);
                (void) orient; // Avoid compiler warning if unused when debugging
                DBG_PRINT_QUATERNION(
                        (snprintf(message, sizeof(message),
                        "QUATERNION %+f %+f %+f %+f\n",
                        orient.w, orient.x, orient.y, orient.z),
                        svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1))));

                tickPast = tickCurrent;
                accNb = 0U;
                gyrNb = 0U;
                magNb = 0U;
                accCumul = Vector3f_null;
                gyrCumul = Vector3f_null;
                magCumul = Vector3f_null;

                msgAutopilot.msgType = AP_MSG_AHRS;
                msgAutopilot.data.IMUData.heading = IMU_get_heading(&imu);
                msgAutopilot.data.IMUData.roll = IMU_get_roll(&imu);
                msgAutopilot.data.IMUData.pitch = IMU_get_pitch(&imu);
                msgAutopilot.data.IMUData.yawRate = IMU_get_yawRate(&imu);
                msgAutopilot.data.IMUData.qHeading = orient;
                msgAutopilot.data.IMUData.deltat = deltat;
                xQueueSend(msgQueueAutoPilot, &msgAutopilot, pdMS_TO_TICKS(10));

                break;

            case MEMS_MSG_CALIBRATE:
                /* Calibrate the sensors */

                status = MEMS_Status_Calibrate;
                /* Stops sending messages for computing AHRS */
                xTimerStop(timerMEMs, (TickType_t )0);
                Calib6_init(&calibreur);

                break;

            case MEMS_MSG_SET_MAG_VS_GYR:

                IMU_set_mag_vs_gyr_prop(&imu, MEMS_Msg.data.mag_vs_gyr);

                break;

            case MEMS_MSG_DISPLAY_CONFIG:
                /* Display the configuration */

                snprintf(message, sizeof(message),
                        "MEMS config : poll : %dms  compute : %dms  mag/gyr %f\n",
                        MEMS_PERIOD_POLL,
                        MEMS_PERIOD_MS,
                        imu.magVsGyr);
                svc_UART_Write(&svc_uart2, message, strlen(message),
                        pdMS_TO_TICKS(1));

                break;

            default:

                break;
            }
        }
        else
        {
            strcpy(message, "MEMS Error receive from queue\n");
            svc_UART_Write(&svc_uart2, message, strlen(message),
                    pdMS_TO_TICKS(100));
        }
    }
}
