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
 *  MEMs driver
 * TODO : improve dialog
 *
 */
#include "string.h"
#include <math.h>

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "mems.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "printf.h"
#include <stdbool.h>

#include <stm32l452xx.h>
#include <stm32l4xx_ll_gpio.h>

#include "util.h"
//#include "gpio.h"
//#include "exti.h"
//#include "nvic.h"
//#include "i2c.h"
#include "aux_usart.h"
#include "geom.h"
#include "autopilot.h"
#include "madgwick.h"

#define MEMS_PERIOD_SEND 200
#define MEMS_FREQUENCY_SEND (configTICK_RATE_HZ / MEMS_PERIOD_SEND)

#define MEMS_MAG_VS_GYRO 0.5F

QueueHandle_t msgQueueMEMs;
TimerHandle_t timerMEMSSend;

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

typedef struct
{
    Vector3f acc;
    Vector3f gyr;
    Vector3f mag;
    float roll, pitch, heading;
    bool initialized;
} MEMS_Status_t;

MEMS_Status_t memsStatus;

void MEMS_init_status(MEMS_Status_t *mstatus)
{
    mstatus->acc = unitmzf;
    mstatus->gyr = Vector3f_null;
    mstatus->mag = Vector3f_null;
    mstatus->roll = NAN;
    mstatus->pitch = NAN;
    mstatus->heading = NAN;
    mstatus->initialized = false;
}

float MEMS_get_roll(MEMS_Status_t *mstatus)
{
    return mstatus->roll;
}

float MEMS_get_pitch(MEMS_Status_t *mstatus)
{
    return mstatus->pitch;
}

float MEMS_get_heading(MEMS_Status_t *mstatus)
{
    return mstatus->heading;
}

float MEMS_get_yawRate(MEMS_Status_t *mstatus)
{
    return mstatus->gyr.z;
}

int MEMS_new_values(MEMS_Status_t *mstatus, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat)
{
    Vector3f dir1, dir3;
    float heading_mag;
    float delta_heading_mag;
    static char message[200];
    int nbcar;

    mstatus->acc = *acc;
    mstatus->gyr = *gyr;
    mstatus->mag = *mag;

    dir1 = vector3f_getCrossProduct(*mag, *acc);
    dir3 = vector3f_getCrossProduct(dir1, unitmzf);

    nbcar = snprintf(message, sizeof(message) - 1, "MEMS magacc %+6f %+6f %+6f   %+6f %+6f %+6f    %+6f %+6f %+6f\n",
                     acc->x, acc->y, acc->z, mag->x, mag->y, mag->z, dir1.x, dir1.y, dir1.z);
    aux_USART_write(aux_usart1, message, nbcar, 0U);

    if ((!mstatus->initialized) || isnan(mstatus->heading))
    {
        mstatus->heading = M_PI + atan2f(dir1.y, dir1.x);
        mstatus->initialized = true;
    }
    else
    {
        if (isnan(gyr->z))
        {
            gyr->z = 0.F;
        }
        heading_mag = M_PI + atan2f(dir3.x, dir3.y);
        float delta_mag = heading_mag - mstatus->heading;
        // float delta_gyr = deltat * gyr->z;
        mstatus->heading = mstatus->heading + (1.F - MEMS_MAG_VS_GYRO) * deltat * gyr->z + MEMS_MAG_VS_GYRO * delta_mag;
        delta_heading_mag = heading_mag - mstatus->heading;
        // mstatus->heading = (deltat * MEMS_MAG_VS_GYRO) * delta_heading_mag + (1 - (deltat * MEMS_MAG_VS_GYRO)) *;
    }

    nbcar = snprintf(message, sizeof(message) - 1, "MEMS Calc %6f  %+8f  %+8f  %+8f\n",
                     deltat, mstatus->heading, heading_mag, delta_heading_mag);
    aux_USART_write(aux_usart1, message, nbcar, 0U);

    dir1 = vector3f_getNormalized(*acc);
    dir3 = vector3f_getCrossProduct(dir1, unitzf);
    mstatus->roll = -acosf(dir3.x) + M_PI_2F;
    mstatus->pitch = acosf(dir3.y) - M_PI_2F;

    return 1;
}

#if 0
I2C_Report_t I2C_readRegs(I2C_Handle_t *i2c, uint8_t addr, uint8_t reg, uint8_t *data, unsigned size, unsigned delay)
{
    uint8_t dmd_acc_code[] = {reg};
    I2C_Transfer_t dmd_acc[] = {
        {
            .data = dmd_acc_code,
            .dir = I2C_DIR_SEND,
            .size = 1,
        },
        {
            .data = data,
            .dir = I2C_DIR_RECEIVE,
            .size = size + 1,
        }};

    return I2C_transfer(i2c, addr, dmd_acc, 2, delay);
}

I2C_Report_t I2C_WriteRegs(I2C_Handle_t *i2c, uint8_t addr, uint8_t reg, uint8_t *data, unsigned size, unsigned delay)
{
    uint8_t dmd_acc_code[1];
    dmd_acc_code[0] = reg;

    I2C_Transfer_t dmd_acc[] = {
        {
            .data = dmd_acc_code,
            .dir = I2C_DIR_SEND,
            .size = 1,
        },
        {
            .data = data,
            .dir = I2C_DIR_SEND,
            .size = size,
        }};
    dmd_acc[0].data = dmd_acc_code;
    dmd_acc[0].dir = I2C_DIR_SEND;
    dmd_acc[0].size = 1;
    dmd_acc[1].data = data;
    dmd_acc[1].dir = I2C_DIR_SEND;
    dmd_acc[1].size = size;

    return I2C_transfer(i2c, addr, dmd_acc, 2, delay);
}

I2C_Report_t I2C_Write(I2C_Handle_t *i2c, uint8_t addr, uint8_t *data, unsigned size, unsigned delay)
{
    I2C_Transfer_t transfer = {
        .data = data,
        .dir = I2C_DIR_SEND,
        .size = size,
    };

    return I2C_transfer(i2c, addr, &transfer, 1, delay);
}

I2C_Report_t MEMS_readVector3x(I2C_Handle_t *i2c, uint8_t addr, uint8_t base, uint8_t *status, int16_t *vector3x, unsigned delay)
{
    int8_t dataint[8];

    I2C_Report_t report = I2C_readRegs(i2c, addr, base, (uint8_t *)dataint, 7, delay);

    if (report.ok)
    {
        *status = dataint[0];

        vector3x[0] = dataint[1] << 8 | dataint[2];
        vector3x[1] = dataint[3] << 8 | dataint[4];
        vector3x[2] = dataint[5] << 8 | dataint[6];
    }
    return report;
}

I2C_Report_t MEMS_readVector3f(I2C_Handle_t *i2c, uint8_t addr, uint8_t base, uint8_t *status, Vector3f *vector, unsigned delay)
{
    int8_t dataint[8];

    I2C_Report_t report = I2C_readRegs(i2c, addr, base, (uint8_t *)dataint, 7, delay);

    if (report.ok)
    {
        *status = dataint[0];

        int16_t ints[3];
        memcpy(ints, &dataint[1], sizeof(int16_t) * 3);
        vector->x = (float)((ints[0]));
        vector->y = (float)((ints[1]));
        vector->z = (float)((ints[2]));
    }
    return report;
}
#endif

int init_taskMEMs(void)
{
    msgQueueMEMs = xQueueCreate(10, sizeof(MEMS_Msg_t));
    if (msgQueueMEMs == (QueueHandle_t)0)
    {
        return -1;
    }
    // Create a timer to send MEMS data periodically
    timerMEMSSend = xTimerCreate("MEMST1", pdMS_TO_TICKS(MEMS_PERIOD_SEND),
                                 pdTRUE, (void *)0, MEMSSendTimerCallback);
    if (timerMEMSSend == NULL)
    {
        return -1; // Timer creation failed
    }
    return 0;
}

void MEMSSendTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    static MEMS_Msg_t msg;
    msg.msgType = MEMS_MSG_SEND;
    msg.number = 0; // For debug purpose

    BaseType_t res = xQueueSend(msgQueueMEMs, &msg, (TickType_t)0);
    if (res != pdPASS)
    {
        aux_USART_write(aux_usart1, "MEMS Send Timer Callback: Queue send failed\n", 44, 0U);
    }
}

void taskMEMs(void *pvParameters)
{
    (void)pvParameters;

    static char message[150];
    int nbcar;
    unsigned nbTours = 0U;
    MEMS_Msg_t msgMEMs;
    // MsgDialog_t msgDialog;
    Vector3f vector, rvector;
    uint8_t status = 0U;

    Vector3f acc = {0.0f, 0.0f, 0.0f};
    Vector3f gyr = {0.0f, 0.0f, 0.0f};
    Vector3f mag = {0.0f, 0.0f, 0.0f};
    // Quaternionf quat;
    int nb_acc = 0;
    int nb_gyr = 0;
    int nb_mag = 0;

    uint8_t data2[10];
    int accrdy = 0;
    //I2C_Report_t report;

    MEMS_Status_t mstatus;
    BaseType_t timestamp;
    BaseType_t timestamp_2;
    float deltat;
    MsgAutoPilot_t msgAutoPilot;

    aux_USART_write(aux_usart1, "\n\n", 2, 0U);

    MEMS_init_status(&mstatus);

    /* Activation des interruptions */
    /* GPIOB2 : Magnetic data ready RDY_M pin of LSM9DS1 */
    EXTI_Select_Edges(2, true, false);
    EXTI_Select_srce_input(2, 1);
    NVIC_SetPriority(NVIC_NUM_IRQ_EXTI2, 0xD);
    NVIC_enable_IRQ(NVIC_NUM_IRQ_EXTI2);
    // EXTI_Intr_enable_line(2, true);

    /* GPIOB0 : Accelerometer data ready INT1 pin of LSM9DS1 */
    EXTI_Select_Edges(0, true, false);
    EXTI_Select_srce_input(0, 1);
    NVIC_SetPriority(NVIC_NUM_IRQ_EXTI0, 0xD);
    NVIC_enable_IRQ(NVIC_NUM_IRQ_EXTI0);
    EXTI_Intr_enable_line(0, true);

    /* GPIOB1 : Gyrometer data ready INT2 pin of LSM9DS1 */
    EXTI_Select_Edges(1, true, false);
    EXTI_Select_srce_input(1, 1);
    NVIC_SetPriority(NVIC_NUM_IRQ_EXTI1, 0xD);
    NVIC_enable_IRQ(NVIC_NUM_IRQ_EXTI1);
    EXTI_Intr_enable_line(1, true);

    xTimerStart(timerMEMSSend, (TickType_t)0);

    /* Initialisation des capteurs MEMs */
    uint8_t cfg_m_1[] = {0x20, 0x54, 0x00, 0x00, 0x08, 0x00};
    // report = I2C_WriteRegs(i2c1, 0x1E, 0x20, cfg_m_1, 6, 100);

    /* Initialisation de l'accéléromètre */
    // uint8_t bidon[] = {0x10, 0x20, 0x05, 0x42};
    uint8_t bidon[] = {0x10, 0x20, 0x00, 0x00};
    // report = I2C_Write(i2c1, 0x6B, bidon, 3 + 2, 100);

    /* Configuration des interruptions du LSM9DS1 */
    uint8_t jerrican[] = {0x0C, 0x01, 0x02};
    // report = I2C_Write(i2c1, 0x6B, jerrican, 2 + 2, 100);

    // report = MEMS_readVector3f(i2c1, 0x1E, 0x27, &status, &vector, 200);
    // report = MEMS_readVector3f(i2c1, 0x6B, 0x17, &status, &vector, 200);
    // report = MEMS_readVector3f(i2c1, 0x6B, 0x27, &status, &vector, 200);

#if 0
    report = I2C_readRegs(i2c1, 0x1E, 0x20, data2, 3, 200);
    nbcar = snprintf(message,
                     sizeof(message),
                     "MEMS Registres 0x20 et suivants %2x %2x %2x %2x %2x %2x\n",
                     (unsigned)data2[0], (unsigned)data2[1], (unsigned)data2[2],
                     (unsigned)data2[3], (unsigned)data2[4], (unsigned)data2[5]);
    aux_USART_write(aux_usart1, message, nbcar, 0U);

#endif

#if 0
    report = I2C_readRegs(i2c1, 0x6B, 0x10, data2, 3, 200);
    nbcar = snprintf(message,
                     sizeof(message),
                     "MEMS Registres 0x10 et suivants %2x %2x %2x %2x %2x %2x\n",
                     (unsigned)data2[0], (unsigned)data2[1], (unsigned)data2[2],
                     (unsigned)data2[3], (unsigned)data2[4], (unsigned)data2[5]);
    aux_USART_write(aux_usart1, message, nbcar, 0U);

    report = I2C_readRegs(i2c1, 0x6B, 0x0C, data2, 3, 200);
    nbcar = snprintf(message,
                     sizeof(message),
                     "MEMS Registres 0x0C et suivants %2x %2x %2x\n",
                     (unsigned)data2[0], (unsigned)data2[1], (unsigned)data2[2]);
    aux_USART_write(aux_usart1, message, nbcar, 0U);
#endif

    timestamp = xTaskGetTickCount();

    I2C_enable(i2c1);

    for (;;)
    {
        // uint8_t addr = 0x6B;
        BaseType_t res;

        res = xQueueReceive(msgQueueMEMs, &msgMEMs, pdMS_TO_TICKS(1000));
        if (res != pdPASS)
        {
            nbcar = snprintf(message, sizeof(message), "MEMs Erreur Réception message\n");
            aux_USART_write(aux_usart1, message, nbcar, 0U);
        }
        else
        {
            nbcar = snprintf(message, sizeof(message), "MEMs message type : %d\n", msgMEMs.msgType);
            // aux_USART_write(aux_usart1, message, nbcar, 0U);

            switch (msgMEMs.msgType)
            {
            case MEMS_MSG_ACC_READY:
                accrdy = GPIO_pin_get(GPIOB, 0);
                // report = MEMS_readVector3f(i2c1, 0x6B, 0x27, &status, &rvector, 200);

                break;

            case MEMS_MSG_GYR_READY:
                accrdy = GPIO_pin_get(GPIOB, 1);
                // report = MEMS_readVector3f(i2c1, 0x6B, 0x17, &status, &rvector, 200);

                // vector = vector3f_getScaled(rvector, (245.0F / 32768.F) * (M_PI / 180.F));

                gyr = vector3f_add(gyr, vector);
                nb_gyr++;

                /* display raw data, without offset and without gain correction */
                nbcar = snprintf(message, sizeof(message),
                                 "MEMS GYR %ld %ld %ld\n",
                                 lrintf(rvector.x), lrintf(rvector.y), lrintf(rvector.z));
                USART_write(usart1, message, nbcar, 0U);

                break;

            case MEMS_MSG_MAG_READY:
                accrdy = GPIO_pin_get(GPIOB, 2);
                // report = MEMS_readVector3f(i2c1, 0x1E, 0x27, &status, &vector, 200);

                // vector = vector3f_getScaled(vector, (4.0F / 32768.F));
#if 0
                if (report.ok)
                {

                    mag = vector3f_add(mag, vector);
                    nb_mag++;

                    /* display raw data, without offset and without gain correction */
                    nbcar = snprintf(message, sizeof(message),
                                     "MEMS MAG %ld %ld %ld\n",
                                     lrintf(vector.x), lrintf(vector.y), lrintf(vector.z));
                    aux_USART_write(aux_usart1, message, nbcar, 0U);
                }
                else
                {
                    reprise_sur_erreur_i2c();
                    nbcar = snprintf(message, sizeof(message), "====>> MEMS MAG reprise suite à erreur\n");
                    aux_USART_write(aux_usart1, message, nbcar, 0U);
                }

#endif
                break;

            case MEMS_MSG_SEND:

                uint8_t reg = 0x20 | 0x80;
                uint8_t buffer[6];

                I2C_Transfer_t seq[] = {
                    {I2C_DIR_SEND, 0x6A, &reg, 1, NULL, 0},
                    {I2C_DIR_RECEIVE, 0x6A, NULL, 0, buffer, 6}};

                if (I2C_transfer_sequence(i2c1, seq, 2, pdMS_TO_TICKS(10)))
                {
                    // lecture OK
                        nbcar = snprintf(message, sizeof(message),
                                         "MEMS %x %x %x \n",
                                         (int)buffer[0],
                                         (int)buffer[1],
                                         (int)buffer[2]);
                        aux_USART_write(usart1, message, nbcar, 0U);
                        nb_mag++;
                }
                else
                {
                    // erreur ou timeout
                }

#if 0
                if (nb_mag == 0)
                {
                    vector.x = 0.F;
                    vector.y = 0.F;
                    vector.z = 0.F;
                    report = MEMS_readVector3f(i2c1, 0x1E, 0x27, &status, &vector, 200);
                    if (report.ok == 1)
                    {
                        mag = vector;
                        nbcar = snprintf(message, sizeof(message),
                                         "MEMS MAG %ld %ld %ld # secours\n",
                                         lrintf(vector.x), lrintf(vector.y), lrintf(vector.z));
                        aux_USART_write(aux_usart1, message, nbcar, 0U);
                        nb_mag++;
                    }
                }
                if (nb_acc == 0 && nb_gyr == 0 && nb_mag == 0)
                {
                    reprise_sur_erreur_i2c();
                    nbcar = snprintf(message, sizeof(message), "MEMS Send reprise suite à absence de données\n");
                    aux_USART_write(aux_usart1, message, nbcar, 0U);
                    break;
                }
                else
                {
                    acc = vector3f_getScaled(acc, 1.0f / nb_acc);
                    gyr = vector3f_getScaled(gyr, 1.0f / nb_gyr);
                    mag = vector3f_getScaled(mag, 1.0f / nb_mag);

                    acc.x += MEMS_ACC_CORR_OFFSET_X;
                    acc.x *= MEMS_ACC_CORR_GAIN_X;
                    acc.y += MEMS_ACC_CORR_OFFSET_Y;
                    acc.y *= MEMS_ACC_CORR_GAIN_Y;
                    acc.z += MEMS_ACC_CORR_OFFSET_Z;
                    acc.z *= MEMS_ACC_CORR_GAIN_Z;

                    mag.x += MEMS_MAG_CORR_OFFSET_X;
                    mag.x *= MEMS_MAG_CORR_GAIN_X;
                    mag.y += MEMS_MAG_CORR_OFFSET_Y;
                    mag.y *= MEMS_MAG_CORR_GAIN_Y;
                    mag.z += MEMS_MAG_CORR_OFFSET_Z;
                    mag.z *= MEMS_MAG_CORR_GAIN_Z;

                    gyr.x += MEMS_GYR_CORR_OFFSET_X;
                    gyr.y += MEMS_GYR_CORR_OFFSET_Y;
                    gyr.z += MEMS_GYR_CORR_OFFSET_Z;

                    acc.x *= -1.F;
                    acc.z *= -1.F;

                    gyr.x *= -1.F;
                    gyr.z *= -1.F;

                    mag.x *= -1.F;
                    mag.y *= -1.F;

                    filter_Update(acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z, mag.x, mag.y, mag.z, (float)MEMS_PERIOD_SEND / (float)configTICK_RATE_HZ);
                    Quaternionf quat = filter_Get_Quaternion();
                    nbcar = snprintf(message, sizeof(message),
                                     "MEMS quat %8f %8f %8f %8f\n",
                                     quat.w, quat.x, quat.y, quat.z);
                    aux_USART_write(aux_usart1, message, nbcar, 0U);

                    // Roll (gîte) autour de x
                    float sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
                    float cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
                    float roll = atan2f(sinr_cosp, cosr_cosp);

                    // Pitch (tangage) autour de y
                    float sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
                    float pitch;
                    if (fabsf(sinp) >= 1.0F)
                        pitch = copysign(M_PI / 2, sinp);
                    else
                    {
                        pitch = asinf(sinp);
                    }

                    // Yaw (cap) autour de z
                    float siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
                    float cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
                    float heading = atan2f(siny_cosp, cosy_cosp);

                    nbcar = snprintf(message, sizeof(message),
                                     "MEMS quatatt %8f %8f %8f\n",
                                     heading, roll, pitch);
                    aux_USART_write(aux_usart1, message, nbcar, 0U);

                    timestamp_2 = xTaskGetTickCount();
                    deltat = ((float)(timestamp_2 - timestamp) / (float)configTICK_RATE_HZ);
                    MEMS_new_values(&mstatus, &acc, &gyr, &mag, (deltat));
                    timestamp = timestamp_2;
                    // MEMS_get_heading(mstatus)

                    msgAutoPilot.msgType = AP_MSG_AHRS;
                    msgAutoPilot.defaultCodes = 0;
                    msgAutoPilot.data.IMUData.heading = MEMS_get_heading(&mstatus);
                    msgAutoPilot.data.IMUData.roll = MEMS_get_roll(&mstatus);
                    msgAutoPilot.data.IMUData.pitch = MEMS_get_pitch(&mstatus);
                    msgAutoPilot.data.IMUData.yawRate = MEMS_get_yawRate(&mstatus);
                    xQueueSend(msgQueueAutoPilot, &msgAutoPilot, pdMS_TO_TICKS(0));
#if 1
                    nbcar = snprintf(message, sizeof(message),
                                     "ATTITUDE %8f %8f %8f\n",
                                     msgAutoPilot.data.IMUData.heading,
                                     msgAutoPilot.data.IMUData.roll,
                                     msgAutoPilot.data.IMUData.pitch);
                    USART_write(usart1, message, nbcar, 0U);
#endif

                    nb_acc = nb_gyr = nb_mag = 0;
                    acc = gyr = mag = (Vector3f){0.0f, 0.0f, 0.0f};
                }
#endif
            default:
                break;
            }
        }

        nbTours++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    (void)accrdy; /* Sometime used for debugging purpose */
}

void EXTI0_Acc_Ready_Handler(void)
{
    static MEMS_Msg_t msgMEMs = {
        .msgType = MEMS_MSG_ACC_READY,
        .number = 0 // For debug purpose
    };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    EXTI_clr_pending_bit(0);
    xQueueSendToBackFromISR(msgQueueMEMs, &msgMEMs, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void EXTI1_Gyr_Ready_Handler(void)
{
    static MEMS_Msg_t msgMEMs = {
        .msgType = MEMS_MSG_GYR_READY,
        .number = 0 // For debug purpose
    };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    EXTI_clr_pending_bit(1);
    xQueueSendToBackFromISR(msgQueueMEMs, &msgMEMs, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void EXTI2_Mag_Ready_Handler(void)
{
    static MEMS_Msg_t msgMEMs = {
        .msgType = MEMS_MSG_MAG_READY,
        .number = 0 // For debug purpose
    };
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    EXTI_clr_pending_bit(2);
    xQueueSendToBackFromISR(msgQueueMEMs, &msgMEMs, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
