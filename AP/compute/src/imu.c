#include <math.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "stm32l4xx_hal.h"

#include "printf.h"
#include "geom.h"
#include "service.h"
#include "imu.h"

int calibreur_addSample(Calibreur_t *calibreur, float currTime, Vector3f *acc, Vector3f *gyr, Vector3f *mag)
{
    if (calibreur->sampleCount >= calibreur->maxSamples)
    {
        return -1; // Calibrator is full, cannot add more samples
    }
    else
    {
        // Add the sample to the calibrator
        calibreur->samples[calibreur->sampleCount].time = currTime;
        calibreur->samples[calibreur->sampleCount].acc = *acc;
        calibreur->samples[calibreur->sampleCount].gyr = *gyr;
        calibreur->samples[calibreur->sampleCount].mag = *mag;
        calibreur->sampleCount++;

        return calibreur->sampleCount; // Return the new sample count
    }
}

Calibreur_t *calibreur_create(int numberOfSamples)
{
    /* Allocate memory for the calibrator */
    Calibreur_t *calibreur = (Calibreur_t *)pvPortMalloc(sizeof(Calibreur_t));
    if (calibreur == NULL)
    {
        return NULL; /* Memory allocation failed */
    }

    calibreur->samples = (MEMsSample_t *)pvPortMalloc(numberOfSamples * sizeof(MEMsSample_t));
    if (calibreur->samples == NULL)
    {
        vPortFree(calibreur);
        return NULL; /* Memory allocation failed */
    }

    calibreur->sampleCount = 0;
    calibreur->maxSamples = numberOfSamples;
    return calibreur;
}

void calibreur_destroy(Calibreur_t *calibreur)
{
    vPortFree(calibreur->samples);
    vPortFree(calibreur);
}

int calibreur_getNumberOfSamples(Calibreur_t *calibreur)
{
    return calibreur->sampleCount;
}

int calibreur_isFull(Calibreur_t *calibreur)
{
    return (calibreur->sampleCount > 0);
}

int calibreur_calibrate(Calibreur_t *calibreur, Vector3f *offset, float M[3][3], float *quality)
{
    if (calibreur->sampleCount < 100)
    {
        return -1; /* Not enough samples */
    }

    *quality = 0.0f;

    return -1; /* Not yet implemented */
}

void IMU_init_status(IMU_Status_t *mstatus)
{
    mstatus->acc = unitmzf;
    mstatus->gyr = Vector3f_null;
    mstatus->mag = Vector3f_null;
    mstatus->roll = NAN;
    mstatus->pitch = NAN;
    mstatus->heading = NAN;
    mstatus->initialized = false;
}

float IMU_get_roll(IMU_Status_t *mstatus)
{
    return mstatus->roll;
}

float IMU_get_pitch(IMU_Status_t *mstatus)
{
    return mstatus->pitch;
}

float IMU_get_heading(IMU_Status_t *mstatus)
{
    return mstatus->heading;
}

float IMU_get_yawRate(IMU_Status_t *mstatus)
{
    return mstatus->gyr.z;
}

int IMU_new_values(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat)
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
#if 0
    nbcar = snprintf(message, sizeof(message) - 1, "MEMS magacc %+6f %+6f %+6f   %+6f %+6f %+6f    %+6f %+6f %+6f\n",
                     acc->x, acc->y, acc->z, mag->x, mag->y, mag->z, dir1.x, dir1.y, dir1.z);
    svc_UART_Write(&svc_uart2, message, nbcar, 0U);
#endif

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
    //svc_UART_Write(&svc_uart2, message, nbcar, 0U);

    dir1 = vector3f_getNormalized(*acc);
    dir3 = vector3f_getCrossProduct(dir1, unitzf);
    mstatus->roll = -acosf(dir3.x) + M_PI_2F;
    mstatus->pitch = acosf(dir3.y) - M_PI_2F;

    return 1;
}
