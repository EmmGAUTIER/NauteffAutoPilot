#include <math.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "stm32l4xx_hal.h"

#include "printf.h"
#include "util.h"
#include "rlib.h"
#include "geom.h"
#include "quat.h"
#include "service.h"
#include "imu.h"

int calibreur_addSample(Calibreur_t *calibreur, float currTime, Vector3f *acc,
        Vector3f *gyr, Vector3f *mag)
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

Calibreur_t* calibreur_create(int numberOfSamples)
{
    /* Allocate memory for the calibrator */
    Calibreur_t *calibreur = (Calibreur_t*) pvPortMalloc(sizeof(Calibreur_t));

    if (calibreur == NULL)
    {
        return NULL; /* Memory allocation failed */
    }

    calibreur->samples = (MEMsSample_t*) pvPortMalloc(
            numberOfSamples * sizeof(MEMsSample_t));

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

int calibreur_calibrate(Calibreur_t *calibreur, Vector3f *offset, float M[3][3],
        float *quality)
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
    mstatus->yawRate = 0.F;
    mstatus->magVsGyr = MEMS_INIT_MAG_VS_GYRO;
    mstatus->orientation = (Quaternionf
            )
            { 1.F, 0.F, 0.F, 0.F };
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
    return - mstatus->yawRate;
}

Quaternionf IMU_getQuaternion(IMU_Status_t *mstatus)
{
    return mstatus->orientation;
}

int IMU_set_mag_vs_gyr_prop(IMU_Status_t *mstatus, float prop)
{
    mstatus->magVsGyr = prop;

    return 0;
}

int IMU_new_values_essai(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr,
        Vector3f *mag, float deltat)
{
    // Vector3f m_h;         /* Horizontal part of magnétic field */
    // Vector3f m_v;         /* Vertical part of magnétic field */
    Vector3f starboard_h; /* Horizontal part of forward idirection */
    Vector3f east; /* East horizontal dir to East */
    // Vector3f north;       /* North horizontal direction  to North */
    Vector3f gravity; /* Gravity */
    Vector3f dir3;
    float cos_hdg;
    float sin_hdg;

    char message[80]; /* TODO remove after debugging */

    /* Compute gravity; gravity is a normalized vector downward */
    gravity = vector3f_getNormalized(*acc);

#if 1
    snprintf(message, sizeof(message), "IMU gravity  %+f %+f %+f\n", gravity.x,
            gravity.y, gravity.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif
    /* horizontal part of forward relative to device */
    starboard_h = vector3f_getCrossProduct(unitmyf, gravity);
    starboard_h = vector3f_getNormalized(starboard_h);
#if 1
    snprintf(message, sizeof(message), "IMU stbd Horizontal  %+f %+f %+f\n",
            starboard_h.x, starboard_h.y, starboard_h.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    east = vector3f_getCrossProduct(*mag, gravity);
    east = vector3f_getNormalized(east);

#if 0
    snprintf(message, sizeof(message), "IMU East  %+f %+f %+f\n", east.x, east.y, east.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    cos_hdg = vector3f_getDotProduct(
            vector3f_getCrossProduct(starboard_h, east), gravity);
    sin_hdg = vector3f_getDotProduct(starboard_h, east);

#if 0
    snprintf(message, sizeof(message), "IMU cos & sin : %+f %+f\n", cos_hdg, sin_hdg);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    mstatus->acc = *acc;
    mstatus->gyr = *gyr;
    mstatus->mag = *mag;

    dir3 = vector3f_getCrossProduct(gravity, unitzf);
    mstatus->roll = -acosf(dir3.x) + M_PI_2F;
    mstatus->pitch = acosf(dir3.y) - M_PI_2F;

    mstatus->heading = atan2f(cos_hdg, sin_hdg);
    mstatus->yawRate = gyr->z;

    return 0;
}

int IMU_new_values(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr,
        Vector3f *mag, float deltat)
{
    Vector3f east, north;
    static char message[160];
    float gyrturn;
    float newdir_x, newdir_y;
    float hdgestim;
    float hdgmag;
    float dirmag_x, dirmag_y;
    float direstim_x, direstim_y;

    mstatus->acc = *acc;
    mstatus->gyr = *gyr;
    mstatus->mag = *mag;

    east = vector3f_getCrossProduct(*acc, *mag);
    north = vector3f_getCrossProduct(unitmzf, east);
    north = vector3f_getNormalized(north);

    hdgmag = -atan2f(north.x, north.y);

#if 1
    snprintf(message, sizeof(message) - 1,
            "IMU mag %+6f %+6f %+6f   acc %+6f %+6f %+6f    east%+6f %+6f %+6f\n",
            acc->x, acc->y, acc->z, mag->x, mag->y, mag->z, east.x, east.y,
            east.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

#if 1
    snprintf(message, sizeof(message) - 1,
            "IMU east %+6f %+6f %+6f   north %+6f %+6f %+6f\n",
            east.x, east.y, east.z, north.x, north.y, north.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    if ((!mstatus->initialized) || isnan(mstatus->heading))
    {
        /* First call heading unknown */
        /* Let's start with magnetometer value */
        mstatus->heading = hdgmag;
        mstatus->initialized = true;
    }
    else
    {
        if (isnan(gyr->z))
        {
            gyr->z = 0.F;
        }

        /* Angle turned since last update : w_z * time since last update */
        gyrturn = -gyr->z * deltat;

        /* Heading estimated */
        hdgestim = mstatus->heading + gyrturn;

        /* The heading is an angle that varies abruptly when crossing x axis */
        /* the difference values may be wrong so we use 2D vectors  */
        /* compute x and y of magnetic direction */
        dirmag_x = cosf(hdgmag);
        dirmag_y = sinf(hdgmag);

        /* compute x and y of estimated direction */
        direstim_x = cosf(hdgestim);
        direstim_y = sinf(hdgestim);

        /* compute the vector beta magnetic value + (1-beta) estimated direction */
        newdir_x = mstatus->magVsGyr * dirmag_x
                + (1.F - mstatus->magVsGyr) * direstim_x;
        newdir_y = mstatus->magVsGyr * dirmag_y
                + (1.F - mstatus->magVsGyr) * direstim_y;
        /* newdir_x,y points to the direction */

#if 0
        snprintf(message, sizeof(message) - 1, "IMU gyr turn  %+6f  north %+7f %+7f dir heading %+7f %+7f\n",
                 gyrturn, north.x, north.y, cosgyr, singyr);
        svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif
#if 0
        snprintf(message, sizeof(message) - 1, "IMU hdgprev  %+6f hdg mag %+6f gyr turn %+6f  direstim %+6f\n",
                 mstatus->heading, hdgmag, gyrturn, hdgestim);
        svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

        /* compute new heading, no need to normalize newdir_x,y for anat2f */
        mstatus->heading = atan2f(newdir_y, newdir_x);
    }

    east = vector3f_getNormalized(*acc);
    north = vector3f_getCrossProduct(east, unitzf);
    mstatus->roll = -acosf(north.x) + M_PI_2F;
    mstatus->pitch = M_PI_2F - acosf(north.y);
    mstatus->yawRate = gyr->z;

    return 1;
}