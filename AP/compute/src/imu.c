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
    mstatus->yawRate = 0.F;
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

/*
 * @brief Compute orientation with new MEMs data
 *
 * @param  mstatus, pointer to IMU status
 * @param acc acc vector (m2/s)
 * @param gyr rotation values (rad/s)
 * @param mag magnetic field (unit ignored)
 * @param deltat time since las update (s)
 * The strength of magnetic field is useless, only its direction is usefull.
 * All units are MKS, angles in radians,...
 * Results are accessed by IMU_GET_XXX() functions.
 */

int IMU_new_values_st(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat)
{
    /*-----------------------------------------------------------*/
    /* From Application Note DT0058 from ST Microelectronics     */
    /* Computing tilt measurement and tilt-compensated e-compass */
    /*-----------------------------------------------------------*/
    float phi = atan2f(acc->y, acc->z);
    float Gz2 = acc->y * sinf(phi) + acc->z * cosf(phi);
    float theta = atanf(-acc->x / Gz2);
    float By2 = mag->z * sinf(phi) - mag->y * cosf(phi);
    float Bz2 = mag->y * sinf(phi) + mag->z * cosf(phi);
    float Bx3 = mag->x * cosf(theta) + Bz2 * sinf(theta);
    float psi = atan2f(By2, Bx3);
    /*-----------------------------------------------------------*/

    mstatus->heading = psi;
    mstatus->roll = phi;
    mstatus->pitch = theta;

    return 0;
}

int IMU_new_values(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat)
{

    Vector3f m_h;       /* Horizontal part of magnétic field */
    Vector3f m_v;       /* Vertical part of magnétic field */
    //Vector3f f_v;       /* Vertical part of forward idirection */
    Vector3f starboard_h; /* Horizontal part of forward idirection */
    Vector3f east;      /* East horizontal dir to East */
    Vector3f north;     /* North horizontal direction  to North */
    Vector3f gravity;   /* Gravity */
    Vector3f dir1;
    Vector3f dir2;
    Vector3f dir3;
    float cos_hdg;
    float sin_hdg;

    char message[80]; /* TODO remove after debugging */

    dir1 = vector3f_getCrossProduct(*acc, *mag);
    dir1 = vector3f_getNormalized(dir1);
    dir2 = vector3f_getCrossProduct(*acc, unitmxf);
    dir2 = vector3f_getNormalized(dir2);

    dir3 = vector3f_getCrossProduct(dir1, dir2);

    mstatus->heading = atan2f(vector3f_getNorm(dir3), vector3f_getDotProduct(dir1, dir2));

    // mstatus->heading = acos(vector3f_getDotProduct(dir1, dir2));

#if 0
    snprintf(message, sizeof(message), "IMU dir1  %+f %+f %+f\n", dir1.x, dir1.y, dir1.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif
#if 0
    snprintf(message, sizeof(message), "IMU dir2  %+f %+f %+f\n", dir2.x, dir2.y, dir2.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif
#if 0
    snprintf(message, sizeof(message), "IMU dir3  %+f %+f %+f\n", dir3.x, dir3.y, dir3.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    /* Compute gravity; gravity is a normalized vector downward */
    gravity = vector3f_getNormalized(*acc);

#if 1
    snprintf(message, sizeof(message), "IMU gravity  %+f %+f %+f\n", gravity.x, gravity.y, gravity.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif
    /* horizontal part of forward relative to device */
    starboard_h = vector3f_getCrossProduct(unitmyf, gravity);
    //forward_h = vector3f_sub(unitmxf, f_v);
    starboard_h = vector3f_getNormalized(starboard_h);
#if 1
    snprintf(message, sizeof(message), "IMU stbd horizontal  %+f %+f %+f\n", starboard_h.x, starboard_h.y, starboard_h.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    /* Vertical part of magnetic field */
    /* (mag dot gravity) * gravity  */
    m_v = vector3f_getScaled(gravity, vector3f_getDotProduct(*mag, gravity));
    m_h = vector3f_sub(*mag, m_v);
#if 0
    snprintf(message, sizeof(message), "IMU m horizontal  %+f %+f %+f\n", m_h.x, m_h.y, m_h.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif
#if 0
    snprintf(message, sizeof(message), "IMU m vertical  %+f %+f %+f\n", m_v.x, m_v.y, m_v.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    east = vector3f_getCrossProduct(*mag, gravity);
    east = vector3f_getNormalized(east);

    //north = vector3f_getCrossProduct(gravity, east);
#if 1
    snprintf(message, sizeof(message), "IMU East  %+f %+f %+f\n", east.x, east.y, east.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif
#if 0
    snprintf(message, sizeof(message), "IMU North  %+f %+f %+f\n", north.x, north.y, north.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    cos_hdg = vector3f_getDotProduct(vector3f_getCrossProduct(starboard_h, east), gravity);
    sin_hdg = vector3f_getDotProduct(starboard_h, east);

#if 1
    snprintf(message, sizeof(message), "IMU cos & sin : %+f %+f\n", cos_hdg, sin_hdg);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    mstatus->acc = *acc;
    mstatus->gyr = *gyr;
    mstatus->mag = *mag;

    dir3 = vector3f_getCrossProduct(gravity, unitzf);
    mstatus->roll = -acosf(dir3.x) + M_PI_2F;
    mstatus->pitch = acosf(dir3.y) - M_PI_2F;
    float me = vector3f_getDotProduct(m_h, east);
    float mn = vector3f_getDotProduct(m_h, north);
#if 0
    snprintf(message, sizeof(message), "IMU me & mn : %+f %+f\n", me, mn);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    mstatus->heading = atan2f(cos_hdg, sin_hdg);
    mstatus->yawRate = gyr->z;
    ;
}

int IMU_new_values_old(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat)
{
    Vector3f dir1, dir3;
    float heading_mag;
    float gyrhdg; /* gyr->z over water, ie with roll and pitch compensation */
    float delta_heading_mag;
    static char message[100];
    float accnorm;

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
        /* First call heading unknown */
        /* Let's start with magnetometer value */
        mstatus->heading = M_PI + atan2f(dir3.y, dir3.x);
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

        accnorm = vector3f_getNorm(*acc);
        // float ax2 = acc->x * acc->x;
        float ay2 = acc->y * acc->y;
        float az2 = acc->z * acc->z;
        float cosphi = acc->y / sqrtf(ay2 + az2);
        float sinphi = acc->z / sqrtf(ay2 + az2);
        float costetha = -acc->x / accnorm;
        float sintetha = sqrtf(ay2 + az2) / accnorm;

        // gyrhdg = gyr->z * cosphi * costetha + gyr->x * sinphi * sintetha - gyr->y * cosphi * sinphi;
        gyrhdg = gyr->z * deltat;

        mstatus->heading = mstatus->heading + (1.F - MEMS_MAG_VS_GYRO) * gyrhdg + MEMS_MAG_VS_GYRO * delta_mag;
        // delta_heading_mag = heading_mag - mstatus->heading;
        // mstatus->heading = (deltat * MEMS_MAG_VS_GYRO) * delta_heading_mag + (1 - (deltat * MEMS_MAG_VS_GYRO)) *;
        //  mstatus->heading = heading_mag;
        mstatus->heading = heading_mag;
    }

#if 1
    snprintf(message, sizeof(message) - 1, "IMU Calc %6f  %+8f  %+8f  %+8f\n",
             deltat, mstatus->heading, heading_mag, delta_heading_mag);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    dir1 = vector3f_getNormalized(*acc);
    dir3 = vector3f_getCrossProduct(dir1, unitzf);
    mstatus->roll = -acosf(dir3.x) + M_PI_2F;
    mstatus->pitch = acosf(dir3.y) - M_PI_2F;
    mstatus->yawRate = gyr->z;

    return 1;
}
