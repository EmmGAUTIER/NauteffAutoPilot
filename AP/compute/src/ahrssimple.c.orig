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
#include "ahrs.h"
#include "ahrssimple.h"

#define MEMS_INIT_MAG_VS_GYRO 0.1F /* Répartition de partie du compas et du gyromètre pour le cap */

const AHRS_Interface_t AHRS_Simple_Interface = {
    .AHRS_init = AHRS_Simple_init,
    .AHRS_get_roll            = AHRS_Simple_get_roll,
    .AHRS_get_pitch           = AHRS_Simple_get_pitch,
    .AHRS_get_heading         = AHRS_Simple_get_heading,
    .AHRS_get_yawRate         = AHRS_Simple_get_yawRate,
    .AHRS_set_mag_vs_gyr_prop = AHRS_Simple_set_mag_vs_gyr_prop,
    .AHRS_update              = AHRS_Simple_update,
    .AHRS_get_Quaternion      = AHRS_Simple_get_Quaternion
};

void AHRS_Simple_init(AHRS_Status_t * mstatus)
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
    mstatus->orientation = (Quaternionf) { 1.F, 0.F, 0.F, 0.F };
}

float AHRS_Simple_get_roll(AHRS_Status_t *mstatus)
{
    return -mstatus->roll;
}

float AHRS_Simple_get_pitch(AHRS_Status_t *mstatus)
{
    return - mstatus->pitch;
}

float AHRS_Simple_get_heading(AHRS_Status_t *mstatus)
{
    return - M_PI_2F - mstatus->heading;
}

float AHRS_Simple_get_yawRate(AHRS_Status_t *mstatus)
{
    return - mstatus->yawRate;
}

void AHRS_Simple_set_mag_vs_gyr_prop(AHRS_Status_t *mstatus, float prop)
{
    mstatus->magVsGyr = prop;

    return;
}

Quaternionf AHRS_Simple_get_Quaternion(AHRS_Status_t *mstatus)
{
    return mstatus->orientation;
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
int AHRS_Simple_update(AHRS_Status_t *mstatus,
                       Vector3f *acc,
                       Vector3f *gyr,
                       Vector3f *mag,
                       float deltat)
{

    Vector3f east, north;
    //static char message[160];
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

#if 0
    snprintf(message, sizeof(message) - 1,
            "IMU mag %+6f %+6f %+6f   acc %+6f %+6f %+6f    east%+6f %+6f %+6f\n",
            acc->x, acc->y, acc->z, mag->x, mag->y, mag->z, east.x, east.y,
            east.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

#if 0
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