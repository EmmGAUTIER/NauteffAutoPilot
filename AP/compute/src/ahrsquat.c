
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
#include "ahrsquat.h"

#define MEMS_INIT_MAG_VS_GYRO 0.1F /* Répartition de partie du compas et du gyromètre pour le cap */

const AHRS_Interface_t AHRS_Quat_Interface =
{
    .AHRS_init = AHRS_Simple_init,
    .AHRS_get_roll            = AHRS_get_roll,
    .AHRS_get_pitch           = AHRS_get_pitch,
    .AHRS_get_heading         = AHRS_get_heading,
    .AHRS_get_yawRate         = AHRS_get_yawRate,
    .AHRS_set_mag_vs_gyr_prop = AHRS_set_mag_vs_gyr_prop,
    .AHRS_update              = AHRS_Quat_update
};

/*
 * @brief Compute new orientation whith mems values
 * @parameter mstatus
 *
 * @note z axis of device points downward
 */
int AHRS_Quat_update(AHRS_Status_t *mstatus, Vector3f *acc, Vector3f *gyr,
                     Vector3f *mag, float deltat)
{
    //char message[100]; /* for debugging only */
    Quaternionf q; /* new quaternion */
    Vector3f down; /* -acc, normalized*/
    Vector3f east; /* east, normalized */
    Vector3f north; /* north, normalized */
    Quaternionf q_pred; /* Quaternion computed with last one and gyro values */
    Quaternionf q_ref; /* reference quaternion computed with acc and mag */
    //float prev_heading; /* previous heading */

    // initial step : get previous heading
    //prev_heading = mstatus->heading;

    // Step 1: Predict new orientation using gyroscope data
    Quaternionf gyro_quat_derivative;
    gyro_quat_derivative.w = 0.0f;
    gyro_quat_derivative.x = -gyr->x;
    gyro_quat_derivative.y = +gyr->y;
    gyro_quat_derivative.z = -gyr->z;

    Quaternionf q_dot = Quaternionf_mul(mstatus->orientation,
                                        gyro_quat_derivative);
    q_dot = Quaternionf_getScaled(q_dot, 0.5f);

    q_pred = Quaternionf_add(mstatus->orientation,
                             Quaternionf_getScaled(q_dot, deltat));
    q_pred = Quaternionf_getNormalized(q_pred);

    // Step 2: Compute the reference orientation using accelerometer and magnetometer data
    //down = vector3f_getScaled(*acc, -1.0f); // Invert to get gravity direction
    down = *acc;
    down = vector3f_getNormalized(down);

    /* Compute Est. Use cross product of down and mag */
    east = vector3f_getCrossProduct(*acc, *mag);
    east = vector3f_getNormalized(east);

    /* Compute North as cross product of down and east */
    north = vector3f_getCrossProduct(east, down);
    north = vector3f_getNormalized(north);

    /* we have North, East and down, let's compute the quaternion of orientation */
    q_ref = calculer_orientation_navire(north, east, down);

    if(mstatus->initialized == false)
    {
        mstatus->orientation = q_ref;
    }
    else
    {
        Quaternionf q1;
        Quaternionf q2;
        q1 = Quaternionf_getScaled(q_ref, mstatus->magVsGyr);
        q2 = Quaternionf_getScaled(q_pred, (1 - mstatus->magVsGyr));
        q = Quaternionf_add(q1, q2);
        mstatus->orientation = Quaternionf_getNormalized(q);
    }

    float roll, pitch, yaw;
    quaternion_vers_euler(q, &roll, &pitch, &yaw);

    if(mstatus->initialized == false)
    {
        mstatus->yawRate = 0.;
    }
    else
    {
        /* Pré-calculs */
        float q1q1 = q.x * q.x;
        float q2q2 = q.y * q.y;
        float q3q3 = q.z * q.z;

        /* Roll rate, around x axis of surface */
        mstatus->rollRate = gyr->x * (1.0f - 2.0f * (q2q2 + q3q3))
                            + gyr->y * (2.0f * (q.x * q.y + q.w * q.z))
                            + gyr->z * (2.0f * (q.x * q.z - q.w * q.y));

        /* Pitch rate, around y axis of surface */
        mstatus->pitchRate = gyr->x * (2.0f * (q.x * q.y - q.w * q.z))
                             + gyr->y * (1.0f - 2.0f * (q1q1 + q3q3))
                             + gyr->z * (2.0f * (q.y * q.z + q.w * q.x));

        /* Yaw rate, along z axis of surface */
        mstatus->yawRate = gyr->x * (2.0f * (q.x * q.z + q.w * q.y))
                           + gyr->y * (2.0f * (q.y * q.z - q.w * q.x))
                           + gyr->z * (1.0f - 2.0f * (q1q1 + q2q2));

    }

    mstatus->roll = -roll;
    mstatus->pitch = pitch;
    mstatus->heading = -yaw;

    mstatus->initialized = true;

#if 0
    snprintf(message, sizeof(message), "IMU east %+f %+f %+f\n", east.x, east.y, east.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    snprintf(message, sizeof(message), "IMU north %+f %+f %+f\n", north.x, north.y, north.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    snprintf(message, sizeof(message), "IMU  down %+f %+f %+f\n", down.x, down.y, down.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    snprintf(message, sizeof(message), "IMU ref quat %+f %+f %+f %+f\n", q_ref.w, q_ref.x, q_ref.y, q_ref.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    snprintf(message, sizeof(message), "IMU angles %+f %+f %+f\n", roll, pitch, yaw);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    return 0;
}