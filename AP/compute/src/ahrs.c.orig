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

/*****************************************************************************\
 * AHRS interface                                                             *
 *                                                                            *
 *                                                                            *
\*****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "util.h"
#include "ahrs.h"
#include "ahrssimple.h"
#include "ahrsquat.h"
#include "ahrsdt0058.h"

const AHRS_Interface_t *AHRS_Interfaces[] = {
    &AHRS_Simple_Interface,
    &AHRS_Quat_Interface,
    &AHRS_DT0058_Interface
};

void AHRS_init(AHRS_Status_t * mstatus)
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

float AHRS_get_roll(AHRS_Status_t *mstatus)
{
    return -mstatus->roll;
}

float AHRS_get_pitch(AHRS_Status_t *mstatus)
{
    return - mstatus->pitch;
}

float AHRS_get_heading(AHRS_Status_t *mstatus)
{
    return mstatus->heading;
}

float AHRS_get_yawRate(AHRS_Status_t *mstatus)
{
    return mstatus->yawRate;
}

void AHRS_set_mag_vs_gyr_prop(AHRS_Status_t *mstatus, float prop)
{
    mstatus->magVsGyr = prop;

    return;
}

Quaternionf AHRS_get_Quaternion(AHRS_Status_t *mstatus)
{
    return mstatus->orientation;
}

Quaternionf calculer_orientation_navire(Vector3f north, Vector3f east, Vector3f down)
{

    Quaternionf q;

    /* Matrice de rotation (repère navire -> repère monde) */
    /* nord correspond à l'axe x du navire */
    /* east correspond à l'axe y du navire */
    /* down correspond à l'axe z du navire (rappel : z vers le bas) */
    float m00 = north.x, m01 = east.x, m02 = down.x;
    float m10 = north.y, m11 = east.y, m12 = down.y;
    float m20 = north.z, m21 = east.z, m22 = down.z;

    /* Conversion matrice de rotation -> quaternion */
    float trace = m00 + m11 + m22;

    if (trace > 0.0f)
    {
        float s = sqrtf(trace + 1.0f) / 2.0f;
        q.w = s;
        q.x = (m21 - m12) / (4 * s);
        q.y = (m02 - m20) / (4 * s);
        q.z = (m10 - m01) / (4 * s);
    }
    else if ((m00 > m11) && (m00 > m22))
    {
        float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        q.w = (m21 - m12) / s;
        q.x = 0.25f * s;
        q.y = (m01 + m10) / s;
        q.z = (m02 + m20) / s;
    }
    else if (m11 > m22)
    {
        float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        q.w = (m02 - m20) / s;
        q.x = (m01 + m10) / s;
        q.y = 0.25f * s;
        q.z = (m12 + m21) / s;
    }
    else
    {
        float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        q.w = (m10 - m01) / s;
        q.x = (m02 + m20) / s;
        q.y = (m12 + m21) / s;
        q.z = 0.25f * s;
    }

    q = Quaternionf_getNormalized(q);

    return q;
}

void quaternion_vers_euler(Quaternionf q, float *roll, float *pitch, float *yaw)
{
    // AnglesEuler angles;

    // Roulis (rotation autour de x)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    *roll = -atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    *pitch = -2 * atan2(sinp, cosp) - M_PI / 2;

    // Lacet (rotation autour de z) = cap
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    *yaw = atan2f(siny_cosp, cosy_cosp);

    return;
}