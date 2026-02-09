/*
 MIT License

 Copyright (c) 2026 Emmanuel Gautier / Nauteff

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

#include <stdint.h>
#include <math.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"

#include "util.h"
#include "printf.h"
#include "rlib.h"
#include "service.h"
#include "kalmanAHRS.h"
#include "geom.h"

/* Constantes pour le filtre ASGD+Kalman en NED */
#define AHRS_MU0            (0.01f)   /* Pas initial pour ASGD */
#define AHRS_BETA           (10.0f)   /* Facteur d'adaptation du pas */
#define AHRS_Q_KALMAN       (1e-6f)   /* Bruit de processus pour Kalman */
#define AHRS_R_KALMAN       (5e-5f)   /* Bruit de mesure pour Kalman */

/* Fonctions sécurisées pour éviter les erreurs numériques */
static float safe_asin(const float x)
{
    if (x >= 1.0f)
        return M_PI / 2.0f;
    if (x <= -1.0f)
        return -M_PI / 2.0f;
    return asinf(x);
}

static float safe_atan2(const float y, const float x)
{
    if ((x == 0.0f) && (y == 0.0f))
        return 0.0f;
    return atan2f(y, x);
}

AHRSStatus_t AHRS_init(AHRSState_t *ahrs)
{
    ahrs->quaternion = (Quaternionf
            )
            { 1.0F, 0.F, 0.F, 0.F };
    ahrs->roll = 0;
    ahrs->pitch = 0;
    ahrs->heading = 0;
    ahrs->gravity = (Vector3f
            )
            { 0.F, 0.F, 0.F };
    ahrs->status = AHRS_UNINITIALIZED;
    return AHRS_OK;
}

AHRSStatus_t AHRS_firstUpdate(AHRSState_t *const ahrs,
        const Vector3f *const acc, const Vector3f *const gyr,
        const Vector3f *const mag, const float deltat)
{

    //ahrs->quaternion =
    return AHRS_OK;
}

AHRSStatus_t AHRS_update(AHRSState_t *const ahrs, const Vector3f *const acc,
        const Vector3f *const gyr, const Vector3f *const mag,
        const float deltat)
{

    char message[100];
    Vector3f acc_body; /* normalized acceleration */
    //Vector3f magNorm; /* normalized magnetic field */
    Vector3f east_body;
    Vector3f north_body;

    /* Normalize mag and acc */
    acc_body = vector3f_getNormalized(*acc);
    //magNorm = vector3f_getNormalized(*mag);

    /* East direction is the cross product of acceleration and magnetic field */
    /* It has to be normalized.  */
    east_body = vector3f_getNormalized(
            vector3f_getCrossProduct(acc_body, *mag));

    /* North direction is the cross product of east and acceleration */
    north_body = vector3f_getNormalized(
            vector3f_getCrossProduct(east_body, acc_body));

    Vector3f v1 = north_body;
    //snprintf(message, sizeof message, "Nord haha %+7.3f %+7.3f %+7.3f\n", v1.x, v1.y, v1.z);
    //svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1));

    Quaternionf qm;

    float tr = north_body.x + east_body.y + acc_body.z;

    int cas = 0;

    if (tr > 0.0f)
    {
        float S = sqrtf(tr + 1.0f) * 2.0f;
        qm.w = 0.25f * S;
        qm.x = (east_body.z - acc_body.y) / S;
        qm.y = (acc_body.x - north_body.z) / S;
        qm.z = (north_body.y - east_body.x) / S;
        cas = 1;
    }
    else if (north_body.x > east_body.y && north_body.x > acc_body.z)
    {
        float S = sqrtf(1.0f + north_body.x - east_body.y - acc_body.z) * 2.0f;
        qm.w = (east_body.z - acc_body.y) / S;
        qm.x = 0.25f * S;
        qm.y = (north_body.y + east_body.x) / S;
        qm.z = (north_body.z + acc_body.x) / S;
        cas = 2;
    }
    else if (east_body.y > acc_body.z)
    {
        float S = sqrtf(1.0f + east_body.y - north_body.x - acc_body.z) * 2.0f;
        qm.w = (acc_body.x - north_body.z) / S;
        qm.x = (north_body.y + east_body.x) / S;
        qm.y = 0.25f * S;
        qm.z = (east_body.z + acc_body.y) / S;
        cas = 3;
    }
    else
    {
        float S = sqrtf(1.0f + acc_body.z - north_body.x - east_body.y) * 2.0f;
        qm.w = (north_body.y - east_body.x) / S;
        qm.x = (north_body.z + acc_body.x) / S;
        qm.y = (east_body.z + acc_body.y) / S;
        qm.z = 0.25f * S;
        cas = 4;
    }

    if (ahrs->status == AHRS_UNINITIALIZED)
    {
        ahrs->quaternion = qm;
    }
    else
    {
        Quaternionf qrot =
        { 1.0f, 0.5f * gyr->x, gyr->y, gyr->z };
        Quaternionf qe = Quaternionf_mul(ahrs->quaternion, qrot);
        //qe.w = ahrs->quaternion.w*cw - ahrs->quaternion.x*sx - ahrs->quaternion.y*sy - ahrs->quaternion.z*sz;
        //qe.x = ahrs->quaternion.w*sx + ahrs->quaternion.x*cw + ahrs->quaternion.y*sz - ahrs->quaternion.z*sy;
        //qe.y = ahrs->quaternion.w*sy - ahrs->quaternion.x*sz + ahrs->quaternion.y*cw + ahrs->quaternion.z*sx;
        //qe.z = ahrs->quaternion.w*sz + ahrs->quaternion.x*sy - ahrs->quaternion.y*sx + ahrs->quaternion.z*cw;

        ahrs->quaternion = Quaternionf_add(
                Quaternionf_getScaled(qm, AHRS_ALPHA),
                Quaternionf_getScaled(qrot, (1.0f - AHRS_ALPHA)));
    }

    snprintf(message, sizeof message,
            "quat  %d  %+7.3f  %+7.3f %+7.3f %+7.3f\n", cas, qm.w, qm.x, qm.y, qm.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), pdMS_TO_TICKS(1));

    /* Compute Euler angles : roll pitch and heading */

    /* roll (X, North) */
    float sinr = 2.0f * (qm.w * qm.x + qm.y * qm.z);
    float cosr = 1 - 2.0f * (qm.x * qm.x + qm.y * qm.y);
    ahrs->roll = atan2f(sinr, cosr);

    /* pitch (Y, East) */
    float s = 2.0f * (qm.w * qm.y - qm.z * qm.x);
    if (s > 1.0f)
        s = 1.0f;
    if (s < -1.0f)
        s = -1.0f;
    ahrs->pitch = asinf(s);

    /* yaw (Z, Down) */
    float siny = 2.0f * (qm.w * qm.z + qm.x * qm.y);
    float cosy = 1 - 2.0f * (qm.y * qm.y + qm.z * qm.z);
    ahrs->heading = atan2f(siny, cosy);

    ahrs->status = AHRS_OK;

    return ahrs->status;

}

/* Fonction principale AHRS_update */
AHRSStatus_t AHRS_updateold(AHRSState_t *const ahrs, const Vector3f *const acc,
        const Vector3f *const gyr, const Vector3f *const mag,
        const float deltat)
{

    if (ahrs->status == AHRS_UNINITIALIZED)
    {
        //AHRS_firstUpdate(&ahrs, &acc, &gyr, &mag, deltat);
    }

    /* --- 1. Calcul du pas adaptatif (AHRS_calculate_adaptive_step) --- */
    const float omega = sqrtf(
            gyr->x * gyr->x + gyr->y * gyr->y + gyr->z * gyr->z);
    const float mu = AHRS_MU0 + AHRS_BETA * omega * deltat;

    /* --- 2. Normalisation de l'accélération --- */
    Vector3f accel_normalized = vector3f_getNormalized(*acc);

    /* --- 3. Mise à jour ASGD (Adaptive-Step Gradient Descent) --- */
    {
        /* Vecteur de gravité en NED */
        const Vector3f g_nav =
        { 0.0f, 0.0f, 9.81f };
        /* Rotation de g_nav dans le repère corps */
        Quaternionf g_body_quat =
        { 0.0f, g_nav.x, g_nav.y, g_nav.z };
        Quaternionf q_conj =
        { ahrs->quaternion.w, -ahrs->quaternion.x, -ahrs->quaternion.y,
                -ahrs->quaternion.z };
        Quaternionf temp1 = Quaternionf_mul(ahrs->quaternion, g_body_quat);
        Quaternionf g_body_quat_rotated = Quaternionf_mul(temp1, q_conj);
        Vector3f g_body =
        { g_body_quat_rotated.x, g_body_quat_rotated.y, g_body_quat_rotated.z };

        /* Erreur entre la mesure et la prédiction */
        const Vector3f error =
        { accel_normalized.y * g_body.z - accel_normalized.z * g_body.y,
                accel_normalized.z * g_body.x - accel_normalized.x * g_body.z,
                accel_normalized.x * g_body.y - accel_normalized.y * g_body.x };

        /* Gradient de la fonction d'erreur */
        Quaternionf error_quat =
        { 0.0f, error.x, error.y, error.z };
        Quaternionf delta_q = Quaternionf_mul(ahrs->quaternion, error_quat);

        /* Mise à jour du quaternion */
        Quaternionf q_obs;
        q_obs.w = ahrs->quaternion.w - mu * delta_q.w;
        q_obs.x = ahrs->quaternion.x - mu * delta_q.x;
        q_obs.y = ahrs->quaternion.y - mu * delta_q.y;
        q_obs.z = ahrs->quaternion.z - mu * delta_q.z;
        q_obs = Quaternionf_getNormalized(q_obs);

        /* --- 4. Mise à jour du filtre de Kalman (AHRS_kalman_update) --- */
        {
            /* Prédiction : q_pred = q_prev + 0.5 * q_prev ⊗ ω * deltat */
            Quaternionf q_pred;
            q_pred.w = ahrs->quaternion.w
                    + 0.5f * deltat
                            * (-ahrs->quaternion.x * gyr->x
                                    - ahrs->quaternion.y * gyr->y
                                    - ahrs->quaternion.z * gyr->z);
            q_pred.x = ahrs->quaternion.x
                    + 0.5f * deltat
                            * (ahrs->quaternion.w * gyr->x
                                    + ahrs->quaternion.y * gyr->z
                                    - ahrs->quaternion.z * gyr->y);
            q_pred.y = ahrs->quaternion.y
                    + 0.5f * deltat
                            * (ahrs->quaternion.w * gyr->y
                                    - ahrs->quaternion.x * gyr->z
                                    + ahrs->quaternion.z * gyr->x);
            q_pred.z = ahrs->quaternion.z
                    + 0.5f * deltat
                            * (ahrs->quaternion.w * gyr->z
                                    + ahrs->quaternion.x * gyr->y
                                    - ahrs->quaternion.y * gyr->x);
            q_pred = Quaternionf_getNormalized(q_pred);

            /* Mise à jour : Correction avec le quaternion observé (gain fixe) */
            const float K = 0.5f;
            ahrs->quaternion.w = q_pred.w + K * (q_obs.w - q_pred.w);
            ahrs->quaternion.x = q_pred.x + K * (q_obs.x - q_pred.x);
            ahrs->quaternion.y = q_pred.y + K * (q_obs.y - q_pred.y);
            ahrs->quaternion.z = q_pred.z + K * (q_obs.z - q_pred.z);
            ahrs->quaternion = Quaternionf_getNormalized(ahrs->quaternion);
        }
    }

    /* --- 5. Calcul du vecteur gravité dans le repère corps --- */
    {
        const Vector3f g_nav =
        { 0.0f, 0.0f, 9.81f };
        Quaternionf g_nav_quat =
        { 0.0f, g_nav.x, g_nav.y, g_nav.z };
        Quaternionf q_conj =
        { ahrs->quaternion.w, -ahrs->quaternion.x, -ahrs->quaternion.y,
                -ahrs->quaternion.z };
        Quaternionf temp1 = Quaternionf_mul(ahrs->quaternion, g_nav_quat);
        Quaternionf gravity_quat = Quaternionf_mul(temp1, q_conj);
        ahrs->gravity.x = gravity_quat.x;
        ahrs->gravity.y = gravity_quat.y;
        ahrs->gravity.z = gravity_quat.z;
    }

    /* --- 6. Mise à jour des angles d'Euler (AHRS_update_euler_angles) --- */
    {
        /* Vecteur "down" dans le repère corps (opposé à la gravité) */
        Vector3f down_body =
        { -ahrs->gravity.x, -ahrs->gravity.y, -ahrs->gravity.z };
        down_body = vector3f_getNormalized(down_body);

        /* Vecteurs de référence dans le repère corps */
        const Vector3f right_body =
        { 0.0f, 1.0f, 0.0f }; /* Axes Y (tribord) */
        const Vector3f forward_body =
        { 1.0f, 0.0f, 0.0f }; /* Axes X (avant) */

        /* roll = asin(down · right) */
        ahrs->roll = safe_asin(
                down_body.x * right_body.x + down_body.y * right_body.y
                        + down_body.z * right_body.z);

        /* pitch = asin(down · forward) */
        ahrs->pitch = safe_asin(
                down_body.x * forward_body.x + down_body.y * forward_body.y
                        + down_body.z * forward_body.z);

        /* Calcul du cap (heading) via le magnétomètre */
        {
            Quaternionf mag_quat =
            { 0.0f, mag->x, mag->y, mag->z };
            Quaternionf mag_body_quat = Quaternionf_mul(ahrs->quaternion,
                    mag_quat);
            Quaternionf q_conj =
            { ahrs->quaternion.w, -ahrs->quaternion.x, -ahrs->quaternion.y,
                    -ahrs->quaternion.z };
            Quaternionf mag_body_quat_rotated = Quaternionf_mul(mag_body_quat,
                    q_conj);
            Vector3f mag_body =
            { mag_body_quat_rotated.x, mag_body_quat_rotated.y, 0.0f /* Projection dans le plan XY */
            };
            mag_body = vector3f_getNormalized(mag_body);

            const Vector3f north_body =
            { 1.0f, 0.0f, 0.0f }; /* Axes X (Nord) */
            const Vector3f cross =
            { north_body.y * mag_body.z - north_body.z * mag_body.y,
                    north_body.z * mag_body.x - north_body.x * mag_body.z,
                    north_body.x * mag_body.y - north_body.y * mag_body.x };

            /* heading = atan2((north × mag).z, north · mag) */
            ahrs->heading = safe_atan2(cross.z,
                    north_body.x * mag_body.x + north_body.y * mag_body.y);
        }
    }

    return AHRS_OK;
}
