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
    mstatus->magVsGyr = MEMS_INIT_MAG_VS_GYRO;
    mstatus->orientation = (Quaternionf){1.F, 0.F, 0.F, 0.F};
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

Quaternionf IMU_getQuaternion(IMU_Status_t *mstatus)
{
    return mstatus->orientation;
}

int IMU_set_mag_vs_gyr_prop(IMU_Status_t *mstatus, float prop)
{
    mstatus->magVsGyr = prop;

    return 0;
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

int IMU_new_values_essai(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat)
{
    // Vector3f m_h;         /* Horizontal part of magnétic field */
    // Vector3f m_v;         /* Vertical part of magnétic field */
    Vector3f starboard_h; /* Horizontal part of forward idirection */
    Vector3f east;        /* East horizontal dir to East */
    // Vector3f north;       /* North horizontal direction  to North */
    Vector3f gravity; /* Gravity */
    Vector3f dir3;
    float cos_hdg;
    float sin_hdg;

    char message[80]; /* TODO remove after debugging */

    /* Compute gravity; gravity is a normalized vector downward */
    gravity = vector3f_getNormalized(*acc);

#if 1
    snprintf(message, sizeof(message), "IMU gravity  %+f %+f %+f\n", gravity.x, gravity.y, gravity.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif
    /* horizontal part of forward relative to device */
    starboard_h = vector3f_getCrossProduct(unitmyf, gravity);
    starboard_h = vector3f_getNormalized(starboard_h);
#if 1
    snprintf(message, sizeof(message), "IMU stbd Horizontal  %+f %+f %+f\n", starboard_h.x, starboard_h.y, starboard_h.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    east = vector3f_getCrossProduct(*mag, gravity);
    east = vector3f_getNormalized(east);

#if 0
    snprintf(message, sizeof(message), "IMU East  %+f %+f %+f\n", east.x, east.y, east.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

    cos_hdg = vector3f_getDotProduct(vector3f_getCrossProduct(starboard_h, east), gravity);
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

#if 0
int IMU_new_values_ref(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat)
{

    Vector3f m_h; /* Horizontal part of magnétic field */
    Vector3f m_v; /* Vertical part of magnétic field */
    // Vector3f f_v;       /* Vertical part of forward idirection */
    Vector3f starboard_h; /* Horizontal part of forward idirection */
    Vector3f east;        /* East horizontal dir to East */
    Vector3f north;       /* North horizontal direction  to North */
    Vector3f gravity;     /* Gravity */
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
#if 0
    snprintf(message, sizeof(message), "IMU att1  %+f\n", mstatus->heading);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

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
    // forward_h = vector3f_sub(unitmxf, f_v);
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

    // north = vector3f_getCrossProduct(gravity, east);
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
#endif

#if 1

int IMU_new_values(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat)
{
    Vector3f dir1, dir3;
    float heading_mag;
    float gyrhdg; /* gyr->z over water, ie with roll and pitch compensation */
    float delta_heading_mag;
    static char message[100];
    // float accnorm;

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
        heading_mag = (M_PI / 2.) + atan2f(dir3.y, dir3.x);
        heading_mag = cvt_dir_norm_rad(heading_mag);
        float delta_mag = heading_mag - mstatus->heading;

        // accnorm = vector3f_getNorm(*acc);
        //  float ax2 = acc->x * acc->x;
        // float ay2 = acc->y * acc->y;
        // float az2 = acc->z * acc->z;
        // float cosphi = acc->y / sqrtf(ay2 + az2);
        // float sinphi = acc->z / sqrtf(ay2 + az2);
        // float costetha = -acc->x / accnorm;
        // float sintetha = sqrtf(ay2 + az2) / accnorm;

        gyrhdg = gyr->z * deltat;
        gyrhdg = cvt_dir_norm_rad(gyrhdg);

        mstatus->heading = mstatus->heading + (1.F - mstatus->magVsGyr) * gyrhdg + mstatus->magVsGyr * delta_mag;
        mstatus->heading = cvt_dir_norm_rad(mstatus->heading);
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
#endif

Quaternionf calculer_orientation_navire(Vector3f nord, Vector3f est, Vector3f bas)
{
    Quaternionf q;

    // Matrice de rotation (repère navire -> repère monde)
    // nord correspond à l'axe x du navire
    // est correspond à l'axe y du navire
    // bas correspond à l'axe z du navire
    float m00 = nord.x, m01 = est.x, m02 = bas.x;
    float m10 = nord.y, m11 = est.y, m12 = bas.y;
    float m20 = nord.z, m21 = est.z, m22 = bas.z;

    // Conversion matrice de rotation -> quaternion
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

// Convertit un quaternion en angles d'Euler (en radians)
// Convention : Yaw-Pitch-Roll (Z-Y-X)
void quaternion_vers_euler(Quaternionf q, float *roll, float *pitch, float *yaw)
{
    // AnglesEuler angles;

    // Roulis (rotation autour de x)
    float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    *pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    // Lacet (rotation autour de z) = cap
    float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
    *yaw = atan2f(siny_cosp, cosy_cosp);

    return;
}

/*
 *
 *
 *
 * @note z axis of device points downward, and acceleration points upward
 *       acceleration points upward and has to be inverted
 *       to point downward (gravity)
 *
 */
void IMU_update_quat(IMU_Status_t *mstatus, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat)
{
    char message[100];  /* for debugging only */
    Quaternionf q_new;  /* new quaternion */
    Vector3f down;      /* -acc, normalized*/
    Vector3f east;      /* east, normalized */
    Vector3f north;     /* north, normalized */
    Quaternionf q_pred; /* Quaternion computed with last one and gyro values */
    // Vector3f rotdir;    /* axe et direction de la rotation */
    //  float cosrot;       /* cos de l'angle de rotation */
    Quaternionf q_ref; /* reference quaternion computed with acc and mag */

    // Step 1: Predict new orientation using gyroscope data
    Quaternionf gyro_quat_derivative;
    gyro_quat_derivative.w = 0.0f;
    gyro_quat_derivative.x = gyr->x;
    gyro_quat_derivative.y = gyr->y;
    gyro_quat_derivative.z = gyr->z;

    Quaternionf q_dot = Quaternionf_mul(mstatus->orientation, gyro_quat_derivative);
    q_dot = Quaternionf_getScaled(q_dot, 0.5f);

    q_pred = Quaternionf_add(mstatus->orientation, Quaternionf_getScaled(q_dot, deltat));
    q_pred = Quaternionf_getNormalized(q_pred);

    // Step 2: Compute the reference orientation using accelerometer and magnetometer data
    down = vector3f_getScaled(*acc, -1.0f); // Invert to get gravity direction
    down = vector3f_getNormalized(down);

    /* Compute Est. Use cross product of down and mag */
    east = vector3f_getCrossProduct(down, *mag);
    east = vector3f_getNormalized(east);

    /* Compute North as cross product of down and east */
    north = vector3f_getCrossProduct(east, down);
    north = vector3f_getNormalized(north);

    /* we have North, East and down, let's compute the quaternion of orientation */
    q_ref = calculer_orientation_navire(north, east, down);
    if (mstatus->initialized == false)
    {
        mstatus->initialized = true;
        mstatus->orientation = q_ref;
    }
    else
    {
        q_new = Quaternionf_getScaled(q_ref, 0.05f);
        q_new = Quaternionf_add(q_new, q_pred);
        mstatus->orientation = Quaternionf_getNormalized(q_new);
    }

    float roll, pitch, yaw;
    quaternion_vers_euler(q_ref, &roll, &pitch, &yaw);
    mstatus->roll = roll;
    mstatus->pitch = pitch;
    mstatus->heading = yaw;

#if 0
    snprintf(message, sizeof(message), "IMU north %+f %+f %+f\n", north.x, north.y, north.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    snprintf(message, sizeof(message), "IMU east %+f %+f %+f\n", east.x, east.y, east.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    snprintf(message, sizeof(message), "IMU  down %+f %+f %+f\n", down.x, down.y, down.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    snprintf(message, sizeof(message), "IMU ref quat %+f %+f %+f %+f\n", q_ref.w, q_ref.x, q_ref.y, q_ref.z);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
    snprintf(message, sizeof(message), "IMU angles %+f %+f %+f\n", roll, pitch, yaw);
    svc_UART_Write(&svc_uart2, message, strlen(message), 0U);
#endif

// Step 3: Apply complementary filter to combine predicted and reference quaternions
#if 0
    q_new = Quaternionf_getScaled(q_predicted, (1 - mstatus->magVsGyr));
    Quaternionf q_ref_scaled = Quaternionf_getScaled(q_ref, mstatus->magVsGyr);
    q_new = Quaternionf_add(q_new, q_ref_scaled);

    // Normalize the final quaternion
    q_new = Quaternionf_getNormalized(q_new);

    // Update the IMU status with the new orientation
    mstatus->orientation = q_new;
#endif
}
