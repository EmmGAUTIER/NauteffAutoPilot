#ifndef AHRS_H
#define AHRS_H

#include "geom.h"
#include "quat.h"

//#include "ahrssimple.h"

#define AHRS_TYPES_NUMBER 1

typedef enum
{
    AHRS_TYPE_NONE = -1,
    AHRS_TYPE_SIMPLE = 0,
    AHRS_TYPE_QUAT,
    AHRS_TYPE_DT0058,
    //AHRS_TYPE_MADGWICK,
    //AHRS_TYPE_KALMAN,
    //AHRS_TYPE_MAHONY,
} AHRS_Types ;

typedef struct
{
    Vector3f acc;
    Vector3f gyr;
    Vector3f mag;
    float roll, pitch, heading;
    float rollRate, pitchRate, yawRate;
    bool initialized;
    float magVsGyr;
    Quaternionf orientation;
    float  hdg_x, hdg_y;
} AHRS_Status_t;

typedef struct
{
    void (*AHRS_init)(AHRS_Status_t*);
    float (*AHRS_get_roll)(AHRS_Status_t*);
    float (*AHRS_get_pitch)(AHRS_Status_t*);
    float (*AHRS_get_heading)(AHRS_Status_t*);
    float (*AHRS_get_yawRate)(AHRS_Status_t*);
    void (*AHRS_set_mag_vs_gyr_prop)(AHRS_Status_t*, float prop);
    int (*AHRS_update)(AHRS_Status_t*, Vector3f *acc, Vector3f *gyr, Vector3f *mag, float deltat);
    Quaternionf(*AHRS_get_Quaternion)(AHRS_Status_t*);
} AHRS_Interface_t;

extern const AHRS_Interface_t *AHRS_Interfaces[];

/*
 * @brief Initialise the AHRS_status_t structure.
 * @param mstatus Pointer to the AHRS_status_t structure to initialize.
 * return None.
 */
void AHRS_init(AHRS_Status_t *mstatus);

/*
 * @brief Compute new orientation whith mems values
 * @parameter mstatus pointer to AHRS structure to update
 * @parameter acc pointer to accelerometer data
 * @parameter gyr pointer to gyroscope data
 * @parameter mag pointer to magnetometer data
 * @parameter deltat time elapsed since last update in seconds
 * @return 1
 */
int AHRS_update(AHRS_Status_t *mstatus,
                Vector3f *acc,
                Vector3f *gyr,
                Vector3f *mag,
                float deltat);

/*
 * @brief Get the roll angle from the AHRS status.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The roll angle in radians.
 */
float AHRS_get_roll(AHRS_Status_t *mstatus);

/*
 * @brief Get the pitch angle from the AHRS status.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The pitch angle in radians.
 */
float AHRS_get_pitch(AHRS_Status_t *mstatus);

/*
 * @brief Get the heading angle from the AHRS status.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The heading angle in radians.
 */
float AHRS_get_heading(AHRS_Status_t *mstatus);

/*
 * @brief Get the yawrate from the AHRS status.
 * yawrate is the rotation speed around z axis, in rad/s
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The heading angle in radians.
 */
float AHRS_get_yawRate(AHRS_Status_t *mstatus);

/*
 * @brief Set the proportion of magnetometer vs gyroscope for heading estimation.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * prop is a value between 0 and 1, where 0 means only gyroscope is used and 1 means only magnetometer is used.
 * @return The heading angle in radians.
 */
void AHRS_set_mag_vs_gyr_prop(AHRS_Status_t *mstatus, float prop);

/*
 * @brief Get the quaternion of orientatoin from the AHRS status.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The quaternion of orientation.
 */
Quaternionf AHRS_get_Quaternion(AHRS_Status_t *mstatus);

/*
 *
 * @brief compute orientation quaternion from north, east and down
 *
 * @param north coordinates of north from sensor
 * @param east coordinates of east from sensor
 * @param down coordinates of down (gravity) from sensor
 *
 * @return normalized quaternion of orientation.
 */
Quaternionf calculer_orientation_navire(Vector3f north,
                                        Vector3f east,
                                        Vector3f down);

/*
 *
 * @brief Converts a quaternion in Euler angles
 * @param quaternion of orientation
 * @param roll, pitch, yaw pointers to float to store Euler angles
 */
void quaternion_vers_euler(Quaternionf q, float *roll, float *pitch, float *yaw);

#endif /* AHRS_H */