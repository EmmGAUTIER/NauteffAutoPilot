
#ifndef AHRSSIMPLE_H
#define AHRSSIMPLE_H

/* Réglages du calul */

#define MEMS_INIT_MAG_VS_GYRO 0.1F /* Répartition de partie du compas et dy gyromètre pour le cap */

extern const AHRS_Interface_t AHRS_Simple_Interface;

/*
 * @brief Initialise the AHRS_Simple_status_t structure.
 * @param mstatus Pointer to the AHRS_Simple_status_t structure to initialize.
 * return None.
 */
void AHRS_Simple_init(AHRS_Status_t *mstatus);

/*
 * @brief Compute new orientation whith mems values
 * @parameter mstatus pointer to AHRS structure to update
 * @parameter acc pointer to accelerometer data
 * @parameter gyr pointer to gyroscope data
 * @parameter mag pointer to magnetometer data
 * @parameter deltat time elapsed since last update in seconds
 * @return 1
 */
int AHRS_Simple_update(AHRS_Status_t *mstatus,
                       Vector3f *acc,
                       Vector3f *gyr,
                       Vector3f *mag,
                       float deltat);

/*
 * @brief Get the roll angle from the AHRS status.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The roll angle in radians.
 */
float AHRS_Simple_get_roll(AHRS_Status_t *mstatus);

/*
 * @brief Get the pitch angle from the AHRS status.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The pitch angle in radians.
 */
float AHRS_Simple_get_pitch(AHRS_Status_t *mstatus);

/*
 * @brief Get the heading angle from the AHRS status.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The heading angle in radians.
 */
float AHRS_Simple_get_heading(AHRS_Status_t *mstatus);

/*
 * @brief Get the yawrate from the AHRS status.
 * yawrate is the rotation speed around z axis, in rad/s
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The heading angle in radians.
 */
float AHRS_Simple_get_yawRate(AHRS_Status_t *mstatus);

/*
 * @brief Set the proportion of magnetometer vs gyroscope for heading estimation.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * prop is a value between 0 and 1, where 0 means only gyroscope is used and 1 means only magnetometer is used.
 * @return The heading angle in radians.
 */
void AHRS_Simple_set_mag_vs_gyr_prop(AHRS_Status_t *mstatus, float prop);

/*
 * @brief Get the quaternion of orientatoin from the AHRS status.
 * @param mstatus Pointer to the AHRS_Status_t structure
 * @return The quaternion of orientation.
 */
Quaternionf AHRS_Simple_get_Quaternion(AHRS_Status_t *mstatus);

#endif /* AHRSSIMPLE_H */
