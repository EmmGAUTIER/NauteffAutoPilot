

#ifndef AHRSQUAT_H
#define AHRSQUAT_H

/* Réglages du calul */

#define MEMS_INIT_MAG_VS_GYRO 0.1F /* Répartition de partie du compas et dy gyromètre pour le cap */

extern const AHRS_Interface_t AHRS_Quat_Interface;

/*
 * @brief Compute new orientation whith mems values
 * @parameter mstatus pointer to AHRS structure to update
 * @parameter acc pointer to accelerometer data
 * @parameter gyr pointer to gyroscope data
 * @parameter mag pointer to magnetometer data
 * @parameter deltat time elapsed since last update in seconds
 * @return 1
 */
int AHRS_Quat_update(AHRS_Status_t *mstatus,
                       Vector3f *acc,
                       Vector3f *gyr,
                       Vector3f *mag,
                       float deltat);

#endif  /* AHRSQUAT_H */