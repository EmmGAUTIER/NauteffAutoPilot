#ifndef AHRS_DT005_H
#define AHRS_DT005_H

extern const AHRS_Interface_t AHRS_DT0058_Interface;

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
int AHRS_DT0058_update(AHRS_Status_t *mstatus, Vector3f *acc, Vector3f *gyr,
                       Vector3f *mag, float deltat);


#endif  /* AHRS_DT005_H */