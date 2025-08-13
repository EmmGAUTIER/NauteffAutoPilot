
#ifndef IMU_H
#define IMU_H

typedef struct
{
    float time;   /* Time of the sample */
    Vector3f acc; /* Accelerometer data */
    Vector3f gyr; /* Gyroscope data */
    Vector3f mag; /* Magnetometer data */
} MEMsSample_t;

typedef struct
{
    MEMsSample_t *samples;   /* Array of samples */
    unsigned int maxSamples; /* Maximum number of samples */
    unsigned sampleCount;    /* Number of samples added */
} Calibreur_t;

/*
 * @brief Create a calibrator with a specified number of samples
 * @param numberOfSamples Number of samples to allocate
 * @return Pointer to the created calibrator, or NULL on failure
 * This function allocates memory for the calibrator and its samples
 *  and uses the FreeRTOS pvPortMalloc() function.
 * It returns a pointer to the calibrator structure.
 * The caller is responsible for freeing the memory using calibreur_destroy.
 */
Calibreur_t *calibreur_create(int numberOfSamples);

/*
 * @brief Add a sample to the calibrator
 * @param calibreur Pointer to the calibrator
 * @param currTime Current time of the sample
 * @param acc Accelerometer data
 * @param gyr Gyroscope data
 * @param mag Magnetometer data
 * @return Number of samples after adding the new sample, or -1 if the calibrator is full
 * This function adds a sample to the calibrator's sample array.
 * When the samples count has reached the maximum, it doesn't add the sample.
 */
int calibreur_addSample(Calibreur_t *calibreur, float currTime, Vector3f *acc, Vector3f *gyr, Vector3f *mag);

/*
 * @brief returns the number of samples in the calibrator
 * @param calibreur Pointer to the calibrator
 * @return Number of samples in the calibrator
 */
int calibreur_getNumberOfSamples(Calibreur_t *calibreur);

/*
 * @brief Check if the calibrator is full
 * @param calibreur Pointer to the calibrator
 * @return 1 if the calibrator is full, 0 otherwise
 * This function checks if the number of samples in the calibrator has reached its maximum.
 */
int calibreur_isFull(Calibreur_t *calibreur);

/*
 * @brief Compute ellipsoid calibration from samples
 * @param calibreur Pointer to the calibrator
 * @param offset Pointer to store the offset vector
 * @param M Pointer to store the calibration matrix
 * @param quality Pointer to store the quality metric of the calibration
 * @return 0 on success, -1 if not enough samples for calibration
 * This function performs calibration logic, such as fitting an ellipsoid to the samples.
 * It computes the offset and M matrix, and returns a quality metric.
 */
int calibreur_calibrate(Calibreur_t *calibreur, Vector3f *offset, float M[3][3], float *quality);

/*
 * @brief Destroyes a calibrator and frees its memory
 * @param calibreur Pointer to the calibrator to destroy
 * @return None
 * This function frees the memory allocated for the calibrator and its samples.
 * It uses the FreeRTOS vPortFree() function.
 */
void calibreur_destroy(Calibreur_t *calibreur);

#endif /* IMU_H */