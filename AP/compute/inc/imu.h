
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

Calibreur_t *calibreur_create(int numberOfSamples);

int calibreur_addSample(Calibreur_t *calibreur, float currTime, Vector3f acc, Vector3f gyr, Vector3f mag);
int calibreur_getNumberOfSamples(Calibreur_t *calibreur);
int calibreur_isFull(Calibreur_t *calibreur);
int calibreur_calibrate(Calibreur_t *calibreur, Vector3f *offset, float M[3][3], float *quality);

void calibreur_destroy(Calibreur_t *calibreur);

#endif /* IMU_H */