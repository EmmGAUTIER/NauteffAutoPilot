#include "FreeRTOS.h"

#include "imu.h"

calibreur_addSample(Calibreur_t calibreur, float currTime, Vector3f acc, Vector3f gyr, Vector3f mag)
{
    // Add the sample to the calibrator
    calibreur->samples[calibreur->sampleCount].time = currTime;
    calibreur->samples[calibreur->sampleCount].acc = acc;
    calibreur->samples[calibreur->sampleCount].gyr = gyr;
    calibreur->samples[calibreur->sampleCount].mag = mag;
    calibreur->sampleCount++;
}
calibreur_create(int numberOfSamples)
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
    if (calibreur != NULL)
    {
        if (calibreur->samples != NULL)
        {
            vPortFree(calibreur->samples);
        }
        vPortFree(calibreur);
    }
}

int calibreur_getNumberOfSamples(Calibreur_t *calibreur)
{
    return calibreur->sampleCount;
}

int calibreur_isFull(Calibreur_t *calibreur)
{
    return calibreur->sampleCount > 0;
}

int calibreur_calibrate(Calibreur_t *calibreur, Vector3f *offset, float M[3][3], float *quality)
{
    if (calibreur->sampleCount < 30)
    {
        return -1; // Not enough samples
    }

    // Perform calibration logic here
    // For example, fit an ellipsoid to the samples and compute offset and M matrix

    // Placeholder for actual calibration logic
    *offset = (Vector3f){0.0f, 0.0f, 0.0f}; // Example offset
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            M[i][j] = 1.0f; // Example identity matrix
        }
    }
    *quality = 1.0f; // Example quality metric

    return 0; // Success
}   