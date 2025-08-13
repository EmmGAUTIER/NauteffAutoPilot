#include "FreeRTOS.h"

#include "geom.h"
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