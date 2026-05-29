/*
 MIT License

 Copyright (c) 2025 Emmanuel Gautier / Nauteff

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

#ifndef CALIB_H
#define CALIB_H

#include "util.h"
#include "geom.h"

typedef enum
{
    Calib6_TypeAcc = 0,
    Calib6_TypeGyr,
    Calib6_TypeMag,
} Calib6_typeSensor_t;

typedef struct
{
    /* Cumul done on 32 bit integers in order to avoid truncating errors */
    /* acc, gyr and mag values as int[3] */
    int started;
    int faceIdx;
    //int onFace; /* 0 : not on a face, 1 : on a face. */
    int stabNumber; /* Number of samples since sensor stabilized */
    int stabOnFace;
    int accumulating;
    unsigned facesDone;

    int32_t acclpf[3];

    int32_t accint[3];
    int32_t gyrint[3];
    int32_t magint[3];

    float acc_x[6];
    float acc_y[6];
    float acc_z[6];
    float gyr_x[6];
    float gyr_y[6];
    float gyr_z[6];
    float mag_x[6];
    float mag_y[6];
    float mag_z[6];

    float magMin_x;
    float magMax_x;
    float magMin_y;
    float magMax_y;
    float magMin_z;
    float magMax_z;

    unsigned nbSampleAcc;
    unsigned nbSampleGyr;
    unsigned nbSampleMag;
} Calib6_t;

typedef struct
{
    float accCorrOffsetx;
    float accCorrOffsety;
    float accCorrOffsetz;
    float accCorrGainx;
    float accCorrGainy;
    float accCorrGainz;

    float gyrCorrOffsetx;
    float gyrCorrOffsety;
    float gyrCorrOffsetz;
    float gyrCorrGainx;
    float gyrCorrGainy;
    float gyrCorrGainz;

    float magCorrOffsetx;
    float magCorrOffsety;
    float magCorrOffsetz;
    float magCorrGainx;
    float magCorrGainy;
    float magCorrGainz;

} Corrector_t;

//INLINE float Calib6_getAccOffsetx (Calib6_t *cal) {return accOffsetx;}
INLINE Vector3f Corrector_getAccCorrected(Corrector_t *corr, Vector3f *acc)
{
    Vector3f result =
    {
        (acc->x + corr->accCorrOffsetx) * corr->accCorrGainx,
        (acc->y + corr->accCorrOffsety) * corr->accCorrGainy,
        (acc->z + corr->accCorrOffsetz) * corr->accCorrGainz
    };
    return result;
}
INLINE Vector3f Corrector_getGyrCorrected(const Corrector_t *corr,
        const Vector3f *gyr)
{
    Vector3f result =
    {
        (gyr->x + corr->gyrCorrOffsetx) * corr->gyrCorrGainx,
        (gyr->y + corr->gyrCorrOffsety) * corr->gyrCorrGainy,
        (gyr->z + corr->gyrCorrOffsetz) * corr->gyrCorrGainz
    };
    return result;
}
INLINE Vector3f Corrector_getMagCorrected(const Corrector_t *corr,
        const Vector3f *mag)
{
    Vector3f result =
    {
        (mag->x + corr->magCorrOffsetx) * corr->magCorrGainx,
        (mag->y + corr->magCorrOffsety) * corr->magCorrGainy,
        (mag->z + corr->magCorrOffsetz) * corr->magCorrGainz
    };
    return result;
}

void Calib6_init(Calib6_t *calib);

int Calib6_getFace(const int *accint);

int Calib6_addSample(Calib6_t *calib, const Calib6_typeSensor_t type, const int *values);

int Calib6_hasEnoughSamples(Calib6_t *calib);

int Calib6_getPhase(Calib6_t *calib);

int Calib6_isPhaseComplete(Calib6_t *calib);

int Calib6_ComputeCorrector(Calib6_t *calib, Corrector_t *corr);

#endif /* CALIB_H*/
