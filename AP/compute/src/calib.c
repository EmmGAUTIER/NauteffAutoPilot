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

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "calib.h"
#include "rlib.h"

#include "util.h"
#include "rlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "util.h"
#include "rlib.h"
#include "service.h"
#include "printf.h"

#define GRAVITY (9.8066F)
#define MIN_SAMPLES_NUMBER 500
#define MAX_SAMPLES_NUMBER 500
#define STABILITY_THRESHOLD 100
#define MIN_STAB_NUMBER 50
#define MAX_DIFFERENCE 100
#define LPF_COEF 0.9F

typedef struct
{
    int s1;
    int dirref;
    int dirother1;
    int dirother2;
} SelectValues;
static SelectValues svs[] =
{
    { +1, 2, 0, 1 }, /* First face : z down, x and y near horizontal */
    /* turn right 90 deg. viewed from 'rear' */
    { +1, 1, 0, 2 }, /* 2nd face : y down, x and z near horizontal */
    /* turn right 90 deg. again */
    { -1, 2, 0, 1 }, /* 3rd face : z up, x and y near horizontal */
    /* turn right 90 deg. again */
    { -1, 1, 0, 2 }, /* 4th face : y up, x and z near horizontal */
    /* turn right 90 deg. again, turn 90 deg. by lifting rear */
    { 1, 0, 1, 2 }, /* 5th face : x down, y and z nearly horizontal */
    /* turn 180 deg. x down */
    { -1, 0, 1, 2 }/* 6th face  x up, y and z nearly horizontal */
};

/*
 * @brief Determine on which face the accelerometer is
 * @param accint pointer to int[3] array containing the accelerometer values (x,y,z)
 * @return index of the face (0..5) or -1 if not on a face
 */

int Calib6_getFace(const int *accint)
{
    int faceIdx = -1;

    for(int i = 0; i < 6 && faceIdx == -1; i++)
    {
        SelectValues *sv = &svs[i];

        if(sv->s1 * accint[sv->dirref]
                > 2 * (abs(accint[sv->dirother1]) + abs(accint[sv->dirother2])))
        {
            faceIdx = i;
        }
    }

    return faceIdx;
}

/*
 * @brief Initilize a Calib6 structure
 * @param calib structure to init
 */
void Calib6_init(Calib6_t *calib)
{
    /* J'avoue que c'est un peu grossier. */
    /* TODO faire une initialisation plus rigoureuse */
    memset((void*) calib, 0x0, sizeof(Calib6_t));
    calib->faceIdx = -1;
}

int Calib6_addSample(Calib6_t *calib, const Calib6_typeSensor_t type,
                     const int *values)
{
    int ret;
    int faceIdx;
    int stabOnFace;
    int faceCalibrated;
    int diff;

    /* differences are computed by adding absolute values of 3 axis */
    /* This is simpler and more efficient than euclidian distance */

    ret = 0;

    switch(type)
    {
    case Calib6_TypeAcc:

        if(calib->started == 0)
        {
            calib->acclpf[0] = values[0];
            calib->acclpf[1] = values[1];
            calib->acclpf[2] = values[2];
            calib->started = 1;
        }

        calib->acclpf[0] = (int)(calib->acclpf[0] * LPF_COEF
                                 + values[0] * (1.F - LPF_COEF));
        calib->acclpf[1] = (int)(calib->acclpf[1] * LPF_COEF
                                 + values[1] * (1.F - LPF_COEF));
        calib->acclpf[2] = (int)(calib->acclpf[2] * LPF_COEF
                                 + values[2] * (1.F - LPF_COEF));
        diff = abs(calib->acclpf[0] - values[0])
               + abs(calib->acclpf[1] - values[1])
               + abs(calib->acclpf[2] - values[2]);
        faceIdx = Calib6_getFace(values);
        stabOnFace = ((faceIdx >= 0) && (faceIdx == calib->faceIdx)
                      && (diff < STABILITY_THRESHOLD));
        faceCalibrated = ((faceIdx >= 0)
                          && (calib->facesDone & (0x1 << faceIdx)));

        if(stabOnFace)
        {
            if(calib->stabNumber < MIN_STAB_NUMBER)
            {
                calib->stabNumber++;
            }
            else
            {
                calib->accumulating = 1;
            }
        }

        if(calib->accumulating && stabOnFace && (!faceCalibrated))
        {
            calib->accint[0] += values[0];
            calib->accint[1] += values[1];
            calib->accint[2] += values[2];
            calib->nbSampleAcc++;
        }

        if(calib->accumulating && ((!stabOnFace) || ((calib->nbSampleAcc >= MAX_SAMPLES_NUMBER)
                                   && (calib->nbSampleGyr >= MAX_SAMPLES_NUMBER)
                                   && (calib->nbSampleMag >= MAX_SAMPLES_NUMBER))))
        {
            if((calib->nbSampleAcc >= MIN_SAMPLES_NUMBER)
                    && (calib->nbSampleGyr >= MIN_SAMPLES_NUMBER)
                    && (calib->nbSampleMag >= MIN_SAMPLES_NUMBER))
            {
                float nbSampleDivider;
                nbSampleDivider = (float) calib->nbSampleAcc;
                calib->acc_x[faceIdx] = ((float) calib->accint[0])
                                        / nbSampleDivider;
                calib->acc_y[faceIdx] = ((float) calib->accint[1])
                                        / nbSampleDivider;
                calib->acc_z[faceIdx] = ((float) calib->accint[2])
                                        / nbSampleDivider;
                nbSampleDivider = (float) calib->nbSampleGyr;
                calib->gyr_x[faceIdx] = ((float) calib->gyrint[0])
                                        / nbSampleDivider;
                calib->gyr_y[faceIdx] = ((float) calib->gyrint[1])
                                        / nbSampleDivider;
                calib->gyr_z[faceIdx] = ((float) calib->gyrint[2])
                                        / nbSampleDivider;
                nbSampleDivider = (float) calib->nbSampleMag;
                calib->mag_x[faceIdx] = ((float) calib->magint[0])
                                        / nbSampleDivider;
                calib->mag_y[faceIdx] = ((float) calib->magint[1])
                                        / nbSampleDivider;
                calib->mag_z[faceIdx] = ((float) calib->magint[2])
                                        / nbSampleDivider;
                calib->facesDone = calib->facesDone | (0x1 << faceIdx);
                char message[128];
                snprintf(message, sizeof(message),
                         "CALIB : face %d done  acc %f %f %f   gyr %+7.2f %+7.2f %+7.2f   mag %+7.2f %+7.2f %+7.2f\n",
                         faceIdx,
                         calib->acc_x[faceIdx], calib->acc_y[faceIdx],
                         calib->acc_z[faceIdx],
                         calib->gyr_x[faceIdx], calib->gyr_y[faceIdx],
                         calib->gyr_z[faceIdx],
                         calib->mag_x[faceIdx], calib->mag_y[faceIdx],
                         calib->mag_z[faceIdx]);
                snprintf(message, sizeof(message), "CALIB acc %f %f %f\n",
                         calib->acc_x[faceIdx], calib->acc_y[faceIdx],
                         calib->acc_z[faceIdx]);
                svc_UART_Write(&svc_uart2, message, strlen(message),
                               pdMS_TO_TICKS(1));
            }
            else
            {
                /* not enough int samples collected clear accumulated values for next trial */
            }

            calib->accint[0] = 0;
            calib->accint[1] = 0;
            calib->accint[2] = 0;
            calib->gyrint[0] = 0;
            calib->gyrint[1] = 0;
            calib->gyrint[2] = 0;
            calib->magint[0] = 0;
            calib->magint[1] = 0;
            calib->magint[2] = 0;
            calib->accumulating = 0;
            calib->nbSampleAcc = 0;
            calib->nbSampleGyr = 0;
            calib->nbSampleMag = 0;
            calib->stabNumber = 0;
        }

        calib->faceIdx = faceIdx;
        calib->stabOnFace = stabOnFace;

        break;

    case Calib6_TypeGyr:
        if(calib->accumulating)
        {
            calib->gyrint[0] += values[0];
            calib->gyrint[1] += values[1];
            calib->gyrint[2] += values[2];
            calib->nbSampleGyr++;
        }

        break;

    case Calib6_TypeMag:
        if(calib->accumulating)
        {
            calib->magint[0] += values[0];
            calib->magint[1] += values[1];
            calib->magint[2] += values[2];
            calib->nbSampleMag++;
        }

        break;
    }

    /* return positive number when calibrated on 6 faces */
    ret = calib->facesDone;

    return ret;
}

int Calib6_hasEnoughSamples(Calib6_t *calib)
{
    return calib->facesDone == 0x3F;
}

int Calib6_isPhaseComplete(Calib6_t *calib)
{
    return 0;
}

int Calib6_getPhase(Calib6_t *calib)
{
    return 0;
}

int Calib6_ComputeCorrector(Calib6_t *calib, Corrector_t *corr)
{
    int res;

    if(calib->facesDone != 0x3F)
    {
        res = -1;
    }
    else
    {
        float meanx, meany, meanz;
        //float cumulx, cumuly, cumulz;
        float minx;
        float maxx;
        float miny;
        float maxy;
        float minz;
        float maxz;

        minx = calib->acc_x[5];
        maxx = calib->acc_x[4];
        miny = calib->acc_y[3];
        maxy = calib->acc_y[1];
        minz = calib->acc_z[2];
        maxz = calib->acc_z[0];

        corr->accCorrOffsetx = -(maxx + minx) / 2.f;
        corr->accCorrOffsety = -(maxy + miny) / 2.f;
        corr->accCorrOffsetz = -(maxz + minz) / 2.f;

        corr->accCorrGainx = GRAVITY / ((maxx - minx) / 2.f);
        corr->accCorrGainy = GRAVITY / ((maxy - miny) / 2.f);
        corr->accCorrGainz = GRAVITY / ((maxz - minz) / 2.f);

        /* Gyro bias */
        meanx = meany = meanz = .0F;

        for(int i = 0; i < 6; i++)
        {
            meanx += calib->gyr_x[i];
            meany += calib->gyr_y[i];
            meanz += calib->gyr_z[i];
        }

        meanx /= 6.F;
        meany /= 6.F;
        meanz /= 6.F;

        corr->gyrCorrOffsetx = meanx;
        corr->gyrCorrOffsety = meany;
        corr->gyrCorrOffsetz = meanz;

        /* Gyro gain :  not yet calibrated */

        /* Magnetometer */

        maxx = minx = calib->mag_x[0];
        maxy = miny = calib->mag_y[0];
        maxz = minz = calib->mag_z[0];

        for(int i = 1; i < 6; i++)
        {
            minx = fminf(minx, calib->mag_x[i]);
            miny = fminf(miny, calib->mag_y[i]);
            minz = fminf(minz, calib->mag_z[i]);
            maxx = fmaxf(maxx, calib->mag_x[i]);
            maxy = fmaxf(maxy, calib->mag_y[i]);
            maxz = fmaxf(maxz, calib->mag_z[i]);
        }

        corr->magCorrOffsetx = -(minx + maxx) / 2.F;
        corr->magCorrOffsety = -(miny + maxy) / 2.F;
        corr->magCorrOffsetz = -(minz + maxz) / 2.F;

        /* arbitrary 1000.F */
        corr->magCorrGainx = 1000.F / (maxx - minx);
        corr->magCorrGainy = 1000.F / (maxy - miny);
        corr->magCorrGainz = 1000.F / (maxz - minz);

        res = 0;
    }

    return res;
}
