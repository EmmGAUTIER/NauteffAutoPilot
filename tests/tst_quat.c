/*
 * tst_quat.c
 *
 *  Created on: 24 avr. 2026
 *      Author: Emmanuel Gautier
 */

#include <stdio.h>
#include "quat.h"

int main()
{
    Quaternionf qr;

    struct
    {
        Quaternionf q1;
        Quaternionf q2;
        Quaternionf qr;
    } dataref_mul[] =
            {
                    {
                            { .w = +0.7071, .x = 0., .y = 0., .z = +0.7071 },
                            { .w = +0.7071, .x = 0., .y = 0., .z = -0.7071 },
                            { 1.0, 0., 0., 0. } },
                    {
                    { 1.0, 2.0, 3., 4. },
                    { 5., 6., 7., 8. },
                    { -60., 12., 30., 24. } },

                    {
                        { 0.7071, 0, 0, 0.7071 },
                        { 0.7071, 0.7071, 0, 0 },
                        {0.5, 0.5, 0.5, 0.5}
                    },
                    {
                        { 0.7071, 0., 0., 0.7071 },
                        { 0.7071, 0., 0., 0.7071 },
                        {0., 0., 0., 1.0}
                    },
                    {Quaternionf_unit, {1.0, 2.0, 3.0, 4.0}, {1.0, 2.0, 3.0, 4.0}}

            };




    for (int i = 0; i < (sizeof(dataref_mul) / sizeof(dataref_mul[0])); i++)
    {

        printf("%d\n", i);
        Quaternionf q1 = dataref_mul[i].q1;
        Quaternionf q2 = dataref_mul[i].q2;
        Quaternionf qa = dataref_mul[i].qr;
        Quaternionf qr = Quaternionf_mul(q1, q2);

        printf(
                "%+8f %+8f %8f %8f    %+8f %+8f %+8f %+8f     %+8f %+8f %+8f %+8f\n",
                q1.w, q1.x, q1.y, q1.z,
                q2.w, q2.x, q2.y, q2.z,
                qr.w - qa.w, qr.x - qa.x, qr.y - qa.y, qr.z - qa.z);

    }

}

